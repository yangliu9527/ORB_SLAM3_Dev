#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../../../include/System.h"

//
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_depth_image_transport/rvl_codec.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

sensor_msgs::Image::Ptr DecodeCompressedDepthImage(const sensor_msgs::CompressedImage &message);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::CompressedImageConstPtr &msgRGB,
                  const sensor_msgs::CompressedImageConstPtr &msgD);

    ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 100);
    message_filters::Subscriber<sensor_msgs::CompressedImage> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw/compressedDepth", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::CompressedImageConstPtr &msgRGB,
                            const sensor_msgs::CompressedImageConstPtr &msgD)
{
    // 解压RGB图像（BGR格式）
    cv::Mat rgb_image = cv::imdecode(cv::Mat(msgRGB->data), cv::IMREAD_COLOR);
    if (rgb_image.empty())
    {
        ROS_ERROR("Failed to decode RGB image");
        return;
    }

    // 解压Depth图像（16位单通道）
    // cv::Mat depth_image = cv::imdecode(cv::Mat(msgD->data), cv::IMREAD_UNCHANGED);
    // if (depth_image.empty())
    // {
    //     ROS_ERROR("Failed to decode Depth image");
    //     return;
    // }

    cv::Mat depth_image;
    try {
        // 使用专用解码函数
        sensor_msgs::Image::Ptr sensor_ptrD = DecodeCompressedDepthImage(*msgD);
        cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(sensor_ptrD);
        depth_image = cv_ptrD->image;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Depth解码失败: %s", e.what());
        return;
    }




    // 注意：timestamp 是从 RGB 图像取的
    double timestamp = msgRGB->header.stamp.toSec();
    //cv::imshow("depth", depth_image);
    mpSLAM->TrackRGBD(rgb_image, depth_image, timestamp);
}

sensor_msgs::Image::Ptr DecodeCompressedDepthImage(const sensor_msgs::CompressedImage &message)
{
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy message header
    cv_ptr->header = message.header;

    // Assign image encoding
    const size_t split_pos = message.format.find(';');
    const std::string image_encoding = message.format.substr(0, split_pos);
    std::string compression_format;
    // Older version of compressed_depth_image_transport supports only png.
    if (split_pos == std::string::npos)
    {
        compression_format = "png";
    }
    else
    {
        std::string format = message.format.substr(split_pos);
        if (format.find("compressedDepth png") != std::string::npos)
        {
            compression_format = "png";
        }
        else if (format.find("compressedDepth rvl") != std::string::npos)
        {
            compression_format = "rvl";
        }
        else if (format.find("compressedDepth") != std::string::npos && format.find("compressedDepth ") == std::string::npos)
        {
            compression_format = "png";
        }
        else
        {
            ROS_ERROR("Unsupported image format: %s", message.format.c_str());
            return sensor_msgs::Image::Ptr();
        }
    }

    // cout <<"compression_format ="<<compression_format<<endl;
    // cout <<"image_encoding ="<<image_encoding;

    cv_ptr->encoding = image_encoding;

    // Decode message data
    if (message.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
    {

        // Read compression type from stream
        compressed_depth_image_transport::ConfigHeader compressionConfig;
        memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

        // Get compressed image data
        const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

        // Depth map decoding
        float depthQuantA, depthQuantB;

        // Read quantization parameters
        depthQuantA = compressionConfig.depthParam[0];
        depthQuantB = compressionConfig.depthParam[1];

        if (enc::bitDepth(image_encoding) == 32)
        {
            cv::Mat decompressed;
            if (compression_format == "png")
            {
                try
                {
                    // Decode image data
                    decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
                }
                catch (cv::Exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    return sensor_msgs::Image::Ptr();
                }
            }
            else
            {
                return sensor_msgs::Image::Ptr();
            }

            size_t rows = decompressed.rows;
            size_t cols = decompressed.cols;

            if ((rows > 0) && (cols > 0))
            {
                cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

                // Depth conversion
                cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                                    itDepthImg_end = cv_ptr->image.end<float>();
                cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                                  itInvDepthImg_end = decompressed.end<unsigned short>();

                for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
                {
                    // check for NaN & max depth
                    if (*itInvDepthImg)
                    {
                        *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
                    }
                    else
                    {
                        *itDepthImg = std::numeric_limits<float>::quiet_NaN();
                    }
                }

                // Publish message to user callback
                return cv_ptr->toImageMsg();
            }
        }
        else
        {
            // Decode raw image
            if (compression_format == "png")
            {
                try
                {
                    cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
                }
                catch (cv::Exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    return sensor_msgs::Image::Ptr();
                }
            }
            else
            {
                return sensor_msgs::Image::Ptr();
            }

            size_t rows = cv_ptr->image.rows;
            size_t cols = cv_ptr->image.cols;

            if ((rows > 0) && (cols > 0))
            {
                // Publish message to user callback
                return cv_ptr->toImageMsg();
            }
        }
    }
    return sensor_msgs::Image::Ptr();
}