#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber() {}

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        imuBuf.push(imu_msg);
    }

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, bool bClahe)
        : mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe)
    {
        mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        if (!img0Buf.empty())
        {
            img0Buf.pop();
            imgDBuf.pop();
        }
        img0Buf.push(msgRGB);
        imgDBuf.push(msgD);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
            return cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
            return cv::Mat();
        }
    }

    cv::Mat GetDepth(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(img_msg);
            return cv_ptrD->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
            return cv::Mat();
        }
    }

    void SyncWithImu()
    {
        while (rclcpp::ok())
        {
            cv::Mat im, depth;
            double tIm = 0;

            if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
            {
                tIm = img0Buf.front()->header.stamp.sec + img0Buf.front()->header.stamp.nanosec * 1e-9;

                if (tIm > mpImuGb->imuBuf.back()->header.stamp.sec + mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9)
                    continue;

                {
                    std::lock_guard<std::mutex> lock(mBufMutex);
                    im = GetImage(img0Buf.front());
                    depth = GetDepth(imgDBuf.front());
                    img0Buf.pop();
                    imgDBuf.pop();
                }

                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                {
                    std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                    while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9 <= tIm)
                    {
                        auto imu_msg = mpImuGb->imuBuf.front();
                        double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                        cv::Point3f acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                        cv::Point3f gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                        vImuMeas.emplace_back(acc, gyr, t);
                        mpImuGb->imuBuf.pop();
                    }
                }

                if (mbClahe)
                {
                    mClahe->apply(im, im);
                }

                mpSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    queue<sensor_msgs::msg::Image::SharedPtr> img0Buf, imgDBuf;
    std::mutex mBufMutex;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 mono_inertial path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        return 1;
    }

    bool bEqual = (argc == 4 && std::string(argv[3]) == "true");

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);

    auto node = rclcpp::Node::make_shared("mono_inertial");

    ImuGrabber imu_grabber;
    ImageGrabber image_grabber(&SLAM, &imu_grabber, bEqual);

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/imu", 1000,
        [&imu_grabber](const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
            imu_grabber.GrabImu(imu_msg);
        });

    auto sub_rgb = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 100,
        [&image_grabber](const sensor_msgs::msg::Image::SharedPtr msgRGB) {
            image_grabber.GrabRGBD(msgRGB, nullptr);
        });

    auto sub_depth = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", 100,
        [&image_grabber](const sensor_msgs::msg::Image::SharedPtr msgD) {
            image_grabber.GrabRGBD(nullptr, msgD);
        });

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &image_grabber);

    rclcpp::spin(node);

    sync_thread.join();
    rclcpp::shutdown();
    return 0;
}
