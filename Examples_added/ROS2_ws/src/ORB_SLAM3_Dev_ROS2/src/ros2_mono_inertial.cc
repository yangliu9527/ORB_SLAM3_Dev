#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

class ImuGrabber : public rclcpp::Node
{
public:
    ImuGrabber() : Node("imu_grabber") {}

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe, const string &paras)
        : Node("image_grabber"), mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe)
    {
        cv::FileStorage fSettings(paras, cv::FileStorage::READ);
        cv::FileNode td_node = fSettings["TimeDelay"]; // t_imu = t_img+td
        if (td_node.empty())
        {
            mtd = 0.0;
        }
        else
        {
            mtd = td_node.real();
        }
        RCLCPP_INFO(this->get_logger(), "Calibrated time shift = %f", mtd);
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;

    double mtd;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;

        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
        {
            tIm = img0Buf.front()->header.stamp.sec + img0Buf.front()->header.stamp.nanosec * 1e-9 + mtd;

            if (tIm > mpImuGb->imuBuf.back()->header.stamp.sec + mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9)
            {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mBufMutex);
                im = GetImage(img0Buf.front());
                img0Buf.pop();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            {
                std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                while (!mpImuGb->imuBuf.empty() &&
                       mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9 <= tIm)
                {
                    auto imu_msg = mpImuGb->imuBuf.front();
                    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                    cv::Point3f acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                    cv::Point3f gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }

            if (mbClahe)
                mClahe->apply(im, im);

            mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        return 1;
    }

    bool bEqual = (argc == 4 && std::string(argv[3]) == "true");

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    auto imu_grabber = std::make_shared<ImuGrabber>();
    auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, imu_grabber.get(), bEqual, argv[2]);

    auto imu_subscription = imu_grabber->create_subscription<sensor_msgs::msg::Imu>(
        "/imu0", 1000, std::bind(&ImuGrabber::GrabImu, imu_grabber.get(), std::placeholders::_1));
    auto image_subscription = image_grabber->create_subscription<sensor_msgs::msg::Image>(
        "/cam0/image_raw", 100, std::bind(&ImageGrabber::GrabImage, image_grabber.get(), std::placeholders::_1));

    std::thread sync_thread(&ImageGrabber::SyncWithImu, image_grabber.get());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imu_grabber);
    executor.add_node(image_grabber);

    executor.spin();

    sync_thread.join();
    rclcpp::shutdown();

    return 0;
}
