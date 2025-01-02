#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const std::string& left_topic, const std::string& right_topic)
        : Node("ImageGrabber"), mpSLAM(pSLAM)
    {
        this->declare_parameter("do_rectify", false);
        do_rectify = this->get_parameter("do_rectify").as_bool();

        // Initialize subscriptions
        left_sub.subscribe(this, left_topic);
        right_sub.subscribe(this, right_topic);

        sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), left_sub, right_sub);
        sync->registerCallback(std::bind(&ImageGrabber::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

        if (do_rectify)
        {
            LoadRectificationParams();
        }
    }

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr& msgRight);

private:
    void LoadRectificationParams();

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync;
};

void ImageGrabber::LoadRectificationParams()
{
    std::string settings_file;
    this->declare_parameter("settings_file", "");
    settings_file = this->get_parameter("settings_file").as_string();

    cv::FileStorage fsSettings(settings_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open settings file: %s", settings_file.c_str());
        rclcpp::shutdown();
        return;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Calibration parameters to rectify stereo are missing!");
        rclcpp::shutdown();
        return;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
}

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft, imRight, msgLeft->header.stamp.sec + msgLeft->header.stamp.nanosec * 1e-9);
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msgLeft->header.stamp.sec + msgLeft->header.stamp.nanosec * 1e-9);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    auto node = std::make_shared<ImageGrabber>(&SLAM, "/cam0/image_raw", "/cam1/image_raw");

    rclcpp::spin(node);

    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    rclcpp::shutdown();

    return 0;
}
