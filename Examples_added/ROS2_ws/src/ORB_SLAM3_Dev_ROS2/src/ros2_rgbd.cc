
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, cv::FileStorage fSettings)
        : Node("rgbd_node"), mpSLAM(pSLAM)
    {
        string rgb_topic = "/jetbot_camera/rgb";
        string depth_topic = "/jetbot_camera/depth";
        cv::FileNode rgb_topic_node = fSettings["topics.rgb"]; // t_imu = t_img+td
        if (!rgb_topic_node.empty())
        {
            rgb_topic = rgb_topic_node.string();
        }

        cv::FileNode depth_topic_node = fSettings["topics.depth"];
        if (!depth_topic_node.empty())
        {
            depth_topic = depth_topic_node.string();
        }

        cout << "rgb topic: " << rgb_topic << ", depth topic: "<<depth_topic<<endl;

        // Subscribers
        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rgb_topic);
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic);

        // Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
        sync_->registerCallback(std::bind(&ImageGrabber::GrabRGBD, this, std::placeholders::_1, std::placeholders::_2));
    }

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
    {
        // Convert the ROS2 image messages to OpenCV Mat
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec + 1e-9 * msgRGB->header.stamp.nanosec);
    }

private:
    ORB_SLAM3::System* mpSLAM;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
};

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);

    // Create the ImageGrabber node
    auto node = std::make_shared<ImageGrabber>(&SLAM,fSettings);

    // Spin the node
    rclcpp::spin(node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();
    return 0;
}
