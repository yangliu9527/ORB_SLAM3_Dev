/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM)
        : Node("mono"), mpSLAM(pSLAM)
    {
        // Declare subscription to camera topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam0/image_raw", 10, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS2 image message to OpenCV image
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackMonocular(cv_ptr->image, rclcpp::Time(msg->header.stamp).seconds());
    }

private:
    ORB_SLAM3::System* mpSLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr << "Usage: ros2 run ORB_SLAM3 mono --ros-args -p path_to_vocabulary:=<path> -p path_to_settings:=<path>" << std::endl;
        return 1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Create ImageGrabber node
    auto image_grabber_node = std::make_shared<ImageGrabber>(&SLAM);

    // Run node
    rclcpp::spin(image_grabber_node);

    // Stop SLAM system
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();
    return 0;
}
