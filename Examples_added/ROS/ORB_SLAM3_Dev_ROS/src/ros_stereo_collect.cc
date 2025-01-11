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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) { 
        ts_file.open(timestamps_save_path, ios::out); 
        ros_ts_file.open(ros_timestamps_save_path, ios::out);
        }

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM3::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    //==========Save Path========
    string save_path = "/home/zhiyu/DataSet/SelfCollected/LYR_AGV/12_27/files";
    string left_imgs_save_path = save_path + "/image_2";
    string right_imgs_save_path = save_path + "/image_3";
    string timestamps_save_path = save_path + "/times.txt";
    string ros_timestamps_save_path = save_path + "/times_ros.txt";
    fstream ts_file;
    fstream ros_ts_file;
    int count = 0;
    double first_ts = -1.0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 4)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {
        igb.M1l = SLAM.GetSettings()->M1l();
        igb.M2l = SLAM.GetSettings()->M2l();
        igb.M1r = SLAM.GetSettings()->M1r();
        igb.M2r = SLAM.GetSettings()->M2r();
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);

        //if (count < 300)
        {
            stringstream filenametream;
            // filenametream <<  setw(6) << setfill('0')<<to_string(kitti_id);
            filenametream << setw(6) << setfill('0') << to_string(int(count));
            string filename;
            filenametream >> filename;
            cv::imwrite(left_imgs_save_path + "/" + filename + ".png", imLeft);
            cv::imwrite(right_imgs_save_path + "/" + filename + ".png", imRight);
            if(first_ts<0)
            {
                first_ts = cv_ptrLeft->header.stamp.toSec();
            }
            double ts = cv_ptrLeft->header.stamp.toSec() - first_ts;
            // f << setprecision(9) << ts << endl;
            ts_file << setprecision(9)<<ts << endl;
            //cout<<"get "<<count<<" th image"<<endl;
            ros_ts_file << setprecision(20)<<cv_ptrLeft->header.stamp.toSec() << endl;
            count++;
        }
        // else
        // {
        //     ts_file.close();
        // }


        //mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        if (count < 100)
        {
            stringstream filenametream;
            // filenametream <<  setw(6) << setfill('0')<<to_string(kitti_id);
            filenametream << setw(6) << setfill('0') << to_string(int(count));
            string filename;
            filenametream >> filename;
            cv::imwrite(left_imgs_save_path + "/" + filename + ".png", cv_ptrLeft->image);
            cv::imwrite(right_imgs_save_path + "/" + filename + ".png", cv_ptrRight->image);
            if(first_ts<0)
            {
                first_ts = cv_ptrLeft->header.stamp.toSec();
            }
            double ts = cv_ptrLeft->header.stamp.toSec() - first_ts;
            // f << setprecision(9) << ts << endl;
            ts_file << setprecision(9)<<ts << endl;
            // f << setprecision(9) << ts << endl;
            ts_file << to_string(count * 0.1) << endl;
            cout<<"get "<<count<<" th image"<<endl;
            count++;
        }
        else
        {
            ts_file.close();
        }

        //mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }
}
