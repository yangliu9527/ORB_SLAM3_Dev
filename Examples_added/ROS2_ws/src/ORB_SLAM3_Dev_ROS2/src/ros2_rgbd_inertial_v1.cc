#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

#include <queue>
#include <mutex>
#include <thread>
#include <iostream>

using namespace std;

class ImuGrabber
{
public:
  ImuGrabber() {}

  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
    //cout<<"get imu";
  }

  queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const string &params)
      : mpSLAM(pSLAM), mpImuGb(pImuGb)
  {
    cv::FileStorage fSettings(params, cv::FileStorage::READ);
    cv::FileNode td_node = fSettings["TimeDelay"];
    mtd = td_node.empty() ? 0.0 : td_node.real();
    RCLCPP_INFO(rclcpp::get_logger("ImageGrabber"), "Calibrated time shift: %f", mtd);
  }

  void GrabImageRGB(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutexRGB);
    imgRGBBuf.push(msg);
    //cout<<"get rgb";
  }

  void GrabImageD(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutexD);
    imgDBuf.push(msg);
    //cout<<"get depth";
  }

  cv::Mat GetRGB(const sensor_msgs::msg::Image::SharedPtr img_msg)
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

  cv::Mat GetD(const sensor_msgs::msg::Image::SharedPtr img_msg)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg);
      return cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
  }


  void SyncWithImu()
  {
    const double maxTimeDiff = 0.01;
    while (rclcpp::ok())
    {
      cv::Mat imRGB, imD;
      double tImRGB = 0, tImD = 0;

      if (!imgRGBBuf.empty() && !imgDBuf.empty() && !mpImuGb->imuBuf.empty())
      {
        {
          std::lock_guard<std::mutex> lock(mBufMutexRGB);
          tImRGB = imgRGBBuf.front()->header.stamp.sec + imgRGBBuf.front()->header.stamp.nanosec * 1e-9 + mtd;
        }
        {
          std::lock_guard<std::mutex> lock(mBufMutexD);
          tImD = imgDBuf.front()->header.stamp.sec + imgDBuf.front()->header.stamp.nanosec * 1e-9 + mtd;
        }

        if (abs(tImRGB - tImD) > maxTimeDiff)
          continue;

        {
          std::lock_guard<std::mutex> lock(mBufMutexRGB);
          imRGB = GetRGB(imgRGBBuf.front());
          imgRGBBuf.pop();
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexD);
          imD = GetD(imgDBuf.front());
          imgDBuf.pop();
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
          while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9 <= tImRGB)
          {
            auto imu_msg = mpImuGb->imuBuf.front();
            double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
            cv::Point3f acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
            cv::Point3f gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
            vImuMeas.emplace_back(acc, gyr, t);
            mpImuGb->imuBuf.pop();
          }
        }

        mpSLAM->TrackRGBD(imRGB, imD, tImRGB, vImuMeas);
      }
    }
  }

  queue<sensor_msgs::msg::Image::SharedPtr> imgRGBBuf, imgDBuf;
  std::mutex mBufMutexRGB, mBufMutexD;

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;
  double mtd;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3 || argc > 4)
  {
    std::cerr << "Usage: ros2 run ORB_SLAM3 stereo_inertial path_to_vocabulary path_to_settings" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("rgbd_inertial");


  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
  ImuGrabber imu_grabber;
  ImageGrabber image_grabber(&SLAM, &imu_grabber, argv[2]);

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
  string rgb_topic = "/jetbot_camera/rgb";
  string depth_topic = "/jetbot_camera/depth";
  string imu_topic = "/jetbot_camera/imu";
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

  cv::FileNode imu_topic_node = fSettings["topics.imu"];
  if (!imu_topic_node.empty())
  {
    imu_topic = imu_topic_node.string();
  }

  cout << "rgb topic: " << rgb_topic << ", depth topic: " << depth_topic << ", imu topic: "<<imu_topic << endl;

  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuGrabber::GrabImu, &imu_grabber, std::placeholders::_1));
  auto sub_img_rgb = node->create_subscription<sensor_msgs::msg::Image>(rgb_topic, 100,
      std::bind(&ImageGrabber::GrabImageRGB, &image_grabber, std::placeholders::_1));
  auto sub_img_depth = node->create_subscription<sensor_msgs::msg::Image>(depth_topic, 100,
      std::bind(&ImageGrabber::GrabImageD, &image_grabber, std::placeholders::_1));

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &image_grabber);

  rclcpp::spin(node);

  sync_thread.join();
  rclcpp::shutdown();
  return 0;
}
