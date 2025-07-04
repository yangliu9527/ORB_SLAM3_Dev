// ROS2 RGBD + IMU同步方案，使用message_filters和ApproximateTime

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "System.h"
#include "ImuTypes.h"

#include <queue>
#include <mutex>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;

class ImuGrabber
{
public:
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
  }

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const std::string &settings)
      : mpSLAM(pSLAM), mpImuGb(pImuGb)
  {
    cv::FileStorage fSettings(settings, cv::FileStorage::READ);
    mtd = fSettings["TimeDelay"].empty() ? 0.0 : fSettings["TimeDelay"].real();
  }

  void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &rgb,
                const sensor_msgs::msg::Image::ConstSharedPtr &depth)
  {
    double tRGB = rclcpp::Time(rgb->header.stamp).seconds() + mtd;

    cv::Mat imRGB = GetRGB(rgb);
    cv::Mat imD = GetD(depth);

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
      std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
      while (!mpImuGb->imuBuf.empty())
      {
        auto imu_msg = mpImuGb->imuBuf.front();
        double t = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (t <= tRGB)
        {
          cv::Point3f acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
          cv::Point3f gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
          vImuMeas.emplace_back(acc, gyr, t);
          mpImuGb->imuBuf.pop();
        }
        else
          break;
      }
    }

    mpSLAM->TrackRGBD(imRGB, imD, tRGB, vImuMeas);
  }

private:
  cv::Mat GetRGB(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      return cv_ptr->image.clone();
    }
    catch (...) { return cv::Mat(); }
  }

  cv::Mat GetD(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvShare(msg);
      return cv_ptr->image.clone();
    }
    catch (...) { return cv::Mat(); }
  }

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;
  double mtd;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc < 3)
  {
    std::cerr << "Usage: ros2 run ORB_SLAM3 rgbd_inertial path_to_vocab path_to_settings" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("rgbd_inertial");

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
  ImuGrabber imu_grabber;
  ImageGrabber image_grabber(&SLAM, &imu_grabber, argv[2]);

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
  std::string rgb_topic = "/jetbot_camera/rgb";
  std::string depth_topic = "/jetbot_camera/depth";
  std::string imu_topic = "/jetbot_camera/imu";

  if (!fSettings["topics.rgb"].empty()) rgb_topic = (std::string)fSettings["topics.rgb"];
  if (!fSettings["topics.depth"].empty()) depth_topic = (std::string)fSettings["topics.depth"];
  if (!fSettings["topics.imu"].empty()) imu_topic = (std::string)fSettings["topics.imu"];

  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuGrabber::GrabImu, &imu_grabber, _1));

  using namespace message_filters;
  typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> RGBDPolicy;

  auto rgb_sub = std::make_shared<Subscriber<sensor_msgs::msg::Image>>(node.get(), rgb_topic);
  auto depth_sub = std::make_shared<Subscriber<sensor_msgs::msg::Image>>(node.get(), depth_topic);
  auto sync = std::make_shared<Synchronizer<RGBDPolicy>>(RGBDPolicy(10), *rgb_sub, *depth_sub);

  sync->registerCallback(std::bind(&ImageGrabber::GrabRGBD, &image_grabber, _1, _2));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
