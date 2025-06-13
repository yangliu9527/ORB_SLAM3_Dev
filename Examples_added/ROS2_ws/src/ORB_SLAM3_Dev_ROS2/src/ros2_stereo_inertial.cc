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
  }

  queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, bool bRect, bool bClahe, const string &params)
      : mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe)
  {
    cv::FileStorage fSettings(params, cv::FileStorage::READ);
    cv::FileNode td_node = fSettings["TimeDelay"];
    mtd = td_node.empty() ? 0.0 : td_node.real();
    RCLCPP_INFO(rclcpp::get_logger("ImageGrabber"), "Calibrated time shift: %f", mtd);
  }

  void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutexLeft);
    imgLeftBuf.push(msg);
  }

  void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mBufMutexRight);
    imgRightBuf.push(msg);
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

  void SyncWithImu()
  {
    const double maxTimeDiff = 0.01;
    while (rclcpp::ok())
    {
      cv::Mat imLeft, imRight;
      double tImLeft = 0, tImRight = 0;

      if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
      {
        {
          std::lock_guard<std::mutex> lock(mBufMutexLeft);
          tImLeft = imgLeftBuf.front()->header.stamp.sec + imgLeftBuf.front()->header.stamp.nanosec * 1e-9 + mtd;
        }
        {
          std::lock_guard<std::mutex> lock(mBufMutexRight);
          tImRight = imgRightBuf.front()->header.stamp.sec + imgRightBuf.front()->header.stamp.nanosec * 1e-9 + mtd;
        }

        if (abs(tImLeft - tImRight) > maxTimeDiff)
          continue;

        {
          std::lock_guard<std::mutex> lock(mBufMutexLeft);
          imLeft = GetImage(imgLeftBuf.front());
          imgLeftBuf.pop();
        }

        {
          std::lock_guard<std::mutex> lock(mBufMutexRight);
          imRight = GetImage(imgRightBuf.front());
          imgRightBuf.pop();
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
          std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
          while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.sec + mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9 <= tImLeft)
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
          mClahe->apply(imLeft, imLeft);
          mClahe->apply(imRight, imRight);
        }

        if (do_rectify)
        {
          cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
          cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
        }

        mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
      }
    }
  }

  queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;
  double mtd;
  const bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;
  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4 || argc > 5)
  {
    std::cerr << "Usage: ros2 run ORB_SLAM3 stereo_inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("stereo_inertial");

  bool do_rectify = std::string(argv[3]) == "true";
  bool do_equalize = (argc == 5 && std::string(argv[4]) == "true");

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);
  ImuGrabber imu_grabber;
  ImageGrabber image_grabber(&SLAM, &imu_grabber, do_rectify, do_equalize, argv[2]);

  string imu_topic = "/imu0";
  string left_topic = "/camera_array/cam0/image_raw";
  string right_topic = "/camera_array/cam1/image_raw";

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
  cv::FileNode td_node = fSettings["topics.imu"];
  if (!fSettings["topics.imu"].empty())
  {
    imu_topic = fSettings["topics.imu"].string();
  }

  if (!fSettings["topics.left_image"].empty())
  {
    left_topic = fSettings["topics.left_image"].string();
  }

  if (!fSettings["topics.right_image"].empty())
  {
    right_topic = fSettings["topics.right_image"].string();
  }

  cout << "imu topic: "<<imu_topic<<", left image topic: "<<left_topic<<", right image topic: "<<right_topic<<endl;

  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1000,
                                                                  std::bind(&ImuGrabber::GrabImu, &imu_grabber, std::placeholders::_1));
  auto sub_img_left = node->create_subscription<sensor_msgs::msg::Image>(left_topic, 100,
                                                                         std::bind(&ImageGrabber::GrabImageLeft, &image_grabber, std::placeholders::_1));
  auto sub_img_right = node->create_subscription<sensor_msgs::msg::Image>(right_topic, 100,
                                                                          std::bind(&ImageGrabber::GrabImageRight, &image_grabber, std::placeholders::_1));

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &image_grabber);

  rclcpp::spin(node);

  sync_thread.join();
  rclcpp::shutdown();
  return 0;
}
