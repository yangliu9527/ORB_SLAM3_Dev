#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
  ImuGrabber() {};
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const string &paras) : mpSLAM(pSLAM), mpImuGb(pImuGb)
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
    cout << "calibrated time shift = " << mtd << endl;
  }
  void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
  void GrabRGB(const sensor_msgs::ImageConstPtr &img_msg);
  void GrabD(const sensor_msgs::ImageConstPtr &img_msg);
  void SyncWithImu();

  cv::Mat GetRGB(const sensor_msgs::ImageConstPtr &img_msg);
  cv::Mat GetD(const sensor_msgs::ImageConstPtr &img_msg);

  queue<sensor_msgs::ImageConstPtr> imgRGBBuf;
  queue<sensor_msgs::ImageConstPtr> imgDBuf;
  std::mutex mBufMutexRGB;
  std::mutex mBufMutexD;
  double mtd;

  ORB_SLAM3::System *mpSLAM;
  ImuGrabber *mpImuGb;

  // const bool mbClahe;
  // cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

cv::Mat ImageGrabber::GetD(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}
void ImageGrabber::GrabRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRGB.lock();
  if (!imgRGBBuf.empty())
    imgRGBBuf.pop();
  imgRGBBuf.push(img_msg);
  mBufMutexRGB.unlock();
}


void ImageGrabber::GrabD(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexD.lock();
  if (!imgDBuf.empty())
    imgDBuf.pop();
  imgDBuf.push(img_msg);
  mBufMutexD.unlock();
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while (1)
  {
    cv::Mat imRGB, imD;
    double tImRGB = 0, tImD = 0;
    if (!imgRGBBuf.empty() && !imgDBuf.empty() && !mpImuGb->imuBuf.empty())
    {
      tImRGB = imgRGBBuf.front()->header.stamp.toSec() + mtd;
      tImD = imgDBuf.front()->header.stamp.toSec() + mtd;

      this->mBufMutexD.lock();
      while ((tImRGB - tImD) > maxTimeDiff && imgDBuf.size() > 1)
      {
        imgDBuf.pop();
        tImD = imgDBuf.front()->header.stamp.toSec() + mtd;
      }
      this->mBufMutexD.unlock();

      this->mBufMutexRGB.lock();
      while ((tImD - tImRGB) > maxTimeDiff && imgRGBBuf.size() > 1)
      {
        imgRGBBuf.pop();
        tImRGB = imgRGBBuf.front()->header.stamp.toSec() + mtd;
      }
      this->mBufMutexRGB.unlock();

      if ((tImRGB - tImD) > maxTimeDiff || (tImD - tImRGB) > maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if (tImRGB > mpImuGb->imuBuf.back()->header.stamp.toSec())
        continue;

      this->mBufMutexRGB.lock();
      imRGB = GetRGB(imgRGBBuf.front());
      imgRGBBuf.pop();
      this->mBufMutexRGB.unlock();

      this->mBufMutexD.lock();
      imD = GetD(imgDBuf.front());
      imgDBuf.pop();
      this->mBufMutexD.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImRGB)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      // if (mbClahe)
      // {
      //   mClahe->apply(imRGB);
      // }

      mpSLAM->TrackRGBD(imRGB, imD, tImRGB, vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  bool bEqual = false;
  if (argc < 3 || argc > 4)
  {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  if (argc == 4)
  {
    std::string sbEqual(argv[3]);
    if (sbEqual == "true")
      bEqual = true;
  }

  // 创建SLAM系统，初始化
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);

  // 创建IMU和图像抓取器
  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb, argv[2]);

  // 订阅IMU数据
  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb);

  // 订阅图像数据
  ros::Subscriber sub_rgb = n.subscribe("/camera/color/image_raw", 1000, &ImageGrabber::GrabRGB, &igb);
  ros::Subscriber sub_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &ImageGrabber::GrabD, &igb);

  // 启动IMU同步线程
  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  // 执行ros循环
  ros::spin();

  return 0;
}
