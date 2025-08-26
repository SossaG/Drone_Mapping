#ifndef IMAGE_GRABBER_HPP
#define IMAGE_GRABBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <queue>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "include/System.h"
#include "include/ImuTypes.h"

class ImageGrabber {
public:
  ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe,
               rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
               rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
               std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name);

  // Existing interfaces
  void grabImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void processImages();
  cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);
  void publishSE3fToOdom(const Sophus::SE3f& Tcw);
  void publishPointCloud(const std::vector<Eigen::Vector3f>& points);
  void savePoseToFile(const Sophus::SE3f &pose, double sec, double nanosec);

  // === IMU intake & packaging for ORB-SLAM3 ===
  void grabImu(const sensor_msgs::msg::Imu::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mImuMutex_);
    imuBuf_.push(msg);
  }

  // Pop IMU samples up to image time (sec) and convert to ORB-SLAM3 IMU::Point
  void getImuUpTo(double t_img, std::vector<ORB_SLAM3::IMU::Point>& out) {
    std::lock_guard<std::mutex> lock(mImuMutex_);
    while (!imuBuf_.empty()) {
      const auto& m = imuBuf_.front();
      const double t = m->header.stamp.sec + 1e-9 * m->header.stamp.nanosec;
      if (t > t_img) break;
      // Your units are already rad/s and m/s^2
      const cv::Point3f a(m->linear_acceleration.x,  m->linear_acceleration.y,  m->linear_acceleration.z);
      const cv::Point3f w(m->angular_velocity.x,     m->angular_velocity.y,     m->angular_velocity.z);
      out.emplace_back(a, w, t);
      imuBuf_.pop();
    }
  }

private:
  // Image buffering
  std::queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
  std::mutex mBufMutex;

  // IMU buffering
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
  std::mutex mImuMutex_;

  // SLAM / processing
  std::shared_ptr<ORB_SLAM3::System> mpSLAM;
  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  bool first_pose = true;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  nav_msgs::msg::Odometry odom_msg_;
  std::shared_ptr<rclcpp::Node> rosNode_;
  const std::string tf_frame;
};

#endif // IMAGE_GRABBER_HPP
