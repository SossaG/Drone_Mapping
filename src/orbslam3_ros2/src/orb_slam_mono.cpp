#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <thread>
#include <string>

#include "include/System.h"
#include "orbslam3_ros2/image_grabber_mono.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("orbslam3_ros2");

  // Parameters
  node->declare_parameter<std::string>("config_path", "");
  node->declare_parameter<std::string>("vocab_path", "");
  std::string config_path = node->get_parameter("config_path").as_string();
  std::string vocab_path  = node->get_parameter("vocab_path").as_string();

  if (config_path.empty() || vocab_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "config_path or vocab_path parameter is empty.");
    return 1;
  }

  bool showPangolin = true;
  bool bEqual = false;

  // Publishers
  auto odom_pub  = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/pointcloud", 10);

  // ORB-SLAM3 in IMU monocular mode
  auto SLAM = std::make_shared<ORB_SLAM3::System>(
      vocab_path, config_path, ORB_SLAM3::System::IMU_MONOCULAR, showPangolin);
      RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 mode: IMU_MONOCULAR");


  // Grabber (keeps your existing wiring)
  auto igb = std::make_shared<ImageGrabber>(SLAM, bEqual, odom_pub, cloud_pub, node, "map");

  // Sensor QoS to avoid drops
  auto sensor_qos = rclcpp::SensorDataQoS();

  // Image sub — keep this topic; rely on your launch remap to /image_raw
  const std::string imgTopic = "/camera/rgb/image_color";
  auto sub_img0 = node->create_subscription<sensor_msgs::msg::Image>(
      imgTopic, sensor_qos, [igb](const sensor_msgs::msg::Image::SharedPtr msg){
        igb->grabImage(msg);
      });

  // IMU sub — your raw topic
  const std::string imuTopic = "/imu/data_raw";
  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
      imuTopic, sensor_qos, [igb](const sensor_msgs::msg::Imu::SharedPtr msg){
        igb->grabImu(msg);
      });

  std::thread image_thread(&ImageGrabber::processImages, igb);

  rclcpp::spin(node);
  rclcpp::shutdown();
  image_thread.join();
  return 0;
}
