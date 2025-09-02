// image_grabber_imu_mono.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include "System.h"
#include "ImuTypes.h"

#include <cmath>

inline bool finite3(double x, double y, double z){
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline bool sane_acc(double ax, double ay, double az){
  // reject absurd values (> ~6g)
  const double A = 6.0 * 9.81;
  return std::abs(ax) < A && std::abs(ay) < A && std::abs(az) < A;
}

inline bool sane_gyr(double gx, double gy, double gz){
  // reject absurd values (> ~1000 deg/s ~ 17.45 rad/s)
  const double G = 17.45;
  return std::abs(gx) < G && std::abs(gy) < G && std::abs(gz) < G;
}


class ImuBuffer {
public:
  void push(const sensor_msgs::msg::Imu::SharedPtr& m){ std::lock_guard<std::mutex> lk(mu_); q_.push(m); }
  bool empty(){ std::lock_guard<std::mutex> lk(mu_); return q_.empty(); }
  sensor_msgs::msg::Imu::SharedPtr front(){ std::lock_guard<std::mutex> lk(mu_); return q_.front(); }
  void pop(){ std::lock_guard<std::mutex> lk(mu_); if(!q_.empty()) q_.pop(); }
  sensor_msgs::msg::Imu::SharedPtr back(){ std::lock_guard<std::mutex> lk(mu_); auto tmp=q_; sensor_msgs::msg::Imu::SharedPtr last; while(!tmp.empty()){ last=tmp.front(); tmp.pop(); } return last; }
private:
  std::queue<sensor_msgs::msg::Imu::SharedPtr> q_;
  std::mutex mu_;
};

class MonoInertialNode : public rclcpp::Node {
public:
  MonoInertialNode()
  : Node("orbslam3_mono_inertial")
  {
    declare_parameter<std::string>("vocab_path", "");
    declare_parameter<std::string>("config_path", "");
    declare_parameter<bool>("equalize", false);
    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("imu_topic", "/imu/data");

    auto vocab = get_parameter("vocab_path").as_string();
    auto cfg   = get_parameter("config_path").as_string();
    equalize_  = get_parameter("equalize").as_bool();

    // ORB-SLAM3 in IMU_MONOCULAR mode
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab, cfg, ORB_SLAM3::System::IMU_MONOCULAR, true);

    auto img_topic = get_parameter("image_topic").as_string();
    auto imu_topic = get_parameter("imu_topic").as_string();

    auto qos = rclcpp::SensorDataQoS();

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      img_topic, qos, std::bind(&MonoInertialNode::onImage, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, [this](sensor_msgs::msg::Imu::SharedPtr m){ imu_buf_.push(m); });

    worker_ = std::thread([this]{ syncLoop(); });
  }

  ~MonoInertialNode() override {
    running_ = false;
    if(worker_.joinable()) worker_.join();
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg){
    static size_t count = 0;
    if ((++count % 30) == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Got %zu images; latest %ux%u enc=%s",
                            count, msg->width, msg->height, msg->encoding.c_str());
    }
    std::lock_guard<std::mutex> lk(img_mu_);
    if(!img_q_.empty()) img_q_.pop();
    img_q_.push(msg);
    }

  cv::Mat toMono8(const sensor_msgs::msg::Image::SharedPtr& msg){
    try {
        // Fast-path: already grayscale
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        return cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
        }

        // Common color encodings -> convert to gray
        if (msg->encoding == sensor_msgs::image_encodings::BGR8 ||
            msg->encoding == sensor_msgs::image_encodings::RGB8) {
        // Force to BGR8 first so the cvtColor code path is deterministic
        cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        return gray;
        }

        // Fallback: try to coerce whatever it is to BGR8, then gray
        cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        return gray;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s (encoding='%s')",
                    e.what(), msg->encoding.c_str());
        return {};
    }
    }


  void syncLoop(){
    using ORB_SLAM3::IMU::Point;
    auto clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    while(rclcpp::ok() && running_){
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
        std::lock_guard<std::mutex> lk(img_mu_);
        if(img_q_.empty()){ std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        img_msg = img_q_.front(); img_q_.pop();
        }
        const double t_im = rclcpp::Time(img_msg->header.stamp).seconds();

        // Guard against future IMU timestamps
        auto last_imu = imu_buf_.back();
        if(last_imu && rclcpp::Time(last_imu->header.stamp).seconds() < t_im){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
        }

        // Collect IMU up to image time
        std::vector<ORB_SLAM3::IMU::Point> vimu;
        {
            static double last_used_imu_t = -1.0;  // persists across frames
            size_t pushed = 0, dropped = 0;

            while(!imu_buf_.empty()){
                auto m = imu_buf_.front();
                const double t = rclcpp::Time(m->header.stamp).seconds();
                if(t > t_im) break;                 // stop at image time

                // pop it now; decide whether to keep
                imu_buf_.pop();

                // enforce strictly increasing timestamps
                if(t <= last_used_imu_t){
                dropped++;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Dropping IMU with non-increasing timestamp: %.9f <= %.9f", t, last_used_imu_t);
                continue;
                }

                // read values
                double ax = m->linear_acceleration.x;
                double ay = m->linear_acceleration.y;
                double az = m->linear_acceleration.z;
                double gx = m->angular_velocity.x;
                double gy = m->angular_velocity.y;
                double gz = m->angular_velocity.z;

                // reject NaN/Inf or absurd outliers
                if(!finite3(ax,ay,az) || !finite3(gx,gy,gz) || !sane_acc(ax,ay,az) || !sane_gyr(gx,gy,gz)){
                dropped++;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Dropping bad IMU sample (acc=[%.3f %.3f %.3f], gyr=[%.3f %.3f %.3f])",
                    ax,ay,az,gx,gy,gz);
                continue;
                }

                vimu.emplace_back(cv::Point3f(ax,ay,az), cv::Point3f(gx,gy,gz), t);
                last_used_imu_t = t;
                pushed++;
            }

            if(dropped && pushed){
                RCLCPP_WARN(this->get_logger(), "IMU samples: kept=%zu, dropped=%zu before t=%.6f",
                            pushed, dropped, t_im);
            }
        }


      cv::Mat im = toMono8(img_msg);
      //Guard empty frames after conversion
      if (im.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Empty image after conversion; encoding was '%s'",
                            img_msg->encoding.c_str());
        continue;
        }

      if(equalize_) clahe->apply(im, im);
      slam_->TrackMonocular(im, t_im, vimu);
      if (vimu.empty()){
        // Still OK to pass empty sometimes, but if this is frequent it hurts tracking.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "No IMU samples up to image time %.6f", t_im);
        }
      if (!vimu.empty()) {
        double t0 = vimu.front().t;
        double t1 = vimu.back().t;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "IMU slice: %zu samples, [%.6f .. %.6f], dT=%.6f",
            vimu.size(), t0, t1, t1 - t0);
        }

    

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_q_;
  std::mutex img_mu_;
  ImuBuffer imu_buf_;
  std::thread worker_;
  std::atomic<bool> running_{true};
  bool equalize_{false};
};
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MonoInertialNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
