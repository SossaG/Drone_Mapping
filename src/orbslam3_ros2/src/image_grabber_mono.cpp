#include "orbslam3_ros2/image_grabber_mono.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include <fstream>
#include <sophus/se3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <limits>  // std::numeric_limits
#include <cmath>   // std::isnan (optional)
#include <algorithm>   // std::sort



ImageGrabber::ImageGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name)
    : mpSLAM(pSLAM), mbClahe(bClahe), first_pose(true), odom_pub_(rospub), cloud_pub_(cloud_pub),
      rosNode_(ros_node), tf_frame(camera_frame_name){
        odom_msg_.header.frame_id = tf_frame;
        odom_msg_.child_frame_id = "odom";
    }

void ImageGrabber::grabImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mImuMutex);
    imuBuf.push_back(msg);

    // Throttled “IMU rx” debug
    static size_t imu_rx = 0;
    imu_rx++;

    // IMPORTANT:
    //  - Use rosNode_->get_logger() and *rosNode_->get_clock()
    //  - DO NOT use this->get_clock() (ImageGrabber is not a Node)
    //  - DO NOT introduce any 'imu_msg' variable; use 'msg' directly
    RCLCPP_INFO_THROTTLE(
        rosNode_->get_logger(),
        *rosNode_->get_clock(),
        2000,  // every 2s
        "IMU rx #%zu  t=%.6f  accel=[%.3f %.3f %.3f]  gyro=[%.3f %.3f %.3f]  buf=%zu",
        imu_rx,
        rclcpp::Time(msg->header.stamp).seconds(),
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z,
        imuBuf.size()
    );
}






std::vector<ORB_SLAM3::IMU::Point> ImageGrabber::takeImuSlice(const double t_img)
{
    // Lower bound of the slice is the previous image time.
    // We require IMU.t to be STRICTLY GREATER than last_img_time and <= t_img.
    static double last_img_time = std::numeric_limits<double>::quiet_NaN();
    if (std::isnan(last_img_time)) {
        // First call: start exactly at this frame; next call will have a real window.
        last_img_time = t_img;
    }

    // Strict lower bound tolerance (avoid zero/negative dt against last_img_time).
    constexpr double MIN_DT = 1e-6;     // 1 microsecond
    // NO upper jitter tolerance here: do NOT accept IMU newer than the image.
    // (Allowing future samples caused your 'post' to go negative.)

    // Filter silly values to protect preintegration.
    constexpr float  A_MAX = 200.0f;    // m/s^2
    constexpr float  G_MAX = 50.0f;     // rad/s

    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(32);

    {
        std::lock_guard<std::mutex> lock(mImuMutex);

        // Collect IMUs in (last_img_time, t_img]  (strictly greater than lower bound)
        for (const auto &m : imuBuf) {
            const double ti = rclcpp::Time(m->header.stamp).seconds();

            if (ti <= last_img_time + MIN_DT) continue;  // strictly greater than previous frame time
            if (ti  > t_img)                 continue;    // never include IMU after the image

            const float ax = static_cast<float>(m->linear_acceleration.x);
            const float ay = static_cast<float>(m->linear_acceleration.y);
            const float az = static_cast<float>(m->linear_acceleration.z);
            const float gx = static_cast<float>(m->angular_velocity.x);
            const float gy = static_cast<float>(m->angular_velocity.y);
            const float gz = static_cast<float>(m->angular_velocity.z);

            if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
                !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
                continue;
            }
            if (std::fabs(ax) > A_MAX || std::fabs(ay) > A_MAX || std::fabs(az) > A_MAX ||
                std::fabs(gx) > G_MAX || std::fabs(gy) > G_MAX || std::fabs(gz) > G_MAX) {
                continue;
            }

            // ORB_SLAM3::IMU::Point ctor in your build: (ax, ay, az, gx, gy, gz, t)
            out.emplace_back(ax, ay, az, gx, gy, gz, ti);
        }

        // Keep buffer bounded (oldest data older than ~1.5s behind current image)
        while (!imuBuf.empty()) {
            const double t_front = rclcpp::Time(imuBuf.front()->header.stamp).seconds();
            if (t_front < t_img - 1.5) imuBuf.pop_front();
            else break;
        }
    } // unlock

    if (out.empty()) {
        // Advance lower bound so we don't reuse stale IMUs next frame.
        last_img_time = t_img;
        return out;
    }

    // Ensure strictly increasing timestamps and drop any duplicates.
    std::sort(out.begin(), out.end(),
              [](const ORB_SLAM3::IMU::Point &a, const ORB_SLAM3::IMU::Point &b) {
                  return a.t < b.t;
              });

    std::vector<ORB_SLAM3::IMU::Point> uniq;
    uniq.reserve(out.size());
    uniq.push_back(out[0]);
    for (size_t i = 1; i < out.size(); ++i) {
        if (out[i].t - uniq.back().t > MIN_DT) {
            uniq.push_back(out[i]);
        }
        // else drop it; zero/negative dt can blow up Sophus::SO3::exp
    }
    out.swap(uniq);

    // Safety: ensure the last sample is not after the image due to rounding.
    while (!out.empty() && out.back().t > t_img) out.pop_back();

    // Slide lower bound for next call AFTER building this slice.
    last_img_time = t_img;
    return out;
}




void ImageGrabber::grabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    img0Buf.push(msg);
}

cv::Mat ImageGrabber::getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        if (mbClahe)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            mClahe->apply(image, image);
        }
        return image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

// Function to save pose to a file
void ImageGrabber::savePoseToFile(const Sophus::SE3f &pose, double sec, double nanosec)
{
    std::ofstream pose_file("pose.txt", std::ios::app); // Open file in append mode
    if (!pose_file.is_open())
    {
        RCLCPP_ERROR(rosNode_->get_logger(), "Failed to open pose.txt for writing.");
        return;
    }

    // Get transformation matrix (4x4)
    Eigen::Matrix4f T = pose.matrix();

    // Write timestamp
    pose_file << sec << "." << nanosec << " ";

    // Write pose matrix (row-wise)
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            pose_file << T(i, j) << " ";

    pose_file << std::endl;
    pose_file.close();
}

void ImageGrabber::processImages()
{   
    static double prev_img_time = std::numeric_limits<double>::quiet_NaN();
    while (rclcpp::ok())
    {   
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(mBufMutex);
            if (img0Buf.empty())
                continue;
            img_msg = img0Buf.front();
            img0Buf.pop();
        }
        cv::Mat image = getImage(img_msg);
        if (image.empty())
            continue;        

        // Track the image *with* IMU and get the camera pose
        const double t_img = img_msg->header.stamp.sec + 1e-9 * img_msg->header.stamp.nanosec;
        std::vector<ORB_SLAM3::IMU::Point> vImu = takeImuSlice(t_img);
        RCLCPP_INFO_THROTTLE(
            rosNode_->get_logger(),
            *rosNode_->get_clock(),
            2000,
            "Feeding IMU to tracker: vImu.size()=%zu  img_t=%.6f%s",
            vImu.size(),
            t_img,
            (vImu.empty() ? "  << NO IMU for this frame" : "")
        );

        if (!vImu.empty()) {
            const double t0 = vImu.front().t;
            const double t1 = vImu.back().t;

            RCLCPP_INFO_THROTTLE(
                rosNode_->get_logger(), *rosNode_->get_clock(), 2000,
                "IMU window used: %zu samples  [%.6f .. %.6f]  for img_t=%.6f  gaps: pre=%.4f post=%.4f",
                vImu.size(), t0, t1, t_img,
                (std::isnan(prev_img_time) ? 0.0 : (t0 - prev_img_time)),  // time since last image to first IMU
                (t_img - t1)                                              // time from last IMU to this image
            );
        } else {
            RCLCPP_WARN_THROTTLE(
                rosNode_->get_logger(), *rosNode_->get_clock(), 2000,
                "IMU window EMPTY for img_t=%.6f (no samples between last image and this one)",
                t_img
            );
            }

        // update for next frame AFTER logging
        prev_img_time = t_img;

        Sophus::SE3f pose = mpSLAM->TrackMonocular(image, t_img, vImu);

        // Save pose to file
        //savePoseToFile(pose, img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);

        // Get the 3D map points from the SLAM system
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = mpSLAM->GetTrackedMapPoints();


        // Convert ORB-SLAM3 MapPoints to Eigen::Vector3f for ROS2 point cloud
        std::vector<Eigen::Vector3f> point_cloud;
        for (auto p : mapPoints)
        {
            if (p && !p->isBad()) // Ensure valid points
            {
                Eigen::Vector3f pos = p->GetWorldPos(); // Get 3D position
                point_cloud.emplace_back(pos[0], pos[1], pos[2]);
            }
        }
        
        // Publish pose and point cloud
        publishSE3fToOdom(pose);
        publishPointCloud(point_cloud);
    }
}



void ImageGrabber::publishSE3fToOdom(const Sophus::SE3f& Tcw)
{    
    // Obtain the position and the orientation
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    odom_msg_.pose.pose.position.x = twc.z();   // Z_OCV → X_ROS
    odom_msg_.pose.pose.position.y = -twc.x();  // -X_OCV → Y_ROS
    odom_msg_.pose.pose.position.z = -twc.y();  // -Y_OCV → Z_ROS  

    odom_msg_.pose.pose.orientation.x = q.z();
    odom_msg_.pose.pose.orientation.y = -q.x();
    odom_msg_.pose.pose.orientation.z = -q.y();
    odom_msg_.pose.pose.orientation.w = q.w();

    // --- Set Covariance Values ---
    double position_variance = 0.01;  // Adjust based on your SLAM system's accuracy
    double orientation_variance = 0.02;

    for (int i = 0; i < 36; i++) odom_msg_.pose.covariance[i] = 0.0;

    odom_msg_.pose.covariance[0] = position_variance;  // x
    odom_msg_.pose.covariance[7] = position_variance;  // y
    odom_msg_.pose.covariance[14] = position_variance; // z

    odom_msg_.pose.covariance[21] = orientation_variance; // roll
    odom_msg_.pose.covariance[28] = orientation_variance; // pitch
    odom_msg_.pose.covariance[35] = orientation_variance; // yaw
    // --------------------------------
    odom_msg_.header.stamp = rosNode_->get_clock()->now();
    odom_pub_->publish(odom_msg_);
}

void ImageGrabber::publishPointCloud(const std::vector<Eigen::Vector3f>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";  // Set a fixed frame for the map 
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& point : points)
    {
        *iter_x = point.z();  // Z_OCV → X_ROS
        *iter_y = -point.x(); // -X_OCV → Y_ROS
        *iter_z = -point.y(); // -Y_OCV → Z_ROS
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    cloud_msg.header.stamp = rosNode_->get_clock()->now();
    cloud_pub_->publish(cloud_msg);
}
