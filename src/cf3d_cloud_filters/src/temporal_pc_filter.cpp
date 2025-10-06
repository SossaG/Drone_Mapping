#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <deque>

class TemporalPointCloudFilter : public rclcpp::Node {
public:
  TemporalPointCloudFilter() : Node("temporal_pc_filter") {
    input_topic_  = this->declare_parameter<std::string>("input_topic",  "/slam/pointcloud");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/slam/pointcloud_filtered");

    leaf_size_     = this->declare_parameter<double>("leaf_size", 0.05);
    mean_k_        = this->declare_parameter<int>("sor_mean_k",150);
    stddev_mul_    = this->declare_parameter<double>("sor_stddev", 0.2);
    radius_        = this->declare_parameter<double>("ror_radius", 0.058);
    min_neighbors_ = this->declare_parameter<int>("ror_min_neighbors", 18 );

    window_size_   = this->declare_parameter<int>("window_size", 6);
    use_window_    = this->declare_parameter<bool>("use_temporal_window", true);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TemporalPointCloudFilter::onCloud, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "temporal_pc_filter listening on %s, publishing %s",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudT::Ptr current(new CloudT);
    pcl::fromROSMsg(*msg, *current);

    // Basic sanity: remove NaNs
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*current, *current, idx);

    // Downsample current to keep memory bounded
    CloudT::Ptr current_ds(new CloudT);
    if (leaf_size_ > 1e-6) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(current);
      vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      vg.filter(*current_ds);
    } else {
      current_ds = current;
    }

    // Maintain window
    if (use_window_) {
      window_.push_back(current_ds);
      while ((int)window_.size() > window_size_) window_.pop_front();
    }

    // Concatenate window or use single cloud
    CloudT::Ptr fused(new CloudT);
    if (use_window_) {
      // Concatenate oldest->newest to encourage stable structure
      fused->reserve( (uint32_t) (window_.size() * current_ds->size()) );
      for (auto &c : window_) *fused += *c;
    } else {
      fused = current_ds;
    }

    // SOR
    CloudT::Ptr sor_out(new CloudT);
    if (fused->size() > (size_t)mean_k_) {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(fused);
      sor.setMeanK(mean_k_);
      sor.setStddevMulThresh(stddev_mul_);
      sor.filter(*sor_out);
    } else {
      sor_out = fused;
    }

    // ROR
    CloudT::Ptr ror_out(new CloudT);
    if (!sor_out->empty()) {
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(sor_out);
      ror.setRadiusSearch(radius_);
      ror.setMinNeighborsInRadius(min_neighbors_);
      ror.filter(*ror_out);
    } else {
      ror_out = sor_out;
    }

    // Optional: light final downsample for uniformity
    CloudT::Ptr final_ds(new CloudT);
    if (leaf_size_ > 1e-6) {
      pcl::VoxelGrid<PointT> vg2;
      vg2.setInputCloud(ror_out);
      vg2.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      vg2.filter(*final_ds);
    } else {
      final_ds = ror_out;
    }

    // right before publishing:
    CloudT::Ptr to_pub = final_ds;
    if (to_pub->empty()) {
    // fallback to pre-filtered (or SOR-only) cloud to avoid zero-size
    to_pub = current_ds;  // or `sor_out` if you prefer
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*final_ds, out);
    out.header = msg->header; // preserve frame & time
    pub_->publish(out);
  }

  // Params
  std::string input_topic_, output_topic_;
  double leaf_size_, stddev_mul_, radius_;
  int mean_k_, min_neighbors_, window_size_;
  bool use_window_;

  // State
  std::deque<CloudT::Ptr> window_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemporalPointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
