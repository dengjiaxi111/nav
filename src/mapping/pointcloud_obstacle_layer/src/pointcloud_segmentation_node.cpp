#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

Eigen::Affine3f transformToEigen(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform.translation;
  const auto& r = transform.transform.rotation;
  Eigen::Quaternionf q(
    static_cast<float>(r.w),
    static_cast<float>(r.x),
    static_cast<float>(r.y),
    static_cast<float>(r.z));
  q.normalize();
  return Eigen::Translation3f(
           static_cast<float>(t.x),
           static_cast<float>(t.y),
           static_cast<float>(t.z)) *
         q;
}

bool isZeroStamp(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0;
}

builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time& time) {
  builtin_interfaces::msg::Time msg;
  const int64_t nanoseconds = time.nanoseconds();
  msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return msg;
}

int fieldOffset(const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
  for (const auto& field : msg.fields) {
    if (field.name == field_name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float readFloat32(const uint8_t* data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

bool pointCloud2ToXYZ(const sensor_msgs::msg::PointCloud2& msg,
                      pcl::PointCloud<pcl::PointXYZ>& cloud) {
  const int x_offset = fieldOffset(msg, "x");
  const int y_offset = fieldOffset(msg, "y");
  const int z_offset = fieldOffset(msg, "z");
  if (x_offset < 0 || y_offset < 0 || z_offset < 0 || msg.point_step == 0) {
    return false;
  }

  cloud.clear();
  cloud.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));
  for (uint32_t row = 0; row < msg.height; ++row) {
    const auto row_start = static_cast<std::size_t>(row) * msg.row_step;
    for (uint32_t col = 0; col < msg.width; ++col) {
      const auto point_start = row_start + static_cast<std::size_t>(col) * msg.point_step;
      if (point_start + msg.point_step > msg.data.size()) {
        continue;
      }
      const uint8_t* point = msg.data.data() + point_start;
      pcl::PointXYZ xyz;
      xyz.x = readFloat32(point + x_offset);
      xyz.y = readFloat32(point + y_offset);
      xyz.z = readFloat32(point + z_offset);
      if (std::isfinite(xyz.x) && std::isfinite(xyz.y) && std::isfinite(xyz.z)) {
        cloud.push_back(xyz);
      }
    }
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = false;
  return true;
}

void xyzToPointCloud2(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                      sensor_msgs::msg::PointCloud2& msg) {
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  msg.fields.resize(3);

  const char* names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>(i * 4);
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.data.resize(static_cast<std::size_t>(msg.row_step));
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto offset = i * msg.point_step;
    std::memcpy(msg.data.data() + offset, &cloud.points[i].x, sizeof(float));
    std::memcpy(msg.data.data() + offset + 4, &cloud.points[i].y, sizeof(float));
    std::memcpy(msg.data.data() + offset + 8, &cloud.points[i].z, sizeof(float));
  }
}

}  // namespace

class PointcloudSegmentationNode : public rclcpp::Node {
public:
  PointcloudSegmentationNode()
  : Node("pointcloud_segmentation"),
    accumulated_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/cloud_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    merge_frame_count_ = std::max(1, static_cast<int>(declare_parameter<int>("merge_frame_count", 2)));

    z_min_base_ = declare_parameter<double>("z_min_base", -0.1);
    z_max_base_ = declare_parameter<double>("z_max_base", 1.5);
    normal_radius_ = declare_parameter<double>("normal_radius", 0.5);
    normal_threads_ = std::max(1, static_cast<int>(declare_parameter<int>("normal_threads", 2)));
    normal_z_abs_max_ = declare_parameter<double>("normal_z_abs_max", 0.8);

    sor_mean_k_ = std::max(1, static_cast<int>(declare_parameter<int>("sor_mean_k", 4)));
    sor_stddev_ = declare_parameter<double>("sor_stddev", 1.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudSegmentationNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "pointcloud_segmentation input=%s output=%s base=%s target=%s",
                input_cloud_topic_.c_str(), output_cloud_topic_.c_str(),
                base_frame_.c_str(), target_frame_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    if (!pointCloud2ToXYZ(*msg, input_cloud)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Input cloud does not contain float32 x/y/z fields");
      return;
    }
    if (input_cloud.empty()) {
      return;
    }

    const std::string source_frame = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ> base_cloud;
    if (!transformCloud(input_cloud, source_frame, base_frame_, base_cloud)) {
      return;
    }

    *accumulated_cloud_ += base_cloud;
    ++accumulated_frames_;
    last_stamp_ = isZeroStamp(msg->header.stamp) ? timeToMsg(now()) : msg->header.stamp;

    if (accumulated_frames_ < merge_frame_count_) {
      return;
    }

    processAccumulatedCloud();
    accumulated_cloud_->clear();
    accumulated_frames_ = 0;
  }

  bool transformCloud(const pcl::PointCloud<pcl::PointXYZ>& input,
                      const std::string& source_frame,
                      const std::string& target_frame,
                      pcl::PointCloud<pcl::PointXYZ>& output) {
    if (source_frame == target_frame) {
      output = input;
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      pcl::transformPointCloud(input, output, transformToEigen(transform));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Failed to transform cloud %s -> %s: %s",
                           source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  void processAccumulatedCloud() {
    auto clipped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(accumulated_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(z_min_base_), static_cast<float>(z_max_base_));
    pass.filter(*clipped);

    if (clipped->empty()) {
      publishCloud(*clipped, base_frame_);
      return;
    }

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setNumberOfThreads(normal_threads_);
    normal_estimator.setInputCloud(clipped);
    normal_estimator.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    normal_estimator.setRadiusSearch(normal_radius_);
    normal_estimator.compute(*normals);

    auto obstacle_indices = std::make_shared<pcl::PointIndices>();
    obstacle_indices->indices.reserve(normals->size());
    for (std::size_t i = 0; i < normals->size(); ++i) {
      const float nz = normals->at(i).normal_z;
      if (std::isfinite(nz) && std::abs(nz) < normal_z_abs_max_) {
        obstacle_indices->indices.push_back(static_cast<int>(i));
      }
    }

    auto obstacle_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(clipped);
    extract.setIndices(obstacle_indices);
    extract.setNegative(false);
    extract.filter(*obstacle_cloud);

    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (static_cast<int>(obstacle_cloud->size()) > sor_mean_k_) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(obstacle_cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_);
      sor.filter(*filtered);
    } else {
      *filtered = *obstacle_cloud;
    }

    publishCloud(*filtered, base_frame_);
  }

  void publishCloud(const pcl::PointCloud<pcl::PointXYZ>& base_cloud,
                    const std::string& cloud_frame) {
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    if (!transformCloud(base_cloud, cloud_frame, target_frame_, target_cloud)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    xyzToPointCloud2(target_cloud, output_msg);
    output_msg.header.stamp = last_stamp_;
    output_msg.header.frame_id = target_frame_;
    cloud_pub_->publish(output_msg);
  }

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string target_frame_;
  std::string base_frame_;
  int merge_frame_count_{2};

  double z_min_base_{-0.1};
  double z_max_base_{1.5};
  double normal_radius_{0.5};
  int normal_threads_{2};
  double normal_z_abs_max_{0.8};
  int sor_mean_k_{4};
  double sor_stddev_{1.1};

  int accumulated_frames_{0};
  builtin_interfaces::msg::Time last_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
