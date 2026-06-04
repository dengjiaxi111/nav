#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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

int64_t packCell(int32_t x, int32_t y) {
  return (static_cast<int64_t>(x) << 32) | static_cast<uint32_t>(y);
}

void unpackCell(int64_t key, int32_t& x, int32_t& y) {
  x = static_cast<int32_t>(key >> 32);
  y = static_cast<int32_t>(key & 0xffffffff);
}

bool finitePoint(const pcl::PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
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
      if (finitePoint(xyz)) {
        cloud.push_back(xyz);
      }
    }
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = false;
  return true;
}

}  // namespace

class LocalObstacleGridNode : public rclcpp::Node {
public:
  LocalObstacleGridNode()
  : Node("local_obstacle_grid") {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_filtered");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/rog_map/map_2d");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    resolution_ = positive(declare_parameter<double>("resolution", 0.05), 0.05);
    range_x_ = positive(declare_parameter<double>("range_x", 3.0), 3.0);
    range_y_ = positive(declare_parameter<double>("range_y", 3.0), 3.0);
    publish_rate_ = positive(declare_parameter<double>("publish_rate", 50.0), 50.0);

    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", -1.0);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.0);
    obstacle_range_ = positive(declare_parameter<double>("obstacle_range", 5.0), 5.0);
    voxel_decay_ = std::max(0.0, declare_parameter<double>("voxel_decay", 0.5));

    occupied_value_ = clampValue(declare_parameter<int>("occupied_value", 100));
    free_value_ = clampValue(declare_parameter<int>("free_value", 0));

    width_ = std::max(1, static_cast<int>(std::ceil(range_x_ / resolution_)));
    height_ = std::max(1, static_cast<int>(std::ceil(range_y_ / resolution_)));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalObstacleGridNode::cloudCallback, this, std::placeholders::_1));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalObstacleGridNode::publishTimerCallback, this));

    RCLCPP_INFO(get_logger(),
                "local_obstacle_grid input=%s output=%s frame=%s base=%s size=%dx%d res=%.3f decay=%.2fs",
                input_cloud_topic_.c_str(), output_map_topic_.c_str(), target_frame_.c_str(),
                base_frame_.c_str(), width_, height_, resolution_, voxel_decay_);
  }

private:
  struct RobotPose {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  static double positive(double value, double fallback) {
    return value > 0.0 ? value : fallback;
  }

  static int8_t clampValue(int value) {
    return static_cast<int8_t>(std::clamp(value, -1, 100));
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RobotPose robot_pose;
    if (!getRobotPose(robot_pose)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (!pointCloud2ToXYZ(*msg, cloud)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Input cloud does not contain float32 x/y/z fields");
      return;
    }
    if (cloud.empty()) {
      return;
    }

    const std::string source_frame = msg->header.frame_id.empty() ? target_frame_ : msg->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    if (!transformCloud(cloud, source_frame, target_frame_, target_cloud)) {
      return;
    }

    const double obstacle_range_sq = obstacle_range_ * obstacle_range_;
    const rclcpp::Time stamp = now();

    std::lock_guard<std::mutex> lock(cells_mutex_);
    for (const auto& point : target_cloud.points) {
      if (!finitePoint(point)) {
        continue;
      }

      const double rel_z = static_cast<double>(point.z) - robot_pose.z;
      if (rel_z < min_obstacle_height_ || rel_z > max_obstacle_height_) {
        continue;
      }

      const double dx = static_cast<double>(point.x) - robot_pose.x;
      const double dy = static_cast<double>(point.y) - robot_pose.y;
      if (dx * dx + dy * dy > obstacle_range_sq) {
        continue;
      }

      const auto gx = static_cast<int32_t>(std::floor(point.x / resolution_));
      const auto gy = static_cast<int32_t>(std::floor(point.y / resolution_));
      occupied_cells_[packCell(gx, gy)] = stamp;
    }
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

  bool getRobotPose(RobotPose& pose) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame_, base_frame_, tf2::TimePointZero);
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.z = transform.transform.translation.z;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Failed to lookup %s -> %s: %s",
                           target_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publishTimerCallback() {
    RobotPose robot_pose;
    if (!getRobotPose(robot_pose)) {
      return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = target_frame_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);

    const double center_x = std::floor(robot_pose.x / resolution_) * resolution_;
    const double center_y = std::floor(robot_pose.y / resolution_) * resolution_;
    const double origin_x = std::floor((center_x - range_x_ * 0.5) / resolution_) * resolution_;
    const double origin_y = std::floor((center_y - range_y_ * 0.5) / resolution_) * resolution_;
    const auto origin_gx = static_cast<int32_t>(std::floor(origin_x / resolution_));
    const auto origin_gy = static_cast<int32_t>(std::floor(origin_y / resolution_));

    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(width_ * height_), free_value_);

    int active_count = 0;
    {
      std::lock_guard<std::mutex> lock(cells_mutex_);
      const rclcpp::Time stamp = now();
      for (auto it = occupied_cells_.begin(); it != occupied_cells_.end();) {
        const double age = (stamp - it->second).seconds();
        if (age > voxel_decay_) {
          it = occupied_cells_.erase(it);
          continue;
        }

        int32_t gx = 0;
        int32_t gy = 0;
        unpackCell(it->first, gx, gy);
        const int lx = gx - origin_gx;
        const int ly = gy - origin_gy;
        if (lx >= 0 && lx < width_ && ly >= 0 && ly < height_) {
          grid.data[static_cast<std::size_t>(ly * width_ + lx)] = occupied_value_;
          ++active_count;
        }
        ++it;
      }
    }

    map_pub_->publish(grid);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Published local obstacle grid with %d active cells", active_count);
  }

  std::string input_cloud_topic_;
  std::string output_map_topic_;
  std::string target_frame_;
  std::string base_frame_;

  double resolution_{0.05};
  double range_x_{3.0};
  double range_y_{3.0};
  double publish_rate_{50.0};
  double min_obstacle_height_{-1.0};
  double max_obstacle_height_{1.0};
  double obstacle_range_{5.0};
  double voxel_decay_{0.5};
  int width_{60};
  int height_{60};
  int8_t occupied_value_{100};
  int8_t free_value_{0};

  std::mutex cells_mutex_;
  std::unordered_map<int64_t, rclcpp::Time> occupied_cells_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalObstacleGridNode>());
  rclcpp::shutdown();
  return 0;
}
