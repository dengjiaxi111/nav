#include "my_nav2_plugins/back_to_freespace.hpp"

namespace my_nav2_behaviors
{
void BackToFreeSpace::onConfigure()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    // 初始化参数
    node->declare_parameter<std::string>("global_frame","/map");
    node->declare_parameter<float>("robot_radius",0.2);
    node->declare_parameter<float>("search_radius",2.0);
    node->declare_parameter<std::string>("costmap_service_name","/local_costmap/get_costmap");
    node->declare_parameter<int>("free_threshold",100);
    node->declare_parameter<bool>("visualize",true);

    node->get_parameter("global_frame",global_frame_);
    node->get_parameter("robot_radius",robot_radius_);
    node->get_parameter("search_radius",search_radius_);
    node->get_parameter("costmap_service_name",costmap_service_name_);
    node->get_parameter("free_threshold",free_threshold_);
    node->get_parameter("visualize",visualize_);

    if(search_radius_ <= robot_radius_){
        RCLCPP_WARN(logger_,"Adjusting search_radius to robot_radius.");
        search_radius_ = robot_radius_;
    }

    // 初始化节点通信
    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(costmap_service_name_);

    if(visualize_){
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("backtofreespace_markers",5);
        marker_pub_->on_activate();
    }
}

void BackToFreeSpace::onCleanup()
{
    marker_pub_.reset();
    costmap_client_.reset();
}

nav2_behaviors::Status BackToFreeSpace::onRun(const std::shared_ptr<const nav2_msgs::action::DriveOnHeading::Goal> command)
{

    // 先获取costmap
    while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
            return nav2_behaviors::Status::FAILED;
        }
        RCLCPP_WARN(logger_, "service not available, waiting again...");
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
        RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
        return nav2_behaviors::Status::FAILED;
    }
    auto costmap = result.get()->map;


    // 获取当前机器人位置
    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
        RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
        return nav2_behaviors::Status::FAILED;
    }

    RCLCPP_INFO(logger_,"robot_base_frame: %s",robot_base_frame_);

    geometry_msgs::msg::Pose2D pose;
    pose.x = initial_pose_.pose.position.x;
    pose.y = initial_pose_.pose.position.y;
    pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

    // 寻找最佳角度
    float best_angle = findBestDirection(costmap, pose, -M_PI, M_PI, search_radius_, M_PI / 4);
    best_angle = findBestDirection(costmap, pose, best_angle - M_PI / 8, best_angle + M_PI / 8, search_radius_, M_PI / 16);

    // 车，移动
    RCLCPP_INFO(logger_,"speed: %f, time_allowance: %f",command->speed, command->time_allowance);
    twist_x_ = std::cos(best_angle) * command->speed;
    twist_y_ = std::sin(best_angle) * command->speed;
    command_x_ = command->target.x;
    command_time_allowance_ = command->time_allowance;

    end_time_ = clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
        RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
        return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "backing up %f meters towards free space at angle %f", command_x_, best_angle);

    if (visualize_) {
        geometry_msgs::msg::Point target_point;
        target_point.x = initial_pose_.pose.position.x + command_x_ * std::cos(best_angle);
        target_point.y = initial_pose_.pose.position.y + command_x_ * std::sin(best_angle);
        visualize(target_point);
    }

    return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackToFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Exceeded time allowance Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

float BackToFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  float best_angle = start_angle;
  float best_safety = std::numeric_limits<float>::infinity();
  for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
    float safety = calculateSafety(costmap, pose, angle, radius);
    if (safety < best_safety) {
      best_safety = safety;
      best_angle = angle;
    }
  }
  return best_angle;
}

float BackToFreeSpace::calculateSafety(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float angle,
  float radius)
{
  float safety = 0.0;
  float resolution = costmap.metadata.resolution;
  float origin_x = costmap.metadata.origin.position.x;
  float origin_y = costmap.metadata.origin.position.y;
  int size_x = costmap.metadata.size_x;
  int size_y = costmap.metadata.size_y;

  for (float r = 0.0; r <= radius; r += resolution) {
    float x = pose.x + r * std::cos(angle);
    float y = pose.y + r * std::sin(angle);
    int i = static_cast<int>((x - origin_x) / resolution);
    int j = static_cast<int>((y - origin_y) / resolution);

    if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
        auto idx = i + j * size_x;
        safety += costmap.data[idx]/255;
    }
  }
  return safety;
}

void BackToFreeSpace::visualize(const geometry_msgs::msg::Point & target_point)
{
  visualization_msgs::msg::MarkerArray markers;

  // Marker for target point
  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = global_frame_;
  target_marker.header.stamp = clock_->now();
  target_marker.ns = "target_point";
  target_marker.id = 0;
  target_marker.type = visualization_msgs::msg::Marker::SPHERE;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.position = target_point;
  target_marker.pose.orientation.w = 1.0;
  target_marker.scale.x = 0.2;
  target_marker.scale.y = 0.2;
  target_marker.scale.z = 0.2;
  target_marker.color.r = 1.0;
  target_marker.color.g = 0.0;
  target_marker.color.b = 0.0;
  target_marker.color.a = 1.0;
  markers.markers.push_back(target_marker);

  marker_pub_->publish(markers);
}
} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_nav2_behaviors::BackToFreeSpace, nav2_core::Behavior)