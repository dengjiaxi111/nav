#include "my_nav2_plugins/back_to_freespace.hpp"

namespace my_nav2_bt
{

void BackToFreeSpace::on_tick(){

    // 构造 Wait Action 的目标
    goal_.time.sec = 5;
    // 先获取参数
    getInput("speed",speed_);
    getInput("search_radius",search_radius_);
    getInput("robot_radius",robot_radius_);
    getInput("free_threshold",free_threshold_);
    getInput("visualize",visualize_);

    point_found_ = false;
    best_angle_= 0;
    RCLCPP_WARN(logger_, "Failed to plan! entering recovery node \n");
};

void BackToFreeSpace::on_wait_for_result(std::shared_ptr<const nav2_msgs::action::Wait::Feedback> )
{
    // 获取当前机器人位置
    if (!nav2_util::getCurrentPose(current_pose_, *tf_, "map", "base_link_fake")) {
        RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
        return;
    }
    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;

    // 获取costmap
    // 发送异步请求并阻塞等待响应
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto future = costmap_client_->async_send_request(request);
    auto result_code = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1));

    /* GPT老师： 
        在 ROS 2 中，服务客户端通过 async_send_request() 发出请求后，它依赖 executor 来处理响应回调。
        如果你调用服务的代码在 BT 节点里直接运行，而且 BT 的主线程没有调用 spin() 或 executor->spin_some()，
        那回调根本就不会执行，future.get() 也就一直卡住。
    */
    if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Failed to get costmap.");
        return;
    }
    
    // 成功获取 costmap
    auto costmap = future.get()->map;
    // RCLCPP_INFO(logger_, "Received costmap: size = %u x %u",
    //             costmap.metadata.size_x, costmap.metadata.size_y);

    // 检测是否离开障碍物
    int index_x = static_cast<int>((current_x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
    int index_y = static_cast<int>((current_y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);
    int index = index_x + index_y * costmap.metadata.size_x;
    if (index < 0 || index >= costmap.data.size()) {
        RCLCPP_ERROR(logger_, "Index out of bounds");
        return;
    }
    RCLCPP_INFO(logger_,"cost: %d",static_cast<int>(costmap.data[index]));
    if (costmap.data[index] < free_threshold_) {
        RCLCPP_INFO(logger_, "Robot is in free space");
        // 机器人已经离开障碍物
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        setStatus(BT::NodeStatus::SUCCESS);
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = 0;
        cmd_vel->linear.y = 0;
        vel_pub_->publish(std::move(cmd_vel));
        return;
    } 

    RCLCPP_WARN(logger_, "Robot is still stuck!");
    // 若尚未离开障碍物，搜索最佳方向
    geometry_msgs::msg::Pose2D pose;
    pose.theta = tf2::getYaw(current_pose_.pose.orientation);
    pose.x = current_x;
    pose.y = current_y;

    // 寻找最佳角度
    if(!point_found_){
        best_angle_ = findBestDirection(costmap, pose, -M_PI, M_PI, search_radius_, M_PI / 8, min_cost_index_);
        point_found_ = true;
    }
    double best_angle_fake_frame = best_angle_ - pose.theta;

    geometry_msgs::msg::Point target_point;
    target_point.x = min_cost_index_ % costmap.metadata.size_x * costmap.metadata.resolution + costmap.metadata.origin.position.x;
    target_point.y = min_cost_index_ / costmap.metadata.size_x * costmap.metadata.resolution + costmap.metadata.origin.position.y;
    if (visualize_) {
        visualize(target_point);
    }
    // 计算移速
    twist_x_ = std::cos(best_angle_fake_frame) * speed_;
    twist_y_ = std::sin(best_angle_fake_frame) * speed_;

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = twist_x_;
    cmd_vel->linear.y = twist_y_;
    vel_pub_->publish(std::move(cmd_vel));
}

float BackToFreeSpace::findBestDirection(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
    float end_angle, float radius, float angle_increment, int &target_point_index)
{
    float best_angle = start_angle;
    float best_cost = std::numeric_limits<float>::infinity();

    for(float dist = 0.0; dist <= radius; dist += costmap.metadata.resolution){
        for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
            int point_index;
            float cost = calculateCost(costmap, pose, angle, dist, point_index);
            if (cost < best_cost) {
                best_cost = cost;
                best_angle = angle;
                target_point_index = point_index;
            }
        }
        if(best_cost < free_threshold_){
            break;
        }
    }
      
    return best_angle;
}
  
float BackToFreeSpace::calculateCost(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float angle,
    float radius, int &best_cost_point_index)
{
    float safety = 0.0;
    float resolution = costmap.metadata.resolution;
    float origin_x = costmap.metadata.origin.position.x;
    float origin_y = costmap.metadata.origin.position.y;
    int size_x = costmap.metadata.size_x;
    int size_y = costmap.metadata.size_y;

    float x = pose.x + radius * std::cos(angle);
    float y = pose.y + radius * std::sin(angle);
    int i = static_cast<int>((x - origin_x) / resolution);
    int j = static_cast<int>((y - origin_y) / resolution);

    if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
        best_cost_point_index = i + j * size_x;
        safety = costmap.data[best_cost_point_index];
    }
    else{
        best_cost_point_index = static_cast<int>((pose.x - origin_x) / resolution) + static_cast<int>((pose.y - origin_y) / resolution) * size_x;
        safety = 255;
    }

    return safety;
}
  
//   std::vector<geometry_msgs::msg::Point> BackToFreeSpace::gatherFreePoints(
//     const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
//   {
//     std::vector<geometry_msgs::msg::Point> results;
//     for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
//       for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
//         auto idx = i + j * costmap.metadata.size_x;
//         auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
//         auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
//         if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
//           geometry_msgs::msg::Point p;
//           p.x = x;
//           p.y = y;
//           results.push_back(p);
//         }
//       }
//     }
//     return results;
//   }
  
  void BackToFreeSpace::visualize(const geometry_msgs::msg::Point & target_point)
  {
    visualization_msgs::msg::MarkerArray markers;
  
    // Marker for target point
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "map";
    target_marker.header.stamp = node_->get_clock()->now();
    target_marker.ns = "";
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
    if(!marker_pub_){
        RCLCPP_ERROR(logger_,"!!!!marker_pub_ invalid!!! ");
    }
    else{
        marker_pub_->publish(markers);
    }
  }
  
}

#include <nlohmann/json.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<my_nav2_bt::BackToFreeSpace>(
        name, "wait", config);
    };
  factory.registerBuilder<my_nav2_bt::BackToFreeSpace>("BackToFreeSpace", builder);
}
