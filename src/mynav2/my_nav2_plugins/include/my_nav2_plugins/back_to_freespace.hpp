#pragma once

#include <string>
#include <vector>

#include "nav2_behaviors/plugins/wait.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace my_nav2_bt
{
/**
 * @class my_nav2_bt::BackToFreeSpace
 * @brief 卡在墙壁、致命障碍物中后直线运动到最近的非障碍物节点
 */

class BackToFreeSpace: public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>
{
public:
    BackToFreeSpace(const std::string & xml_tag_name, const std::string & action_name, const BT::NodeConfiguration & conf)
      : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf), logger_(node_->get_logger())
    {
        if (search_radius_ < robot_radius_) {
            RCLCPP_WARN(logger_, "max_radius < robot_radius. Adjusting max_radius.");
            search_radius_ = robot_radius_;
        }
        
        costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");


        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "back_up_free_space_markers", 1);

        tf_  = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        point_found_ = false;
        best_angle_ = 0;
    }

    static BT::PortsList providedPorts()
    {
        
        return providedBasicPorts(
        {
            BT::InputPort<double>("speed", 0.8, "moving speed"),
            BT::InputPort<double>("search_radius", 1.0, "search_radius"),
            BT::InputPort<double>("robot_radius", 0.3, "robot_radius"),
            BT::InputPort<double>("free_threshold", 200, "free_threshold"),
            BT::InputPort<double>("time_allowance", 5, "time allowance"),
            BT::InputPort<bool>("visualize", true, "visualize"),
        });

    }

    void on_tick() override;
    void on_wait_for_result(std::shared_ptr<const nav2_msgs::action::Wait::Feedback>) override;

protected:
    /**
   * @brief 寻找可行点
   * @return 可行点集合
   */
    std::vector<geometry_msgs::msg::Point> gatherFreePoints(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius);

    /**
   * @brief 计算代价值
   * @return 代价值
   */
    float calculateCost(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float angle,
        float radius, int &best_cost_point_index);

    /**
     * @brief 寻找最佳方向
     * @return 最佳方向, 返回目标点索引的引用
    */
    float findBestDirection(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
        float end_angle, float radius, float angle_increment, int &target_point_index);

     /**
   * @brief 可视化
   * @return void
   */
    void visualize(const geometry_msgs::msg::Point & target_point);

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;

    // parameters
    std::string costmap_service_name_;
    double speed_;  //速度
    double twist_x_, twist_y_;  //速度
    double search_radius_, robot_radius_; //搜索距离
    int free_threshold_;        //阈值
    double time_allowance_;
    bool visualize_;     
    
    // nav info
    geometry_msgs::msg::PoseStamped current_pose_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    double cmd_dist_;
    geometry_msgs::msg::Point cmd_point_;
    int min_cost_index_;
    bool point_found_;
    double best_angle_;
    // others
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_pub_;

    void sendGoal(const nav2_msgs::action::Wait::Goal & goal)
    {
        // Implement the logic to send the goal to the action server
        RCLCPP_INFO(logger_, "Sending goal to action server...");
        // Example: action_client_->async_send_goal(goal);
    }
};

}