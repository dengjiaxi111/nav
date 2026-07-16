#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class WaypointNavigationNode : public rclcpp::Node
{
public:
    WaypointNavigationNode() : Node("waypoint_navigation_node")
    {
        // 目标点坐标 (x, y, z, yaw)
        goal_points_ = {
            {0, 0, 0.2, 0.0},  // Goal 1
            {5.23, -3.64, 0.2, 0.0},  // Goal 2
            {4.9, -1.91, 0.2, 0.0},  // Goal 3
            {5.23, -3.64, 0.2, 0.0},  // Goal 4
            {2.74, -1.51, 0.2, 0.0},  // Goal 5
        };

        // 创建发布目标点的动作客户端
        using namespace std::chrono_literals;
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // 发布目标点数组
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("goal_poses", 10);

        // 启动定时器，但不立即发送目标点
        timer_ = this->create_wall_timer(
            1s, std::bind(&WaypointNavigationNode::send_next_goal, this));
    }

private:
    void send_next_goal()
    {
        if (current_goal_index_ >= goal_points_.size()) {
            RCLCPP_INFO(this->get_logger(), "All goals reached.");
            return;
        }

        // 获取当前目标点
        auto goal = goal_points_[current_goal_index_];

        // 创建目标点 PoseStamped
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.header.frame_id = "map";  // 假设使用地图坐标系
        goal_pose.pose.position.x = goal[0];
        goal_pose.pose.position.y = goal[1];
        goal_pose.pose.position.z = goal[2];
        goal_pose.pose.orientation.w = 1.0;  // 无旋转 (可以根据目标yaw进行调整)

        // 创建目标请求
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        // 发送目标点到动作服务器
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&WaypointNavigationNode::on_goal_result, this, std::placeholders::_1);

        // 发送目标点
        action_client_->async_send_goal(goal_msg, send_goal_options);

        // 发布目标点
        publish_goal_poses();

        RCLCPP_INFO(this->get_logger(), "Sending goal %zu", current_goal_index_ + 1);
    }

    void publish_goal_poses()
    {
        // 创建 PoseArray 消息
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = "map";  // 假设使用地图坐标系

        // 填充 PoseArray 中的目标点
        for (size_t i = 0; i <= current_goal_index_; i++) {
            auto goal = goal_points_[i];

            geometry_msgs::msg::Pose goal_pose;
            goal_pose.position.x = goal[0];
            goal_pose.position.y = goal[1];
            goal_pose.position.z = goal[2];
            goal_pose.orientation.w = 1.0;  // 无旋转 (可以根据目标yaw调整)

            pose_array.poses.push_back(goal_pose);
        }

        // 发布 PoseArray
        goal_pub_->publish(pose_array);
    }

    void on_goal_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal %zu reached successfully", current_goal_index_);

            // 当前目标点到达成功，发送下一个目标点
            current_goal_index_++;
            send_next_goal();  // 发送下一个目标点
        } 
        else {
            RCLCPP_INFO(this->get_logger(),"Goal running!");
            //RCLCPP_ERROR(this->get_logger(), "Goal %zu failed", current_goal_index_);
            // 可以选择重试当前目标点，或者跳过
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t current_goal_index_ = 0;
    std::vector<std::vector<double>> goal_points_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
