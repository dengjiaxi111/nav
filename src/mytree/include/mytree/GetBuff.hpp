#ifndef __GETBUFF__
#define __GETBUFF__

#include <iostream>
#include <string>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/float64.hpp"
#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/my_path.hpp"
#include "robots_msgs/msg/mode_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

using namespace std;
namespace myBT {
/*
节点逻辑：
    1：在允许时间内且未尝试打符，可以进入running状态
    2：在running状态下，首先前往Buff点
    3：到达Buff点后，云台停转并等待1s，获取此时的yaw角差
    4: 更新Yaw角差并发送给serial节点，设置云台为角度控制模式 (gimbal_mode为0b10)
    5: Yaw角误差小于阈值后，开启打符模式 (gimbal_mode为0b11)
    6：若超出允许时间或开符成功，更新标志位，退出打符模式
*/

class GetBuff : public BT::StatefulActionNode {
private:
    bool debug_flag_;
    bool goal_reached_;
    bool start_hit_buff_;
    bool buff_get_over_;
    rclcpp::Time reach_time_;
    rclcpp::Time buff_aim_time_;
    rclcpp::Time last_warn_;
    double dist_;
    double yaw_diff_;
    const double TIMEOUT_DURATION = 15.0;  
    const double REACH_THRESHOLD = 0.3;   

    shared_ptr<rclcpp::Node> parent_node_ptr_;
    string yaml_path_;
    string package_path_;

    // 创建参数客户端
    shared_ptr<rclcpp::AsyncParametersClient> param_client_;

    shared_ptr<robots_msgs::msg::ModeCmd> modecmd_BB_;
    shared_ptr<robots_msgs::msg::GameStatus> game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus> robot_status_BB_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;

    geometry_msgs::msg::PoseStamped red_buff_goal_;     
    geometry_msgs::msg::PoseStamped blue_buff_goal_;     
    geometry_msgs::msg::PoseStamped buff_point_;
    shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;
    shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> yaw_diff_publisher_;
    rclcpp::Time last_publish_time_;
    rclcpp::Time start_time_;

public:
    GetBuff(const string &name, const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) 
        : BT::StatefulActionNode(name, config)
    {
        parent_node_ptr_ = parent_node;
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");

        parent_node_ptr_->get_parameter_or("GetBuff_debug", debug_flag_, true);
        parent_node_ptr_->get_parameter_or("footprints_path", yaml_path_, package_path_+"/config/footprints.yaml");
        
        // Read buff points from yaml
        parent_node_ptr_->declare_parameter<double>("red.buff.x", 2.0);
        parent_node_ptr_->declare_parameter<double>("red.buff.y", 0.0);
        parent_node_ptr_->declare_parameter<double>("blue.buff.x", -2.0);
        parent_node_ptr_->declare_parameter<double>("blue.buff.y", 0.0);
        parent_node_ptr_->declare_parameter<double>("buff_point.x", -2.0);
        parent_node_ptr_->declare_parameter<double>("buff_point.y", 0.0);

        parent_node_ptr_->get_parameter("red.buff.x", red_buff_goal_.pose.position.x);
        parent_node_ptr_->get_parameter("red.buff.y", red_buff_goal_.pose.position.y);
        parent_node_ptr_->get_parameter("blue.buff.x", blue_buff_goal_.pose.position.x);
        parent_node_ptr_->get_parameter("blue.buff.y", blue_buff_goal_.pose.position.y);
        parent_node_ptr_->get_parameter("buff_point.x", buff_point_.pose.position.x);
        parent_node_ptr_->get_parameter("buff_point.y", buff_point_.pose.position.y);

        red_buff_goal_.header.frame_id = "map";
        blue_buff_goal_.header.frame_id = "map";
        buff_point_.header.frame_id = "map";

        goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        yaw_diff_publisher_ = parent_node_ptr_->create_publisher<std_msgs::msg::Float64>("/yaw_diff", 10);
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(parent_node_ptr_, "controller_server");

        buff_aim_time_ = reach_time_ = last_warn_ = last_publish_time_ = parent_node_ptr_->get_clock()->now();
        goal_reached_ = false;
        buff_get_over_ = false;
        start_hit_buff_ = false;
        
        start_time_ = parent_node_ptr_->get_clock()->now();

        if(debug_flag_) {
            cout << CYAN << "-----GETBUFF NODE INITIALIZED----" << RESET << endl;
        }
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        // ports_list.insert(BT::OutputPort<bool>("in_buff_mode"));
        ports_list.insert(BT::BidirectionalPort<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd"));
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
        ports_list.insert(BT::InputPort<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos"));
        return ports_list;
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

BT::NodeStatus GetBuff::onStart()
{
    if(debug_flag_) {
        cout << RED << "-----GETBUFF onSTART!----" << RESET << endl;
    }

    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus", game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus", robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos", transform_BB_);
    getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd", modecmd_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr)) {
        cout << RED << "NODE GETBUFF GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }
    
    if(game_status_BB_->stage_remain_time >= 395 && !buff_get_over_){
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;    
}

BT::NodeStatus GetBuff::onRunning()
{
    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus", game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus", robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos", transform_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr)) {
        cout << RED << "NODE GETBUFF GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }
    
    geometry_msgs::msg::PoseStamped* goal_ptr;
    if(robot_status_BB_->robot_id == 7) {
        goal_ptr = &red_buff_goal_;
    }
    else if(robot_status_BB_->robot_id == 107) {
        goal_ptr = &blue_buff_goal_;
    }
    else {
        cout << RED << "NODE GETBUFF INVALID ROBOT ID" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否满足退出条件
    auto current_time = parent_node_ptr_->get_clock()->now();
    if(((current_time - reach_time_).seconds() > 20 && goal_reached_) || game_status_BB_->stage_remain_time < 390 || game_status_BB_->our_small_buff_activated){
        if(debug_flag_) {
            cout << YELLOW << "exiting GETBUFF" << RESET << endl;
        }
        buff_get_over_ = true;
        // 等待服务可用
        rclcpp::Parameter new_param("PurePursuit.desired_linear_vel", 2.2);
        std::vector<rclcpp::Parameter> param_list = { new_param };
       // 设置参数（异步方式）
        param_client_->set_parameters(
        param_list,
        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> result_future) {
          auto results = result_future.get();
          for (const auto & res : results) {
            if (res.successful) {
              RCLCPP_INFO(parent_node_ptr_->get_logger(), "参数设置成功！");
            } else {
              RCLCPP_WARN(parent_node_ptr_->get_logger(), "参数设置失败: %s", res.reason.c_str());
            }
          }
        });
        sleep(0.5);

        // auto results = param_client_->set_parameters(param_list);
        // for (const auto & res : results) {
        //     if (res.successful) {
        //         RCLCPP_INFO(parent_node_ptr_->get_logger(), "参数设置成功！");
        //     } else {
        //         RCLCPP_WARN(parent_node_ptr_->get_logger(), "参数设置失败: %s", res.reason.c_str());
        //     }

        return BT::NodeStatus::SUCCESS;
    }
    // 运行到这里，说明仍在打符阶段且未超时

    // 提高前往打符点的速度
    // 等待服务可用
    if (!param_client_->service_is_ready()) {
        if ((parent_node_ptr_->get_clock()->now() - last_warn_).seconds() > 1.0) {
          RCLCPP_WARN(parent_node_ptr_->get_logger(), "参数服务未就绪");
          last_warn_ = parent_node_ptr_->get_clock()->now();
        }
        return BT::NodeStatus::RUNNING; // 下次再试
    }
      // 修改 RegulatedPurePursuit.desired_linear_vel
    rclcpp::Parameter new_param("PurePursuit.desired_linear_vel", 3.5);
    std::vector<rclcpp::Parameter> param_list = { new_param };
   // 设置参数（异步方式）
    param_client_->set_parameters(
    param_list,
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> result_future) {
      auto results = result_future.get();
      for (const auto & res : results) {
        if (res.successful) {
          RCLCPP_INFO(parent_node_ptr_->get_logger(), "参数设置成功！");
        } else {
          RCLCPP_WARN(parent_node_ptr_->get_logger(), "参数设置失败: %s", res.reason.c_str());
        }
      }
    });

    dist_ = sqrt(pow(transform_BB_->transform.translation.x - goal_ptr->pose.position.x, 2) 
               + pow(transform_BB_->transform.translation.y - goal_ptr->pose.position.y, 2));

    if(debug_flag_){
        cout << CYAN << "dist: " << dist_ << RESET << endl;

    }
    // 检查是已抵达打符点
    if(!goal_reached_ && dist_ <= REACH_THRESHOLD) {
        goal_reached_ = true;
        reach_time_ = parent_node_ptr_->get_clock()->now();
        if(debug_flag_) {
            cout << YELLOW << "Reached buff point" << RESET << endl;
        }
    }
    else if(goal_reached_){
    }
    else{
        modecmd_BB_->should_gimbal_patrol = 0;
        modecmd_BB_->should_chassis_spin = 0;
        auto pub_time_diff = parent_node_ptr_->get_clock()->now() - last_publish_time_;
        if(pub_time_diff.seconds() >= 1.0) {
            cout << YELLOW << "go buff point" << RESET << endl;
            last_publish_time_ = parent_node_ptr_->get_clock()->now();
            goal_ptr->header.stamp = last_publish_time_;
            goal_publisher_->publish(*goal_ptr);
        }
        return BT::NodeStatus::RUNNING;
    }

    // 测量当前与大符的yaw角差并发布
    // 将符的坐标从地图坐标系转换为底盘坐标系
    geometry_msgs::msg::PoseStamped buff_pose;
    buff_pose.header.frame_id = "map";
    buff_pose.pose.position.x = buff_point_.pose.position.x;
    buff_pose.pose.position.y = buff_point_.pose.position.y;
    buff_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); 
    geometry_msgs::msg::PoseStamped buff_pose_baselink;

    // 利用 tf2 库直接求逆
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform_BB_->transform, tf2_transform);

    tf2::Transform tf2_inverse = tf2_transform.inverse();

    geometry_msgs::msg::TransformStamped inverse_transform;
    inverse_transform.header = transform_BB_->header;
    inverse_transform.child_frame_id = transform_BB_->child_frame_id;
    inverse_transform.transform = tf2::toMsg(tf2_inverse);

    tf2::doTransform(buff_pose, buff_pose_baselink, inverse_transform);

    double x = buff_pose_baselink.pose.position.x;
    double y = buff_pose_baselink.pose.position.y;
    yaw_diff_ = atan2(y, x);  // 计算从底盘坐标系到buff点的角度差
    yaw_diff_ = yaw_diff_ / M_PI * 180.0;  // 转换为度
    std_msgs::msg::Float64 yaw_diff_msg;
    yaw_diff_msg.data = yaw_diff_;
    yaw_diff_publisher_->publish(yaw_diff_msg);
    if(debug_flag_){
        cout << CYAN  << ", Yaw difference: " << yaw_diff_ << "(deg)"<< RESET << endl;
    }

    //等待1.0秒
    if((parent_node_ptr_->get_clock()->now() - reach_time_).seconds() <0.4){
        if(debug_flag_) {
            cout << YELLOW << "Waiting at buff point..." << RESET << endl;
        }
        // 令云台和底盘停转
        modecmd_BB_->rebirth = 0;
        modecmd_BB_->should_chassis_spin = 0;
        modecmd_BB_->should_gimbal_patrol = 0;
        modecmd_BB_->use_capacity = 0;
        modecmd_BB_->buy_bullet = 0;
        return BT::NodeStatus::RUNNING;
    }
    // 阈值为2度
    if(abs(yaw_diff_) < 2.0 && !start_hit_buff_) {
        if(debug_flag_) {
            cout << GREEN << "activating buffmode" << RESET << endl;
        }
        buff_aim_time_ = parent_node_ptr_->get_clock()->now();
        start_hit_buff_ = true;
    }
    else if(!start_hit_buff_){
        if(debug_flag_) {
            cout << YELLOW << "waiting for yaw" << RESET << endl;
        }
        modecmd_BB_->should_gimbal_patrol = 0b10;
    }
    else if(start_hit_buff_ && (parent_node_ptr_->get_clock()->now() - buff_aim_time_).seconds() > 0.3) {
        // 开始打符
        modecmd_BB_->should_gimbal_patrol = 0b11;

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void GetBuff::onHalted()
{
    setOutput("in_buff_mode", false);
}

}  // namespace myBT

#endif