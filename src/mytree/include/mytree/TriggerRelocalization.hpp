#ifndef __TRIGGERRELOCALIZATION__
#define __TRIGGERRELOCALIZATION__
#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/chassis_odom.hpp"

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
class TriggerRelocalization : public BT::StatefulActionNode
{
private:

    enum class State {
        WAITING_START,
        CALL_SERVICE,
        WAIT_COMPLETE
    };

    bool debug_flag_;
    bool use_relocalization_;
    shared_ptr<rclcpp::Node> parent_node_ptr_;
    string package_path_;
    rclcpp::Time state_start_time_;
    rclcpp::Time last_relocalization_time_;
    State current_state_ = State::WAITING_START;

    // 重定位服务
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr relocalization_client_;
    shared_future<shared_ptr<std_srvs::srv::Trigger::Response>> relocalization_pending_future_;

    // 取消导航点action
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    // 订阅电控里程计
    rclcpp::Subscription<robots_msgs::msg::ChassisOdom>::SharedPtr odom_sub_;
    robots_msgs::msg::ChassisOdom current_odom_;

    shared_ptr<robots_msgs::msg::GameStatus> game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus> robot_status_BB_;
public:
    TriggerRelocalization(const string &name, const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node)
        : BT::StatefulActionNode(name, config), parent_node_ptr_(parent_node)
    {
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");
        parent_node_ptr_->declare_parameter<bool>("TriggerRelocalization_debug", true);
        parent_node_ptr_->declare_parameter<bool>("use_relocalization", false);
        parent_node_ptr_->get_parameter("TriggerRelocalization_debug", debug_flag_);
        parent_node_ptr_->get_parameter("use_relocalization", use_relocalization_);

        relocalization_client_ = parent_node_ptr_->create_client<std_srvs::srv::Trigger>("/relocalization_service");
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(parent_node_ptr_, "navigate_to_pose");
        
        odom_sub_ = parent_node_ptr_->create_subscription<robots_msgs::msg::ChassisOdom>(
            "chassis_odom", 10,
            [this](const robots_msgs::msg::ChassisOdom::SharedPtr msg) {
                current_odom_ = *msg;
            });

        last_relocalization_time_ = parent_node_ptr_->get_clock()->now();
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
        ports_list.insert(BT::OutputPort<bool>("in_relocalization_mode"));
        return ports_list;
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

};

BT::NodeStatus TriggerRelocalization::onStart()
{
    cout << GREEN << "---TriggerRelocalization on start---" << RESET << endl;

    getInput("gamestatus", game_status_BB_);
    getInput("robotstatus", robot_status_BB_);
    if(game_status_BB_ == nullptr || robot_status_BB_ == nullptr) {
        RCLCPP_ERROR(parent_node_ptr_->get_logger(), "NODE TRIGGERRELOCALIZATION GET INFO FAILED");
        return BT::NodeStatus::FAILURE;
    }

    if(!use_relocalization_){
        return BT::NodeStatus::FAILURE;
    }

    if (!relocalization_client_->wait_for_service(chrono::seconds(0))) {
            RCLCPP_WARN(parent_node_ptr_->get_logger(), 
                "Relocalization service not available");
        return BT::NodeStatus::FAILURE;
    }

    // 满足自动触发重定位的条件：
    auto relocalization_time_diff = parent_node_ptr_->get_clock()->now() - last_relocalization_time_;
    bool in_game = (game_status_BB_->game_type == 1) && (game_status_BB_->game_progress == 4);
    bool not_in_battle = (robot_status_BB_->ttk_status == 1) && (pow(current_odom_.speed_x,2) + pow(current_odom_.speed_y,2) < 0.04);
    
    // 条件1，在赛内且死亡
    bool condition1 = in_game && (game_status_BB_->my_hp == 0) && relocalization_time_diff.seconds() > 30;
    // 条件2，脱战且距上次重定位超过 1分钟
    //bool condition2 = in_game && not_in_battle && (relocalization_time_diff.seconds() > 60);
    bool condition2 = false;
    // TODO: 条件3，长时间卡在某处
    bool condition3 = false;
    if(debug_flag_){
        cout << BLUE << "condition1:" << condition1 << endl;
        cout << BLUE << "condition2:" << condition2 << endl;
        cout << BLUE << "condition3:" << condition3 << endl;
    }

    if(condition1 || condition2 || condition3){
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;

}

// 进入running则说明触发重定位
BT::NodeStatus TriggerRelocalization::onRunning()
{
    cout << GREEN << "---TriggerRelocalization on running---" << RESET << endl;

    getInput("gamestatus", game_status_BB_);
    getInput("robotstatus", robot_status_BB_);
    
    auto current_time = parent_node_ptr_->get_clock()->now();
    if(current_state_ == State::WAITING_START){

        setOutput("in_relocalization_mode", true);
        // 取消当前导航点
        auto goal_handle = action_client_->async_cancel_all_goals();

        state_start_time_ = current_time;
        current_state_ = State::CALL_SERVICE;

        if (debug_flag_) {
            cout << BLUE << "Triggering Relocalization " << RESET << endl;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 等待1s后调用服务
    if (current_state_ == State::CALL_SERVICE) {
        if((current_time - state_start_time_).seconds() > 1.0){
            auto request = make_shared<std_srvs::srv::Trigger::Request>();
            relocalization_pending_future_ = relocalization_client_->async_send_request(request).future.share();
            
            state_start_time_ = current_time;
            current_state_ = State::WAIT_COMPLETE;
            return BT::NodeStatus::RUNNING;
        }
    }
    // 调用完成后再等待1s退出重定位模式
    else if(current_state_ == State::WAIT_COMPLETE){
        // 首先检查服务调用是否完成
        if (relocalization_pending_future_.wait_for(chrono::seconds(0)) == future_status::ready) {
            try {
                auto result = relocalization_pending_future_.get();
                if (!result->success) {
                    RCLCPP_ERROR(parent_node_ptr_->get_logger(), 
                        "Relocalization failed: %s", 
                        result->message.c_str());
                    return BT::NodeStatus::FAILURE;
                }
                
                // 服务调用成功，等待额外1秒
                if ((current_time - state_start_time_).seconds() > 1.0) {
                    setOutput("in_relocalization_mode", false);
                    last_relocalization_time_ = current_time;
                    current_state_ = State::WAITING_START;
                    return BT::NodeStatus::SUCCESS;
                }
            }
            catch(const exception& e) {
                RCLCPP_ERROR(parent_node_ptr_->get_logger(), 
                    "Exception while getting service response: %s", 
                    e.what());
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    return BT::NodeStatus::RUNNING;
}

void TriggerRelocalization::onHalted()
{
    // 重置状态
    current_state_ = State::WAITING_START;
    setOutput("in_relocalization_mode", false);
    cout << YELLOW << "---TriggerRelocalization halt!---" << RESET << endl;
}

}
#endif
