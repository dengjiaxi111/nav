#ifndef __WAITINGFORSTART__
#define __WAITINGFORSTART__
#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
namespace myBT{

    // 比赛正式开始后返回Failure
    class WaitingForStart:public BT::StatefulActionNode
    {
    private:

        bool debug_flag_;
        bool debug_mode_;
        shared_ptr<rclcpp::Node> parent_node_ptr_;
        string yaml_path_;
        string package_path_;

        shared_ptr<robots_msgs::msg::ModeCmd>       modecmd_BB_;
        shared_ptr<robots_msgs::msg::GameStatus>    game_status_BB_;
        shared_ptr<robots_msgs::msg::RobotStatus>   robot_status_BB_;

        // 添加 NavigateToPose Action Client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    public:
        WaitingForStart(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
        {
            debug_mode_ = false;
            parent_node_ptr_ = parent_node;
            package_path_ = ament_index_cpp::get_package_share_directory("mytree");
            
            parent_node_ptr_->declare_parameter<bool>("WaitingForStart_debug",true);

            parent_node_ptr_->get_parameter("WaitingForStart_debug",debug_flag_);
            parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");

            // 初始化 NavigateToPose Action Client
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(parent_node_ptr_, "navigate_to_pose");

        }
        static BT::PortsList providedPorts()
        {
            BT::PortsList ports_list;
            ports_list.insert(BT::BidirectionalPort<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd"));
            ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
            ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
            ports_list.insert(BT::InputPort<bool>("debugmode"));
            return ports_list;
        }

        void cancelNavigationGoal(){
            if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
                RCLCPP_ERROR(parent_node_ptr_->get_logger(), "NavigateToPose action server not available!");
                return;
            }
            auto goal_handle = action_client_->async_cancel_all_goals();
            
        }
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;
    };

    BT::NodeStatus WaitingForStart::onStart()
    {
        cout << GREEN << "---WaitingForStart onStart---" << RESET << endl;

        getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
        getInput<bool>("debugmode",debug_mode_);

        if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr))
        {
            cout << RED << "NODE WAITINGFORSTART GET INFO FAILED" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
        if((game_status_BB_->game_type == 1) && (game_status_BB_->game_progress == 4))
        {   
            return BT::NodeStatus::FAILURE;
        }

        if(debug_mode_){
            cout<<GREEN<< "WAITING FOR GAME START"<<RESET<<endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus WaitingForStart::onRunning()
    {   
        cout << GREEN << "---WaitingForStart onRunning---" << RESET << endl;

        getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

        if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr) || (modecmd_BB_ == nullptr))
        {
            cout << RED << "NODE WAITINGFORSTART GET INFO FAILED" << RESET << endl;
            return BT::NodeStatus::RUNNING;
        }

        if(debug_flag_)
        {
            cout << BLUE << "game progress: " << int(game_status_BB_->game_progress) << RESET << endl;
            cout << BLUE << "game type: " << int(game_status_BB_->game_type) << RESET << endl;
        }

        /*
            bit 0-3：比赛类型
            • 1：RoboMaster 机甲大师超级对抗赛
            • 2：RoboMaster 机甲大师高校单项赛
            • 3：ICRA RoboMaster 高校人工智能挑战赛
            • 4：RoboMaster 机甲大师高校联盟赛 3V3 对抗
            • 5：RoboMaster 机甲大师高校联盟赛步兵对抗
            bit 4-7：当前比赛阶段
            • 0：未开始比赛
            • 1：准备阶段
            • 2：十五秒裁判系统自检阶段
            • 3：五秒倒计时
            • 4：比赛中
            • 5：比赛结算中
        */
        if((game_status_BB_->game_type == 1) && (game_status_BB_->game_progress == 4))
        {
            // 在比赛中
            modecmd_BB_->should_gimbal_patrol = 1;
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            // 比赛尚未正式开始
            if((game_status_BB_->game_type == 1) && (game_status_BB_->game_progress < 4))
            {
                modecmd_BB_->use_capacity = 0;
                modecmd_BB_->should_gimbal_patrol = 1;
                modecmd_BB_->should_chassis_spin = 0;
                if(game_status_BB_->game_progress >= 2){
                    modecmd_BB_->should_chassis_spin = 2;
                }
                modecmd_BB_->rebirth = 0;
                modecmd_BB_->buy_bullet = 0;
            }
            // 比赛结束
            else if((game_status_BB_->game_type == 1) && (game_status_BB_->game_progress == 5))
            {
                modecmd_BB_->use_capacity = 0;
                modecmd_BB_->should_gimbal_patrol = 0;
                modecmd_BB_->should_chassis_spin = 0;
                modecmd_BB_->rebirth = 0;
                modecmd_BB_->buy_bullet = 0;
            }
            // 其他
            else
            {
                modecmd_BB_->use_capacity = 0;
                modecmd_BB_->should_gimbal_patrol = 0;
                modecmd_BB_->should_chassis_spin = 0;
                modecmd_BB_->rebirth = 0;
                modecmd_BB_->buy_bullet = 0;
            }

            // 取消当前导航目标
            cancelNavigationGoal();

            setOutput("modecmd",modecmd_BB_);
            return BT::NodeStatus::RUNNING;

        }
    }
    void WaitingForStart::onHalted()
    {
        cout << YELLOW << "---WaitingForStart onHalt---" << RESET << endl;
    }
 

}
#endif