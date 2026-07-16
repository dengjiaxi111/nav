#ifndef __GOSUPPLY__
#define __GOSUPPLY__
#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

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

namespace myBT
{

    // 需要回血时返回SUCESS，否则返回FAILURE
    class GoSupply:public BT::StatefulActionNode
    {
    private:

        bool debug_flag_;
        shared_ptr<rclcpp::Node> parent_node_ptr_;
        string yaml_path_;
        string package_path_;

        shared_ptr<robots_msgs::msg::ModeCmd>               modecmd_BB_;
        shared_ptr<robots_msgs::msg::GameStatus>      game_status_BB_;
        shared_ptr<robots_msgs::msg::RobotStatus>     robot_status_BB_;
        shared_ptr<geometry_msgs::msg::TransformStamped> current_pos_;

        bool need_healing_;
        bool need_bullet_;
        bool free_bullet_ready_;
        bool supply_area_get_;

        rclcpp::Time last_supply_time_;
        int last_game_time_;

        shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;
        rclcpp::Time last_publish_time_;
        rclcpp::Time supply_get_time_;

        geometry_msgs::msg::PoseStamped supply_area_goal_red_;
        geometry_msgs::msg::PoseStamped supply_area_goal_blue_;

        double dist_ ;

    public:
        GoSupply(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
        {
            parent_node_ptr_ = parent_node;
            package_path_ = ament_index_cpp::get_package_share_directory("mytree");
            
            parent_node_ptr_->declare_parameter<bool>("GoSupply_debug",true);

            parent_node_ptr_->get_parameter("GoSupply_debug",debug_flag_);
            parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");

            {
            //从yaml中读取点
            parent_node_ptr_->declare_parameter<double>("red.supply.x",-0.374);
            parent_node_ptr_->declare_parameter<double>("red.supply.y",0.3);
            parent_node_ptr_->declare_parameter<double>("blue.supply.x",-0.374);
            parent_node_ptr_->declare_parameter<double>("blue.supply.y",0.3);

            parent_node_ptr_->get_parameter("red.supply.x",supply_area_goal_red_.pose.position.x);
            parent_node_ptr_->get_parameter("red.supply.y",supply_area_goal_red_.pose.position.y);
            parent_node_ptr_->get_parameter("blue.supply.x",supply_area_goal_blue_.pose.position.x);
            parent_node_ptr_->get_parameter("blue.supply.y",supply_area_goal_blue_.pose.position.y);

            supply_area_goal_red_.header.frame_id = "map";
            supply_area_goal_blue_.header.frame_id = "map";
            }

            need_healing_ = 0;
            need_bullet_ = 0;
            free_bullet_ready_ =false ;
            supply_area_get_ = false;

            last_game_time_ = 419;
            last_supply_time_ = parent_node_ptr_->get_clock()->now();
            last_publish_time_ = parent_node_ptr_->get_clock()->now();
            supply_get_time_ = parent_node_ptr_->get_clock()->now();
            goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);
        }

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports_list;
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

    BT::NodeStatus GoSupply::onStart()
    {
        cout << GREEN << "---GoSupply onStart---" << RESET << endl;
        
        getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
        getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",current_pos_);
        getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

        if (!current_pos_) {
            cout << RED << "FAILED TO GET CURRENT POSITION FROM INPUT PORT" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
        
        if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr))
        {
            cout << RED << "Gosupply GET INFO FAILED" << RESET << endl;
            }

        // 计算是否发放了免费子弹
        if(game_status_BB_->game_progress == 4 && game_status_BB_->game_type == 1)
        {
           if((last_game_time_/ 60) > (game_status_BB_->stage_remain_time / 60))
           {
                // 说明发放了免费子弹
                free_bullet_ready_ = true;
           }
           last_game_time_ = game_status_BB_->stage_remain_time;
        }

        auto supply_time_diff = parent_node_ptr_->get_clock()->now() - last_supply_time_;
        if(game_status_BB_->my_hp <= 120)
        {
            need_healing_ = true;
            if(debug_flag_)
            {
                cout << CYAN << "NEED HEALING!" << RESET << endl;
                cout << BLUE << "supply time: " << supply_time_diff.seconds() << " s" << RESET << endl;
                cout << BLUE << "current hp: " << int(game_status_BB_->my_hp) << RESET << endl;
            }
        }
        
        if(robot_status_BB_->projectile_allowance_17mm <= 20 && free_bullet_ready_)
        {
            need_bullet_ = true;
            if(debug_flag_)
            {
                cout << CYAN << "NEED BULLET!" << RESET << endl;
                cout << BLUE << "supply time: " << supply_time_diff.seconds() << " s" << RESET << endl;
                cout << BLUE << "current hp: " << int(game_status_BB_->my_hp) << RESET << endl;
            }
        }

        if(!need_healing_ && (!need_bullet_))
        {
// 无需补给
            if(debug_flag_)
            {
                cout << CYAN << "NO SUPPLY NEEDED" << RESET << endl;
                cout << BLUE << "supply time: " << supply_time_diff.seconds() << " s" << RESET << endl;
                cout << BLUE << "current hp: " << int(game_status_BB_->my_hp) << RESET << endl;
            }
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GoSupply::onRunning()
    {
        cout << GREEN << "---GoSupply onRunning---" << RESET << endl;
        
        getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
        getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",current_pos_);
        if(debug_flag_)
        {
            //TODO: 完成DEBUG信息打印
            cout<<CYAN<<"Go supply on RUNNING"<<RESET<<endl;

        }
        // 判定是否已经补给
        if((game_status_BB_->my_hp >= 370 && robot_status_BB_->supply_blood_area == 1) || (supply_area_get_ && 
            (parent_node_ptr_->get_clock()->now() - supply_get_time_).seconds() > 8))
        {
            // 补给完成
            if(debug_flag_)
            {
                cout<<GREEN<<"SUPPLY FINISHED!" << RESET << endl;
            }
            free_bullet_ready_ = false; 
            modecmd_BB_->use_capacity = 0;
            need_healing_ = need_bullet_ = 0;
            last_supply_time_ = parent_node_ptr_->get_clock()->now();
            supply_area_get_ = false;
            return BT::NodeStatus::SUCCESS;
        }

        geometry_msgs::msg::PoseStamped supply_area_goal;
        if(robot_status_BB_->robot_id == 7) //红方
        {
            supply_area_goal = supply_area_goal_red_;
        }
        else if(robot_status_BB_->robot_id == 107) //蓝方
        {
            supply_area_goal = supply_area_goal_blue_;
        }
        else // 哈哈 主控ID设置错了
        {
            cout << RED << "INVALID ROBOT ID!" << RESET << endl;
            supply_area_goal = supply_area_goal_red_;
        }

        dist_ = sqrt(pow(current_pos_->transform.translation.x - supply_area_goal.pose.position.x, 2) +
                     pow(current_pos_->transform.translation.y - supply_area_goal.pose.position.y, 2));

        // 通过距离判断是否到到达补血点以及到达时间
        if((dist_ < 0.3 || robot_status_BB_->supply_blood_area == 1) && supply_area_get_ == false )
        {
            supply_area_get_ = true;
            supply_get_time_ = parent_node_ptr_->get_clock()->now();
            if(debug_flag_)
            {
                cout << GREEN << "REACHED SUPPLY AREA!" << RESET << endl;
            }
        }

        if(robot_status_BB_->supply_blood_area == 1)
        {
            modecmd_BB_->use_capacity = 0;
            modecmd_BB_->should_chassis_spin = 0;
        }
        else
        {
            modecmd_BB_->use_capacity = 1;
            modecmd_BB_->should_chassis_spin = 2;
        }

        auto send_goal_time_diff_ = parent_node_ptr_->get_clock()->now() - last_publish_time_;

        // 每隔1s才发布一次目标点，这样可以避免浪费太多导航资源，其他节点同
        if(send_goal_time_diff_.seconds() >= 1)
        {
            cout << CYAN << "SENDING SUPPLY GOAL" << RESET << endl;
            supply_area_goal.header.stamp = parent_node_ptr_->get_clock()->now();
            goal_publisher_->publish(supply_area_goal);
            last_publish_time_ = parent_node_ptr_->get_clock()->now();

            if(debug_flag_)
            {
                //TODO: 完成DEBUG信息打印
                cout<<BLUE<<"moving to supply point" << RESET << endl;
                cout<<BLUE<< "target goal x: " << supply_area_goal.pose.position.x << RESET << endl;
                cout<<BLUE<< "target goal y: " << supply_area_goal.pose.position.y << RESET << endl;                    
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void GoSupply::onHalted()
    {
        cout << YELLOW << "---GoSupply onHalt---" << RESET << endl;
        supply_area_get_ = false;
        need_healing_ = false;
        need_bullet_ = false;
    }


}
#endif