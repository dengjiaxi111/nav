#ifndef __PATROL__                  //这个节点用于航点导航。
#define __PATROL__
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>  
#include <ctime>    

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
namespace myBT{
const bool GOHIGHLAND = true;

enum class PatrolArea{
    HOME,
    HIGHLAND,
    ENEMY
};

class Patrol:public BT::StatefulActionNode
{
private:
    bool debug_flag_;
    bool start_defence_ ;

    shared_ptr<rclcpp::Node> parent_node_ptr_;
    rclcpp::Time last_publish_time_;
    rclcpp::Time last_update_time_;
    string yaml_path_;
    string package_path_;

    shared_ptr<robots_msgs::msg::ModeCmd>       modecmd_BB_;
    shared_ptr<robots_msgs::msg::GameStatus>    game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus>   robot_status_BB_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;
    vector<EnemyInfo> enemy_info_;


    std::vector<geometry_msgs::msg::PoseStamped> red_highland_point_goals_;
    std::vector<geometry_msgs::msg::PoseStamped> blue_highland_point_goals_;
    std::vector<geometry_msgs::msg::PoseStamped> red_home_point_goals_;
    std::vector<geometry_msgs::msg::PoseStamped> blue_home_point_goals_;
    std::vector<geometry_msgs::msg::PoseStamped> red_enemy_point_goals_;
    std::vector<geometry_msgs::msg::PoseStamped> blue_enemy_point_goals_;
    PatrolArea patrol_area_;
    PatrolArea last_patrol_area_;

    std::vector<geometry_msgs::msg::PoseStamped>* point_goals_ptr_;

    shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;

    // 巡逻相关变量 
    int current_goal_index_ = 0;     // 当前目标点索引
    bool is_forward_ = true;            // 巡逻方向(true为正向,false为反向)
    rclcpp::Time arrive_time_;          // 到达目标点的时间
    bool at_goal_ = false;              // 是否在目标点
    const double GOAL_THRESHOLD = 0.6;   // 到达目标点的阈值(米)
    const double STAY_DURATION = 10.0;   // 在目标点停留时间(秒)
    rclcpp::Time last_enemy_visible_time;
    bool reached_enemy_highland;
    bool fk_enemy_;

    // 吊射相关变量
    rclcpp::Time last_snipe_time_;
    int last_base_hp_;
    int snipe_cnt_;

public:
    Patrol(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
    {   

        parent_node_ptr_ = parent_node;
        last_update_time_ = parent_node_ptr_->get_clock()->now();
        last_publish_time_ = parent_node_ptr_->get_clock()->now();
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");
        
        parent_node_ptr_->declare_parameter<bool>("Patrol_debug",true);

        parent_node_ptr_->get_parameter("Patrol_debug",debug_flag_);
        parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");

        {
        //从yaml中读取点
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_highland.x", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_highland.y", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_home.x", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_home.y", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_enemy.x", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red.patrol_enemy.y", std::vector<double>());
            
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue.patrol_highland.x", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue.patrol_highland.y", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue.patrol_home.x", std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue.patrol_home.y", std::vector<double>());
            parent_node_ptr_->declare_parameter<bool>("fk_enemy", false);
        }
        
        parent_node_ptr_->get_parameter("fk_enemy", fk_enemy_);


        // 读取坐标数组
        std::vector<double> red_highland_x, red_highland_y;
        std::vector<double> red_home_x, red_home_y;
        parent_node_ptr_->get_parameter("red.patrol_highland.x", red_highland_x);
        parent_node_ptr_->get_parameter("red.patrol_highland.y", red_highland_y);
        parent_node_ptr_->get_parameter("red.patrol_home.x", red_home_x);
        parent_node_ptr_->get_parameter("red.patrol_home.y", red_home_y);

        // 转换为PoseStamped并存入red_point_goal_
        for(size_t i = 0; i < red_highland_x.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = red_highland_x[i];
            pose.pose.position.y = red_highland_y[i];
            pose.pose.orientation.w = 1.0;
            red_highland_point_goals_.push_back(pose);
        }
    
        for(size_t i = 0; i < red_home_x.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = red_home_x[i];
            pose.pose.position.y = red_home_y[i];
            pose.pose.orientation.w = 1.0;
            red_home_point_goals_.push_back(pose);
        }

        // 同样处理蓝方的点位
        std::vector<double> blue_highland_x, blue_highland_y;
        std::vector<double> blue_home_x, blue_home_y;
        parent_node_ptr_->get_parameter("blue.patrol_highland.x", blue_highland_x);
        parent_node_ptr_->get_parameter("blue.patrol_highland.y", blue_highland_y);
        parent_node_ptr_->get_parameter("blue.patrol_home.x", blue_home_x);
        parent_node_ptr_->get_parameter("blue.patrol_home.y", blue_home_y);
        for(size_t i = 0; i < blue_highland_x.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = blue_highland_x[i];
            pose.pose.position.y = blue_highland_y[i];
            pose.pose.orientation.w = 1.0;
            blue_highland_point_goals_.push_back(pose);
        }
        for(size_t i = 0; i < blue_home_x.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = blue_home_x[i];
            pose.pose.position.y = blue_home_y[i];
            pose.pose.orientation.w = 1.0;
            blue_home_point_goals_.push_back(pose);
        }

        goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        last_publish_time_ = parent_node_ptr_->get_clock()->now();
        last_enemy_visible_time = parent_node_ptr_->get_clock()->now();
        last_snipe_time_ = parent_node_ptr_->get_clock()->now();
        last_base_hp_ = 5000;
        snipe_cnt_ = 0;
        patrol_area_ = PatrolArea::HIGHLAND;
        last_patrol_area_ = PatrolArea::HIGHLAND;
        reached_enemy_highland = false;
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::BidirectionalPort<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd"));
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
        ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
        ports_list.insert(BT::InputPort<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos"));
        ports_list.insert(BT::InputPort<vector<EnemyInfo>>("enemypose"));
        return ports_list;
    }

    bool isEnemySniping();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

BT::NodeStatus Patrol::onStart()
{
    cout << GREEN << "---Patrol onStart---" << RESET << endl;
    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);
    getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);
    getInput<vector<EnemyInfo>>("enemypose", enemy_info_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr) || (transform_BB_ == nullptr)){
        cout << RED << "NODE Patrol GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus Patrol::onRunning()
{   
    cout << GREEN << "---Patrol onRunning---" << RESET << endl;
    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);
    getInput<vector<EnemyInfo>>("enemypose", enemy_info_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr) || (transform_BB_ == nullptr))
    {
        cout << RED << "NODE PATROL GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::RUNNING;
    }

    modecmd_BB_->should_chassis_spin = 1;
    modecmd_BB_->should_gimbal_patrol = 1;
    modecmd_BB_->use_capacity = 0;
    modecmd_BB_->buy_bullet = 0;
    modecmd_BB_->rebirth = 0;

    setOutput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

    // 考虑通过基地掉血判断地方英雄是否开启吊射
    bool enemy_sniping = isEnemySniping();

    // TODO根据场上情况选择对应大区点位
    patrol_area_ = PatrolArea::HIGHLAND;

    // 巡逻区域发生变化
    if(last_patrol_area_ != patrol_area_){
        current_goal_index_ = 0;
    }
    last_patrol_area_ = patrol_area_;



    if(robot_status_BB_->robot_id == 7) //红方
    {
        switch(patrol_area_){
            case PatrolArea::HIGHLAND:
                point_goals_ptr_ = &red_highland_point_goals_;
                break;
            case PatrolArea::ENEMY:
                break;
            case PatrolArea::HOME:
                point_goals_ptr_ = &red_home_point_goals_;
                break;
        }
    }
    else if(robot_status_BB_->robot_id == 107) // 蓝方
    {
        switch(patrol_area_){
            case PatrolArea::HIGHLAND:
                point_goals_ptr_ = &blue_highland_point_goals_;
                break;
            case PatrolArea::ENEMY:
                break;
            case PatrolArea::HOME:
                point_goals_ptr_ = &blue_home_point_goals_;
                break;
        }
    }
    else{
        cout << RED << "INVALID ROBOT ID!" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }

    if (point_goals_ptr_ == nullptr || point_goals_ptr_->empty()) {
        cout << RED << "No patrol points available!" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }


    // if(!reached_enemy_highland){
    //     current_goal_index_ = 1;
    // }

    auto current_goal = (*point_goals_ptr_)[current_goal_index_];
    double dist = sqrt(pow(transform_BB_->transform.translation.x - current_goal.pose.position.x, 2) +
                      pow(transform_BB_->transform.translation.y - current_goal.pose.position.y, 2));

    if(debug_flag_)
    {
        cout << BLUE << "current position: x: " << transform_BB_->transform.translation.x 
            << " y: " << transform_BB_->transform.translation.y << RESET << endl;
        cout << BLUE << "target point: x: " << current_goal.pose.position.x 
                << " y: " << current_goal.pose.position.y << RESET << endl;
        cout << BLUE << "distance to target: " << dist << RESET << endl;
    }

    // 检查是否到达目标点
    if(dist <= GOAL_THRESHOLD) {
        if(!fk_enemy_){
            if(current_goal_index_ == 1){
                reached_enemy_highland = true;
            }
            if(!at_goal_) {
                // 刚到达目标点，记录时间
                arrive_time_ = parent_node_ptr_->get_clock()->now();
                at_goal_ = true;
                if(debug_flag_) {
                    cout << CYAN << "REACHED PATROL POINT " << current_goal_index_ << RESET << endl;
                }
            }
            for (auto& enemy : enemy_info_) {
                if (enemy._visible && enemy._shootable && (enemy._id != 0)) {
                    last_enemy_visible_time = parent_node_ptr_->get_clock()->now();
                }
            }
            // 检查是否需要继续停留
            if(robot_status_BB_->ttk_status == 1){
                auto stay_time = parent_node_ptr_->get_clock()->now() - arrive_time_;
                auto enemy_visible_time = parent_node_ptr_->get_clock()->now() - last_enemy_visible_time;
                bool our_advantage = (game_status_BB_->my_base_hp > game_status_BB_->enemy_base_hp + 300);
    
                if(current_goal_index_ == 0 && stay_time.seconds() >= 40){
                    current_goal_index_ = 0;
                    at_goal_ = false;
                }
                else if(current_goal_index_ == 1 && stay_time.seconds() > 8 && enemy_visible_time.seconds() > 8){
                    current_goal_index_ = 0;
                    at_goal_ = false;
                }
                
                // if(our_advantage){
                //     current_goal_index_ = 0;
                // }
                // else{
                //     current_goal_index_ = 0;
                // }
                
                // if (stay_time.seconds() >= 5.0) {
                //     if(debug_flag_) {
                //         cout << GREEN << "STAYED AT GOAL FOR " << stay_time.seconds() << " SECONDS, MOVING TO NEXT GOAL" << RESET << endl;
                //     }
                //     at_goal_ = false; // 重置到达状态
                //     // 切换到下一个目标点
                //     if(is_forward_) {
                //         current_goal_index_++;
                //         if(current_goal_index_ >= point_goals_ptr_->size()) {
                //             is_forward_ = false;
                //             current_goal_index_ = point_goals_ptr_->size() - 1; // 回到最后一个点
                //         }
                //     }
                //     else{
                //         current_goal_index_--;
                //         if(current_goal_index_ < 0) {
                //             is_forward_ = true;
                //             current_goal_index_ = 0; // 回到第一个点
                //         }
                //     }
                // }
            }
        }
        else{
            
            if(!at_goal_) {
                // 刚到达目标点，记录时间
                arrive_time_ = parent_node_ptr_->get_clock()->now();
                at_goal_ = true;
                if(debug_flag_) {
                    cout << CYAN << "REACHED PATROL POINT " << current_goal_index_ << RESET << endl;
                }
            }
            if(robot_status_BB_->ttk_status == 1){
                auto stay_time = parent_node_ptr_->get_clock()->now() - arrive_time_;
                if (stay_time.seconds() >= 10.0) {
                    if(debug_flag_) {
                        cout << GREEN << "STAYED AT GOAL FOR " << stay_time.seconds() << " SECONDS, MOVING TO NEXT GOAL" << RESET << endl;
                    }
                    at_goal_ = false; // 重置到达状态
                    // 切换到下一个目标点
                    if(is_forward_) {
                        current_goal_index_++;
                        if(current_goal_index_ >= point_goals_ptr_->size()) {
                            is_forward_ = false;
                            current_goal_index_ = point_goals_ptr_->size() - 1; // 回到最后一个点
                        }
                    }
                    else{
                        current_goal_index_--;
                        if(current_goal_index_ < 0) {
                            is_forward_ = true;
                            current_goal_index_ = 0; // 回到第一个点
                        }
                    }
                }
            }
            
        }
    }

    auto send_goal_time_diff = parent_node_ptr_->get_clock()->now() - last_publish_time_;
    if(send_goal_time_diff.seconds() >= 1.0) {
        current_goal.header.stamp = parent_node_ptr_->get_clock()->now();
        goal_publisher_->publish(current_goal);
        last_publish_time_ = parent_node_ptr_->get_clock()->now();
    }
    return BT::NodeStatus::RUNNING;

}


void Patrol::onHalted()
{
    cout << YELLOW << "---Patrol onHalt---" << RESET << endl;
    current_goal_index_ = 0;
    at_goal_ = false;
    is_forward_ = true;
}

bool Patrol::isEnemySniping()
{
    // 检查敌方英雄是否在吊射
    int current_base_hp = this->game_status_BB_->enemy_base_hp;

    //敌方吊射造成伤害
    if(last_base_hp_ - current_base_hp >= 200 && last_base_hp_ - current_base_hp < 625){
        snipe_cnt_++;
        last_snipe_time_ = parent_node_ptr_->get_clock()->now();
        if(debug_flag_) {
            cout << RED << "ENEMY SNIPING DETECTED! "<< RESET << endl;
        }
    }
    last_base_hp_ = current_base_hp;

    // 长时间未掉血认为停止
    if((parent_node_ptr_->get_clock()->now() - last_snipe_time_).seconds() > 6.0) {
        snipe_cnt_ = 0; // 重置计数
    }

    if(snipe_cnt_ >= 2) {
        if(debug_flag_) {
            cout << RED << "ENEMY SNIPING CONFIRMED! "<< RESET << endl;
        }
        return true; // 敌方正在吊射
    }
    else{
        return false; // 敌方没有吊射
    }
}

}

#endif
