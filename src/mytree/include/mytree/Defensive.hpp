#ifndef __Defensive__                  //这个节点用于守家
#define __Defensive__
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

const bool GOFORTRESS = false;

// 运行中返回RUNNING
class Defensive:public BT::StatefulActionNode
{
private:

    bool debug_flag_;
    bool on_buff_point_;
    bool will_on_buff_point_;   
    bool defensive_;                  //能否进到这个节点的判断，根据能量值和基地血量和堡垒占领状态
    bool base_open_;
    bool fortress_capturing_;
    rclcpp::Time start_capture_time_;

    shared_ptr<rclcpp::Node> parent_node_ptr_;
    rclcpp::Time last_publish_time_;
    rclcpp::Time last_update_time_;
    string yaml_path_;
    string package_path_;

    shared_ptr<robots_msgs::msg::ModeCmd>       modecmd_BB_;
    shared_ptr<robots_msgs::msg::GameStatus>    game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus>   robot_status_BB_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;

    geometry_msgs::msg::PoseStamped point_goal_red_;
    geometry_msgs::msg::PoseStamped point_goal_blue_;

    shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;


public:
    Defensive(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
    {   
        defensive_= false;
        fortress_capturing_ = false;
        on_buff_point_ = false;
        will_on_buff_point_ = false;
        base_open_ = false;

        parent_node_ptr_ = parent_node;
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");
        
        parent_node_ptr_->declare_parameter<bool>("Defensive_debug",true);

        parent_node_ptr_->get_parameter("Defensive_debug",debug_flag_);
        parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");

        std::srand(std::time(nullptr));

        {
        //从yaml中读取点
        parent_node_ptr_->declare_parameter<double>("red.fortress.x",2.0);
        parent_node_ptr_->declare_parameter<double>("red.fortress.y",2.0);
        parent_node_ptr_->declare_parameter<double>("blue.fortress.x",-2.0);
        parent_node_ptr_->declare_parameter<double>("blue.fortress.y",2.0);

        parent_node_ptr_->get_parameter("red.fortress.x",point_goal_red_.pose.position.x);
        parent_node_ptr_->get_parameter("red.fortress.y",point_goal_red_.pose.position.y);
        parent_node_ptr_->get_parameter("blue.fortress.x",point_goal_blue_.pose.position.x);
        parent_node_ptr_->get_parameter("blue.fortress.y",point_goal_blue_.pose.position.y);
        }

        goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        start_capture_time_ = last_publish_time_ = parent_node_ptr_->get_clock()->now();

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

BT::NodeStatus Defensive::onStart() {
    cout << GREEN << "---Defensive onStart---" << RESET << endl;

    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);
    getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);
    setOutput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr) || (transform_BB_ == nullptr))
    {
        cout << RED << "NODE Defensive GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }
    if(game_status_BB_->our_fortress_status == 0b00){
        fortress_capturing_ = false;
    }
    if(!fortress_capturing_ && game_status_BB_->our_fortress_status == 0b10){
        fortress_capturing_ = true;
        start_capture_time_ = parent_node_ptr_->get_clock()->now();
    }
    if(fortress_capturing_ && (parent_node_ptr_->get_clock()->now() - start_capture_time_).seconds() > 5.0){
        return BT::NodeStatus::RUNNING;
    }

    if( (game_status_BB_->my_base_hp <= 2000 || robot_status_BB_->remaining_energy == 0x10 || robot_status_BB_->remaining_energy == 0x00) == 1){
        cout << static_cast<int>(robot_status_BB_->remaining_energy) << endl;
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Defensive::onRunning() {   
    cout << GREEN << "---Defensive onRunning---" << RESET << endl;

    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr) || (transform_BB_ == nullptr))
    {
        cout << RED << "NODE Defensive GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::RUNNING;
    }

    geometry_msgs::msg::PoseStamped* defensive_goal_ptr;
    if(robot_status_BB_->robot_id == 7) //红方
    {
        defensive_goal_ptr = &point_goal_red_;
    }
    else if(robot_status_BB_->robot_id == 107) //蓝方
    {
        defensive_goal_ptr = &point_goal_blue_;
    }
    else // 哈哈 主控ID设置错了
    {
        cerr << "INVALID ROBOT ID!" << endl;
        defensive_goal_ptr = &point_goal_blue_;
    }

    if(game_status_BB_->our_fortress_status == 0b00){
        fortress_capturing_ = false;
    }
    if(fortress_capturing_ && (parent_node_ptr_->get_clock()->now() - start_capture_time_).seconds() > 20.0){
        base_open_ = true;
    }
    if(!base_open_ && !fortress_capturing_  && (game_status_BB_->my_base_hp>2000) && robot_status_BB_->remaining_energy >= 0x10){
        return BT::NodeStatus::FAILURE;
    }



    double dist = sqrt(pow(transform_BB_->transform.translation.x - defensive_goal_ptr->pose.position.x, 2) +
                            pow(transform_BB_->transform.translation.y - defensive_goal_ptr->pose.position.y, 2));

    if(debug_flag_)
    {
        cout << BLUE << "target point: x: " << defensive_goal_ptr->pose.position.x 
             << " y: " << defensive_goal_ptr->pose.position.y << RESET << endl;
        cout << BLUE << "current position: x: " << transform_BB_->transform.translation.x
             << " y: " << transform_BB_->transform.translation.y << RESET << endl;

        cout << BLUE << "distance to target: " << dist << RESET << endl;

auto pub_time_diff = parent_node_ptr_->get_clock()->now() - last_publish_time_;
        if(will_on_buff_point_){
            cout << CYAN << "MOVING TO BUFF POINT" << RESET << endl;
        }
        if(on_buff_point_){
            cout << CYAN << "ON BUFF POINT" << RESET << endl;
        }
        else{
            cout << CYAN << "MOVING TO DEFENSIVE POINT" << RESET << endl;
        }
    }
    if(GOFORTRESS){
        if(dist <=0.40){

            modecmd_BB_->should_chassis_spin = 0;
            modecmd_BB_->should_gimbal_patrol = 1;
            modecmd_BB_->use_capacity = 0;
            modecmd_BB_->buy_bullet = 0;
            modecmd_BB_->rebirth = 0;
    
            on_buff_point_ = true;
            will_on_buff_point_ = false; 
        }
        else if(dist <= 2.5){
            will_on_buff_point_ = true;
            modecmd_BB_->should_chassis_spin = 2;
            modecmd_BB_->should_gimbal_patrol = 0;
            modecmd_BB_->use_capacity = 1;
            modecmd_BB_->buy_bullet = 0;
            modecmd_BB_->rebirth = 0;
        }
    }
    else{
        if(robot_status_BB_->remaining_energy == 0x10 || robot_status_BB_->remaining_energy == 0x00){
            
            if(dist <= 0.4){
                modecmd_BB_->should_chassis_spin = 2;
            }
            else{
                modecmd_BB_->should_chassis_spin = 0;
            }
            
        }
        else{
            modecmd_BB_->should_chassis_spin = 2;
        }
            
        modecmd_BB_->should_gimbal_patrol = 1;
        modecmd_BB_->use_capacity = 0;
        modecmd_BB_->buy_bullet = 0;
        modecmd_BB_->rebirth = 0;
        if(robot_status_BB_->remaining_energy > 0x10){
            modecmd_BB_->should_chassis_spin = 1;
        }
    }

    setOutput("modecmd",modecmd_BB_);

    auto send_goal_time_diff_ = parent_node_ptr_->get_clock()->now() - last_publish_time_;
    if(send_goal_time_diff_ .seconds() >= 1){
        defensive_goal_ptr->header.frame_id = "map";
        defensive_goal_ptr->header.stamp = parent_node_ptr_->get_clock()->now();
        goal_publisher_->publish(*defensive_goal_ptr);
        last_publish_time_ = parent_node_ptr_->get_clock()->now();
    }
    return BT::NodeStatus::RUNNING;
}


void Defensive::onHalted() {
    cout << YELLOW << "---Defensive onHalt---" << RESET << endl;
}


}

#endif
