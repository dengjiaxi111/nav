//开局的节点
#ifndef __ATTACKDPOINT__
#define __ATTACKDPOINT__

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
namespace myBT{

// 是否前往高地
const bool GOTO_HIGHLAND = true;
// 比赛正式开始后返回Failure
class AttackDPoint:public BT::StatefulActionNode
{
private:

    bool debug_flag_;
    bool highland_get_;
    bool toilet_get_;
    bool toilet_out_;
    rclcpp::Time highland_get_time_;
    double dist_;

    shared_ptr<rclcpp::Node> parent_node_ptr_;
    string yaml_path_;
    string package_path_;

    shared_ptr<robots_msgs::msg::ModeCmd>       modecmd_BB_;
    shared_ptr<robots_msgs::msg::GameStatus>    game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus>   robot_status_BB_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;

    // 添加新的成员变量
    std::vector<geometry_msgs::msg::PoseStamped> red_point_goals_;     
    std::vector<geometry_msgs::msg::PoseStamped> blue_point_goals_;
    size_t current_goal_index_;
    
    geometry_msgs::msg::PoseStamped     red_point_goal_;     
    geometry_msgs::msg::PoseStamped     blue_point_goal_;     
    geometry_msgs::msg::PoseStamped     red_out_point_goal_;
    geometry_msgs::msg::PoseStamped     blue_out_point_goal_;
    shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;

    rclcpp::Time last_publish_time_;

public:
    AttackDPoint(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
    {
        parent_node_ptr_ = parent_node;
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");

        parent_node_ptr_->get_parameter_or("AttackDPoint_debug",debug_flag_,true);
        parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");
        {
        //从yaml中读取点
        std::vector<double> red_x, red_y, blue_x, blue_y;
        parent_node_ptr_->declare_parameter<std::vector<double>>("red.highland.x", {2.0});
        parent_node_ptr_->declare_parameter<std::vector<double>>("red.highland.y", {0.0});
        parent_node_ptr_->declare_parameter<std::vector<double>>("blue.highland.x", {-2.0});
        parent_node_ptr_->declare_parameter<std::vector<double>>("blue.highland.y", {0.0});
        parent_node_ptr_->declare_parameter<double>("red.toilet.x",2.0);
        parent_node_ptr_->declare_parameter<double>("red.toilet.y",0.0);
        parent_node_ptr_->declare_parameter<double>("blue.toilet.x",-2.0);
        parent_node_ptr_->declare_parameter<double>("blue.toilet.y",0.0);
        parent_node_ptr_->declare_parameter<double>("red.toilet_out.x",2.0);
        parent_node_ptr_->declare_parameter<double>("red.toilet_out.y",0.0);
        parent_node_ptr_->declare_parameter<double>("blue.toilet_out.x",-2.0);
        parent_node_ptr_->declare_parameter<double>("blue.toilet_out.y",0.0);
        if(GOTO_HIGHLAND){
            parent_node_ptr_->get_parameter("red.highland.x", red_x);
            parent_node_ptr_->get_parameter("red.highland.y", red_y);
            parent_node_ptr_->get_parameter("blue.highland.x", blue_x);
            parent_node_ptr_->get_parameter("blue.highland.y", blue_y);
            // 构建目标点数组
            for(size_t i = 0; i < red_x.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = red_x[i];
                pose.pose.position.y = red_y[i];
                red_point_goals_.push_back(pose);
            }

            for(size_t i = 0; i < blue_x.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = blue_x[i];
                pose.pose.position.y = blue_y[i];
                blue_point_goals_.push_back(pose);
            }
        }
        else{
            parent_node_ptr_->get_parameter("red.toilet.x",red_point_goal_.pose.position.x);
            parent_node_ptr_->get_parameter("red.toilet.y",red_point_goal_.pose.position.y);
            parent_node_ptr_->get_parameter("blue.toilet.x",blue_point_goal_.pose.position.x);
            parent_node_ptr_->get_parameter("blue.toilet.y",blue_point_goal_.pose.position.y);
            
            parent_node_ptr_->get_parameter("red.toilet_out.x",red_out_point_goal_.pose.position.x);
            parent_node_ptr_->get_parameter("red.toilet_out.y",red_out_point_goal_.pose.position.y);
            parent_node_ptr_->get_parameter("blue.toilet_out.x",blue_out_point_goal_.pose.position.x);
            parent_node_ptr_->get_parameter("blue.toilet_out.y",blue_out_point_goal_.pose.position.y);
        }
            red_point_goal_.header.frame_id = "map";
            blue_point_goal_.header.frame_id = "map";
        }
        goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        last_publish_time_ = parent_node_ptr_->get_clock()->now();
        highland_get_ = false;
        toilet_get_ = false;
        toilet_out_ =false;
        current_goal_index_ = 0;
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

BT::NodeStatus AttackDPoint::onStart()
{         
    cout << GREEN << "-----ATTACKDPOINT onSTART!----" << RESET << endl;

    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);
    getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr)){
        cout << RED << "NODE ATTACKDPOINT GET INFO FAILED" << RESET << endl;
    }

    if(GOTO_HIGHLAND){
        if(highland_get_ == true)
        {
            cout << RED << "ATTACKDPOINT finished(highland_get_)" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    else{
        // 前哨站已掉
        if(game_status_BB_->enemy_outpost_hp == 0 && toilet_out_ == true)
        {
            cout<<BLUE<<"ATTACKDPOINT finished(enemy_outpost_hp)"<<RESET<<endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
}

BT::NodeStatus AttackDPoint::onRunning()
{   
    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);

    if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr))
    {
        cout << RED << "NODE AttackDPoint GET INFO FAILED" << RESET << endl;
        return BT::NodeStatus::FAILURE;
    }
    std::vector<geometry_msgs::msg::PoseStamped>* goals_array;
    geometry_msgs::msg::PoseStamped*  goal_ptr;
    if(GOTO_HIGHLAND){
        // 红方
        if(robot_status_BB_->robot_id == 7) {
            goals_array = &red_point_goals_;
        }
        // 蓝方
        else if(robot_status_BB_->robot_id == 107){
            goals_array = &blue_point_goals_;
        }
        else{ // ID错误
            cout << RED << "NODE AttackDPoint GET INFO FAILED" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
        if(goals_array->empty()) {
            cout << RED << "No goals defined" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
        // 从数组中获取当前目标点
        goal_ptr = &((*goals_array)[current_goal_index_]);
    }
    else{
            // 红方
        if(robot_status_BB_->robot_id == 7) {
            goal_ptr = &red_point_goal_;
        }
        // 蓝方
        else if(robot_status_BB_->robot_id == 107){
            goal_ptr = &blue_point_goal_;
        }
        else{ // ID错误
            cout << RED << "NODE AttackDPoint GET INFO FAILED" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    cout << GREEN << "-----AttackDPoint onRunning!----" << RESET << endl;
    if(debug_flag_){   
        if(robot_status_BB_->robot_id == 7){
            cout << CYAN << " RED goal point .x:   "<< goal_ptr->pose.position.x<<RESET<<endl;
            cout << CYAN << " RED goal point .y:   "<< goal_ptr->pose.position.y<<RESET<<endl;
        }
        else{
            cout << CYAN << " BLUE goal point .x:   "<< goal_ptr->pose.position.x<<RESET<<endl;
            cout << CYAN << " BLUE goal point .y:   "<< goal_ptr->pose.position.y<<RESET<<endl;
        }
        cout << CYAN << "current point .x   "<<transform_BB_->transform.translation.x<<RESET<<endl;
        cout << CYAN << "current point .y   "<<transform_BB_->transform.translation.y<<RESET<<endl;
    }

    dist_ = sqrt(pow(transform_BB_->transform.translation.x - goal_ptr->pose.position.x,2) 
    +pow(transform_BB_->transform.translation.y - goal_ptr->pose.position.y,2));


    if(GOTO_HIGHLAND){
        if(current_goal_index_ == 0){
            modecmd_BB_->should_gimbal_patrol = 1;
            modecmd_BB_->should_chassis_spin  = 1;
        }
        else{
            modecmd_BB_->should_gimbal_patrol = 1;
            modecmd_BB_->should_chassis_spin  = 1;
        }
        modecmd_BB_->use_capacity = 0;
        modecmd_BB_->buy_bullet = 0;
        modecmd_BB_->rebirth = 0;
        if(!highland_get_ && dist_ <= 1.0){
            // 到达当前点，移动到下一个目标点
            current_goal_index_++;

            if(current_goal_index_ >= goals_array->size()) {
                highland_get_ = true;
                highland_get_time_ = parent_node_ptr_->get_clock()->now();
                cout << GREEN << "get highland" << RESET << endl;
                current_goal_index_ = goals_array->size() - 1; // 保持在最后一个点
            }
        }

        if(highland_get_ && (parent_node_ptr_->get_clock()->now() - highland_get_time_).seconds() >= 1.0)
        {
            cout << GREEN << "---AttackDPoint finished---" << RESET << endl;
            return BT::NodeStatus::FAILURE;
        }
    }
    else{
        modecmd_BB_->should_chassis_spin = 0;
        modecmd_BB_->should_gimbal_patrol = 0;
        modecmd_BB_->use_capacity = 0;
        modecmd_BB_->buy_bullet = 0;
        modecmd_BB_->rebirth = 0;
        // 前哨未掉且已就位，则可以小陀螺
        if(game_status_BB_->enemy_outpost_hp > 0){
            if(dist_ <= 0.5){
                toilet_get_ = true;
                modecmd_BB_->should_chassis_spin = 1;
                modecmd_BB_->should_gimbal_patrol = 1;
            }
        }
        // 否则不陀螺,且离开吊射点位
        else{
            if(robot_status_BB_->robot_id == 7) // 红方
            {
                goal_ptr = &red_out_point_goal_;
            }
            else // 蓝方
            {
                goal_ptr = &blue_out_point_goal_;
            }
            // 判断是否到到达toiletout位
            auto dist = sqrt(pow(transform_BB_->transform.translation.x - goal_ptr->pose.position.x,2)
            +pow(transform_BB_->transform.translation.y - goal_ptr->pose.position.y,2));
            if(dist <= 0.3){
                toilet_out_ = true;
            }

        }
        setOutput("modecmd",modecmd_BB_);

        // 判断退出条件
        if(game_status_BB_->enemy_outpost_hp == 0 && toilet_get_ == true){
            cout<< GREEN <<"---finish attack D point(cover hero)---"<<RESET<<endl;
            return BT::NodeStatus::FAILURE;
        }
    }
    // 发布目标点
    {
        auto pub_time_diff = parent_node_ptr_->get_clock()->now() - last_publish_time_;

        if(pub_time_diff.seconds() >= 1.0)
        {
            last_publish_time_ = parent_node_ptr_->get_clock()->now();
            goal_ptr->header.stamp = last_publish_time_;
            goal_publisher_->publish(*goal_ptr);
        }

        if(debug_flag_)
        {
            if(GOTO_HIGHLAND){
                cout << CYAN << "GOING TO HIGHLAND" << RESET << endl;
            }
            else{
                cout << CYAN << "GOING TO TOILET" << RESET << endl;
            }
        }
    }
    return BT::NodeStatus::RUNNING;
}
void AttackDPoint::onHalted()
{
    toilet_get_ = false;
    highland_get_ = false;
}


}

#endif
