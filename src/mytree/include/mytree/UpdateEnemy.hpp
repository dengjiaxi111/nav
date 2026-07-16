/*
    这个节点负责更新场上的敌人信息，用于发布敌人位置以及控制是否击打
*/
#ifndef __UPDATEENEMY__
#define __UPDATEENEMY__
#include <iostream>
#include <string>
#include <bitset>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

#include "std_msgs/msg/u_int8.hpp"
#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/enemy_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "common/enemy_info.hpp" 

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
    class UpdateEnemy: public BT::SyncActionNode
    {
    private:

        static constexpr uint8_t ENEMY_OUTPOST = 0x01;  // bit0: 敌方前哨站
        static constexpr uint8_t ENEMY_1 = 0x02;        // bit1: 敌方1号
        static constexpr uint8_t ENEMY_2 = 0x04;        // bit2: 敌方2号
        static constexpr uint8_t ENEMY_3 = 0x08;        // bit3: 敌方3号
        static constexpr uint8_t ENEMY_4 = 0x10;        // bit4: 敌方4号
        static constexpr uint8_t ENEMY_5 = 0x20;        // bit5: 敌方5号(保留)
        static constexpr uint8_t ENEMY_6 = 0x40;        // bit6: 敌方6号(保留)
        static constexpr uint8_t ENEMY_7 = 0x80;        // bit7: 敌方7号
        // debug模式
        bool debug_flag_;

        shared_ptr<rclcpp::Node> parent_node_ptr_;
        string yaml_path_;
        string package_path_;

        uint8_t priority_;
        robots_msgs::msg::EnemyPose enemy_pose_;
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr priority_pub_;
        rclcpp::Subscription<robots_msgs::msg::EnemyPose>::SharedPtr enemy_pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // 从黑板获取的数据
        shared_ptr<robots_msgs::msg::GameStatus> game_status_BB_;
        shared_ptr<robots_msgs::msg::RobotStatus> robot_status_BB_;

        // 发布在黑板上的数据
        vector<EnemyInfo> enemy_info_;

        // 补给区域数据结构
        std::vector<std::pair<double, double>> red_supply_zone_;
        std::vector<std::pair<double, double>> blue_supply_zone_;

        // 从yaml加载多边形顶点
        void loadPolygonFromParams(const std::string& prefix, std::vector<std::pair<double, double>>& polygon) {
            std::vector<double> x_coords, y_coords;
            
            parent_node_ptr_->declare_parameter<std::vector<double>>(prefix + ".x");
            parent_node_ptr_->declare_parameter<std::vector<double>>(prefix + ".y");
            
            x_coords = parent_node_ptr_->get_parameter(prefix + ".x").as_double_array();
            y_coords = parent_node_ptr_->get_parameter(prefix + ".y").as_double_array();

            polygon.clear();
            for (size_t i = 0; i < x_coords.size(); ++i) {
                polygon.push_back({x_coords[i], y_coords[i]});
            }
        }

    public:
    UpdateEnemy(const string &name, const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) 
        : BT::SyncActionNode(name, config),
        parent_node_ptr_(parent_node),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(parent_node->get_clock())),
        tf_listener_(*tf_buffer_) 
        {
            parent_node_ptr_ = parent_node;
            package_path_ = ament_index_cpp::get_package_share_directory("mytree");
            costmap_ = make_shared<nav_msgs::msg::OccupancyGrid>();
            // 参数初始化
            priority_ = 0b11111110; // 默认全部地面兵种
            /*
            bit0: 敌方前哨站
            bit1: 敌方1号
            bit2: 敌方2号
            bit3: 敌方3号
            bit4: 敌方4号
            bit5: 敌方5号(保留)
            bit6: 敌方6号(保留)
            bit7: 敌方7号
            */

            enemy_info_.push_back(EnemyInfo(1, parent_node_ptr_->get_clock()));
            enemy_info_.push_back(EnemyInfo(2, parent_node_ptr_->get_clock()));
            enemy_info_.push_back(EnemyInfo(3, parent_node_ptr_->get_clock()));
            enemy_info_.push_back(EnemyInfo(4, parent_node_ptr_->get_clock()));
            enemy_info_.push_back(EnemyInfo(7, parent_node_ptr_->get_clock()));

            parent_node_ptr_->declare_parameter<bool>("UpdateEnemy_debug", true);
            parent_node_ptr_->get_parameter("UpdateEnemy_debug", debug_flag_);
            priority_pub_ = parent_node_ptr_->create_publisher<std_msgs::msg::UInt8>("/enemy_priority", 10);
            costmap_sub_ = parent_node_ptr_->create_subscription<nav_msgs::msg::OccupancyGrid>( "/global_costmap/costmap",10,std::bind(&UpdateEnemy::costmapCallback, this, std::placeholders::_1));
            enemy_pose_sub_ = parent_node_ptr_->create_subscription<robots_msgs::msg::EnemyPose>("/EnemyPose",10,bind(&UpdateEnemy::enemyposeCallback,this,placeholders::_1));
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(parent_node_ptr_);
            // 加载补给区域
            loadPolygonFromParams("red_supply_zone_area", red_supply_zone_);
            loadPolygonFromParams("blue_supply_zone_area", blue_supply_zone_);
        }

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports_list;
            ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
            ports_list.insert(BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
            ports_list.insert(BT::OutputPort<vector<EnemyInfo>>("enemypose"));
            return ports_list;
        }

        BT::NodeStatus tick() override
        {
            cout << GREEN << "---UpdateEnemy tick---" << RESET << endl;

            getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus", game_status_BB_);
            getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus", robot_status_BB_);

            if((game_status_BB_ == nullptr) || (robot_status_BB_ == nullptr))
            {
                cout << RED << "NODE UPDATEENEMY GET INFO FAILED" << RESET << endl;
                return BT::NodeStatus::FAILURE;
            }

            // 更新敌人血量
            enemy_info_[0].updateHP(game_status_BB_->enemy_1_robot_hp);
            enemy_info_[1].updateHP(game_status_BB_->enemy_2_robot_hp);
            enemy_info_[2].updateHP(game_status_BB_->enemy_3_robot_hp);
            enemy_info_[3].updateHP(game_status_BB_->enemy_4_robot_hp);
            enemy_info_[4].updateHP(game_status_BB_->enemy_7_robot_hp);

            // 更新敌人位姿以及是否可击打
            for(auto &enemy : enemy_info_)
            {
                enemy.updateShootable(costmap_,red_supply_zone_, blue_supply_zone_);
            }

            // 设置优先级
            {
                priority_ = 0;  
                priority_ |= ENEMY_5;
                priority_ |= ENEMY_6;
                if (enemy_info_[0]._shootable) priority_ |= ENEMY_1;
                if (enemy_info_[1]._shootable) priority_ |= ENEMY_2;
                if (enemy_info_[2]._shootable) priority_ |= ENEMY_3;
                if (enemy_info_[3]._shootable) priority_ |= ENEMY_4;
                if (enemy_info_[4]._shootable) priority_ |= ENEMY_7;
            }

            if (debug_flag_) {
                cout << BLUE << "Priority bits: " << std::bitset<8>(priority_) << RESET << endl;
            }

            auto msg = std_msgs::msg::UInt8();
            msg.data = priority_;
            priority_pub_->publish(msg);
            setOutput("enemypose", enemy_info_);

            return BT::NodeStatus::FAILURE;
        }

        void enemyposeCallback(const robots_msgs::msg::EnemyPose::SharedPtr msg)
        {
            this->enemy_pose_ = *msg;
            // 创建 base_link 坐标系下的 PoseStamped
            geometry_msgs::msg::PoseStamped pose_in_base;
            pose_in_base.header.frame_id = "base_link";
            pose_in_base.header.stamp = parent_node_ptr_->get_clock()->now();
            pose_in_base.pose.position.x = msg->enemy_x;
            pose_in_base.pose.position.y = msg->enemy_y;
            pose_in_base.pose.position.z = 0.0;
            pose_in_base.pose.orientation.w = 1.0; // 单位四元数
            geometry_msgs::msg::PoseStamped pose_in_map;
            try
            {
                // 等待变换并转换到 map 坐标系
                pose_in_map = tf_buffer_->transform(pose_in_base, "map", tf2::durationFromSec(0.1));

                // 更新 enemy info
                robots_msgs::msg::EnemyPose pose_in_map_custom;
                pose_in_map_custom.enemy_num = msg->enemy_num;
                pose_in_map_custom.enemy_x = pose_in_map.pose.position.x;
                pose_in_map_custom.enemy_y = pose_in_map.pose.position.y;
                for (auto &enemy : enemy_info_)
                {
                    enemy.updatePose(pose_in_map_custom);
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(parent_node_ptr_->get_logger(), "Transform failed: %s", ex.what());
            }

            geometry_msgs::msg::TransformStamped enemy_tf;
            enemy_tf.header.stamp = parent_node_ptr_->get_clock()->now();
            enemy_tf.header.frame_id = "map";      
            enemy_tf.child_frame_id = "enemy";      
            enemy_tf.transform.translation.x = pose_in_map.pose.position.x;
            enemy_tf.transform.translation.y = pose_in_map.pose.position.y;
            enemy_tf.transform.translation.z = 0.5;
            enemy_tf.transform.rotation.x = 0.0;
            enemy_tf.transform.rotation.y = 0.0;
            enemy_tf.transform.rotation.z = 0.0;
            enemy_tf.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(enemy_tf);
        }

        void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            *(this->costmap_) = *msg;
        }

    };
}

#endif