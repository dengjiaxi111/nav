#ifndef __ENEMYINFO__
#define __ENEMYINFO__
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/enemy_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace myBT
{
    struct EnemyInfo
    {
        int _id;
        int _current_hp;
        int _last_hp;
        bool _visible;
        bool _invincible;
        bool _shootable; 
        bool _instant_revive; // 是否买活

        geometry_msgs::msg::PoseStamped _pose;
        rclcpp::Clock::SharedPtr _clock;
        rclcpp::Time _last_revive_time;

        EnemyInfo(int id,rclcpp::Clock::SharedPtr clock) : _id(id), _current_hp(150), _last_hp(150), _visible(false), _invincible(false), _shootable(false), _instant_revive(false)
        {
            _clock = clock;
            _last_revive_time = clock->now();
        }

        void updateHP(int hp)
        {
            _last_hp = _current_hp;
            _current_hp = hp;
            if(_current_hp > 0 && _last_hp == 0)
            {
                // 刚刚复活
                _last_revive_time = _clock->now();
                if(_current_hp >= 100) // 认定为买活
                {
                    _instant_revive = true;
                }
                else
                {
                    _instant_revive = false;
                }
            }

            int wait_time;
            if(_instant_revive){
                wait_time = 2;
            }
            else if(this->_id == 7) { // 特殊处理哨兵
                wait_time = 60;
            }
            else{
                wait_time = 8;
            }

            if(_current_hp == 0)
            {
                _invincible = true;
            }
            else 
            {
                if((_clock->now() - _last_revive_time).seconds() > wait_time){
                    _invincible = false;
                }
                if(this->_id == 7 && _last_hp > 0 && _current_hp > _last_hp) 
                {
                    _invincible = false; 
                }
            }
            
        }

        void updatePose(const robots_msgs::msg::EnemyPose enemy_pose)
        {
            if(enemy_pose.enemy_num == _id){
                _pose.header.stamp = _clock->now();
                _pose.header.frame_id = "base_link";
                _pose.pose.position.x = enemy_pose.enemy_x;
                _pose.pose.position.y = enemy_pose.enemy_y;
                _pose.pose.position.z = 0.0;
                _pose.pose.orientation.x = 0.0;
                _pose.pose.orientation.y = 0.0;
                _pose.pose.orientation.z = 0.0;
                _pose.pose.orientation.w = 1.0;
                _visible = true;
            }
            else if((_clock->now() - _pose.header.stamp).seconds() > 1.0)
            {
                _visible = false;
            }
        }

        // 判断点是否在多边形内
        bool isPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
            bool inside = false;
            int j = polygon.size() - 1;
            
            for (int i = 0; i < polygon.size(); i++) {
                if (((polygon[i].second > y) != (polygon[j].second > y)) &&
                    (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                        (polygon[j].second - polygon[i].second) + polygon[i].first)) {
                    inside = !inside;
                }
                j = i;
            }
            return inside;
        }


        void updateShootable(nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
                             const std::vector<std::pair<double, double>>& red_supply_zone,
                             const std::vector<std::pair<double, double>>& blue_supply_zone)
        {
            // TODO: TEST!!!!!!!!!
            _shootable = true;
            return;
            if (_invincible) {
                return;
            }

            // 检查是否在可击打区域内
            bool in_red_supply = isPointInPolygon(_pose.pose.position.x, 
                _pose.pose.position.y, 
                red_supply_zone);
            bool in_blue_supply = isPointInPolygon(_pose.pose.position.x, 
                 _pose.pose.position.y, 
                 blue_supply_zone);

            bool in_supply_area = in_red_supply || in_blue_supply;

            if(costmap->data.empty())
            {
                _shootable = true;
                return;
            }

            if(_visible && in_supply_area)
            {
                _shootable = false;
            }
        }

    };
}

#endif