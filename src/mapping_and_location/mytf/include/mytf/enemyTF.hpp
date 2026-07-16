#include "rclcpp/rclcpp.hpp"
#include "robots_msgs/msg/enemy_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <queue>

constexpr int STORAGE_LEN = 20;

struct EnemyInfo{
    uint8_t _enemy_id;
    std::queue<geometry_msgs::msg::PoseStamped> _pose_queue;    
    EnemyInfo():_enemy_id(0){}
};

class EnemyTF:public rclcpp::Node
{
public:
    EnemyTF(): Node("EnemyTF")
    {
        RCLCPP_INFO(this->get_logger(),"Node EnemyTF running!");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EnemyTF::timer_callback, this)); //0.1s发布一次
        enemy_sub_ = this->create_subscription<robots_msgs::msg::EnemyPose>("EnemyData", 100, std::bind(&EnemyTF::enemy_callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_enemy_time_ = this->get_clock()->now();
    }
    void timer_callback();

private:
    void send_tf(float x, float y);
    void enemy_callback(robots_msgs::msg::EnemyPose::SharedPtr);
    void predict_enemy_pose(float enemy_x_list[], float enemy_y_list[], int& valid_data_num);

    EnemyInfo enemy_info_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_ ;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<robots_msgs::msg::EnemyPose>::SharedPtr enemy_sub_;

    rclcpp::Time last_enemy_time_;

    std::mutex mtx_;
};


