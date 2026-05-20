#ifndef ENEMY_STATE_FUSION_HPP
#define ENEMY_STATE_FUSION_HPP

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include "decision_messages/msg/enemy_robot_state.hpp"

class EnemyStateFusion {
public:
    explicit EnemyStateFusion(tf2_ros::Buffer& tf_buffer);

    void applyVisualLockCorrection(const decision_messages::msg::EnemyRobotState& input,
                                   const rclcpp::Time& stamp,
                                   decision_messages::msg::EnemyRobotState& output) const;

private:
    tf2_ros::Buffer& tf_buffer_;to

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    bool transformPointToMap(const geometry_msgs::msg::PointStamped& point_in,
                             geometry_msgs::msg::PointStamped& point_out) const;
};

#endif // ENEMY_STATE_FUSION_HPP
