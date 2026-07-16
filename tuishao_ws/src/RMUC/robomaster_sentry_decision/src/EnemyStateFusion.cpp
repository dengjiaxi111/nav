#include "sentry_decision/EnemyStateFusion.hpp"
#include <cmath>
#include <tf2/exceptions.h>

EnemyStateFusion::EnemyStateFusion(tf2_ros::Buffer& tf_buffer)
    : tf_buffer_(tf_buffer)
{}

void EnemyStateFusion::applyVisualLockCorrection(
    const decision_messages::msg::EnemyRobotState& input,
    const rclcpp::Time& stamp,
    decision_messages::msg::EnemyRobotState& output) const
{
    if (input.enemy_id <= 0 || input.enemy_x == 0.0 || input.enemy_y == 0.0) return;

    geometry_msgs::msg::PointStamped enemy_in_base;
    enemy_in_base.header.stamp = stamp;
    enemy_in_base.header.frame_id = "base_link";
    enemy_in_base.point.x = input.enemy_x;
    enemy_in_base.point.y = input.enemy_y;
    enemy_in_base.point.z = 0.0;

    geometry_msgs::msg::PointStamped enemy_in_map;
    if (!transformPointToMap(enemy_in_base, enemy_in_map)) return;

    const double x_cm = enemy_in_map.point.x * 100.0;
    const double y_cm = enemy_in_map.point.y * 100.0;

    switch (input.enemy_id) {
        case 1:
            output.enemy_hero_x = x_cm;
            output.enemy_hero_y = y_cm;
            break;
        case 2:
            output.enemy_engineer_x = x_cm;
            output.enemy_engineer_y = y_cm;
            break;
        case 3:
            output.enemy_infantry3_x = x_cm;
            output.enemy_infantry3_y = y_cm;
            break;
        case 4:
            output.enemy_infantry4_x = x_cm;
            output.enemy_infantry4_y = y_cm;
            break;
        case 7:
            output.enemy_sentry_x = x_cm;
            output.enemy_sentry_y = y_cm;
            break;
        default:
            break;
    }
}

double EnemyStateFusion::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool EnemyStateFusion::transformPointToMap(const geometry_msgs::msg::PointStamped& point_in,
                                           geometry_msgs::msg::PointStamped& point_out) const {
    try {
        const auto transform = tf_buffer_.lookupTransform("map", point_in.header.frame_id, tf2::TimePointZero);
        const double base_x = transform.transform.translation.x;
        const double base_y = transform.transform.translation.y;
        const double base_yaw = yawFromQuaternion(transform.transform.rotation);

        const double cos_yaw = std::cos(base_yaw);
        const double sin_yaw = std::sin(base_yaw);

        point_out.header.stamp = point_in.header.stamp;
        point_out.header.frame_id = "map";
        point_out.point.x = base_x + cos_yaw * point_in.point.x - sin_yaw * point_in.point.y;
        point_out.point.y = base_y + sin_yaw * point_in.point.x + cos_yaw * point_in.point.y;
        point_out.point.z = transform.transform.translation.z + point_in.point.z;
        return true;
    } catch (const tf2::TransformException& ex) {
        return false;
    }
}
