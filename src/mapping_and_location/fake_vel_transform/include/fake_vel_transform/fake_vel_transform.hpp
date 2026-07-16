#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <mutex>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <robots_msgs/msg/chassis_odom.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::mutex mtx;

double AngleLimiter(double angle)
{
    if(angle > M_PI)
        angle -= M_PI*2;
    if(angle < -M_PI)
        angle += M_PI*2;
    return angle;
}
namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
    explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void localPlanCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomECCallback(const robots_msgs::msg::ChassisOdom::SharedPtr msg);
    void publishTransform();

    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub_;
    rclcpp::Subscription<robots_msgs::msg::ChassisOdom>::SharedPtr odom_EC_msg_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_gimbal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_EC_pub_;

    // Broadcast tf from base_link to base_link_fake
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path local_plan_;
    rclcpp::Time local_plan_time_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    geometry_msgs::msg::PoseStamped planner_local_pose_;

    double fake_angle_diff_;
    double last_fake_angle_diff_;
    double fake_base_link_angle_;
    double last_base_link_fake_angle_;
    double fake_angle_diff_filtered_;
    double real_base_link_angle_;

    double current_robot_base_angle_;

    float spin_speed_;
    bool use_fake_vel_;
    bool fake_vel_on_;
    double alpha_;
    double fake_angular_speed_coefficient_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
