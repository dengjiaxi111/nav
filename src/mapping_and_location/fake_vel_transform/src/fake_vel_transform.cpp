#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string LOCAL_PLAN_TOPIC  = "/local_plan";
const std::string ODOM_TOPIC        = "/Odometry";

const int TF_PUBLISH_FREQUENCY = 50;  // base_link to base_link_fake. Frequency in Hz.
const int CMD_VEL_FREQUENCY = 50;

// 将角度归一化到 [-pi, pi]
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
    RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

    this->declare_parameter<float>("spin_speed", 0.0);
    this->declare_parameter<bool>("use_fake_vel",true);
    this->declare_parameter<double>("alpha", 0.95); 
    this->declare_parameter<bool>("fake_vel_on", true);
    this->declare_parameter<double>("fake_angular_speed_coefficient", 1.2); // 用于调整角速度的系数
    this->get_parameter("spin_speed", spin_speed_);
    this->get_parameter("use_fake_vel", use_fake_vel_);

    this->get_parameter("alpha", alpha_);
    this->get_parameter("fake_vel_on", fake_vel_on_);
    this->get_parameter("fake_angular_speed_coefficient", fake_angular_speed_coefficient_);
    
    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Create Publisher and Subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
    
    cmd_vel_gimbal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_gimbal", rclcpp::QoS(rclcpp::KeepLast(1)));
    
    odom_EC_msg_sub_ = this->create_subscription<robots_msgs::msg::ChassisOdom>(
        "/ChassisOdom", 10, std::bind(&FakeVelTransform::odomECCallback, this, std::placeholders::_1));
    odom_EC_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/Odometry/EC", rclcpp::QoS(rclcpp::KeepLast(1)));

    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
        std::bind(&FakeVelTransform::publishTransform, this));

    fake_base_link_angle_ = 0.785398; // 初始角度为45度
    last_fake_angle_diff_ = 0;
    fake_angle_diff_filtered_ = 0;
    last_base_link_fake_angle_ = 0;
}


//  更新并滤波角度，将调整后的速度指令发布到 /cmd_vel_gimbal 话题
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx);

    geometry_msgs::msg::TransformStamped transform_stamped;
    try{
        transform_stamped = tf2_buffer_->lookupTransform("base_link", "base_link_fake",tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(),"Failed to get base_link_fake when transforming cmd_vel");
        return;
    }
    
    double fake_angle_diff = tf2::getYaw(transform_stamped.transform.rotation); 
    double delta = fake_angle_diff - last_fake_angle_diff_;

    // 低通滤波
    delta = std::atan2(std::sin(delta), std::cos(delta)); // 归一化 [-pi, pi]

    // 低通滤波
    fake_angle_diff_filtered_ = last_fake_angle_diff_ + alpha_ * delta;

    normalize_angle(fake_angle_diff_filtered_);

    // 更新 last_fake_angle_diff_
    last_fake_angle_diff_ = fake_angle_diff_filtered_;

    fake_angle_diff_filtered_ = std::atan2(std::sin(fake_angle_diff_filtered_), std::cos(fake_angle_diff_filtered_)); // 归一化 [-pi, pi]
    //std::cout << "fake_angle_diff_filtered_" << fake_angle_diff_filtered_ << std::endl;

    geometry_msgs::msg::Twist aft_tf_vel;

    aft_tf_vel.angular.z = msg->angular.z;
    aft_tf_vel.linear.x = msg->linear.x * cos(fake_angle_diff_filtered_) - msg->linear.y * sin(fake_angle_diff_filtered_);
    aft_tf_vel.linear.y = msg->linear.x * sin(fake_angle_diff_filtered_) + msg->linear.y * cos(fake_angle_diff_filtered_);

    cmd_vel_gimbal_pub_->publish(aft_tf_vel);
    fake_base_link_angle_ += msg->angular.z / CMD_VEL_FREQUENCY * fake_angular_speed_coefficient_; // 为适应purepursuit而增加的系数
    fake_base_link_angle_ = AngleLimiter(fake_base_link_angle_);

    last_fake_angle_diff_ = fake_angle_diff;
}

// 发布base_link_fake到base_link的变换
void FakeVelTransform::publishTransform()
{
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link_static";
    t.child_frame_id = "base_link_fake";
    tf2::Quaternion q;

    this->get_parameter("fake_vel_on", fake_vel_on_);

    if(use_fake_vel_){
        if(fake_vel_on_){
            q.setRPY(0, 0, fake_base_link_angle_);
        }
        else{
            t.header.frame_id = "base_link";
            q.setRPY(0, 0, 0);
        }
    }
    else{
        q.setRPY(0, 0, 0);
    }

    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

}
// 处理电控反馈的轮式里程计消息
void FakeVelTransform::odomECCallback(const robots_msgs::msg::ChassisOdom::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try{
        transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link_fake",tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
        return;
    }
    
    tf2::Quaternion q;
    tf2::fromMsg(transform_stamped.transform.rotation, q);
    double yaw = tf2::getYaw(q);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link_fake";
    odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
    odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
    odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
    odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;
    odom_msg.twist.twist.linear.x = msg->speed_x;
    odom_msg.twist.twist.linear.y = msg->speed_y;
    odom_msg.twist.twist.angular.z = (yaw - last_base_link_fake_angle_) * CMD_VEL_FREQUENCY;
    last_base_link_fake_angle_ = yaw;

    odom_EC_pub_->publish(odom_msg);
}

}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
