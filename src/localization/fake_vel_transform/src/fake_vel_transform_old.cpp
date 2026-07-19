#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string CMD_VEL_TOPIC     = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL  = "/cmd_vel_gimbal";
const std::string LOCAL_PLAN_TOPIC  = "/local_plan";
const std::string ODOM_TOPIC        = "/odometry";
const float CONTROLLER_INVAVID_TIME = 0.2;

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

    // Declare and get the spin speed parameter
    this->declare_parameter<float>("spin_speed", 0.0);
    this->declare_parameter<bool>("use_fake_vel",true);
    this->get_parameter("spin_speed", spin_speed_);
    this->get_parameter("use_fake_vel", use_fake_vel_);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Create Publisher and Subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        CMD_VEL_TOPIC, 10, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

    // 使用localplan的时间戳作为cmd_vel的时间戳
    local_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(LOCAL_PLAN_TOPIC, 10,std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));
    
    cmd_vel_gimbal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
    
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
        std::bind(&FakeVelTransform::publishTransform, this));

    fake_base_link_angle_ = 0;
    fake_angle_diff_filtered_ = 0;
    local_plan_time_ = this->get_clock()->now();
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // 我们理想地认为localplan和cmd_vel是同时发布的
    local_plan_time_ = msg->header.stamp;
    local_plan_ = *msg;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try 
    {
        transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link",tf2::TimePointZero,tf2::durationFromSec(0.05));
        std::lock_guard<std::mutex> lock(mtx);
        {
            last_real_base_link_angle_ = real_base_link_angle_;
            real_base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

            fake_angle_diff_ = fake_base_link_angle_ - real_base_link_angle_;
            fake_angle_diff_ = normalize_angle(fake_angle_diff_);
            double alpha = 0.9; // 滤波系数，视需求调整
            
            // slh:很多史！！！！！
            if (fabs(fake_angle_diff_) > M_PI_2 && 
                std::signbit(fake_angle_diff_) != std::signbit(fake_angle_diff_filtered_)) 
            {
                double temp_diff = fake_angle_diff_filtered_;
                if (std::signbit(fake_angle_diff_)) {
                    fake_angle_diff_ += 2 * M_PI;
                } 
                else {
                    temp_diff += 2 * M_PI;
                }
                fake_angle_diff_filtered_ = alpha * fake_angle_diff_ + (1 - alpha) * temp_diff;
                RCLCPP_INFO(this->get_logger(), "Raw diff: %.2f Filtered: %.2f",
                    fake_angle_diff_, fake_angle_diff_filtered_);
            } 
            else 
            {
                // 正常滤波
                fake_angle_diff_filtered_ = alpha * fake_angle_diff_ + (1 - alpha) * fake_angle_diff_filtered_;
            }
            fake_angle_diff_filtered_ = normalize_angle(fake_angle_diff_filtered_);

            {
                geometry_msgs::msg::Twist aft_tf_vel;
                aft_tf_vel.angular.z = latest_cmd_vel_.angular.z;
                //RCLCPP_INFO(this->get_logger(),"-fake_angle_diff:%f ", -fake_angle_diff_filtered_);
                aft_tf_vel.linear.x = latest_cmd_vel_.linear.x * cos(fake_angle_diff_filtered_) + latest_cmd_vel_.linear.y * sin(fake_angle_diff_filtered_);
                aft_tf_vel.linear.y = latest_cmd_vel_.linear.x * sin(fake_angle_diff_filtered_) + latest_cmd_vel_.linear.y * cos(fake_angle_diff_filtered_);

                cmd_vel_gimbal_pub_->publish(aft_tf_vel);
            }
        }
    } 
    catch (tf2::TransformException & ex) 
    {
        
        RCLCPP_WARN(this->get_logger(),"FAILED TO GET TRANS FROM odom2baselink when receiving Path!");

    }


}


// Transform the velocity from base_link_fake to base_link
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    fake_base_link_angle_ += msg->angular.z / CMD_VEL_FREQUENCY;
    fake_base_link_angle_ = AngleLimiter(fake_base_link_angle_);

    if((this->get_clock()->now() - local_plan_time_).seconds() >= CONTROLLER_INVAVID_TIME){
        geometry_msgs::msg::Twist aft_tf_vel;
        aft_tf_vel.angular.z = msg->angular.z;
        aft_tf_vel.linear.x = msg->linear.x * cos(fake_angle_diff_filtered_) - msg->linear.y * sin(fake_angle_diff_filtered_);
        aft_tf_vel.linear.y = msg->linear.x * sin(fake_angle_diff_filtered_) + msg->linear.y * cos(fake_angle_diff_filtered_);

        cmd_vel_gimbal_pub_->publish(aft_tf_vel);
        RCLCPP_INFO(this->get_logger(),"out of time limit!");
    }
    else{
        latest_cmd_vel_ = *msg;
    }
}

// 发布base_link_fake到base_link的变换
//TODO: 查明导致伪车头摆动的原因
void FakeVelTransform::publishTransform()
{

    geometry_msgs::msg::TransformStamped transform_stamped;
    if((this->get_clock()->now() - local_plan_time_).seconds() >= CONTROLLER_INVAVID_TIME){
        try 
        {
            
            transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link",tf2::TimePointZero);
            last_real_base_link_angle_ = real_base_link_angle_;
            real_base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);
            fake_angle_diff_filtered_ = fake_base_link_angle_ - real_base_link_angle_;
        }
        catch (tf2::TransformException & ex) 
        {
            RCLCPP_INFO(this->get_logger(), "Could not get trans odom to base_link: %s", ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_link_fake";
    tf2::Quaternion q;

    if(use_fake_vel_){
        q.setRPY(0, 0, fake_angle_diff_filtered_);
    }
    else{
        q.setRPY(0, 0, 0);
    }

    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
