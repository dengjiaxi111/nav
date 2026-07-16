#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/EnemyStateFusion.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using namespace GameConstants;

class SentryDecisionNode : public rclcpp::Node {
public:
    SentryDecisionNode()
        : Node("sentry_decision"),
          game_started_(false),
          tf_buffer_(this->get_clock()),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)),
          enemy_state_fusion_(tf_buffer_),
          target_publish_interval_ms_(2000),
          target_arrived_publish_interval_ms_(5000),
          last_target_publish_time_(0),
          last_published_target_x_(0.0),
          last_published_target_y_(0.0)
    {
        decision_manager_ = std::make_shared<DecisionManager>();
        auto blackboard = decision_manager_->getBlackboard();

        std::string config_path =
            ament_index_cpp::get_package_share_directory("sentry_decision") +
            "/config/sentry_decision_params.yaml";
        if (std::ifstream(config_path).good()) {
            if (!blackboard->loadConfigFromYAML(config_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load config");
                rclcpp::shutdown();
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Config file not found: %s", config_path.c_str());
            rclcpp::shutdown();
            return;
        }

        int publish_interval_ms = 100;
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["publish_interval_ms"])
                publish_interval_ms = config["publish_interval_ms"].as<int>();
            if (config["target_publish_interval_ms"])
                target_publish_interval_ms_ = config["target_publish_interval_ms"].as<int>();
            if (config["target_arrived_publish_interval_ms"])
                target_arrived_publish_interval_ms_ = config["target_arrived_publish_interval_ms"].as<int>();
        } catch (...) {}

        if (target_publish_interval_ms_ <= 0) target_publish_interval_ms_ = 2000;
        if (target_arrived_publish_interval_ms_ <= 0) target_arrived_publish_interval_ms_ = 5000;

        enemy_state_sub_ = this->create_subscription<decision_messages::msg::EnemyRobotState>(
            "/decision_messages/EnemyRobotState", 10,
            std::bind(&SentryDecisionNode::enemyStateCallback, this, std::placeholders::_1));

        our_state_sub_ = this->create_subscription<decision_messages::msg::OurRobotState>(
            "/decision_messages/OurRobotState", 10,
            std::bind(&SentryDecisionNode::ourStateCallback, this, std::placeholders::_1));

        game_state_sub_ = this->create_subscription<decision_messages::msg::GameState>(
            "/decision_messages/GameState", 10,
            std::bind(&SentryDecisionNode::gameStateCallback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/sentry/target_position", 10);
        control_pub_ = this->create_publisher<sentry_decision::msg::SentryControl>("/sentry/control", 10);

        corrected_enemy_pub_ = this->create_publisher<decision_messages::msg::EnemyRobotState>(
            "/decision_messages/EnemyRobotState_fused", 10);

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms),
                                         std::bind(&SentryDecisionNode::decisionLoop, this));
    }

private:
    void ourStateCallback(const decision_messages::msg::OurRobotState::SharedPtr msg) {
        decision_manager_->updateOurState(msg);
    }

    void enemyStateCallback(const decision_messages::msg::EnemyRobotState::SharedPtr msg) {
        auto corrected_msg = std::make_shared<decision_messages::msg::EnemyRobotState>(*msg);
        enemy_state_fusion_.applyVisualLockCorrection(*msg, now(), *corrected_msg);

        decision_manager_->updateEnemyState(corrected_msg);
        corrected_enemy_pub_->publish(*corrected_msg);
    }

    void gameStateCallback(const decision_messages::msg::GameState::SharedPtr msg) {
        decision_manager_->updateGameState(msg);
        if (msg->stage == STAGE_BATTLE && !game_started_) {
            game_started_ = true;
        } else if (msg->stage != STAGE_BATTLE && game_started_) {
            game_started_ = false;
        }
    }

    bool updatePositionFromTF() {
        try {
            auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            auto blackboard = decision_manager_->getBlackboard();
            double x_m = t.transform.translation.x;
            double y_m = t.transform.translation.y;
            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            blackboard->updatePositionFromTF(x_m, y_m, yaw);
            return true;
        } catch (const tf2::TransformException& ex) {
            return false;
        }
    }

    void decisionLoop() {
        try {
            updatePositionFromTF();
            DecisionOutput output = decision_manager_->executeDecision();

            if (game_started_) {
                publishDecisionTargetIfNeeded(output);
                publishControl(output.control_msg);
            } else {
                publishStopControl();
            }
        } catch (const std::exception& e) {
            // silent
        }
    }

    bool isValidTarget(const geometry_msgs::msg::Point& target) const {
        return target.x != 0.0 || target.y != 0.0;
    }

    void publishDecisionTargetIfNeeded(const DecisionOutput& output) {
        auto blackboard = decision_manager_->getBlackboard();
        geometry_msgs::msg::Point target;
        bool has_target = false;
        bool target_reached = false;

        if (output.target_needs_publishing && isValidTarget(output.target_position)) {
            target = output.target_position;
            has_target = true;
        } else if (blackboard->at_current_target && isValidTarget(blackboard->current_behavior.target)) {
            target = blackboard->current_behavior.target;
            has_target = true;
            target_reached = true;
        }

        if (!has_target) return;

        const bool same_point =
            std::abs(target.x - last_published_target_x_) < 0.01 &&
            std::abs(target.y - last_published_target_y_) < 0.01;
        const double interval_sec =
            (target_reached ? target_arrived_publish_interval_ms_ : target_publish_interval_ms_) / 1000.0;
        rclcpp::Time now = this->now();

        if (!same_point || (now.seconds() - last_target_publish_time_ > interval_sec)) {
            publishTarget(target);
            publishNav2Goal(target, output.control_msg.target_yaw_deg * M_PI / 180.0);
            last_published_target_x_ = target.x;
            last_published_target_y_ = target.y;
            last_target_publish_time_ = now.seconds();
        }
    }

    void publishTarget(const geometry_msgs::msg::Point& target) {
        auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = now();
        msg->header.frame_id = "map";
        msg->point.x = target.x / 100.0;
        msg->point.y = target.y / 100.0;
        target_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "[TARGET] (%.3f, %.3f)", msg->point.x, msg->point.y);
    }

    void publishNav2Goal(const geometry_msgs::msg::Point& target, double yaw_rad) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = now();
        goal.header.frame_id = "map";
        goal.pose.position.x = target.x / 100.0;
        goal.pose.position.y = target.y / 100.0;
        goal.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();

        goal_pub_->publish(goal);
    }

    void publishControl(const sentry_decision::msg::SentryControl& ctrl) {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>(ctrl);
        clearTargetYaw(*msg);
        control_pub_->publish(*msg);
        const char* gimbal_str[] = {"不动","打人","打前哨站"};
        const char* spin_str[] = {"不转","高速转","低速转"};
        const char* posture_str[] = {"","进攻姿态","防御姿态","移动姿态"};
        const char* gimbal_text = ctrl.gimbal_mode < 3 ? gimbal_str[ctrl.gimbal_mode] : "未知云台";
        const char* spin_text = ctrl.spin_mode < 3 ? spin_str[ctrl.spin_mode] : "未知小陀螺";
        const char* posture_text = ctrl.posture < 4 ? posture_str[ctrl.posture] : "未知姿态";
        RCLCPP_INFO(this->get_logger(), "[CONTROL] %s, %s, %s",
                    gimbal_text, spin_text, posture_text);
    }

    void publishStopControl() {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>();
        msg->gimbal_mode = GIMBAL_IDLE;
        msg->spin_mode = SPIN_OFF;
        msg->posture = POSTURE_MOVE;
        msg->target_yaw_deg = 0.0;
        msg->target_yaw_valid = false;
        control_pub_->publish(*msg);
    }

    void clearTargetYaw(sentry_decision::msg::SentryControl& msg) {
        msg.target_yaw_valid = false;
        msg.target_yaw_deg = 0.0;
    }

    rclcpp::Subscription<decision_messages::msg::OurRobotState>::SharedPtr our_state_sub_;
    rclcpp::Subscription<decision_messages::msg::EnemyRobotState>::SharedPtr enemy_state_sub_;
    rclcpp::Subscription<decision_messages::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sentry_decision::msg::SentryControl>::SharedPtr control_pub_;
    rclcpp::Publisher<decision_messages::msg::EnemyRobotState>::SharedPtr corrected_enemy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    EnemyStateFusion enemy_state_fusion_;

    int target_publish_interval_ms_;
    int target_arrived_publish_interval_ms_;
    double last_target_publish_time_;
    double last_published_target_x_;
    double last_published_target_y_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
