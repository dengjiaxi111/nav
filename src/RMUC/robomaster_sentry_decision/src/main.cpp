#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
          last_target_publish_time_(0),
          last_published_target_x_(0.0),
          last_published_target_y_(0.0)
    {
        decision_manager_ = std::make_shared<DecisionManager>();
        auto blackboard = decision_manager_->getBlackboard();

        std::string config_path = "robomaster_sentry_decision/config/sentry_decision_params.yaml";
        if (std::ifstream(config_path).good()) {
            if (!blackboard->loadConfigFromYAML(config_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load config, shutting down");
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
        } catch (...) {
            publish_interval_ms = 100;
        }

        our_state_sub_ = this->create_subscription<decision_messages::msg::OurRobotState>(
            "/decision_messages/OurRobotState", 10,
            std::bind(&SentryDecisionNode::ourStateCallback, this, std::placeholders::_1));
        enemy_state_sub_ = this->create_subscription<decision_messages::msg::EnemyRobotState>(
            "/decision_messages/EnemyRobotState", 10,
            std::bind(&SentryDecisionNode::enemyStateCallback, this, std::placeholders::_1));
        game_state_sub_ = this->create_subscription<decision_messages::msg::GameState>(
            "/decision_messages/GameState", 10,
            std::bind(&SentryDecisionNode::gameStateCallback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/sentry/target_position", 10);
        control_pub_ = this->create_publisher<sentry_decision::msg::SentryControl>("/sentry/control", 10);

        // 单一主循环，发布周期100ms
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms),
                                         std::bind(&SentryDecisionNode::decisionLoop, this));
    }

private:
    void ourStateCallback(const decision_messages::msg::OurRobotState::SharedPtr msg) {
        decision_manager_->updateOurState(msg);
    }
    void enemyStateCallback(const decision_messages::msg::EnemyRobotState::SharedPtr msg) {
        decision_manager_->updateEnemyState(msg);
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
            blackboard->updatePositionFromTF(t.transform.translation.x, t.transform.translation.y);
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
                auto blackboard = decision_manager_->getBlackboard();
                
                // 处理控制指令（保持100ms发布）
                const bool should_stream_outpost_yaw =
                    output.control_msg.gimbal_mode == GIMBAL_OUTPOST &&
                    output.control_msg.spin_mode == SPIN_OFF;
                if (output.control_needs_publishing || should_stream_outpost_yaw) {
                    publishControl(output.control_msg);
                    blackboard->setControlPublished(true);
                }

                // 处理目标点发布（差异化频率）
                if (output.target_needs_publishing && (output.target_position.x != 0 || output.target_position.y != 0)) {
                    geometry_msgs::msg::Point new_target = output.target_position;
                    bool same_point = (std::abs(new_target.x - last_published_target_x_) < 0.01 &&
                                       std::abs(new_target.y - last_published_target_y_) < 0.01);
                    rclcpp::Time now = this->now();

                    // 不同点立即发布（当前100ms周期直接发）；相同点需间隔5秒
                    if (!same_point || 
                        (now.seconds() - last_target_publish_time_ > 5.0)) {
                        publishTarget(new_target);
                        last_published_target_x_ = new_target.x;
                        last_published_target_y_ = new_target.y;
                        last_target_publish_time_ = now.seconds();
                        blackboard->setTargetPublished(true);
                    }
                }
            } else {
                publishStopControl();
            }
        } catch (const std::exception& e) {
            // silent
        }
    }

    void publishTarget(const geometry_msgs::msg::Point& target) {
        auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = now();
        msg->header.frame_id = "map";
        msg->point.x = target.x / 100.0;
        msg->point.y = target.y / 100.0;
        target_pub_->publish(*msg);
        std::cout << "[TARGET] (" << msg->point.x << ", " << msg->point.y << ")" << std::endl;
    }

    void publishControl(const sentry_decision::msg::SentryControl& ctrl) {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>(ctrl);
        fillOutpostYawIfNeeded(*msg);
        control_pub_->publish(*msg);
        const char* gimbal_str[] = {"不动","打人","打前哨站","打基地"};
        const char* spin_str[] = {"不转","转动"};
        const char* posture_str[] = {"","进攻姿态","防御姿态","移动姿态"};
        std::cout << "[CONTROL] " << gimbal_str[ctrl.gimbal_mode] << ", "
                  << spin_str[ctrl.spin_mode] << ", "
                  << posture_str[ctrl.posture] << std::endl;
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

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    bool computeOutpostYawDeg(double& yaw_deg) {
        auto blackboard = decision_manager_->getBlackboard();
        const auto outpost = blackboard->getEnemyOutpostPoint();
        if (outpost.x == 0.0 && outpost.y == 0.0) return false;
        try {
            auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            const double base_x = t.transform.translation.x;
            const double base_y = t.transform.translation.y;
            const double target_x = outpost.x / 100.0;
            const double target_y = outpost.y / 100.0;
            const double target_yaw_map = std::atan2(target_y - base_y, target_x - base_x);
            const double base_yaw_map = yawFromQuaternion(t.transform.rotation);
            yaw_deg = normalizeAngle(target_yaw_map - base_yaw_map) * 180.0 / M_PI;
            return true;
        } catch (const tf2::TransformException& ex) {
            return false;
        }
    }

    void fillOutpostYawIfNeeded(sentry_decision::msg::SentryControl& msg) {
        msg.target_yaw_valid = false;
        msg.target_yaw_deg = 0.0;
        if (msg.gimbal_mode != GIMBAL_OUTPOST || msg.spin_mode != SPIN_OFF) return;
        double yaw_deg = 0.0;
        if (computeOutpostYawDeg(yaw_deg)) {
            msg.target_yaw_deg = yaw_deg;
            msg.target_yaw_valid = true;
        }
    }

    rclcpp::Subscription<decision_messages::msg::OurRobotState>::SharedPtr our_state_sub_;
    rclcpp::Subscription<decision_messages::msg::EnemyRobotState>::SharedPtr enemy_state_sub_;
    rclcpp::Subscription<decision_messages::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sentry_decision::msg::SentryControl>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 目标点发布频率控制
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
