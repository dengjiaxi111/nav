#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/Constants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using namespace SentryConstants;

class SentryDecisionNode : public rclcpp::Node {
public:
    SentryDecisionNode()
    : Node("sentry_decision"),
      game_started_(false),
      target_republish_interval_sec_(0.2),
      has_last_published_target_(false),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        decision_manager_ = std::make_shared<DecisionManager>();
        auto blackboard = decision_manager_->getBlackboard();

        // 仅从 robomaster_sentry_decision/config/sentry_decision_params.yaml 加载配置
        std::string config_path = "robomaster_sentry_decision/config/sentry_decision_params.yaml";
        if (std::ifstream(config_path).good()) {
            if (!blackboard->loadConfigFromYAML(config_path)) {
                RCLCPP_WARN(this->get_logger(), "无法加载配置文件 %s，使用默认配置", config_path.c_str());
            }
            try {
                YAML::Node config = YAML::LoadFile(config_path);
                if (config["target_republish_interval_sec"]) {
                    target_republish_interval_sec_ = config["target_republish_interval_sec"].as<double>();
                }
            } catch (const YAML::Exception& e) {
                RCLCPP_WARN(this->get_logger(), "读取目标重发频率失败: %s，使用默认值 %.2f s",
                            e.what(), target_republish_interval_sec_);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "配置文件 %s 不存在，使用默认配置", config_path.c_str());
        }

        our_state_sub_ = this->create_subscription<decision_messages::msg::OurRobotState>(
            "/decision_messages/OurRobotState", 10,
            std::bind(&SentryDecisionNode::ourStateCallback, this, std::placeholders::_1));

        game_state_sub_ = this->create_subscription<decision_messages::msg::GameState>(
            "/decision_messages/GameState", 10,
            std::bind(&SentryDecisionNode::gameStateCallback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/sentry/target_position", 10);
        control_pub_ = this->create_publisher<sentry_decision::msg::SentryControl>("/sentry/control", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&SentryDecisionNode::decisionLoop, this));

        std::cout << "[SYSTEM] Sentry Decision System (TF Position) Started" << std::endl;
    }

private:
    void ourStateCallback(const decision_messages::msg::OurRobotState::SharedPtr msg) {
        decision_manager_->updateOurState(msg);
    }

    void gameStateCallback(const decision_messages::msg::GameState::SharedPtr msg) {
        decision_manager_->updateGameState(msg);
        if (msg->stage == 4 && !game_started_) {
            game_started_ = true;
            std::cout << "[SYSTEM] 比赛开始!" << std::endl;
        } else if (msg->stage != 4 && game_started_) {
            game_started_ = false;
            std::cout << "[SYSTEM] 比赛结束" << std::endl;
        }
    }

    bool updatePositionFromTF() {
        try {
            auto t = tf_buffer_.lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero);

            auto blackboard = decision_manager_->getBlackboard();
            blackboard->updatePositionFromTF(t.transform.translation.x, t.transform.translation.y);
            return true;
        } catch (const tf2::TransformException& ex) {
            std::cerr << "[TF] 查询 map->base_link 失败: " << ex.what() << std::endl;
            return false;
        }
    }

    void decisionLoop() {
        try {
            updatePositionFromTF();

            DecisionOutput output = decision_manager_->executeDecision();

            if (game_started_) {
                auto blackboard = decision_manager_->getBlackboard();

                if (output.target_needs_publishing &&
                    (output.target_position.x != 0 || output.target_position.y != 0)) {
                    publishTarget(output.target_position);
                    blackboard->setTargetPublished(true);
                }

                maybeRepublishTarget();

                if (output.control_needs_publishing) {
                    publishControl(output.control_msg);
                    blackboard->setControlPublished(true);
                }
            } else {
                clearRetainedTarget();
                publishStopControl();
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Decision loop: " << e.what() << std::endl;
        }
    }

    void publishTarget(const geometry_msgs::msg::Point& target) {
        auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = now();
        msg->header.frame_id = "map";
        msg->point.x = target.x / 100.0;
        msg->point.y = target.y / 100.0;
        target_pub_->publish(*msg);
        last_target_publish_time_ = now();
        last_published_target_ = target;
        has_last_published_target_ = true;
        std::cout << "[TARGET] (" << msg->point.x << ", " << msg->point.y << ")" << std::endl;
    }

    bool hasActiveMovementTarget() const {
        auto blackboard = decision_manager_->getBlackboard();
        const auto& behavior = blackboard->current_behavior;

        if (blackboard->at_current_target) {
            return false;
        }

        if (behavior.state != BehaviorState::MOVING) {
            return false;
        }

        return behavior.type == BehaviorType::MOVE_TO_ATTACK ||
               behavior.type == BehaviorType::MOVE_TO_SUPPLY;
    }

    void maybeRepublishTarget() {
        auto blackboard = decision_manager_->getBlackboard();

        if (!hasActiveMovementTarget()) {
            clearRetainedTarget();
            return;
        }

        const auto& target = blackboard->current_behavior.target;
        const bool target_changed = !has_last_published_target_ ||
                                    target.x != last_published_target_.x ||
                                    target.y != last_published_target_.y;

        if (target_changed ||
            (now() - last_target_publish_time_).seconds() >= target_republish_interval_sec_) {
            publishTarget(target);
        }
    }

    void clearRetainedTarget() {
        has_last_published_target_ = false;
    }

    void publishControl(const sentry_decision::msg::SentryControl& ctrl) {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>(ctrl);
        control_pub_->publish(*msg);

        auto blackboard = decision_manager_->getBlackboard();

        std::string gimbal = (ctrl.gimbal_mode == 1) ? "打人" : "不动";
        std::string spin;
        switch (ctrl.spin_mode) {
            case 0: spin = "不动"; break;
            case 1: spin = "旋转"; break;
            default: spin = "未知";
        }

        std::cout << "[CONTROL] " << gimbal << ", " << spin << std::endl;
        std::cout << "[DEBUG] 比赛阶段: " << (int)blackboard->stage 
                  << ", 云台模式: " << (int)ctrl.gimbal_mode 
                  << ", 底盘模式: " << (int)ctrl.spin_mode << std::endl;
    }

    void publishStopControl() {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>();
        auto blackboard = decision_manager_->getBlackboard();
        msg->gimbal_mode = blackboard->getGimbalModeByStage();
        msg->spin_mode = 0;
        control_pub_->publish(*msg);

        std::cout << "[CONTROL] 停止模式, 云台模式: " << (int)msg->gimbal_mode << ", 底盘不动" << std::endl;
        std::cout << "[DEBUG] 比赛阶段: " << (int)blackboard->stage 
                  << ", 云台模式: " << (int)msg->gimbal_mode 
                  << ", 底盘模式: 0" << std::endl;
    }

    rclcpp::Subscription<decision_messages::msg::OurRobotState>::SharedPtr our_state_sub_;
    rclcpp::Subscription<decision_messages::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sentry_decision::msg::SentryControl>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;
    double target_republish_interval_sec_;
    rclcpp::Time last_target_publish_time_;
    geometry_msgs::msg::Point last_published_target_;
    bool has_last_published_target_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
