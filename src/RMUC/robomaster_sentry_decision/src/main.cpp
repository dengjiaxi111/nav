#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

        // 订阅原始 EnemyRobotState（包含裁判系统数据和视觉锁敌原始数据）
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

        // 修正：发布融合后的敌人坐标到新话题，避免自己订阅自己
        corrected_enemy_pub_ = this->create_publisher<decision_messages::msg::EnemyRobotState>(
            "/decision_messages/EnemyRobotState_fused", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms),
                                         std::bind(&SentryDecisionNode::decisionLoop, this));
    }

private:
    void ourStateCallback(const decision_messages::msg::OurRobotState::SharedPtr msg) {
        decision_manager_->updateOurState(msg);
    }

    void enemyStateCallback(const decision_messages::msg::EnemyRobotState::SharedPtr msg) {
        auto corrected_msg = std::make_shared<decision_messages::msg::EnemyRobotState>(*msg);

        // 视觉锁敌数据有效时，解算地图坐标并更新对应敌人字段
        if (msg->enemy_id > 0 && msg->enemy_x != 0.0 && msg->enemy_y != 0.0) {
            auto blackboard = decision_manager_->getBlackboard();
            double robot_x = blackboard->x / 100.0;   // 米
            double robot_y = blackboard->y / 100.0;
            double robot_yaw = blackboard->robot_yaw;
            double base_yaw = msg->base_yaw;
            double e_x = msg->enemy_x;
            double e_y = msg->enemy_y;

            auto [x_map, y_map] = gimbalToMap(robot_x, robot_y, robot_yaw, base_yaw, e_x, e_y);
            double x_cm = x_map * 100.0;
            double y_cm = y_map * 100.0;

            switch (msg->enemy_id) {
                case 1:
                    corrected_msg->enemy_hero_x = x_cm;
                    corrected_msg->enemy_hero_y = y_cm;
                    break;
                case 2:
                    corrected_msg->enemy_engineer_x = x_cm;
                    corrected_msg->enemy_engineer_y = y_cm;
                    break;
                case 3:
                    corrected_msg->enemy_infantry3_x = x_cm;
                    corrected_msg->enemy_infantry3_y = y_cm;
                    break;
                case 4:
                    corrected_msg->enemy_infantry4_x = x_cm;
                    corrected_msg->enemy_infantry4_y = y_cm;
                    break;
                case 7:
                    corrected_msg->enemy_sentry_x = x_cm;
                    corrected_msg->enemy_sentry_y = y_cm;
                    break;
                default:
                    break;
            }
        }

        // 用修正后的数据更新黑板（决策核心）
        decision_manager_->updateEnemyState(corrected_msg);

        // 发布融合后的消息到独立话题，供其他节点使用
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
            // 提取 yaw (弧度)
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
                auto blackboard = decision_manager_->getBlackboard();
                const bool should_stream_outpost_yaw =
                    output.control_msg.gimbal_mode == GIMBAL_OUTPOST &&
                    output.control_msg.spin_mode == SPIN_OFF;
                if (output.control_needs_publishing || should_stream_outpost_yaw) {
                    publishControl(output.control_msg);
                    blackboard->setControlPublished(true);
                }

                if (output.target_needs_publishing && (output.target_position.x != 0 || output.target_position.y != 0)) {
                    geometry_msgs::msg::Point new_target = output.target_position;
                    bool same_point = (std::abs(new_target.x - last_published_target_x_) < 0.01 &&
                                       std::abs(new_target.y - last_published_target_y_) < 0.01);
                    rclcpp::Time now = this->now();
                    if (!same_point || (now.seconds() - last_target_publish_time_ > 5.0)) {
                        publishTarget(new_target, output.decision_reason);
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

    std::string behaviorTypeToString(BehaviorType type) const {
        switch (type) {
            case BehaviorType::NONE: return "NONE";
            case BehaviorType::INIT_MOVE: return "INIT_MOVE";
            case BehaviorType::INIT_ATTACK: return "INIT_ATTACK";
            case BehaviorType::MOVE_TO_ATTACK_HERO: return "MOVE_TO_ATTACK_HERO";
            case BehaviorType::ATTACK_HERO: return "ATTACK_HERO";
            case BehaviorType::MOVE_TO_ATTACK_ROBOT: return "MOVE_TO_ATTACK_ROBOT";
            case BehaviorType::ATTACK_ROBOT: return "ATTACK_ROBOT";
            case BehaviorType::MOVE_TO_SUPPLY: return "MOVE_TO_SUPPLY";
            case BehaviorType::SUPPLY: return "SUPPLY";
            case BehaviorType::RESURRECTION_MOVE: return "RESURRECTION_MOVE";
            case BehaviorType::RESURRECTING: return "RESURRECTING";
            case BehaviorType::MOVE_TO_BASE_DEFENSE: return "MOVE_TO_BASE_DEFENSE";
            case BehaviorType::BASE_DEFENSE: return "BASE_DEFENSE";
            case BehaviorType::MOVE_TO_GAIN_POINT: return "MOVE_TO_GAIN_POINT";
            case BehaviorType::OCCUPY_GAIN_POINT: return "OCCUPY_GAIN_POINT";
            case BehaviorType::MOVE_TO_FORTRESS: return "MOVE_TO_FORTRESS";
            case BehaviorType::OCCUPY_FORTRESS: return "OCCUPY_FORTRESS";
            case BehaviorType::MOVE_TO_ENEMY_FORTRESS: return "MOVE_TO_ENEMY_FORTRESS";
            case BehaviorType::OCCUPY_ENEMY_FORTRESS: return "OCCUPY_ENEMY_FORTRESS";
            case BehaviorType::MOVE_TO_RAMP: return "MOVE_TO_RAMP";
            case BehaviorType::MOVE_TO_GUARD: return "MOVE_TO_GUARD";
            case BehaviorType::GUARD: return "GUARD";
            case BehaviorType::MOVE_TO_SAFE_POINT: return "MOVE_TO_SAFE_POINT";
        }
        return "UNKNOWN";
    }

    std::string behaviorStateToString(BehaviorState state) const {
        switch (state) {
            case BehaviorState::IDLE: return "IDLE";
            case BehaviorState::MOVING: return "MOVING";
            case BehaviorState::EXECUTING: return "EXECUTING";
            case BehaviorState::COMPLETED: return "COMPLETED";
        }
        return "UNKNOWN";
    }

    void publishTarget(const geometry_msgs::msg::Point& target, const std::string& reason) {
        auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = now();
        msg->header.frame_id = "map";
        msg->point.x = target.x / 100.0;
        msg->point.y = target.y / 100.0;
        target_pub_->publish(*msg);

        auto blackboard = decision_manager_->getBlackboard();
        std::cout << "[TARGET] state=" << reason
                  << ", behavior=" << behaviorTypeToString(blackboard->current_behavior.type)
                  << ", behavior_state=" << behaviorStateToString(blackboard->current_behavior.state)
                  << ", target=(" << msg->point.x << ", " << msg->point.y << ")"
                  << ", hp=" << blackboard->current_hp << "/" << blackboard->getMaxHp()
                  << ", ammo=" << blackboard->allowance_17mm << "/" << blackboard->getMaxAmmo()
                  << ", stage=" << int(blackboard->stage)
                  << ", time=" << blackboard->stage_remaining_time
                  << ", res=" << (blackboard->resurrection_flag ? "true" : "false")
                  << ", init_done=" << (blackboard->initialization_complete ? "true" : "false")
                  << ", enemy_hero_vis=" << (blackboard->enemy_hero.visible ? "true" : "false")
                  << ", enemy_eng_vis=" << (blackboard->enemy_engineer.visible ? "true" : "false")
                  << std::endl;
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
        fillOutpostYawIfNeeded(*msg, true);
        control_pub_->publish(*msg);
    }

    std::pair<double, double> gimbalToMap(
        double robot_x, double robot_y, double robot_yaw,
        double base_yaw, double enemy_x_g, double enemy_y_g)
    {
        double cb = std::cos(base_yaw);
        double sb = std::sin(base_yaw);
        double x_ch = cb * enemy_x_g - sb * enemy_y_g;
        double y_ch = sb * enemy_x_g + cb * enemy_y_g;

        double cr = std::cos(robot_yaw);
        double sr = std::sin(robot_yaw);
        double x_map = cr * x_ch - sr * y_ch + robot_x;
        double y_map = sr * x_ch + cr * y_ch + robot_y;

        return {x_map, y_map};
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

    void fillOutpostYawIfNeeded(sentry_decision::msg::SentryControl& msg, bool force_fill = false) {
        msg.target_yaw_valid = false;
        msg.target_yaw_deg = 0.0;
        if (!force_fill && (msg.gimbal_mode != GIMBAL_OUTPOST || msg.spin_mode != SPIN_OFF)) return;
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
    rclcpp::Publisher<decision_messages::msg::EnemyRobotState>::SharedPtr corrected_enemy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
