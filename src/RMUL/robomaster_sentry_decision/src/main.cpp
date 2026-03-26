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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;
using namespace SentryConstants;

class SentryDecisionNode : public rclcpp::Node {
public:
    SentryDecisionNode()
    : Node("sentry_decision"),
      game_started_(false),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        decision_manager_ = std::make_shared<DecisionManager>();
        auto blackboard = decision_manager_->getBlackboard();

        // 加载配置文件：环境变量 > 当前目录 > config/ > robomaster_sentry_decision/config > 安装目录
        std::string config_path = getConfigPath();
        if (!config_path.empty()) {
            if (!blackboard->loadConfigFromYAML(config_path)) {
                RCLCPP_WARN(this->get_logger(), "无法加载配置文件 %s，使用默认配置", config_path.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未找到配置文件，使用默认配置");
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
    std::string getConfigPath() {
        // 1. 环境变量
        const char* env_config = std::getenv("SENTRY_DECISION_CONFIG");
        if (env_config && std::ifstream(env_config).good()) {
            return std::string(env_config);
        }

        // 2. 当前工作目录
        std::string cwd_config = "sentry_decision_params.yaml";
        if (std::ifstream(cwd_config).good()) {
            return cwd_config;
        }

        // 3. 当前工作目录下的 config 子目录
        std::string cwd_config_sub = "config/sentry_decision_params.yaml";
        if (std::ifstream(cwd_config_sub).good()) {
            return cwd_config_sub;
        }

        // 4. 当前工作目录下的 robomaster_sentry_decision/config 子目录（您的源码结构）
        std::string src_config = "robomaster_sentry_decision/config/sentry_decision_params.yaml";
        if (std::ifstream(src_config).good()) {
            return src_config;
        }

        // 5. 安装目录默认配置
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("sentry_decision");
            std::string default_config = package_share_dir + "/config/sentry_decision_params.yaml";
            if (std::ifstream(default_config).good()) {
                return default_config;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "获取包共享目录失败: %s", e.what());
        }

        return "";
    }

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
            RCLCPP_INFO(this->get_logger(), "updatePositionFromTF: %f, %f", t.transform.translation.x, t.transform.translation.y);
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

                if (output.control_needs_publishing) {
                    publishControl(output.control_msg);
                    blackboard->setControlPublished(true);
                }
            } else {
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
        std::cout << "[TARGET] (" << msg->point.x << ", " << msg->point.y << ")" << std::endl;
    }

    void publishControl(const sentry_decision::msg::SentryControl& ctrl) {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>(ctrl);
        control_pub_->publish(*msg);
        std::string gimbal = (ctrl.gimbal_mode == 1) ? "打人" : "不动";
        std::string spin;
        switch (ctrl.spin_mode) {
            case 0: spin = "不动"; break;
            case 1: spin = "旋转"; break;
            default: spin = "未知";
        }
        std::cout << "[CONTROL] " << gimbal << ", " << spin << std::endl;
    }

    void publishStopControl() {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>();
        auto blackboard = decision_manager_->getBlackboard();
        msg->gimbal_mode = blackboard->getGimbalModeByStage();
        msg->spin_mode = 0;
        control_pub_->publish(*msg);
    }

    rclcpp::Subscription<decision_messages::msg::OurRobotState>::SharedPtr our_state_sub_;
    rclcpp::Subscription<decision_messages::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sentry_decision::msg::SentryControl>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;

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
