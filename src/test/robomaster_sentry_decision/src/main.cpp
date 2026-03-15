#include "sentry_decision/DecisionManager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>

// ROS2消息类型
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"

// 自定义消息类型
#include "sentry_decision/msg/sentry_control.hpp"

using namespace std::chrono_literals;

class SentryDecisionNode : public rclcpp::Node {
public:
    SentryDecisionNode() : Node("sentry_decision"), game_started_(false) {
        // 初始化决策管理器
        decision_manager_ = std::make_shared<DecisionManager>();
        
        // 订阅器
        our_state_sub_ = this->create_subscription<decision_messages::msg::OurRobotState>(
            "/decision_messages/OurRobotState", 10,
            std::bind(&SentryDecisionNode::ourStateCallback, this, std::placeholders::_1));
        
        enemy_state_sub_ = this->create_subscription<decision_messages::msg::EnemyRobotState>(
            "/decision_messages/EnemyRobotState", 10,
            std::bind(&SentryDecisionNode::enemyStateCallback, this, std::placeholders::_1));
        
        game_state_sub_ = this->create_subscription<decision_messages::msg::GameState>(
            "/decision_messages/GameState", 10,
            std::bind(&SentryDecisionNode::gameStateCallback, this, std::placeholders::_1));
        
        // 发布器
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/sentry/target_position", 10);
        control_pub_ = this->create_publisher<sentry_decision::msg::SentryControl>("/sentry/control", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("/sentry/debug", 10);
        
        // 定时器（10Hz）
        timer_ = this->create_wall_timer(100ms, std::bind(&SentryDecisionNode::decisionLoop, this));
        
        std::cout << "[SYSTEM] Sentry Decision System Started" << std::endl;
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
        
        // 检查比赛是否开始 - 修复：只有当stage从非4变为4时才打印开始信息
        // 避免在比赛过程中频繁打印开始/结束信息
        if (msg->stage == 4 && !game_started_) {
            game_started_ = true;
            std::cout << "[SYSTEM] 比赛开始!" << std::endl;
        } else if (msg->stage != 4 && game_started_) {
            game_started_ = false;
            std::cout << "[SYSTEM] 比赛结束" << std::endl;
        }
    }
    
    void decisionLoop() {
        try {
            // 执行决策
            DecisionOutput output = decision_manager_->executeDecision();
            
            // 只在比赛开始后才发布
            if (game_started_) {
                auto blackboard = decision_manager_->getBlackboard();
                
                // 根据决策输出决定发布内容
                if (output.target_needs_publishing && 
                    output.target_position.x != 0 && output.target_position.y != 0) {
                    std::cout << "[PUBLISH] 发布目标点: (" 
                              << output.target_position.x/100.0 << ", " 
                              << output.target_position.y/100.0 << ")" 
                              << " 原因: " << output.decision_reason << std::endl;
                    publishTarget(output.target_position);
                    blackboard->setTargetPublished(true);
                }
                
                if (output.control_needs_publishing) {
                    std::cout << "[PUBLISH] 发布控制消息" << std::endl;
                    publishControl(output.control_msg);
                    blackboard->setControlPublished(true);
                }
                
            } else {
                // 比赛未开始，只发布停止的控制消息
                publishStopControl();
            }
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Decision loop error: " << e.what() << std::endl;
        }
    }
    
    void publishTarget(const geometry_msgs::msg::Point& target) {
        auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "map";
        msg->point.x = target.x / 100.0;  // cm → m
        msg->point.y = target.y / 100.0;  // cm → m
        msg->point.z = 0.0;  // z坐标通常为0
        target_pub_->publish(*msg);
        
        // 精简输出：只显示目标点
        std::cout << "[TARGET] 发布目标点: (" 
                  << static_cast<double>(msg->point.x) << ", " 
                  << static_cast<double>(msg->point.y) << ")" << std::endl;
    }
    
    void publishControl(const sentry_decision::msg::SentryControl& control_msg) {
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>(control_msg);
        control_pub_->publish(*msg);
        
        // 精简输出：只显示姿态切换
        std::string gimbal_mode_str, spin_mode_str, posture_str, ramp_mode_str;
        
        switch (control_msg.gimbal_mode) {
            case 0: gimbal_mode_str = "打符"; break;
            case 1: gimbal_mode_str = "打人"; break;
            case 2: gimbal_mode_str = "打前哨站"; break;
            case 3: gimbal_mode_str = "不动"; break;
            default: gimbal_mode_str = "未知";
        }
        
        switch (control_msg.spin_mode) {
            case 0: spin_mode_str = "不动"; break;
            case 1: spin_mode_str = "低速转"; break;
            case 2: spin_mode_str = "变速转"; break;
            case 3: spin_mode_str = "高速转"; break;
            default: spin_mode_str = "未知";
        }
        
        switch (control_msg.posture) {
            case 1: posture_str = "进攻姿态"; break;
            case 2: posture_str = "防御姿态"; break;
            case 3: posture_str = "移动姿态"; break;
            default: posture_str = "未知姿态";
        }
        
        switch (control_msg.ramp_mode) {
            case 0: ramp_mode_str = "不飞坡"; break;
            case 1: ramp_mode_str = "飞坡"; break;
            default: ramp_mode_str = "未知";
        }
        
        std::cout << "[CONTROL] 姿态切换: " << gimbal_mode_str << ", " 
                  << spin_mode_str << ", " << posture_str << ", " 
                  << ramp_mode_str << std::endl;
    }
    
    void publishStopControl() {
        // 发布停止的控制消息
        auto msg = std::make_shared<sentry_decision::msg::SentryControl>();
        msg->gimbal_mode = 3;  // GIMBAL_IDLE
        msg->spin_mode = 0;    // SPIN_OFF
        msg->posture = 2;      // POSTURE_DEFENSE
        msg->ramp_mode = 0;    // RAMP_OFF
        control_pub_->publish(*msg);
    }
    
    // 成员变量
    rclcpp::Subscription<decision_messages::msg::OurRobotState>::SharedPtr our_state_sub_;
    rclcpp::Subscription<decision_messages::msg::EnemyRobotState>::SharedPtr enemy_state_sub_;
    rclcpp::Subscription<decision_messages::msg::GameState>::SharedPtr game_state_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sentry_decision::msg::SentryControl>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<DecisionManager> decision_manager_;
    bool game_started_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SentryDecisionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
