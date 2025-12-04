// nav_components/src/nav_server.cpp
// 导航服务器 - 核心状态机

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <nav_core/nav_fsm.hpp>
#include "nav_interfaces/action/navigate.hpp"
#include "nav_components/simple_planner.hpp"
#include "nav_components/pure_pursuit.hpp"
#include "nav_components/backup_recovery.hpp"
#include "nav_components/spin_recovery.hpp"
#include "nav_components/recovery_manager.hpp"
#include "nav_components/static_map_loader.hpp"

using Navigate = nav_interfaces::action::Navigate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;

class NavServer : public rclcpp::Node {
public:
    NavServer() : Node("nav_server"), fsm_(get_logger()) {
        control_rate_ = declare_parameter("control_rate", 20.0);
        goal_timeout_ = declare_parameter("goal_timeout", 60.0);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);
        yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.1);
        map_file_ = declare_parameter("map_file", "");
        map_frame_ = declare_parameter("map_frame", "map");
        base_frame_ = declare_parameter("base_frame", "base_link");
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
        
        // 加载静态地图或订阅地图topic
        if (!map_file_.empty()) {
            map_ = nav_components::StaticMapLoader::load(map_file_, get_logger());
            if (map_) {
                planner_.setMap(map_);
                // 定时发布地图，兼容不同QoS的订阅者
                map_timer_ = create_wall_timer(
                    std::chrono::milliseconds(50),
                    [this]() {
                        if (map_) {
                            map_->header.stamp = now();
                            map_pub_->publish(*map_);
                        }
                    });
                RCLCPP_INFO(get_logger(), "静态地图已加载");
            }
        } else {
            map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", rclcpp::QoS(1).transient_local(),
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                    map_ = msg;
                    planner_.setMap(map_);
                    RCLCPP_INFO(get_logger(), "地图更新: %dx%d", msg->info.width, msg->info.height);
                });
        }
        
        initComponents();
        
        action_server_ = rclcpp_action::create_server<Navigate>(
            this, "navigate",
            std::bind(&NavServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavServer::handleCancel, this, std::placeholders::_1),
            std::bind(&NavServer::handleAccepted, this, std::placeholders::_1));
        
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&NavServer::controlLoop, this));
        
        RCLCPP_INFO(get_logger(), "导航服务器启动");
    }

private:
    void initComponents() {
        planner_.initialize(this);
        controller_.initialize(this);
        controller_.setTolerance(goal_tolerance_, yaw_tolerance_);
        
        auto vel_pub = [this](const geometry_msgs::msg::Twist& cmd) {
            cmd_vel_pub_->publish(cmd);
        };
        
        auto backup = std::make_shared<nav_components::BackupRecovery>();
        backup->initialize(this, vel_pub);
        recovery_mgr_.addRecovery(backup);
        
        auto spin = std::make_shared<nav_components::SpinRecovery>();
        spin->initialize(this, vel_pub);
        recovery_mgr_.addRecovery(spin);
    }
    
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const Navigate::Goal> goal) {
        
        if (fsm_.state() != nav_core::NavState::IDLE &&
            fsm_.state() != nav_core::NavState::SUCCEEDED &&
            fsm_.state() != nav_core::NavState::FAILED) {
            RCLCPP_WARN(get_logger(), "导航中，拒绝新目标");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(get_logger(), "目标: (%.2f, %.2f)",
            goal->goal_pose.pose.position.x, goal->goal_pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(get_logger(), "取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
        goal_handle_ = goal_handle;
        goal_ = goal_handle->get_goal()->goal_pose;
        fsm_.reset();
        fsm_.transitionTo(nav_core::NavState::PLANNING);
        start_time_ = now();
    }
    
    void controlLoop() {
        if (!goal_handle_) return;
        
        // 获取机器人当前位姿
        if (!updateRobotPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "无法获取 %s -> %s 变换", map_frame_.c_str(), base_frame_.c_str());
            return;
        }
        
        if (goal_handle_->is_canceling()) {
            stopRobot();
            auto result = std::make_shared<Navigate::Result>();
            result->success = false;
            result->message = "已取消";
            goal_handle_->canceled(result);
            goal_handle_ = nullptr;
            fsm_.transitionTo(nav_core::NavState::IDLE);
            return;
        }
        
        if ((now() - start_time_).seconds() > goal_timeout_) {
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::TIMEOUT);
        }
        
        switch (fsm_.state()) {
            case nav_core::NavState::PLANNING:    doPlanning(); break;
            case nav_core::NavState::CONTROLLING: doControlling(); break;
            case nav_core::NavState::RECOVERY:    doRecovery(); break;
            case nav_core::NavState::SUCCEEDED:   finishSuccess(); break;
            case nav_core::NavState::FAILED:      finishFailure(); break;
            default: break;
        }
        
        publishFeedback();
    }
    
    // 通过 TF 查询机器人位姿
    bool updateRobotPose() {
        try {
            auto transform = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
            
            current_pose_.header.stamp = transform.header.stamp;
            current_pose_.header.frame_id = map_frame_;
            current_pose_.pose.position.x = transform.transform.translation.x;
            current_pose_.pose.position.y = transform.transform.translation.y;
            current_pose_.pose.position.z = transform.transform.translation.z;
            current_pose_.pose.orientation = transform.transform.rotation;
            return true;
        } catch (const tf2::TransformException& e) {
            return false;
        }
    }
    
    void doPlanning() {
        if (!map_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "等待地图...");
            return;
        }
        
        nav_msgs::msg::Path path;
        if (planner_.plan(current_pose_, goal_, path)) {
            current_path_ = path;
            controller_.setPath(path);
            path_pub_->publish(path);
            fsm_.transitionTo(nav_core::NavState::CONTROLLING);
        } else {
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::PLANNING_FAILED);
        }
    }
    
    void doControlling() {
        geometry_msgs::msg::Twist cmd;
        auto result = controller_.computeVelocity(current_pose_, cmd);
        
        switch (result) {
            case nav_core::ControlResult::RUNNING:
                cmd_vel_pub_->publish(cmd);
                break;
            case nav_core::ControlResult::SUCCEEDED:
                stopRobot();
                fsm_.transitionTo(nav_core::NavState::SUCCEEDED);
                break;
            case nav_core::ControlResult::FAILED:
                fsm_.triggerRecovery(nav_core::RecoveryTrigger::CONTROL_FAILED);
                break;
        }
    }
    
    void doRecovery() {
        if (fsm_.recoveryExhausted()) {
            fsm_.transitionTo(nav_core::NavState::FAILED);
            return;
        }
        
        auto status = recovery_mgr_.update(current_pose_);
        
        if (status == nav_core::RecoveryStatus::SUCCEEDED) {
            recovery_mgr_.reset();
            fsm_.transitionTo(nav_core::NavState::PLANNING);
        } else if (status == nav_core::RecoveryStatus::FAILED) {
            fsm_.transitionTo(nav_core::NavState::FAILED);
        } else if (status == nav_core::RecoveryStatus::IDLE) {
            recovery_mgr_.start(fsm_.recoveryTrigger(), current_pose_);
        }
    }
    
    void finishSuccess() {
        auto result = std::make_shared<Navigate::Result>();
        result->success = true;
        result->total_time = (now() - start_time_).seconds();
        goal_handle_->succeed(result);
        goal_handle_ = nullptr;
        fsm_.transitionTo(nav_core::NavState::IDLE);
        RCLCPP_INFO(get_logger(), "导航成功 (%.2fs)", result->total_time);
    }
    
    void finishFailure() {
        stopRobot();
        auto result = std::make_shared<Navigate::Result>();
        result->success = false;
        result->message = "恢复失败";
        goal_handle_->abort(result);
        goal_handle_ = nullptr;
        fsm_.transitionTo(nav_core::NavState::IDLE);
        RCLCPP_ERROR(get_logger(), "导航失败");
    }
    
    void publishFeedback() {
        if (!goal_handle_) return;
        
        auto fb = std::make_shared<Navigate::Feedback>();
        fb->current_pose = current_pose_;
        fb->state = static_cast<uint8_t>(fsm_.state());
        fb->recovery_count = fsm_.recoveryCount();
        fb->distance_remaining = std::hypot(
            goal_.pose.position.x - current_pose_.pose.position.x,
            goal_.pose.position.y - current_pose_.pose.position.y);
        goal_handle_->publish_feedback(fb);
    }
    
    void stopRobot() {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    
    nav_core::NavFSM fsm_;
    nav_components::SimplePlanner planner_;
    nav_components::PurePursuit controller_;
    nav_components::RecoveryManager recovery_mgr_;
    
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> goal_handle_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Path current_path_;
    
    std::string map_file_, map_frame_, base_frame_;
    double control_rate_, goal_timeout_, goal_tolerance_, yaw_tolerance_;
    rclcpp::Time start_time_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavServer>());
    rclcpp::shutdown();
    return 0;
}
