// nav_components/src/nav_server.cpp
// 导航服务器 - 核心状态机

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <nav_core/nav_fsm.hpp>
#include <nav_core/map_interface.hpp>
#include "nav_interfaces/action/navigate.hpp"
#include "nav_components/simple_planner.hpp"
#include "nav_components/pure_pursuit.hpp"
#include "nav_components/nmpc.hpp"
#include "nav_components/backup_recovery.hpp"
#include "nav_components/spin_recovery.hpp"
#include "nav_components/recovery_manager.hpp"
#include "nav_components/layered_map_manager.hpp"

using Navigate = nav_interfaces::action::Navigate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;

class NavServer : public rclcpp::Node {
public:
    NavServer() : Node("nav_server"), fsm_(get_logger()) {
        // === 核心控制参数 ===
        declare_parameter("control_rate", 20.0);
        declare_parameter("goal_timeout", 60.0);
        declare_parameter("goal_tolerance", 0.1);
        declare_parameter("yaw_tolerance", 0.1);
        declare_parameter("controller_timeout", 10.0);
        declare_parameter("controller_progress_threshold", 0.15);
        
        control_rate_ = get_parameter("control_rate").as_double();
        goal_timeout_ = get_parameter("goal_timeout").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        yaw_tolerance_ = get_parameter("yaw_tolerance").as_double();
        controller_timeout_ = get_parameter("controller_timeout").as_double();
        controller_progress_threshold_ = get_parameter("controller_progress_threshold").as_double();
        
        // === 路径检查参数 ===
        declare_parameter("path_check_period", 2.0);
        declare_parameter("path_lateral_tolerance", 0.5);
        
        path_check_period_ = get_parameter("path_check_period").as_double();
        path_lateral_tolerance_ = get_parameter("path_lateral_tolerance").as_double();
        
        // === 脱困系统参数 ===
        declare_parameter("escape.trigger_costmap_threshold", 99);
        declare_parameter("escape.target_safe_esdf", 0.3);
        declare_parameter("escape.search_radius", 5.0);
        declare_parameter("escape.rotation_speed", 0.3);
        declare_parameter("escape.forward_speed", 0.15);
        declare_parameter("escape.yaw_tolerance", 0.1);
        declare_parameter("escape.position_tolerance", 0.15);
        declare_parameter("escape.timeout", 10.0);
        
        escape_trigger_costmap_threshold_ = get_parameter("escape.trigger_costmap_threshold").as_int();
        escape_target_safe_esdf_ = get_parameter("escape.target_safe_esdf").as_double();
        escape_search_radius_ = get_parameter("escape.search_radius").as_double();
        escape_rotation_speed_ = get_parameter("escape.rotation_speed").as_double();
        escape_forward_speed_ = get_parameter("escape.forward_speed").as_double();
        escape_yaw_tolerance_ = get_parameter("escape.yaw_tolerance").as_double();
        escape_position_tolerance_ = get_parameter("escape.position_tolerance").as_double();
        escape_timeout_ = get_parameter("escape.timeout").as_double();
        
        // === 坐标系配置 ===
        declare_parameter("map_file", "");
        declare_parameter("map_frame", "map");
        declare_parameter("base_frame", "base_link");
        declare_parameter("odom_frame", "odom");
        
        map_file_ = get_parameter("map_file").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        
        // === ESDF 配置 ===
        declare_parameter("enable_esdf", false);
        declare_parameter("esdf_vis_max_dist", 2.0);
        
        enable_esdf_ = get_parameter("enable_esdf").as_bool();
        esdf_vis_max_dist_ = get_parameter("esdf_vis_max_dist").as_double();
        
        // === 性能日志 ===
        declare_parameter("enable_performance_logging", false);
        bool enable_performance_logging = get_parameter("enable_performance_logging").as_bool();
        
        // === 参数诊断日志 ===
        RCLCPP_INFO(get_logger(), "========== NavServer 参数诊断 ==========");
        RCLCPP_INFO(get_logger(), "control_rate: %.1f Hz", control_rate_);
        RCLCPP_INFO(get_logger(), "controller_timeout: %.1f s", controller_timeout_);
        RCLCPP_INFO(get_logger(), "controller_progress_threshold: %.2f m", controller_progress_threshold_);
        RCLCPP_INFO(get_logger(), "path_check_period: %.2f s", path_check_period_);
        RCLCPP_INFO(get_logger(), "path_lateral_tolerance: %.3f m ⚠️", path_lateral_tolerance_);
        RCLCPP_INFO(get_logger(), "escape.trigger_costmap_threshold: %d", escape_trigger_costmap_threshold_);
        RCLCPP_INFO(get_logger(), "escape.target_safe_esdf: %.2f m", escape_target_safe_esdf_);
        RCLCPP_INFO(get_logger(), "escape.search_radius: %.1f m", escape_search_radius_);
        RCLCPP_INFO(get_logger(), "escape.rotation_speed: %.2f rad/s", escape_rotation_speed_);
        RCLCPP_INFO(get_logger(), "escape.forward_speed: %.2f m/s", escape_forward_speed_);
        RCLCPP_INFO(get_logger(), "========================================");
        
        // 分层地图配置
        declare_parameter("enable_static_layer", true);
        declare_parameter("enable_dynamic_layer", false);
        declare_parameter("dynamic_layer_topic", "/rog_map/map_2d");
        
        enable_static_layer_ = get_parameter("enable_static_layer").as_bool();
        enable_dynamic_layer_ = get_parameter("enable_dynamic_layer").as_bool();
        dynamic_layer_topic_ = get_parameter("dynamic_layer_topic").as_string();
        
        // 膨胀参数
        declare_parameter("inflation.radius", 0.5);
        declare_parameter("inflation.inscribed_radius", 0.2);
        declare_parameter("inflation.cost_scaling", 3.0);
        declare_parameter("inflation.decay_type", "exponential");
        
        nav_components::InflationParams inflation_params;
        inflation_params.inflation_radius = get_parameter("inflation.radius").as_double();
        inflation_params.inscribed_radius = get_parameter("inflation.inscribed_radius").as_double();
        inflation_params.cost_scaling = get_parameter("inflation.cost_scaling").as_double();
        std::string decay_str = get_parameter("inflation.decay_type").as_string();
        inflation_params.decay_type = (decay_str == "linear") ? 
            nav_components::DecayType::LINEAR : nav_components::DecayType::EXPONENTIAL;
        inflation_params_ = inflation_params;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
        static_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("static_map", rclcpp::QoS(1).transient_local().reliable());
        fused_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("fused_map", rclcpp::QoS(1).transient_local().reliable());
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(1).transient_local().reliable());
        if (enable_esdf_) {
            esdf_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("esdf_map", rclcpp::QoS(1).transient_local().reliable());
        }
        
        // RViz 目标订阅
        goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&NavServer::goalPoseCallback, this, std::placeholders::_1));
        
        // 地图管理器（使用分层地图管理器）
        map_manager_ = std::make_shared<nav_components::LayeredMapManager>();
        map_manager_->initialize(this, tf_buffer_);
        map_manager_->setEsdfEnabled(enable_esdf_);
        map_manager_->setEsdfVisMaxDist(esdf_vis_max_dist_);
        map_manager_->setEnablePerformanceLogging(enable_performance_logging);
        map_manager_->setStaticLayerEnabled(enable_static_layer_);  // 设置静态层开关
        
        initComponents();
        
        if (enable_static_layer_) {
            // 静态层启用时加载静态地图
            if (!map_file_.empty()) {
                // 从文件加载静态地图
                if (map_manager_->loadStaticMap(map_file_, inflation_params)) {
                    planner_.setMap(map_manager_);
                    startMapPublisher();
                    RCLCPP_INFO(get_logger(), "静态地图加载完成");
                }
            } else {
                // 订阅外部静态地图
                static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                    "map", rclcpp::QoS(1).transient_local(),
                    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                        map_manager_->setInflationParams(inflation_params_);
                        map_manager_->setStaticMap(msg);
                        planner_.setMap(map_manager_);
                        RCLCPP_INFO(get_logger(), "静态地图更新: %dx%d", msg->info.width, msg->info.height);
                    });
            }
        } else {
            // 静态层禁用时，创建空白地图框架（50m x 50m @ 0.05m/cell）
            map_manager_->createBlankStaticMap(50.0, 50.0, 0.05, inflation_params);
            planner_.setMap(map_manager_);
            startMapPublisher();
            RCLCPP_WARN(get_logger(), "静态层已禁用");
        }
        
        // 订阅动态障碍物层（来自rog_map）
        if (enable_dynamic_layer_) {
            const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
            dynamic_layer_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                dynamic_layer_topic_, qos,
                std::bind(&NavServer::dynamicLayerCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "动态层订阅: %s", dynamic_layer_topic_.c_str());
        }
        
        action_server_ = rclcpp_action::create_server<Navigate>(
            this, "navigate",
            std::bind(&NavServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavServer::handleCancel, this, std::placeholders::_1),
            std::bind(&NavServer::handleAccepted, this, std::placeholders::_1));
        
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&NavServer::controlLoop, this));
        
        RCLCPP_INFO(get_logger(), "导航服务器启动 (静态层: %s, 动态层: %s)", 
                    enable_static_layer_ ? "启用" : "禁用",
                    enable_dynamic_layer_ ? "启用" : "禁用");
    }

private:
    // 动态障碍物层回调
    void dynamicLayerCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (map_manager_) {
            map_manager_->updateDynamicLayer(msg);
        }
    }
    
    // 启动地图发布定时器
    void startMapPublisher() {
        map_pub_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz 发布（动态层更新较快）
            [this]() {
                if (!map_manager_) {
                    return;
                }
                
                // 发布静态地图
                if (auto grid = map_manager_->getStaticMap()) {
                    static_map_pub_->publish(*grid);
                }
                // 发布融合地图
                if (auto fused = map_manager_->getFusedMap()) {
                    fused_map_pub_->publish(*fused);
                }
                // 发布膨胀代价地图
                if (auto costmap = map_manager_->getCostmap()) {
                    costmap_pub_->publish(*costmap);
                }
                // 发布 ESDF 可视化
                if (enable_esdf_) {
                    if (auto vis = map_manager_->getEsdfVis()) {
                        esdf_pub_->publish(*vis);
                    }
                }
            });
    }
    
    // RViz 2D Goal Pose 回调
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "RViz 目标: (%.2f, %.2f)", 
                    msg->pose.position.x, msg->pose.position.y);
        
        // 如果有旧任务，先中止
        if (goal_handle_) {
            auto result = std::make_shared<Navigate::Result>();
            result->success = false;
            result->message = "被 RViz 目标抢占";
            goal_handle_->abort(result);
            goal_handle_ = nullptr;
        }
        
        // 直接设置目标并开始导航（无需 Action Client）
        goal_ = *msg;
        goal_.header.frame_id = map_frame_;
        stopRobot();
        recovery_mgr_.reset();
        fsm_.reset();
        fsm_.transitionTo(nav_core::NavState::PLANNING);
        start_time_ = std::chrono::steady_clock::now();
        rviz_goal_active_ = true;
    }
    
    void initComponents() {
        planner_.initialize(this);
        controller_.initialize(this);
        controller_.setTolerance(goal_tolerance_, yaw_tolerance_);
        
        // 将地图接口传递给控制器（用于 ESDF 障碍物代价）
        if (map_manager_) {
            controller_.setMap(map_manager_);
        }
        
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
        
        // 如果正在导航中，将取消当前任务并执行新目标
        RCLCPP_INFO(get_logger(), "收到目标: (%.2f, %.2f)",
            goal->goal_pose.pose.position.x, goal->goal_pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(get_logger(), "取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
        // 如果有旧任务，先中止
        if (goal_handle_) {
            auto result = std::make_shared<Navigate::Result>();
            result->success = false;
            result->message = "被新目标抢占";
            goal_handle_->abort(result);
            RCLCPP_INFO(get_logger(), "旧目标已中止");
        }
        
        goal_handle_ = goal_handle;
        goal_ = goal_handle->get_goal()->goal_pose;
        stopRobot();
        recovery_mgr_.reset();
        fsm_.reset();
        fsm_.transitionTo(nav_core::NavState::PLANNING);
        start_time_ = std::chrono::steady_clock::now();
        rviz_goal_active_ = false;  // Action 目标
    }
    
    void controlLoop() {
        // 支持 Action 和 RViz 两种目标来源
        if (!goal_handle_ && !rviz_goal_active_) {
            return;
        }
        
        // 获取机器人当前位姿
        if (!updateRobotPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "无法获取 %s -> %s 变换", map_frame_.c_str(), base_frame_.c_str());
            return;
        }
        
        // Action 取消处理
        if (goal_handle_ && goal_handle_->is_canceling()) {
            stopRobot();
            auto result = std::make_shared<Navigate::Result>();
            result->success = false;
            result->message = "已取消";
            goal_handle_->canceled(result);
            goal_handle_ = nullptr;
            fsm_.transitionTo(nav_core::NavState::IDLE);
            return;
        }
        
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count() > goal_timeout_) {
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::TIMEOUT);
        }
        
        switch (fsm_.state()) {
            case nav_core::NavState::PLANNING:    doPlanning(); break;
            case nav_core::NavState::CONTROLLING: doControlling(); break;
            case nav_core::NavState::ESCAPING:    doEscaping(); break;
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
        if (!map_manager_ || !map_manager_->hasMap()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "等待地图...");
            return;
        }
        
        // 在规划前检查起点是否在障碍物中（仅用 Costmap）
        auto costmap = map_manager_->getCostmap();
        if (costmap) {
            int mx, my;
            double resolution = costmap->info.resolution;
            double origin_x = costmap->info.origin.position.x;
            double origin_y = costmap->info.origin.position.y;
            mx = static_cast<int>((current_pose_.pose.position.x - origin_x) / resolution);
            my = static_cast<int>((current_pose_.pose.position.y - origin_y) / resolution);
            
            if (mx >= 0 && mx < costmap->info.width && my >= 0 && my < costmap->info.height) {
                int idx = my * costmap->info.width + mx;
                int8_t cost = costmap->data[idx];
                if (cost >= escape_trigger_costmap_threshold_) {
                    RCLCPP_WARN(get_logger(), 
                        "⚠️ 起点在障碍物中: costmap=%d >= %d，进入脱困模式", 
                        cost, escape_trigger_costmap_threshold_);
                    fsm_.transitionTo(nav_core::NavState::ESCAPING);
                    escape_start_time_ = std::chrono::steady_clock::now();
                    escape_target_x_ = -1.0;
                    escape_phase_ = 0;
                    return;
                }
            }
        }
        
        // 起点安全，正常规划
        nav_msgs::msg::Path path;
        if (planner_.plan(current_pose_, goal_, path)) {
            current_path_ = path;
            controller_.setPath(path);
            path_pub_->publish(path);
            
            control_count_ = 0;
            last_progress_time_ = std::chrono::steady_clock::now();
            last_progress_pose_ = current_pose_;
            last_path_check_wall_time_ = std::chrono::steady_clock::now();
            
            fsm_.transitionTo(nav_core::NavState::CONTROLLING);
        } else {
            RCLCPP_ERROR(get_logger(), "规划失败：A*无法找到路径");
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::PLANNING_FAILED);
        }
    }
    
    void doEscaping() {
        // 脱困模式：搜索最近的安全点，原地转向后直线前往
        // 策略：BFS 搜索 ESDF > target_safe_esdf 的格子，差速底盘先转向再前进
        
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - escape_start_time_).count();
        
        if (elapsed > escape_timeout_) {
            RCLCPP_ERROR(get_logger(), "⏱️ 脱困超时(%.1fs)，进入恢复模式", escape_timeout_);
            stopRobot();
            escape_target_x_ = -1.0;  // 重置标记
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::STUCK);
            return;
        }
        
        // 首次进入：搜索安全目标点（只执行一次）
        if (escape_target_x_ < -0.5) {  // 使用 -1.0 作为"未初始化"标记
            // BFS 搜索最近的安全点
            if (!findSafeEscapeTarget()) {
                RCLCPP_ERROR(get_logger(), "❌ 无法找到安全脱困点，进入恢复模式");
                stopRobot();
                escape_target_x_ = -1.0;  // 重置标记
                fsm_.triggerRecovery(nav_core::RecoveryTrigger::STUCK);
                return;
            }
            escape_phase_ = 0;  // 阶段0: 转向
            RCLCPP_WARN(get_logger(), 
                "🎯 脱困目标: (%.2f, %.2f), 距离=%.2fm", 
                escape_target_x_, escape_target_y_,
                std::hypot(escape_target_x_ - current_pose_.pose.position.x,
                          escape_target_y_ - current_pose_.pose.position.y));
        }
        
        // 获取当前朝向
        tf2::Quaternion q;
        tf2::fromMsg(current_pose_.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 计算到目标的方向和距离
        double dx = escape_target_x_ - current_pose_.pose.position.x;
        double dy = escape_target_y_ - current_pose_.pose.position.y;
        double target_yaw = std::atan2(dy, dx);
        double dist_to_target = std::hypot(dx, dy);
        
        // 归一化角度差
        double yaw_error = target_yaw - yaw;
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
        
        geometry_msgs::msg::Twist cmd;
        
        // 阶段0: 原地旋转对齐目标
        if (escape_phase_ == 0) {
            if (std::abs(yaw_error) > escape_yaw_tolerance_) {
                cmd.angular.z = std::copysign(escape_rotation_speed_, yaw_error);
                cmd_vel_pub_->publish(cmd);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                    "🔄 阶段0: 旋转对齐, yaw_error=%.2f°", yaw_error * 180.0 / M_PI);
            } else {
                RCLCPP_INFO(get_logger(), "✅ 旋转完成，开始前进");
                escape_phase_ = 1;
            }
            return;
        }
        
        // 阶段1: 直线前进到目标
        if (escape_phase_ == 1) {
            // 检查是否到达目标
            if (dist_to_target < escape_position_tolerance_) {
                RCLCPP_INFO(get_logger(), "✅ 脱困成功！到达安全点，重新规划");
                stopRobot();
                escape_target_x_ = -1.0;  // 重置目标
                fsm_.transitionTo(nav_core::NavState::PLANNING);
                return;
            }
            
            // 检查是否已经进入安全区域（提前退出）
            if (map_manager_ && map_manager_->hasEsdf()) {
                double gx, gy;  // 梯度（不使用）
                double esdf_dist = map_manager_->getEsdfDistanceWithGradient(
                    current_pose_.pose.position.x, current_pose_.pose.position.y,
                    &gx, &gy);
                if (esdf_dist > escape_target_safe_esdf_) {
                    RCLCPP_INFO(get_logger(), 
                        "✅ 已进入安全区域(ESDF=%.2fm > %.2fm)，重新规划", 
                        esdf_dist, escape_target_safe_esdf_);
                    stopRobot();
                    escape_target_x_ = -1.0;
                    fsm_.transitionTo(nav_core::NavState::PLANNING);
                    return;
                }
            }
            
            // 前进时保持方向修正（轻微转向）
            if (std::abs(yaw_error) > escape_yaw_tolerance_ * 2) {
                // 角度偏离过大，停止前进，重新进入旋转阶段
                RCLCPP_WARN(get_logger(), "⚠️ 方向偏离过大(%.1f°)，重新旋转", 
                    yaw_error * 180.0 / M_PI);
                escape_phase_ = 0;
                return;
            }
            
            cmd.linear.x = escape_forward_speed_;
            cmd.angular.z = std::copysign(std::min(0.1, std::abs(yaw_error)), yaw_error);  // 轻微修正
            cmd_vel_pub_->publish(cmd);
            
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "➡️  阶段1: 前进中, 剩余=%.2fm, yaw_error=%.1f°, 用时=%.1fs", 
                dist_to_target, yaw_error * 180.0 / M_PI, elapsed);
        }
    }
    
    // 搜索安全脱困点（BFS 螺旋搜索 ESDF > threshold 的格子）
    bool findSafeEscapeTarget() {
        if (!map_manager_ || !map_manager_->hasEsdf()) {
            RCLCPP_ERROR(get_logger(), "ESDF 地图不可用，无法搜索安全点");
            return false;
        }
        
        auto costmap = map_manager_->getCostmap();
        if (!costmap) {
            RCLCPP_ERROR(get_logger(), "Costmap 不可用");
            return false;
        }
        
        double resolution = costmap->info.resolution;
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;
        int width = costmap->info.width;
        int height = costmap->info.height;
        
        // 当前位置转地图坐标
        int cx = static_cast<int>((current_pose_.pose.position.x - origin_x) / resolution);
        int cy = static_cast<int>((current_pose_.pose.position.y - origin_y) / resolution);
        
        RCLCPP_INFO(get_logger(), 
            "🔍 开始BFS搜索: 当前位置(%.2f, %.2f) -> 地图坐标(%d, %d), 地图尺寸=%dx%d, 分辨率=%.2fm",
            current_pose_.pose.position.x, current_pose_.pose.position.y,
            cx, cy, width, height, resolution);
        
        // BFS 螺旋搜索，使用配置的最大半径
        int max_radius = static_cast<int>(escape_search_radius_ / resolution);
        RCLCPP_INFO(get_logger(), 
            "🔍 搜索参数: 半径=%.2fm(%d格), 目标ESDF>%.2fm, cost<99",
            escape_search_radius_, max_radius, escape_target_safe_esdf_);
        
        int checked_cells = 0;
        int valid_cells = 0;
        double max_esdf_found = -1.0;
        
        for (int r = 1; r <= max_radius; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    // 只搜索外环
                    if (std::abs(dx) != r && std::abs(dy) != r) continue;
                    
                    int mx = cx + dx;
                    int my = cy + dy;
                    
                    // 边界检查
                    if (mx < 0 || mx >= width || my < 0 || my >= height) continue;
                    
                    checked_cells++;
                    
                    // 转世界坐标
                    double wx = origin_x + (mx + 0.5) * resolution;
                    double wy = origin_y + (my + 0.5) * resolution;
                    
                    // 检查 ESDF
                    double gx, gy;  // 梯度（不使用）
                    double esdf_dist = map_manager_->getEsdfDistanceWithGradient(wx, wy, &gx, &gy);
                    
                    // 检查 costmap（必须可通行）
                    int idx = my * width + mx;
                    int8_t cost = costmap->data[idx];
                    
                    // 记录最大ESDF值（用于调试）
                    if (cost < 99 && esdf_dist > max_esdf_found) {
                        max_esdf_found = esdf_dist;
                    }
                    
                    // 每10个格子输出一次样本调试信息
                    if (checked_cells % 50 == 0) {
                        RCLCPP_DEBUG(get_logger(), 
                            "  样本[%d]: (%.2f,%.2f) ESDF=%.3fm, cost=%d, r=%d",
                            checked_cells, wx, wy, esdf_dist, cost, r);
                    }
                    
                    if (esdf_dist > escape_target_safe_esdf_ && cost < 99) {
                        valid_cells++;
                        escape_target_x_ = wx;
                        escape_target_y_ = wy;
                        double search_dist = r * resolution;
                        RCLCPP_INFO(get_logger(), 
                            "✅ 找到安全点: (%.2f, %.2f), ESDF=%.3fm > %.2fm, cost=%d < 99, 搜索距离=%.2fm (检查%d格)", 
                            wx, wy, esdf_dist, escape_target_safe_esdf_, cost, search_dist, checked_cells);
                        return true;
                    }
                }
            }
        }
        
        RCLCPP_ERROR(get_logger(), 
            "❌ 搜索失败: 检查了%d个格子, 无满足条件的点 (最大ESDF=%.3fm < %.2fm或cost>=99)",
            checked_cells, max_esdf_found, escape_target_safe_esdf_);
        
        return false;
    }
    
    void doControlling() {
        control_count_++;
        if (control_count_ == 1) {
            RCLCPP_INFO(get_logger(), "🎮 Controller: 开始控制循环");
        }
        
        auto wall_now = std::chrono::steady_clock::now();
        double time_since_check = std::chrono::duration<double>(
            wall_now - last_path_check_wall_time_).count();
        if (time_since_check > path_check_period_) {
            last_path_check_wall_time_ = wall_now;
            
            if (!current_path_.poses.empty() && !planner_.validatePath(current_path_)) {
                RCLCPP_WARN(get_logger(), 
                    "⚠️  路径验证失败: 检测到动态障碍物，触发重新规划");
                stopRobot();
                fsm_.transitionTo(nav_core::NavState::PLANNING);
                return;
            }
            
            // 检查2: 当前位置是否偏离路径过远（横向偏移检测）
            if (!current_path_.poses.empty()) {
                // 计算机器人到路径的最短距离
                double min_dist = std::numeric_limits<double>::max();
                size_t closest_idx = 0;
                
                for (size_t i = 0; i < current_path_.poses.size(); ++i) {
                    const auto& pose = current_path_.poses[i].pose.position;
                    double dx = current_pose_.pose.position.x - pose.x;
                    double dy = current_pose_.pose.position.y - pose.y;
                    double dist = std::hypot(dx, dy);
                    
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_idx = i;
                    }
                }
                
                // 横向偏移阈值（可通过参数调整）
                if (min_dist > path_lateral_tolerance_) {
                    RCLCPP_WARN(get_logger(), 
                        "⚠️  横向偏离路径过远 (%.2f m > %.2f m, 最近点索引: %zu/%zu), 触发重新规划", 
                        min_dist, path_lateral_tolerance_, closest_idx, current_path_.poses.size());
                    planner_.clearCache();  // 强制下一次规划不复用旧路径
                    stopRobot();
                    fsm_.transitionTo(nav_core::NavState::PLANNING);
                    return;
                }
            }
        }
        
        // 超时检测：如果长时间无进展，触发恢复
        double time_since_progress = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_progress_time_).count();
        
        if (time_since_progress > controller_timeout_) {
            RCLCPP_ERROR(get_logger(), 
                "❌ Controller: 控制超时 (%.1f秒无进展 > %.1f秒阈值)", 
                time_since_progress, controller_timeout_);
            stopRobot();
            fsm_.triggerRecovery(nav_core::RecoveryTrigger::CONTROL_FAILED);
            return;
        }
        
        // 进展检测：计算距离上次进展位置的距离
        double dx = current_pose_.pose.position.x - last_progress_pose_.pose.position.x;
        double dy = current_pose_.pose.position.y - last_progress_pose_.pose.position.y;
        double dist_moved = std::hypot(dx, dy);
        
        if (dist_moved > controller_progress_threshold_) {
            // 有明显进展，更新时间戳和位置
            last_progress_time_ = std::chrono::steady_clock::now();
            last_progress_pose_ = current_pose_;
        }
        
        geometry_msgs::msg::Twist cmd;
        auto result = controller_.computeVelocity(current_pose_, cmd);
        
        if (control_count_ % 20 == 0) {
            RCLCPP_INFO(get_logger(), "🎮 Controller: result=%d, v=%.2f, ω=%.2f, no_progress=%.1fs",
                static_cast<int>(result), cmd.linear.x, cmd.angular.z, time_since_progress);
        }
        
        switch (result) {
            case nav_core::ControlResult::RUNNING:
                cmd_vel_pub_->publish(cmd);
                break;
            case nav_core::ControlResult::SUCCEEDED:
                stopRobot();
                RCLCPP_INFO(get_logger(), "✅ Controller: 到达目标!");
                fsm_.transitionTo(nav_core::NavState::SUCCEEDED);
                break;
            case nav_core::ControlResult::FAILED:
                RCLCPP_WARN(get_logger(), "❌ Controller: 控制失败");
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
        stopRobot();  
        
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();
        
        if (goal_handle_) {
            auto result = std::make_shared<Navigate::Result>();
            result->success = true;
            result->total_time = elapsed;
            goal_handle_->succeed(result);
            goal_handle_ = nullptr;
        }
        
        rviz_goal_active_ = false;
        fsm_.transitionTo(nav_core::NavState::IDLE);
        RCLCPP_INFO(get_logger(), "导航成功 (%.2fs)", elapsed);
    }
    
    void finishFailure() {
        stopRobot();  // 🚨 安全第一：立即停止机器人
        
        if (goal_handle_) {
            auto result = std::make_shared<Navigate::Result>();
            result->success = false;
            result->message = "恢复失败";
            goal_handle_->abort(result);
            goal_handle_ = nullptr;
        }
        
        rviz_goal_active_ = false;
        fsm_.transitionTo(nav_core::NavState::IDLE);
        RCLCPP_ERROR(get_logger(), "导航失败");
    }
    
    void publishFeedback() {
        if (!goal_handle_) {
            return;
        }
        
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
    nav_components::NMPC controller_;  // 控制器: NMPC (可替换为 PurePursuit)
    nav_components::RecoveryManager recovery_mgr_;
    
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> goal_handle_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_layer_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;      // 控制循环
    rclcpp::TimerBase::SharedPtr map_pub_timer_;      // 地图发布（可视化）
    
    // 分层地图管理器：管理静态层、动态层、融合地图、ESDF
    std::shared_ptr<nav_components::LayeredMapManager> map_manager_;
    
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Path current_path_;
    
    std::string map_file_, map_frame_, base_frame_, odom_frame_;
    std::string dynamic_layer_topic_;
    double control_rate_, goal_timeout_, goal_tolerance_, yaw_tolerance_;
    double controller_timeout_;  // 控制器无进展超时阈值(秒)
    double controller_progress_threshold_;  // 进展检测距离阈值(米)
    double esdf_vis_max_dist_;
    bool enable_esdf_;
    bool enable_static_layer_;   // 是否启用静态地图层
    bool enable_dynamic_layer_;  // 是否启用动态障碍物层
    bool rviz_goal_active_ = false;  // RViz 目标激活标志
    nav_components::InflationParams inflation_params_;  // 膨胀参数缓存
    
    // 路径检查相关
    double path_check_period_ = 2.0;  // 路径检查周期(秒)
    double path_lateral_tolerance_ = 0.5;  // 横向偏离路径容忍度(米)
    std::chrono::steady_clock::time_point last_path_check_wall_time_;  // 使用wall clock
    
    // 脱困系统参数
    int escape_trigger_costmap_threshold_;  // Costmap 触发阈值
    double escape_target_safe_esdf_;        // 目标 ESDF 距离
    double escape_search_radius_;           // BFS 搜索半径
    double escape_rotation_speed_;          // 旋转速度
    double escape_forward_speed_;           // 前进速度
    double escape_yaw_tolerance_;           // 角度容差
    double escape_position_tolerance_;      // 位置容差
    double escape_timeout_;                 // 超时时长
    
    std::chrono::steady_clock::time_point start_time_;           // 导航开始时间(wall clock)
    std::chrono::steady_clock::time_point last_progress_time_;   // 上次有进展的时间(wall clock)
    std::chrono::steady_clock::time_point escape_start_time_;    // 脱困开始时间(wall clock)
    geometry_msgs::msg::PoseStamped last_progress_pose_;  // 上次有进展的位置
    int control_count_ = 0;  // 控制循环计数器（用于调试日志）
    
    // 脱困相关状态
    double escape_target_x_ = -1.0;  // 脱困目标点 X（-1表示未设置）
    double escape_target_y_ = -1.0;  // 脱困目标点 Y
    int escape_phase_ = 0;  // 脱困阶段：0=旋转, 1=前进
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavServer>());
    rclcpp::shutdown();
    return 0;
}
