/**
 * @file localization_initializer_node.cpp
 * @brief 定位初始化节点 - 使用 NDT+GICP 三阶段配准实现全局定位
 * 
 * 工作流程:
 *   1. 加载官方地图 (PCD 文件)
 *   2. 等待用户在 RViz 中提供初始位姿估计 (/initialpose)
 *   3. 订阅当前激光扫描 (/cloud_registered)
 *   4. 执行三阶段配准:
 *      - NDT 粗配准 (resolution=1.0, 快速搜索)
 *      - NDT 精配准 (resolution=0.5, 提升精度)
 *      - GICP 优化 (各向异性匹配, 最终优化)
 *   5. 发布 map → odom 静态 TF
 *   6. 发布可视化信息 (配准结果、质量评分)
 * 
 * @author navigation2026 team
 * @date 2026-02-08
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>  // 🔒 多线程保护

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <algorithm>
#include <numeric>
#include <cmath>

class LocalizationInitializer : public rclcpp::Node {
public:
    LocalizationInitializer() : Node("localization_initializer") {
        // ==================== 参数声明 ====================
        declare_parameter("map_file", "");
        declare_parameter("initial_x", 0.0);
        declare_parameter("initial_y", 0.0);
        declare_parameter("initial_z", 0.0);
        declare_parameter("initial_yaw", 0.0);
        
        // NDT 参数 (Magnusson 博士论文优化 - 三阶段纯 NDT)
        declare_parameter("ndt_stage1_resolution", 1.0);     // 阶段1: 粗配准
        declare_parameter("ndt_stage2_resolution", 0.5);     // 阶段2: 中配准
        declare_parameter("ndt_stage3_resolution", 0.25);    // 阶段3: 精配准
        declare_parameter("ndt_max_iterations", 100);
        declare_parameter("ndt_transformation_epsilon", 1e-6);  // 论文建议值
        declare_parameter("ndt_step_size", 0.1);                // 论文建议值
        declare_parameter("ndt_threads", 8);
        declare_parameter("ndt_outlier_ratio", 0.55);           // 论文建议 0.5-0.6
        declare_parameter("ndt_enable_linked_cells", true);     // 网格链接 (DIRECT7)
        declare_parameter("ndt_enable_trilinear_interp", false); // 三线性插值加权（基于邻域体素）
        
        // GICP 参数 (可选,默认禁用)
        declare_parameter("enable_gicp", false);                // 禁用 GICP
        
        // 质量评估阈值
        declare_parameter("ndt_fitness_threshold", 1.0);
        
        // 降采样参数 (VoxelGrid - 空间均匀分布)
        declare_parameter("map_voxel_size", 0.1);
        declare_parameter("scan_voxel_size", 0.05);
        
        // 多帧累积参数
        declare_parameter("accumulate_scans", false);           // 是否启用多帧累积
        declare_parameter("accumulate_frames", 3);              // 累积帧数
        declare_parameter("accumulate_duration", 0.0);          // 累积时长（秒），0表示按帧数
        declare_parameter("accumulate_voxel_size", 0.1);        // 累积点云降采样
        
        // 点云空间过滤参数（按坐标轴过滤）
        declare_parameter("spatial_filter_enable", false);      // 是否启用空间过滤
        declare_parameter("filter_x_min", -100.0);              // X 轴最小值（米），过滤机器人后方
        declare_parameter("filter_x_max", 100.0);               // X 轴最大值（米），过滤机器人前方
        declare_parameter("filter_y_min", -100.0);              // Y 轴最小值（米），过滤左侧
        declare_parameter("filter_y_max", 100.0);               // Y 轴最大值（米），过滤右侧
        declare_parameter("filter_z_min", -1.0);                // Z 轴最小值（米），过滤地面/低处
        declare_parameter("filter_z_max", 3.0);                 // Z 轴最大值（米），过滤天花板
        
        // Yaw 多假设验证参数
        declare_parameter("yaw_seed_enable", false);
        declare_parameter("yaw_seed_half_range_deg", 15.0);
        declare_parameter("yaw_seed_step_deg", 2.0);
        declare_parameter("yaw_seed_ndt_resolution", 1.5);
        declare_parameter("yaw_seed_ndt_max_iter", 10);
        declare_parameter("yaw_seed_fallback_threshold", 5.0);
        
        // 自动初始化（如果不使用 RViz）
        declare_parameter("auto_initialize", false);
        
        // ==================== 读取参数 ====================
        map_file_ = get_parameter("map_file").as_string();
        auto_initialize_ = get_parameter("auto_initialize").as_bool();
        
        if (auto_initialize_) {
            user_initial_guess_(0, 3) = get_parameter("initial_x").as_double();
            user_initial_guess_(1, 3) = get_parameter("initial_y").as_double();
            user_initial_guess_(2, 3) = get_parameter("initial_z").as_double();
            double yaw = get_parameter("initial_yaw").as_double();
            Eigen::AngleAxisf rotation(yaw, Eigen::Vector3f::UnitZ());
            user_initial_guess_.block<3,3>(0,0) = rotation.toRotationMatrix();
            has_initial_guess_ = true;
        }
        
        // ==================== 订阅与发布 ====================
        scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&LocalizationInitializer::scanCallback, this, std::placeholders::_1)
        );
        
        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&LocalizationInitializer::initialPoseCallback, this, std::placeholders::_1)
        );
        
        // 发布配准结果可视化
        aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization/aligned_scan", 10
        );
        
        map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization/map_cloud", rclcpp::QoS(10).transient_local()
        );
        
        // 发布特征提取后的点云（用于可视化调试）
        feature_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization/feature_cloud", 10
        );
        
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/localization/status", 10
        );
        
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/localization/fitness_marker", 10
        );
        
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // ==================== 加载地图 ====================
        if (map_file_.empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 参数 'map_file' 未设置！");
            return;
        }
        
        if (!loadMapCloud()) {
            RCLCPP_ERROR(get_logger(), "❌ 地图加载失败，节点退出");
            return;
        }
        
        // ==================== 定时发布地图点云 ====================
        // ⚠️ 必须在地图加载成功后再启动定时器，避免空指针访问
        map_publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LocalizationInitializer::publishMapCloud, this)
        );
        
        publishMapCloud();
        
        RCLCPP_INFO(get_logger(), "✅ 定位初始化节点启动成功");
        logNdtSearchConfig("启动时");
        if (!auto_initialize_) {
            RCLCPP_INFO(get_logger(), "⏳ 请在 RViz 中使用 '2D Pose Estimate' 工具设置初始位姿");
        } else {
            RCLCPP_INFO(get_logger(), "🤖 自动初始化模式: [%.2f, %.2f, %.2f°]",
                       user_initial_guess_(0,3), user_initial_guess_(1,3),
                       get_parameter("initial_yaw").as_double() * 180.0 / M_PI);
        }
    }
    
    // ==================== 析构函数 ====================
    ~LocalizationInitializer() {
        // 停止定时器，避免在析构过程中触发回调
        if (map_publish_timer_) {
            map_publish_timer_->cancel();
        }

        RCLCPP_INFO(get_logger(), " 定位初始化节点已安全关闭");
    }
    
private:
    // ==================== 地图加载 ====================
    bool loadMapCloud() {
        RCLCPP_INFO(get_logger(), "📦 正在加载地图: %s", map_file_.c_str());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file_, *raw_map) == -1) {
            RCLCPP_ERROR(get_logger(), "❌ 无法加载地图文件: %s", map_file_.c_str());
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "   原始点云: %zu 点", raw_map->size());
        
        // 降采样
        double voxel_size = get_parameter("map_voxel_size").as_double();
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.setInputCloud(raw_map);
        
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        voxel_filter.filter(*map_cloud_);
        
        RCLCPP_INFO(get_logger(), "   降采样后: %zu 点 (voxel_size=%.2fm)", 
                   map_cloud_->size(), voxel_size);
        RCLCPP_INFO(get_logger(), "✅ 地图加载成功");
        
        // 发布初始 map → odom TF（单位变换），使 RViz 能够显示地图
        publishInitialMapFrame();
        
        return true;
    }
    
    // ==================== 发布初始 map frame ====================
    void publishInitialMapFrame() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";
        
        // 单位变换（identity）
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(tf_msg);
        
        RCLCPP_INFO(get_logger(), "📍 发布初始 map → odom TF（单位变换），RViz 可显示地图");
    }
    
    // ==================== 发布地图点云（用于 RViz 可视化）====================
    void publishMapCloud() {
        if (!map_cloud_ || map_cloud_->empty()) {
            return;  // 地图未加载，跳过
        }
        
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud_, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = now();
        map_cloud_pub_->publish(map_msg);
        
        // 只在第一次发布时输出日志
        static bool first_publish = true;
        if (first_publish) {
            RCLCPP_INFO(get_logger(), "📍 地图点云已发布到 /localization/map_cloud (frame: map)");
            RCLCPP_INFO(get_logger(), "   💡 地图会每秒持续发布，确保 RViz 能收到");
            first_publish = false;
        }
    }
    
    // ==================== 初始位姿回调 ====================
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        bool was_initialized = localization_initialized_;
        if (was_initialized) {
            RCLCPP_WARN(get_logger(), "🔄 收到新的初始位姿，重置初始化状态并重新配准");
        }

        // 无论是否已完成初始化，只要收到新的初始位姿，就重新开始当前会话的配准流程
        localization_initialized_ = false;
        clearScanBuffer();

        user_initial_guess_ = poseToMatrix(msg->pose.pose);
        has_initial_guess_ = true;
        
        double yaw = getYawFromQuaternion(msg->pose.pose.orientation);
        RCLCPP_INFO(get_logger(), "📍 收到初始位姿估计: [%.2f, %.2f, %.2f°]",
                   msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   yaw * 180.0 / M_PI);
        
        if (was_initialized) {
            publishStatus("🔄 已接收新的初始位姿，开始重新定位...");
        } else {
            publishStatus("初始位姿已接收，等待扫描数据...");
        }
    }
    
    // ==================== 扫描回调 ====================
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (localization_initialized_) {
            return;  // 已完成初始化，不再处理
        }
        
        if (!has_initial_guess_) {
            static auto last_warn = now();
            if ((now() - last_warn).seconds() > 2.0) {
                RCLCPP_WARN(get_logger(), "⏳ 等待初始位姿 (RViz '2D Pose Estimate' 或设置 auto_initialize)");
                last_warn = now();
            }
            return;
        }
        
        // 转换点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *scan_cloud);
        
        // 检查是否启用多帧累积
        bool accumulate_enabled = get_parameter("accumulate_scans").as_bool();
        
        if (accumulate_enabled) {
            // 累积模式
            handleScanAccumulation(scan_cloud, msg->header.stamp);
        } else {
            // 单帧模式（原有逻辑）
            processSingleScan(scan_cloud);
        }
    }
    
    // ==================== 处理单帧扫描（原有逻辑）====================
    void processSingleScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud) {
        RCLCPP_INFO(get_logger(), "📡 收到扫描数据，开始配准...");
        publishStatus("正在执行配准...");
        
        // 降采样
        double scan_voxel = get_parameter("scan_voxel_size").as_double();
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setLeafSize(scan_voxel, scan_voxel, scan_voxel);
        voxel_filter.setInputCloud(scan_cloud);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        voxel_filter.filter(*scan_downsampled);
        
        RCLCPP_INFO(get_logger(), "   📦 降采样: %zu → %zu 点", scan_cloud->size(), scan_downsampled->size());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud = scan_downsampled;

        // 仅进行空间过滤（移除特征提取与墙壁过滤）
        processed_cloud = applySpatialFilter(processed_cloud);
        
        // Yaw 多假设验证（在三阶段 NDT 前搜索最优 yaw）
        Eigen::Matrix4f optimized_guess = yawSeedSearch(processed_cloud, user_initial_guess_);
        
        // 执行单假设配准
        Eigen::Matrix4f final_pose;
        bool success = performRegistration(processed_cloud, optimized_guess, final_pose);
        
        // 执行配准
        if (success) {
            RCLCPP_INFO(get_logger(), "🎉 ========================================");
            RCLCPP_INFO(get_logger(), "🎉 定位初始化完成！开始发布结果...");
            
            // 发布 TF
            publishMapToOdomTF(final_pose);
            RCLCPP_INFO(get_logger(), "   ✅ map → odom TF 已发布");
            
            // 发布配准后的点云（蓝色点云）
            publishAlignedCloud(processed_cloud, final_pose);
            RCLCPP_INFO(get_logger(), "   ✅ 配准点云已发布到 /localization/aligned_scan");
            
            localization_initialized_ = true;
            
            RCLCPP_INFO(get_logger(), "🎉 系统已准备好导航。");
            RCLCPP_INFO(get_logger(), "🎉 ========================================");
            
            publishStatus("✅ 定位成功！map → odom TF 已发布");
            publishFitnessMarker(final_pose, true);
        } else {
            RCLCPP_ERROR(get_logger(), "❌ 配准失败，请调整初始位姿后重试");
            publishStatus("❌ 配准失败，请重新设置初始位姿");
            has_initial_guess_ = false;  // 允许重试
        }
    }
    
    // ==================== 处理多帧累积 ====================
    void handleScanAccumulation(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud,
        const rclcpp::Time& timestamp
    ) {
        // ✅ 单帧过滤: 先过滤再累积(避免运动拖影)
        auto filtered_scan = filterSingleFrame(scan_cloud);
        
        if (!filtered_scan || filtered_scan->empty()) {
            RCLCPP_WARN(get_logger(), "⚠️ 单帧过滤后为空,跳过该帧");
            return;
        }
        
        bool should_process = false;
        std::string accumulation_info;
        
        // 🔒 临界区: 只在访问 scan_buffer_ 时持锁
        {
            std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
            
            // 记录第一帧时间
            if (scan_buffer_.empty()) {
                accumulation_start_time_ = timestamp;
                RCLCPP_INFO(get_logger(), "📦 开始累积点云...");
            }
            
            // 添加已过滤的点云到缓冲区
            scan_buffer_.push_back(filtered_scan);
            
            // 计算累积时长
            double elapsed = (timestamp - accumulation_start_time_).seconds();
            
            // 检查是否满足累积条件
            double accumulate_duration = get_parameter("accumulate_duration").as_double();
            int accumulate_frames = get_parameter("accumulate_frames").as_int();
            
            if (accumulate_duration > 0.0) {
                // 按时长累积
                should_process = (elapsed >= accumulate_duration);
                accumulation_info = std::to_string(elapsed) + "s/" + 
                                  std::to_string(accumulate_duration) + "s";
            } else {
                // 按帧数累积
                should_process = (scan_buffer_.size() >= static_cast<size_t>(accumulate_frames));
                accumulation_info = std::to_string(scan_buffer_.size()) + "/" + 
                                  std::to_string(accumulate_frames) + " 帧";
            }
            
            // 简化日志: 只在开始、每10帧、完成时输出
            if (!should_process) {
                size_t frame_count = scan_buffer_.size();
                if (frame_count == 1 || frame_count % 10 == 0) {
                    RCLCPP_INFO(get_logger(), "   📦 累积中: %s", accumulation_info.c_str());
                }
            } else {
                RCLCPP_INFO(get_logger(), "📦 累积完成 (%s)，开始合并与配准...", accumulation_info.c_str());
            }
        }
        // 🔓 锁已释放,可以调用 mergeAccumulatedScans()
        
        if (!should_process) {
            publishStatus("累积点云中: " + accumulation_info);
            return;
        }
        
        // ✅ mergeAccumulatedScans() 内部已完成:
        //    1. 原始合并
        //    2. 空间过滤 (去除地面/天花板)
        //    3. 降采样
        auto processed_cloud = mergeAccumulatedScans();
        
        if (!processed_cloud || processed_cloud->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 点云合并/过滤失败");
            clearScanBuffer();
            has_initial_guess_ = false;
            return;
        }
        
        RCLCPP_INFO(get_logger(), "   ✅ 点云处理完成: %zu 点 (已完成空间过滤+降采样)", 
                   processed_cloud->size());
        publishStatus("开始配准 (使用处理后的累积点云)...");
        
        // Yaw 多假设验证（在三阶段 NDT 前搜索最优 yaw）
        Eigen::Matrix4f optimized_guess = yawSeedSearch(processed_cloud, user_initial_guess_);
        
        // 执行单假设配准
        Eigen::Matrix4f final_pose;
        bool success = performRegistration(processed_cloud, optimized_guess, final_pose);
        
        if (success) {
            publishMapToOdomTF(final_pose);
            publishAlignedCloud(processed_cloud, final_pose);  // 使用实际配准的点云
            localization_initialized_ = true;
            
            RCLCPP_INFO(get_logger(), "🎉 ========================================");
            RCLCPP_INFO(get_logger(), "🎉 定位初始化完成！系统已准备好导航。");
            RCLCPP_INFO(get_logger(), "🎉 ========================================");
            
            publishStatus("✅ 定位成功！map → odom TF 已发布");
            publishFitnessMarker(final_pose, true);
        } else {
            RCLCPP_ERROR(get_logger(), "❌ 配准失败，请调整初始位姿后重试");
            publishStatus("❌ 配准失败，请重新设置初始位姿");
            
            // 清空缓冲区，允许重新累积
            clearScanBuffer();
            has_initial_guess_ = false;
        }
    }
    
    // ==================== 合并累积的点云 ====================
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergeAccumulatedScans() {
        // 🔒 深拷贝缓冲区,避免在合并过程中被回调修改
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> local_buffer;
        {
            std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
            if (scan_buffer_.empty()) {
                return nullptr;
            }
            local_buffer = scan_buffer_;  // 拷贝 shared_ptr vector (浅拷贝指针)
        }
        
        // 创建合并后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
        
        // 简单合并所有点云（假设在同一坐标系下）
        for (const auto& scan : local_buffer) {
            *merged += *scan;
        }
        
        RCLCPP_INFO(get_logger(), "   ✅ 合并原始点云: %zu 帧 → %zu 点", 
                   local_buffer.size(), merged->size());
        
        // ✅ 在累积后应用空间过滤
        RCLCPP_INFO(get_logger(), "   🔍 应用空间过滤（累积后）...");
        auto spatially_filtered = applySpatialFilter(merged);
        
        if (!spatially_filtered || spatially_filtered->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 空间过滤后点云为空！");
            return nullptr;
        }
        
        RCLCPP_INFO(get_logger(), "   ✅ 空间过滤: %zu → %zu 点", 
                   merged->size(), spatially_filtered->size());
        
        // ❌ 移除动态物体与曲率特征提取逻辑（不再需要）
        pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessed = spatially_filtered;
        pcl::PointCloud<pcl::PointXYZI>::Ptr feature_enhanced = preprocessed;
        
        // 🔄 优化顺序3: 最后降采样（去除重复点）
        double voxel_size = get_parameter("accumulate_voxel_size").as_double();
        
        RCLCPP_INFO(get_logger(), "   准备降采样: %zu 点", feature_enhanced->size());
        
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.setInputCloud(feature_enhanced);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        RCLCPP_INFO(get_logger(), "   执行降采样...");
        try {
            voxel_filter.filter(*final_cloud);
            RCLCPP_INFO(get_logger(), "   ✅ 降采样完成: %zu → %zu 点 (voxel=%.2fm)", 
                       feature_enhanced->size(), final_cloud->size(), voxel_size);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "❌ 降采样异常: %s", e.what());
            return feature_enhanced;
        }
        
        RCLCPP_INFO(get_logger(), "   清空缓冲区...");
        clearScanBuffer();
        
        RCLCPP_INFO(get_logger(), "   ✅ 合并完成，返回点云");
        return final_cloud;
    }
    
    // ==================== 计算 Fitness Score ====================
    double computeFitnessScore(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud,
        const Eigen::Matrix4f& pose
    ) {
        // 方法1：手动计算点到地图的平均距离（更准确）
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*scan_cloud, *transformed, pose);
        
        // 使用 KD-Tree 查找最近邻
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(map_cloud_);
        
        double total_distance = 0.0;
        int valid_points = 0;
        const double max_correspondence_dist = 1.0;  // 最大对应距离
        
        for (const auto& point : transformed->points) {
            std::vector<int> indices(1);
            std::vector<float> distances(1);
            
            if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
                if (distances[0] < max_correspondence_dist * max_correspondence_dist) {
                    total_distance += std::sqrt(distances[0]);
                    valid_points++;
                }
            }
        }
        
        if (valid_points == 0) {
            return std::numeric_limits<double>::max();  // 没有有效对应点
        }
        
        return total_distance / valid_points;  // 返回平均距离
    }
    
    // ==================== 空间过滤（按坐标轴过滤）====================
    pcl::PointCloud<pcl::PointXYZI>::Ptr applySpatialFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud
    ) {
        bool filter_enabled = get_parameter("spatial_filter_enable").as_bool();
        
        if (!filter_enabled) {
            // 未启用过滤，直接返回原点云
            return cloud;
        }
        
        double x_min = get_parameter("filter_x_min").as_double();
        double x_max = get_parameter("filter_x_max").as_double();
        double y_min = get_parameter("filter_y_min").as_double();
        double y_max = get_parameter("filter_y_max").as_double();
        double z_min = get_parameter("filter_z_min").as_double();
        double z_max = get_parameter("filter_z_max").as_double();
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(cloud->size());
        
        size_t removed_x = 0;
        size_t removed_y = 0;
        size_t removed_z = 0;
        
        for (const auto& point : cloud->points) {
            bool keep = true;
            
            // X 轴过滤（前后方向）
            if (point.x < x_min || point.x > x_max) {
                removed_x++;
                keep = false;
            }
            
            // Y 轴过滤（左右方向）
            if (keep && (point.y < y_min || point.y > y_max)) {
                removed_y++;
                keep = false;
            }
            
            // Z 轴过滤（高度方向）
            if (keep && (point.z < z_min || point.z > z_max)) {
                removed_z++;
                keep = false;
            }
            
            // 保留在有效范围内的点
            if (keep) {
                filtered->push_back(point);
            }
        }
        
        // 简化日志: 只在DEBUG模式输出详细信息
        RCLCPP_DEBUG(get_logger(), "   🔍 空间过滤: %zu → %zu 点", 
                    cloud->size(), filtered->size());
        RCLCPP_DEBUG(get_logger(), "      X 范围: [%.1f, %.1f]m, Y 范围: [%.1f, %.1f]m, Z 范围: [%.1f, %.1f]m",
                    x_min, x_max, y_min, y_max, z_min, z_max);
        
        if (removed_x > 0) {
            RCLCPP_DEBUG(get_logger(), "      过滤 X 轴外点: %zu", removed_x);
        }
        
        if (removed_y > 0) {
            RCLCPP_DEBUG(get_logger(), "      过滤 Y 轴外点: %zu", removed_y);
        }
        
        if (removed_z > 0) {
            RCLCPP_DEBUG(get_logger(), "      过滤 Z 轴外点: %zu (天花板/地面)", removed_z);
        }
        
        // 检查过滤后点云是否太少
        if (filtered->size() < 100) {
            RCLCPP_WARN(get_logger(), "⚠️ 过滤后点云过少 (%zu 点)，可能影响配准质量", 
                       filtered->size());
            RCLCPP_WARN(get_logger(), "   建议: 放宽过滤范围参数");
        }
        
        return filtered;
    }
    
    // ==================== 单帧过滤（禁用，累积后再处理）====================
    // 直接返回原始点云，不做任何处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterSingleFrame(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud
    ) {
        // 不做任何过滤，直接返回原始点云
        return scan_cloud;
    }
    
    // ==================== Yaw 多假设验证 (Yaw Seed Search) ====================
    /**
     * @brief 在 [yaw0 - Δ, yaw0 + Δ] 范围内搜索最优 yaw 角
     * 
     * 方法: 对每个候选 yaw，用粗分辨率 NDT 跑少量迭代，取 fitness score 最低的作为最优。
     * 仅修改 initial_guess 的旋转部分（绕 Z 轴），平移不变。
     * 
     * @param scan_cloud    预处理后的扫描点云
     * @param initial_guess 用户提供的初始位姿（含 XY 平移 + 初始 yaw）
     * @return 优化后的 initial_guess（yaw 已替换为最优候选）
     */
    Eigen::Matrix4f yawSeedSearch(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud,
        const Eigen::Matrix4f& initial_guess
    ) {
        bool enable = get_parameter("yaw_seed_enable").as_bool();
        if (!enable) {
            return initial_guess;
        }
        
        double half_range_deg = get_parameter("yaw_seed_half_range_deg").as_double();
        double step_deg = get_parameter("yaw_seed_step_deg").as_double();
        double seed_resolution = get_parameter("yaw_seed_ndt_resolution").as_double();
        int seed_max_iter = get_parameter("yaw_seed_ndt_max_iter").as_int();
        
        // 从初始位姿提取当前 yaw
        Eigen::Matrix3f rot = initial_guess.block<3,3>(0,0);
        Eigen::Quaternionf q_init(rot);
        double center_yaw = std::atan2(
            2.0 * (q_init.w() * q_init.z() + q_init.x() * q_init.y()),
            1.0 - 2.0 * (q_init.y() * q_init.y() + q_init.z() * q_init.z())
        );
        
        // 提取平移部分（保持不变）
        Eigen::Vector3f translation = initial_guess.block<3,1>(0,3);
        
        double half_range_rad = half_range_deg * M_PI / 180.0;
        double step_rad = step_deg * M_PI / 180.0;
        
        int num_candidates = static_cast<int>(std::ceil(2.0 * half_range_rad / step_rad)) + 1;
        
        RCLCPP_INFO(get_logger(), "🔄 ========== Yaw 多假设验证 ==========");
        RCLCPP_INFO(get_logger(), "   中心 yaw=%.1f°, 范围=[%.1f°, %.1f°], 步长=%.1f°, 候选数=%d",
                   center_yaw * 180.0 / M_PI,
                   (center_yaw - half_range_rad) * 180.0 / M_PI,
                   (center_yaw + half_range_rad) * 180.0 / M_PI,
                   step_deg, num_candidates);
        RCLCPP_INFO(get_logger(), "   评分 NDT: resolution=%.2fm, max_iter=%d",
                   seed_resolution, seed_max_iter);
        
        auto search_start = std::chrono::high_resolution_clock::now();
        
        // 创建一个粗分辨率 NDT 用于快速评分
        // NDT 对象只创建一次，每次只改 initial_guess
        logNdtSearchConfig("Yaw-seed 评分");
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_seed(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()
        );
        ndt_seed->setResolution(seed_resolution);
        ndt_seed->setNumThreads(get_parameter("ndt_threads").as_int());
        ndt_seed->setNeighborhoodSearchMethod(getNdtSearchMethod());
        ndt_seed->setEnableTrilinearInterpolation(get_parameter("ndt_enable_trilinear_interp").as_bool());
        ndt_seed->setMaximumIterations(seed_max_iter);
        ndt_seed->setTransformationEpsilon(0.01);  // 宽松收敛，快速退出
        ndt_seed->setStepSize(0.2);                // 大步长，粗略搜索
        ndt_seed->setOulierRatio(get_parameter("ndt_outlier_ratio").as_double());
        ndt_seed->setInputTarget(map_cloud_);
        ndt_seed->setInputSource(scan_cloud);
        
        double best_score = std::numeric_limits<double>::max();
        double best_yaw = center_yaw;
        int best_idx = -1;
        
        // 存储所有候选的分数用于日志
        struct YawCandidate {
            double yaw_rad;
            double fitness;
        };
        std::vector<YawCandidate> candidates;
        candidates.reserve(num_candidates);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_tmp(new pcl::PointCloud<pcl::PointXYZI>());
        
        for (double yaw = center_yaw - half_range_rad; 
             yaw <= center_yaw + half_range_rad + 1e-6;
             yaw += step_rad)
        {
            // 构造候选 pose（只改 yaw，保持 XY 平移不变）
            Eigen::Matrix4f candidate_pose = Eigen::Matrix4f::Identity();
            candidate_pose(0, 0) = std::cos(yaw);
            candidate_pose(0, 1) = -std::sin(yaw);
            candidate_pose(1, 0) = std::sin(yaw);
            candidate_pose(1, 1) = std::cos(yaw);
            candidate_pose.block<3,1>(0,3) = translation;
            
            // 用粗 NDT 跑少量迭代
            ndt_seed->align(*aligned_tmp, candidate_pose);
            
            // 取 NDT 内部 fitness score（越小越好）
            double score = ndt_seed->getFitnessScore();
            
            candidates.push_back({yaw, score});
            
            if (score < best_score) {
                best_score = score;
                best_yaw = yaw;
                best_idx = static_cast<int>(candidates.size()) - 1;
            }
        }
        
        auto search_end = std::chrono::high_resolution_clock::now();
        double search_ms = std::chrono::duration<double, std::milli>(search_end - search_start).count();
        
        // 打印 top-5 候选
        std::vector<size_t> sorted_indices(candidates.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::partial_sort(sorted_indices.begin(), 
                         sorted_indices.begin() + std::min<size_t>(5, sorted_indices.size()),
                         sorted_indices.end(),
                         [&](size_t a, size_t b) { return candidates[a].fitness < candidates[b].fitness; });
        
        RCLCPP_INFO(get_logger(), "   📊 Top-5 候选:");
        for (size_t i = 0; i < std::min<size_t>(5, sorted_indices.size()); ++i) {
            auto& c = candidates[sorted_indices[i]];
            RCLCPP_INFO(get_logger(), "      %s yaw=%+.1f° fitness=%.4f",
                       (sorted_indices[i] == static_cast<size_t>(best_idx)) ? "★" : " ",
                       c.yaw_rad * 180.0 / M_PI, c.fitness);
        }
        
        double yaw_shift = (best_yaw - center_yaw) * 180.0 / M_PI;
        RCLCPP_INFO(get_logger(), "   ✅ 最优 yaw=%.1f° (偏移 %+.1f°), fitness=%.4f, 耗时=%.0fms",
                   best_yaw * 180.0 / M_PI, yaw_shift, best_score, search_ms);
        RCLCPP_INFO(get_logger(), "🔄 ========================================");
        
        // ===== 置信度检查：若最优 score 仍超过阈值，说明评分不可信，回退到原始 yaw =====
        double fallback_threshold = get_parameter("yaw_seed_fallback_threshold").as_double();
        if (best_score > fallback_threshold) {
            RCLCPP_WARN(get_logger(), 
                "⚠️  Yaw-seed 评分不可信 (best_score=%.4f > threshold=%.4f)，回退到用户原始 yaw=%.1f°",
                best_score, fallback_threshold, center_yaw * 180.0 / M_PI);
            RCLCPP_WARN(get_logger(),
                "   可能原因: 点云质量差 / 车位置偏差过大 / seed_ndt_resolution 过粗");
            RCLCPP_WARN(get_logger(),
                "   建议: 查看 Top-5 正常 fitness 值后调整 yaw_seed_fallback_threshold");
            return initial_guess;  // 原样返回用户给的初始位姿
        }
        
        // 构造最优 initial_guess
        Eigen::Matrix4f best_guess = Eigen::Matrix4f::Identity();
        best_guess(0, 0) = std::cos(best_yaw);
        best_guess(0, 1) = -std::sin(best_yaw);
        best_guess(1, 0) = std::sin(best_yaw);
        best_guess(1, 1) = std::cos(best_yaw);
        best_guess.block<3,1>(0,3) = translation;
        
        return best_guess;
    }
    
    // ==================== 三阶段配准 ====================
    bool performRegistration(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud,
        const Eigen::Matrix4f& initial_guess,
        Eigen::Matrix4f& final_pose
    ) {
        // 🔧 安全检查 1: 验证输入点云
        if (!scan_cloud || scan_cloud->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ [SAFETY] 输入点云为空或未初始化！");
            return false;
        }
        
        // 🔧 安全检查 2: 验证目标地图
        if (!map_cloud_ || map_cloud_->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ [SAFETY] 目标地图为空或未初始化！");
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "✅ [SAFETY] 安全检查通过: scan=%zu点, map=%zu点",
                   scan_cloud->size(), map_cloud_->size());
        logNdtSearchConfig("三阶段配准");
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // ===== 阶段 0: 评估初始位姿质量 =====
        double initial_fitness = computeFitnessScore(scan_cloud, initial_guess);
        RCLCPP_INFO(get_logger(), "📊 初始位姿质量评估: fitness=%.4f", initial_fitness);
        
        // ===== 阶段 1: NDT 粗配准 (1.0m 捕获大偏差) =====
        RCLCPP_INFO(get_logger(), "🔍 阶段 1/3: NDT 粗配准 (分辨率=%.2fm)...", 
                   get_parameter("ndt_stage1_resolution").as_double());
        
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_stage1(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()
        );
        
        // 📚 Magnusson 论文优化: 迭代网格 + 网格链接
        ndt_stage1->setResolution(get_parameter("ndt_stage1_resolution").as_double());
        ndt_stage1->setNumThreads(get_parameter("ndt_threads").as_int());
        
        // 🔹 网格链接 (Linked Cells) - 论文建议!
        // 作用: 点落在空格子时,搜索最近的有效格子 (鲁棒性↑15%, 耗时仅↑2%)
        ndt_stage1->setNeighborhoodSearchMethod(getNdtSearchMethod());
        ndt_stage1->setEnableTrilinearInterpolation(get_parameter("ndt_enable_trilinear_interp").as_bool());
        
        // 🔹 离群点处理 - 论文建议 0.5-0.6
        ndt_stage1->setOulierRatio(get_parameter("ndt_outlier_ratio").as_double());
        
        // 🔧 固定使用配置文件参数（不做动态调整）
        ndt_stage1->setMaximumIterations(get_parameter("ndt_max_iterations").as_int());
        ndt_stage1->setTransformationEpsilon(get_parameter("ndt_transformation_epsilon").as_double());
        ndt_stage1->setStepSize(get_parameter("ndt_step_size").as_double());
        
        RCLCPP_INFO(get_logger(), "   🔧 NDT 参数: iters=%ld, eps=%.1e, step=%.2f",
                   get_parameter("ndt_max_iterations").as_int(),
                   get_parameter("ndt_transformation_epsilon").as_double(),
                   get_parameter("ndt_step_size").as_double());
        
        ndt_stage1->setInputTarget(map_cloud_);
        ndt_stage1->setInputSource(scan_cloud);
        
        RCLCPP_INFO(get_logger(), "   🔧 NDT 输入: source=%zu点, target=%zu点", 
                   scan_cloud->size(), map_cloud_->size());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr stage1_aligned(new pcl::PointCloud<pcl::PointXYZI>());
        
        RCLCPP_INFO(get_logger(), "   🚀 开始 NDT 配准...");
        try {
            ndt_stage1->align(*stage1_aligned, initial_guess);
            RCLCPP_INFO(get_logger(), "   ✅ NDT align 执行完成");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "   ❌ NDT 配准异常: %s", e.what());
            return false;
        }
        
        if (!ndt_stage1->hasConverged()) {
            RCLCPP_ERROR(get_logger(), "   ❌ NDT 阶段1未收敛");
            return false;
        }
        
        double stage1_score = ndt_stage1->getFitnessScore();
        Eigen::Matrix4f stage1_pose = ndt_stage1->getFinalTransformation();
        
        RCLCPP_INFO(get_logger(), "   ✅ 收敛: fitness=%.3f, iters=%d", 
                   stage1_score, ndt_stage1->getFinalNumIteration());
        
        // 检查配准是否让结果变差
        double stage1_fitness = computeFitnessScore(scan_cloud, stage1_pose);
        RCLCPP_INFO(get_logger(), "   📊 阶段1后几何质量: %.4f (初始: %.4f)", 
                   stage1_fitness, initial_fitness);
        
        if (stage1_score > get_parameter("ndt_fitness_threshold").as_double()) {
            RCLCPP_WARN(get_logger(), "   ⚠️ 阶段1质量不佳 (fitness=%.3f > %.1f)，继续尝试...",
                       stage1_score, get_parameter("ndt_fitness_threshold").as_double());
        }
        
        // ===== 阶段 2: NDT 中配准 (0.5m 进一步对齐) =====
        RCLCPP_INFO(get_logger(), "🔍 阶段 2/3: NDT 中配准 (分辨率=%.2fm)...",
                   get_parameter("ndt_stage2_resolution").as_double());
        
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_stage2(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()
        );
        
        // 📚 Magnusson 论文优化: 中配准阶段参数
        ndt_stage2->setResolution(get_parameter("ndt_stage2_resolution").as_double());
        ndt_stage2->setNumThreads(get_parameter("ndt_threads").as_int());
        ndt_stage2->setNeighborhoodSearchMethod(getNdtSearchMethod());
        ndt_stage2->setEnableTrilinearInterpolation(get_parameter("ndt_enable_trilinear_interp").as_bool());
        ndt_stage2->setOulierRatio(get_parameter("ndt_outlier_ratio").as_double());
        
        // 🔧 固定使用配置文件参数（不做动态调整）
        ndt_stage2->setMaximumIterations(get_parameter("ndt_max_iterations").as_int());
        ndt_stage2->setTransformationEpsilon(get_parameter("ndt_transformation_epsilon").as_double());
        ndt_stage2->setStepSize(get_parameter("ndt_step_size").as_double());
        
        ndt_stage2->setInputTarget(map_cloud_);
        ndt_stage2->setInputSource(scan_cloud);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr stage2_aligned(new pcl::PointCloud<pcl::PointXYZI>());
        ndt_stage2->align(*stage2_aligned, stage1_pose);
        
        // 🎯 强制使用阶段2结果（无论收敛状态和精度如何）
        double stage2_score = ndt_stage2->getFitnessScore();
        Eigen::Matrix4f stage2_pose = ndt_stage2->getFinalTransformation();
        
        RCLCPP_INFO(get_logger(), "   %s 收敛: fitness=%.3f, iters=%d",
                   ndt_stage2->hasConverged() ? "✅" : "⚠️",
                   stage2_score, ndt_stage2->getFinalNumIteration());
        
        // 计算并显示精度变化，但无条件采用阶段2结果
        double stage2_fitness = computeFitnessScore(scan_cloud, stage2_pose);
        RCLCPP_INFO(get_logger(), "   📊 阶段2后几何质量: %.4f (阶段1: %.4f)",
                   stage2_fitness, stage1_fitness);
        
        // 🔥 强制采用阶段2结果
        final_pose = stage2_pose;
        if (stage2_fitness < stage1_fitness) {
            RCLCPP_INFO(get_logger(), "   ✅ 阶段2改善了结果 (Δ=%.4f)", stage1_fitness - stage2_fitness);
        } else {
            RCLCPP_WARN(get_logger(), "   ⚠️ 阶段2精度变差 (Δ=+%.4f)，但仍强制采用", stage2_fitness - stage1_fitness);
        }
        
        // ===== 阶段 3: NDT 精配准 (0.25m 死磕细节) =====
        if (!get_parameter("enable_gicp").as_bool()) {
            // 使用三阶段纯 NDT
            RCLCPP_INFO(get_logger(), "🔍 阶段 3/3: NDT 精配准 (分辨率=%.2fm)...",
                       get_parameter("ndt_stage3_resolution").as_double());
            
            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_stage3(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()
            );
            
            // 📚 Magnusson 论文优化: 精配准阶段参数
            ndt_stage3->setResolution(get_parameter("ndt_stage3_resolution").as_double());
            ndt_stage3->setNumThreads(get_parameter("ndt_threads").as_int());
            ndt_stage3->setNeighborhoodSearchMethod(getNdtSearchMethod());
            ndt_stage3->setEnableTrilinearInterpolation(get_parameter("ndt_enable_trilinear_interp").as_bool());
            ndt_stage3->setOulierRatio(get_parameter("ndt_outlier_ratio").as_double());
            
            // 🔧 固定使用配置文件参数（不做动态调整）
            ndt_stage3->setMaximumIterations(get_parameter("ndt_max_iterations").as_int());
            ndt_stage3->setTransformationEpsilon(get_parameter("ndt_transformation_epsilon").as_double());
            ndt_stage3->setStepSize(get_parameter("ndt_step_size").as_double());
            
            ndt_stage3->setInputTarget(map_cloud_);
            ndt_stage3->setInputSource(scan_cloud);
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr stage3_aligned(new pcl::PointCloud<pcl::PointXYZI>());
            ndt_stage3->align(*stage3_aligned, final_pose);
            
            // 🎯 强制使用阶段3结果（无论收敛状态和精度如何）
            double stage3_score = ndt_stage3->getFitnessScore();
            Eigen::Matrix4f stage3_pose = ndt_stage3->getFinalTransformation();
            
            RCLCPP_INFO(get_logger(), "   %s 收敛: fitness=%.3f, iters=%d",
                       ndt_stage3->hasConverged() ? "✅" : "⚠️",
                       stage3_score, ndt_stage3->getFinalNumIteration());
            
            // 计算并显示精度变化，但无条件采用阶段3结果
            double stage3_fitness = computeFitnessScore(scan_cloud, stage3_pose);
            double stage2_final_fitness = computeFitnessScore(scan_cloud, final_pose);
            RCLCPP_INFO(get_logger(), "   📊 阶段3后几何质量: %.4f (阶段2: %.4f)",
                       stage3_fitness, stage2_final_fitness);
            
            // 🔥 强制采用阶段3结果
            final_pose = stage3_pose;
            if (stage3_fitness < stage2_final_fitness) {
                RCLCPP_INFO(get_logger(), "   ✅ 阶段3改善了结果 (Δ=%.4f)", stage2_final_fitness - stage3_fitness);
            } else {
                RCLCPP_WARN(get_logger(), "   ⚠️ 阶段3精度变差 (Δ=+%.4f)，但仍强制采用", stage3_fitness - stage2_final_fitness);
            }
            
            // 直接输出 NDT 最终结果
            double ndt_final_fitness = computeFitnessScore(scan_cloud, final_pose);
            RCLCPP_INFO(get_logger(), "📊 三阶段纯 NDT 最终几何质量: %.4f", ndt_final_fitness);
        } else {
            RCLCPP_WARN(get_logger(), "⚠️ GICP 已启用 (不推荐!)，跳过 NDT 阶段3");
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(get_logger(), "⏱️  总耗时: %ld ms", duration.count());
        
        // 输出最终位姿
        Eigen::Vector3f trans = final_pose.block<3,1>(0,3);
        Eigen::Matrix3f rot = final_pose.block<3,3>(0,0);
        Eigen::Quaternionf q(rot);
        double yaw = std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()), 
                               1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
        
        RCLCPP_INFO(get_logger(), "📍 最终位姿: [x=%.3f, y=%.3f, z=%.3f, yaw=%.2f°]",
                   trans.x(), trans.y(), trans.z(), yaw * 180.0 / M_PI);
        
        return true;
        
        
    }
    
    // ==================== 发布 TF ====================
    void publishMapToOdomTF(const Eigen::Matrix4f& T_map_to_odom) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";
        
        tf_msg.transform.translation.x = T_map_to_odom(0, 3);
        tf_msg.transform.translation.y = T_map_to_odom(1, 3);
        tf_msg.transform.translation.z = T_map_to_odom(2, 3);
        
        Eigen::Quaternionf q(T_map_to_odom.block<3,3>(0,0));
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(tf_msg);
        
        RCLCPP_INFO(get_logger(), "🔗 map → odom TF 已发布 (静态)");
    }
    
    // ==================== 发布配准后点云 ====================
    void publishAlignedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
                            const Eigen::Matrix4f& transform) {
        if (!scan || scan->empty()) {
            RCLCPP_WARN(get_logger(), "⚠️ 无法发布点云：输入点云为空");
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*scan, *aligned, transform);
        
        sensor_msgs::msg::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_msg.header.frame_id = "map";
        aligned_msg.header.stamp = now();
        
        aligned_cloud_pub_->publish(aligned_msg);
        
        RCLCPP_INFO(get_logger(), "   📤 配准点云已发布: %zu 点 (frame: map)", aligned->size());
    }
    
    // ==================== 发布特征点云（可视化调试）====================
    void publishFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& feature_cloud) {
        sensor_msgs::msg::PointCloud2 feature_msg;
        pcl::toROSMsg(*feature_cloud, feature_msg);
        feature_msg.header.frame_id = "odom";  // 特征点云在 odom 坐标系
        feature_msg.header.stamp = now();
        
        feature_cloud_pub_->publish(feature_msg);
        
        RCLCPP_DEBUG(get_logger(), "📤 发布特征点云: %zu 点", feature_cloud->size());
    }
    
    // ==================== 发布状态信息 ====================
    void publishStatus(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }
    
    // ==================== 发布质量标记 ====================
    void publishFitnessMarker(const Eigen::Matrix4f& pose, bool success) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "localization";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = pose(0, 3);
        marker.pose.position.y = pose(1, 3);
        marker.pose.position.z = pose(2, 3) + 1.0;
        
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        if (success) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.text = "✓ 定位成功";
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.text = "✗ 定位失败";
        }
        
        marker_pub_->publish(marker);
    }
    
    // ==================== 工具函数 ====================
    Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose& pose) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        
        mat(0, 3) = pose.position.x;
        mat(1, 3) = pose.position.y;
        mat(2, 3) = pose.position.z;
        
        Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
                            pose.orientation.y, pose.orientation.z);
        mat.block<3,3>(0,0) = q.toRotationMatrix();
        
        return mat;
    }
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 mat(tf_q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void clearScanBuffer() {
        std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
        scan_buffer_.clear();
    }

    pclomp::NeighborSearchMethod getNdtSearchMethod() {
        // 说明：
        // 开启三线性插值时使用 DIRECT26 邻域，并在 pclomp 内部启用加权评分。
        if (get_parameter("ndt_enable_trilinear_interp").as_bool()) {
            return pclomp::DIRECT26;
        }
        if (get_parameter("ndt_enable_linked_cells").as_bool()) {
            return pclomp::DIRECT7;
        }
        return pclomp::DIRECT1;
    }

    std::string getNdtSearchMethodName() {
        auto method = getNdtSearchMethod();
        switch (method) {
            case pclomp::DIRECT26:
                return "DIRECT26 (trilinear-weighted)";
            case pclomp::DIRECT7:
                return "DIRECT7 (linked-cells)";
            case pclomp::DIRECT1:
                return "DIRECT1";
            case pclomp::KDTREE:
                return "KDTREE";
            default:
                return "UNKNOWN";
        }
    }

    void logNdtSearchConfig(const std::string& stage) {
        bool trilinear_interp = get_parameter("ndt_enable_trilinear_interp").as_bool();
        bool linked_cells = get_parameter("ndt_enable_linked_cells").as_bool();
        RCLCPP_INFO(
            get_logger(),
            "🔧 [%s] NDT 邻域配置: ndt_enable_trilinear_interp=%s, ndt_enable_linked_cells=%s -> 生效模式=%s",
            stage.c_str(),
            trilinear_interp ? "true" : "false",
            linked_cells ? "true" : "false",
            getNdtSearchMethodName().c_str()
        );
    }
    
    // ==================== 成员变量 ====================
    std::string map_file_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
    
    Eigen::Matrix4f user_initial_guess_ = Eigen::Matrix4f::Identity();
    bool has_initial_guess_ = false;
    bool localization_initialized_ = false;
    bool auto_initialize_ = false;
    
    // 多帧累积缓冲区（需要互斥锁保护并发访问）
    std::mutex scan_buffer_mutex_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_buffer_;
    rclcpp::Time accumulation_start_time_{0, 0, RCL_ROS_TIME};
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;  // 定时发布地图点云
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
