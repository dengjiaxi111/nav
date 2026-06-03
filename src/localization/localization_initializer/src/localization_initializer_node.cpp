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
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
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
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>

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
        declare_parameter("use_last_ndt_result", true);         // true: 始终采用最后一阶段结果
        
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

        // 基于法向量的简易地面过滤
        declare_parameter("ground_filter_enable", false);
        declare_parameter("ground_normal_radius", 0.25);
        declare_parameter("ground_normal_z_threshold", 0.92);
        declare_parameter("ground_max_height", -0.15);
        declare_parameter("ground_max_curvature", 0.15);

        // Multi-radius DBSCAN 动态障碍过滤
        declare_parameter("cluster_filter_enable", false);
        declare_parameter("cluster_near_range", 8.0);
        declare_parameter("cluster_mid_range", 16.0);
        declare_parameter("cluster_near_epsilon", 0.18);
        declare_parameter("cluster_mid_epsilon", 0.28);
        declare_parameter("cluster_far_epsilon", 0.40);
        declare_parameter("cluster_near_min_points", 18);
        declare_parameter("cluster_mid_min_points", 12);
        declare_parameter("cluster_far_min_points", 8);
        declare_parameter("cluster_occupancy_voxel_size", 0.10);
        declare_parameter("cluster_dynamic_max_range", 12.0);
        declare_parameter("cluster_dynamic_min_height", 0.25);
        declare_parameter("cluster_dynamic_max_height", 2.20);
        declare_parameter("cluster_dynamic_max_width", 1.40);
        declare_parameter("cluster_dynamic_max_depth", 1.40);
        declare_parameter("cluster_dynamic_max_volume", 3.00);
        declare_parameter("cluster_dynamic_min_density", 40.0);
        declare_parameter("cluster_dynamic_min_occupancy_ratio", 0.015);
        declare_parameter("cluster_dynamic_max_points", 2500);
        declare_parameter("cluster_pre_voxel_size", 0.15);
        declare_parameter("cluster_visualization_enable", true);
        declare_parameter("cluster_label_scale", 0.35);

        // 时序稳定性过滤（多帧累计拖影过滤）
        declare_parameter("temporal_filter_enable", true);
        declare_parameter("temporal_filter_voxel_size", 0.12);
        declare_parameter("temporal_filter_min_hits", 3);
        declare_parameter("temporal_filter_min_hit_ratio", 0.15);
        
        // Yaw 多假设验证参数
        declare_parameter("yaw_seed_enable", false);
        declare_parameter("yaw_seed_half_range_deg", 15.0);
        declare_parameter("yaw_seed_step_deg", 2.0);
        declare_parameter("yaw_seed_ndt_resolution", 1.5);
        declare_parameter("yaw_seed_ndt_max_iter", 10);
        declare_parameter("yaw_seed_fallback_threshold", 5.0);
        declare_parameter("yaw_seed_eval_max_correspondence_dist", 0.60);
        declare_parameter("yaw_seed_eval_trim_ratio", 0.70);
        declare_parameter("yaw_seed_inlier_ratio_weight", 0.35);
        declare_parameter("yaw_seed_ndt_score_weight", 0.03);
        declare_parameter("yaw_seed_min_inlier_ratio", 0.20);
        declare_parameter("yaw_seed_max_trimmed_distance", 0.45);

        declare_parameter("ndt_eval_max_correspondence_dist", 0.60);
        declare_parameter("ndt_eval_trim_ratio", 0.70);
        declare_parameter("ndt_eval_trimmed_distance_weight", 0.73);
        declare_parameter("ndt_eval_inlier_ratio_weight", 0.22);
        declare_parameter("ndt_eval_ndt_score_weight", 0.05);
        
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

        cluster_debug_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization/cluster_debug_cloud", 10
        );

        cluster_label_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/localization/cluster_labels", 10
        );
        
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/localization/status", 10
        );
        
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/localization/fitness_marker", 10
        );
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LocalizationInitializer::publishCurrentMapToOdomTF, this)
        );
        
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
        
        RCLCPP_INFO(get_logger(), "定位初始化节点启动成功");
        logNdtSearchConfig("启动时");
        if (!auto_initialize_) {
            RCLCPP_INFO(get_logger(), " 请在 RViz 中使用 '2D Pose Estimate' 工具设置初始位姿");
        } else {
            RCLCPP_INFO(get_logger(), "自动初始化模式: [%.2f, %.2f, %.2f°]",
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
    struct DbscanParams {
        double epsilon;
        int min_points;
    };

    struct ClusterMetrics {
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        float min_z;
        float max_z;
        double width;
        double depth;
        double height;
        double volume;
        double density;
        double occupancy_ratio;
        double centroid_range;
        double centroid_x;
        double centroid_y;
        double centroid_z;
        std::size_t point_count;
    };

    struct AlignmentQualityMetrics {
        double trimmed_distance = std::numeric_limits<double>::max();
        double inlier_ratio = 0.0;
        int valid_points = 0;
    };

    std::size_t makeVoxelKey(std::int64_t vx, std::int64_t vy, std::int64_t vz) const {
        std::size_t key = std::hash<std::int64_t>{}(vx);
        key ^= std::hash<std::int64_t>{}(vy) + 0x9e3779b97f4a7c15ULL + (key << 6) + (key >> 2);
        key ^= std::hash<std::int64_t>{}(vz) + 0x9e3779b97f4a7c15ULL + (key << 6) + (key >> 2);
        return key;
    }

    std::size_t makeVoxelKey(const pcl::PointXYZI& point, double voxel_size) const {
        std::int64_t vx = static_cast<std::int64_t>(std::floor(point.x / voxel_size));
        std::int64_t vy = static_cast<std::int64_t>(std::floor(point.y / voxel_size));
        std::int64_t vz = static_cast<std::int64_t>(std::floor(point.z / voxel_size));
        return makeVoxelKey(vx, vy, vz);
    }

    // ==================== 地图加载 ====================
    bool loadMapCloud() {
        RCLCPP_INFO(get_logger(), " 正在加载地图: %s", map_file_.c_str());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file_, *raw_map) == -1) {
            RCLCPP_ERROR(get_logger(), " 无法加载地图文件: %s", map_file_.c_str());
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
        RCLCPP_INFO(get_logger(), " 地图加载成功");
        
        // 发布初始 map → odom TF（单位变换），使 RViz 能够显示地图
        publishInitialMapFrame();
        
        return true;
    }
    
    // ==================== 发布初始 map frame ====================
    void publishInitialMapFrame() {
        current_map_to_odom_tf_.header.frame_id = "map";
        current_map_to_odom_tf_.child_frame_id = "odom";
        
        // 单位变换（identity）
        current_map_to_odom_tf_.transform.translation.x = 0.0;
        current_map_to_odom_tf_.transform.translation.y = 0.0;
        current_map_to_odom_tf_.transform.translation.z = 0.0;
        current_map_to_odom_tf_.transform.rotation.x = 0.0;
        current_map_to_odom_tf_.transform.rotation.y = 0.0;
        current_map_to_odom_tf_.transform.rotation.z = 0.0;
        current_map_to_odom_tf_.transform.rotation.w = 1.0;
        has_map_to_odom_tf_ = true;

        publishCurrentMapToOdomTF();
        
        RCLCPP_INFO(get_logger(), " 发布初始 map → odom TF（单位变换，动态发布），RViz 可显示地图");
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
            RCLCPP_INFO(get_logger(), " 地图点云已发布到 /localization/map_cloud (frame: map)");
            RCLCPP_INFO(get_logger(), "    地图会每秒持续发布，确保 RViz 能收到");
            first_publish = false;
        }
    }
    
    // ==================== 初始位姿回调 ====================
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        bool was_initialized = localization_initialized_;
        if (was_initialized) {
            RCLCPP_WARN(get_logger(), " 收到新的初始位姿，重置初始化状态并重新配准");
        }

        // 无论是否已完成初始化，只要收到新的初始位姿，就重新开始当前会话的配准流程
        localization_initialized_ = false;
        clearScanBuffer();

        Eigen::Matrix4f T_map_to_base = poseToMatrix(msg->pose.pose);
        user_initial_guess_ = T_map_to_base;

        // RViz 的 /initialpose 表示 map -> base_link。
        // NDT 对齐的是 odom 系下的 /cloud_registered，因此初值必须是 map -> odom：
        //   T_map_base = T_map_odom * T_odom_base
        //   T_map_odom = T_map_base * inverse(T_odom_base)
        try {
            auto odom_to_base_msg = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
            Eigen::Matrix4f T_odom_to_base = transformToMatrix(odom_to_base_msg.transform);
            user_initial_guess_ = T_map_to_base * T_odom_to_base.inverse();
            RCLCPP_INFO(get_logger(), " 已将 /initialpose 从 map->base_link 换算为 map->odom 初值");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                get_logger(),
                "⚠️ 查询 odom->base_link 失败，回退为原始 /initialpose 作为初值: %s",
                ex.what());
        }
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
        if (!processed_cloud || processed_cloud->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 降采样后点云为空，无法执行配准");
            publishStatus("❌ 点云为空，无法执行配准");
            return;
        }
        publishFeatureCloud(processed_cloud);
        
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
        // ✅ 单帧不做过滤，直接累积，统一在合并后预处理
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
            
            // 添加原始单帧点云到缓冲区
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
        //    2. 空间/地面过滤
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
        publishFeatureCloud(processed_cloud);
        
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

        auto temporally_filtered = applyTemporalStabilityFilter(local_buffer, "累积后");
        if (!temporally_filtered || temporally_filtered->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 时序稳定性过滤后点云为空！");
            return nullptr;
        }
        
        auto spatially_filtered = preprocessCloudForRegistration(temporally_filtered, true, "累积后");
        
        if (!spatially_filtered || spatially_filtered->empty()) {
            RCLCPP_ERROR(get_logger(), "❌ 累积后预处理后的点云为空！");
            return nullptr;
        }
        
        RCLCPP_INFO(get_logger(), "   ✅ 累积后预处理: %zu → %zu 点", 
                   temporally_filtered->size(), spatially_filtered->size());
        
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

    AlignmentQualityMetrics evaluateAlignmentQuality(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud,
        const Eigen::Matrix4f& pose,
        const pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
        double max_correspondence_dist,
        double trim_ratio
    ) {
        AlignmentQualityMetrics metrics;
        if (!scan_cloud || scan_cloud->empty()) {
            return metrics;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*scan_cloud, *transformed, pose);

        const double max_dist_sq = max_correspondence_dist * max_correspondence_dist;
        std::vector<double> valid_distances;
        valid_distances.reserve(transformed->size());

        for (const auto& point : transformed->points) {
            std::vector<int> indices(1);
            std::vector<float> distances(1);

            if (kdtree.nearestKSearch(point, 1, indices, distances) > 0 &&
                distances[0] < max_dist_sq) {
                valid_distances.push_back(std::sqrt(distances[0]));
            }
        }

        metrics.valid_points = static_cast<int>(valid_distances.size());
        metrics.inlier_ratio = static_cast<double>(metrics.valid_points) /
                               static_cast<double>(std::max<std::size_t>(1, transformed->size()));

        if (valid_distances.empty()) {
            metrics.trimmed_distance = max_correspondence_dist * 10.0;
            return metrics;
        }

        std::sort(valid_distances.begin(), valid_distances.end());
        double clamped_trim_ratio = std::clamp(trim_ratio, 0.1, 1.0);
        std::size_t trimmed_count = static_cast<std::size_t>(
            std::ceil(valid_distances.size() * clamped_trim_ratio));
        trimmed_count = std::max<std::size_t>(1, std::min(trimmed_count, valid_distances.size()));

        double trimmed_sum = std::accumulate(
            valid_distances.begin(),
            valid_distances.begin() + static_cast<std::ptrdiff_t>(trimmed_count),
            0.0
        );
        metrics.trimmed_distance = trimmed_sum / static_cast<double>(trimmed_count);
        return metrics;
    }

    double computeCombinedAlignmentScore(
        const AlignmentQualityMetrics& metrics,
        double ndt_score,
        double trimmed_distance_weight,
        double inlier_ratio_weight,
        double ndt_score_weight
    ) {
        return trimmed_distance_weight * metrics.trimmed_distance
             - inlier_ratio_weight * metrics.inlier_ratio
             + ndt_score_weight * ndt_score;
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
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterSingleFrame(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_cloud
    ) {
        return scan_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessCloudForRegistration(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        bool enable_cluster_filter,
        const std::string& stage_name
    ) {
        if (!input_cloud || input_cloud->empty()) {
            return input_cloud;
        }

        auto filtered = applySpatialFilter(input_cloud);
        if (!filtered || filtered->empty()) {
            RCLCPP_WARN(get_logger(), "⚠️ [%s] 空间过滤后点云为空", stage_name.c_str());
            return filtered;
        }

        filtered = removeGroundByNormals(filtered, stage_name);
        if (!filtered || filtered->empty()) {
            RCLCPP_WARN(get_logger(), "⚠️ [%s] 地面过滤后点云为空", stage_name.c_str());
            return filtered;
        }

        if (enable_cluster_filter) {
            filtered = filterDynamicObstaclesWithMultiRadiusDbscan(filtered, stage_name);
        }

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr applyTemporalStabilityFilter(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& frames,
        const std::string& stage_name
    ) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
        if (frames.empty()) {
            return merged;
        }

        for (const auto& frame : frames) {
            if (frame) {
                *merged += *frame;
            }
        }

        if (!get_parameter("temporal_filter_enable").as_bool() || frames.size() <= 1) {
            return merged;
        }

        double voxel_size = std::max(0.02, get_parameter("temporal_filter_voxel_size").as_double());
        int min_hits = static_cast<int>(get_parameter("temporal_filter_min_hits").as_int());
        double min_hit_ratio = get_parameter("temporal_filter_min_hit_ratio").as_double();
        int frame_count = static_cast<int>(frames.size());
        int required_hits = std::max(min_hits, static_cast<int>(std::ceil(frame_count * min_hit_ratio)));
        required_hits = std::min(required_hits, frame_count);
        required_hits = std::max(1, required_hits);

        std::unordered_map<std::size_t, int> voxel_hits;
        voxel_hits.reserve(merged->size());

        for (const auto& frame : frames) {
            if (!frame || frame->empty()) {
                continue;
            }

            std::unordered_set<std::size_t> frame_voxels;
            frame_voxels.reserve(frame->size());
            for (const auto& point : frame->points) {
                frame_voxels.insert(makeVoxelKey(point, voxel_size));
            }

            for (const auto& voxel_key : frame_voxels) {
                ++voxel_hits[voxel_key];
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(merged->size());

        std::size_t removed_points = 0;
        for (const auto& frame : frames) {
            if (!frame) {
                continue;
            }

            for (const auto& point : frame->points) {
                auto it = voxel_hits.find(makeVoxelKey(point, voxel_size));
                if (it != voxel_hits.end() && it->second >= required_hits) {
                    filtered->push_back(point);
                } else {
                    ++removed_points;
                }
            }
        }

        std::size_t stable_voxels = 0;
        for (const auto& [_, hits] : voxel_hits) {
            if (hits >= required_hits) {
                ++stable_voxels;
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "   [%s] 时序稳定性过滤: frames=%d, voxel=%.2f, hits>=%d (min_hits=%d, ratio=%.2f) -> %zu → %zu 点, stable_voxels=%zu, removed=%zu",
            stage_name.c_str(),
            frame_count,
            voxel_size,
            required_hits,
            min_hits,
            min_hit_ratio,
            merged->size(),
            filtered->size(),
            stable_voxels,
            removed_points
        );

        if (filtered->empty()) {
            RCLCPP_WARN(get_logger(), "⚠️ [%s] 时序稳定性过滤后为空，回退到原始合并点云", stage_name.c_str());
            return merged;
        }

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr removeGroundByNormals(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::string& stage_name
    ) {
        if (!get_parameter("ground_filter_enable").as_bool()) {
            return cloud;
        }

        if (!cloud || cloud->size() < 30) {
            return cloud;
        }

        double radius = get_parameter("ground_normal_radius").as_double();
        double normal_z_threshold = get_parameter("ground_normal_z_threshold").as_double();
        double ground_max_height = get_parameter("ground_max_height").as_double();
        double ground_max_curvature = get_parameter("ground_max_curvature").as_double();

        pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(cloud);
        normal_estimation.setRadiusSearch(radius);
        int normal_threads = static_cast<int>(get_parameter("ndt_threads").as_int());
        normal_estimation.setNumberOfThreads(std::max(1, normal_threads));

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        normal_estimation.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        normal_estimation.compute(*normals);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(cloud->size());

        std::size_t removed_ground = 0;
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto& point = cloud->points[i];
            const auto& normal = normals->points[i];

            bool normal_valid = std::isfinite(normal.normal_x) &&
                                std::isfinite(normal.normal_y) &&
                                std::isfinite(normal.normal_z) &&
                                std::isfinite(normal.curvature);
            bool looks_like_ground = normal_valid &&
                                     std::abs(normal.normal_z) >= normal_z_threshold &&
                                     normal.curvature <= ground_max_curvature &&
                                     point.z <= ground_max_height;

            if (looks_like_ground) {
                ++removed_ground;
                continue;
            }

            filtered->push_back(point);
        }

        RCLCPP_INFO(
            get_logger(),
            "   [%s] 法向量地面过滤: %zu → %zu 点 (移除 %zu, radius=%.2f, |nz|>=%.2f, z<=%.2f)",
            stage_name.c_str(),
            cloud->size(),
            filtered->size(),
            removed_ground,
            radius,
            normal_z_threshold,
            ground_max_height
        );

        return filtered;
    }

    DbscanParams getDbscanParamsForRange(double range) {
        if (range <= get_parameter("cluster_near_range").as_double()) {
            return {
                get_parameter("cluster_near_epsilon").as_double(),
                static_cast<int>(get_parameter("cluster_near_min_points").as_int())
            };
        }

        if (range <= get_parameter("cluster_mid_range").as_double()) {
            return {
                get_parameter("cluster_mid_epsilon").as_double(),
                static_cast<int>(get_parameter("cluster_mid_min_points").as_int())
            };
        }

        return {
            get_parameter("cluster_far_epsilon").as_double(),
            static_cast<int>(get_parameter("cluster_far_min_points").as_int())
        };
    }

    ClusterMetrics computeClusterMetrics(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::vector<int>& indices
    ) {
        ClusterMetrics metrics{};
        metrics.min_x = std::numeric_limits<float>::max();
        metrics.min_y = std::numeric_limits<float>::max();
        metrics.min_z = std::numeric_limits<float>::max();
        metrics.max_x = std::numeric_limits<float>::lowest();
        metrics.max_y = std::numeric_limits<float>::lowest();
        metrics.max_z = std::numeric_limits<float>::lowest();
        metrics.point_count = indices.size();

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double voxel_size = std::max(0.02, get_parameter("cluster_occupancy_voxel_size").as_double());
        std::unordered_set<std::size_t> occupied_voxels;
        occupied_voxels.reserve(indices.size());

        for (int idx : indices) {
            const auto& p = cloud->points[idx];
            metrics.min_x = std::min(metrics.min_x, p.x);
            metrics.max_x = std::max(metrics.max_x, p.x);
            metrics.min_y = std::min(metrics.min_y, p.y);
            metrics.max_y = std::max(metrics.max_y, p.y);
            metrics.min_z = std::min(metrics.min_z, p.z);
            metrics.max_z = std::max(metrics.max_z, p.z);
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;

            occupied_voxels.insert(makeVoxelKey(p, voxel_size));
        }

        metrics.width = std::max(0.0, static_cast<double>(metrics.max_x - metrics.min_x));
        metrics.depth = std::max(0.0, static_cast<double>(metrics.max_y - metrics.min_y));
        metrics.height = std::max(0.0, static_cast<double>(metrics.max_z - metrics.min_z));
        metrics.volume = std::max(1e-6, metrics.width * metrics.depth * std::max(metrics.height, 0.05));
        metrics.density = static_cast<double>(metrics.point_count) / metrics.volume;

        double centroid_x = sum_x / static_cast<double>(metrics.point_count);
        double centroid_y = sum_y / static_cast<double>(metrics.point_count);
        double centroid_z = sum_z / static_cast<double>(metrics.point_count);
        metrics.centroid_range = std::hypot(centroid_x, centroid_y);
        metrics.centroid_x = centroid_x;
        metrics.centroid_y = centroid_y;
        metrics.centroid_z = centroid_z;

        int grid_x = std::max(1, static_cast<int>(std::ceil(metrics.width / voxel_size)));
        int grid_y = std::max(1, static_cast<int>(std::ceil(metrics.depth / voxel_size)));
        int grid_z = std::max(1, static_cast<int>(std::ceil(std::max(metrics.height, 0.05) / voxel_size)));
        double total_voxels = static_cast<double>(grid_x) * grid_y * grid_z;
        metrics.occupancy_ratio =
            total_voxels > 0.0 ? static_cast<double>(occupied_voxels.size()) / total_voxels : 0.0;

        return metrics;
    }

    bool isDynamicObstacleCluster(const ClusterMetrics& metrics) {
        if (metrics.centroid_range > get_parameter("cluster_dynamic_max_range").as_double()) {
            return false;
        }

        if (metrics.point_count > static_cast<std::size_t>(get_parameter("cluster_dynamic_max_points").as_int())) {
            return false;
        }

        if (metrics.height < get_parameter("cluster_dynamic_min_height").as_double() ||
            metrics.height > get_parameter("cluster_dynamic_max_height").as_double()) {
            return false;
        }

        if (metrics.width > get_parameter("cluster_dynamic_max_width").as_double() ||
            metrics.depth > get_parameter("cluster_dynamic_max_depth").as_double()) {
            return false;
        }

        if (metrics.volume > get_parameter("cluster_dynamic_max_volume").as_double()) {
            return false;
        }

        if (metrics.density < get_parameter("cluster_dynamic_min_density").as_double()) {
            return false;
        }

        if (metrics.occupancy_ratio < get_parameter("cluster_dynamic_min_occupancy_ratio").as_double()) {
            return false;
        }

        return true;
    }

    void publishClusterDebugVisualization(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::vector<std::vector<int>>& clusters,
        const std::vector<bool>& dynamic_cluster_mask
    ) {
        if (!get_parameter("cluster_visualization_enable").as_bool()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
            for (int point_idx : clusters[cluster_id]) {
                pcl::PointXYZI point = cloud->points[point_idx];
                point.intensity = dynamic_cluster_mask[cluster_id]
                    ? static_cast<float>(1000 + cluster_id)
                    : static_cast<float>(cluster_id + 1);
                cluster_cloud->push_back(point);
            }
        }

        pcl::toROSMsg(*cluster_cloud, cluster_msg);
        cluster_msg.header.frame_id = "odom";
        cluster_msg.header.stamp = now();
        cluster_debug_cloud_pub_->publish(cluster_msg);

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "odom";
        clear_marker.header.stamp = now();
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        double label_scale = get_parameter("cluster_label_scale").as_double();
        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
            if (clusters[cluster_id].empty()) {
                continue;
            }

            ClusterMetrics metrics = computeClusterMetrics(cloud, clusters[cluster_id]);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "odom";
            text_marker.header.stamp = now();
            text_marker.ns = "cluster_labels";
            text_marker.id = static_cast<int>(cluster_id);
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.orientation.w = 1.0;
            text_marker.pose.position.x = metrics.centroid_x;
            text_marker.pose.position.y = metrics.centroid_y;
            text_marker.pose.position.z = metrics.max_z + 0.25;
            text_marker.scale.z = label_scale;
            text_marker.color.a = 1.0;
            if (dynamic_cluster_mask[cluster_id]) {
                text_marker.color.r = 1.0;
                text_marker.color.g = 0.2;
                text_marker.color.b = 0.2;
                text_marker.text = "D" + std::to_string(cluster_id);
            } else {
                text_marker.color.r = 0.1;
                text_marker.color.g = 0.9;
                text_marker.color.b = 1.0;
                text_marker.text = std::to_string(cluster_id);
            }
            marker_array.markers.push_back(text_marker);
        }

        cluster_label_pub_->publish(marker_array);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterDynamicObstaclesWithMultiRadiusDbscan(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::string& stage_name
    ) {
        if (!get_parameter("cluster_filter_enable").as_bool()) {
            return cloud;
        }

        if (!cloud || cloud->size() < 50) {
            return cloud;
        }

        double dynamic_max_range = get_parameter("cluster_dynamic_max_range").as_double();
        double cluster_pre_voxel_size = std::max(0.05, get_parameter("cluster_pre_voxel_size").as_double());

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustering_roi(new pcl::PointCloud<pcl::PointXYZI>());
        clustering_roi->reserve(cloud->size());
        for (const auto& point : cloud->points) {
            if (std::hypot(point.x, point.y) <= dynamic_max_range) {
                clustering_roi->push_back(point);
            }
        }

        if (clustering_roi->size() < 30) {
            RCLCPP_INFO(
                get_logger(),
                "   [%s] 聚类 ROI 点太少: full=%zu, roi=%zu, range<=%.2fm，跳过 DBSCAN",
                stage_name.c_str(),
                cloud->size(),
                clustering_roi->size(),
                dynamic_max_range
            );
            return cloud;
        }

        pcl::VoxelGrid<pcl::PointXYZI> cluster_voxel_filter;
        cluster_voxel_filter.setLeafSize(cluster_pre_voxel_size, cluster_pre_voxel_size, cluster_pre_voxel_size);
        cluster_voxel_filter.setInputCloud(clustering_roi);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cluster_voxel_filter.filter(*cluster_cloud);

        if (cluster_cloud->size() < 30) {
            RCLCPP_INFO(
                get_logger(),
                "   [%s] 聚类降采样后点太少: roi=%zu, voxel=%.2f, downsampled=%zu，跳过 DBSCAN",
                stage_name.c_str(),
                clustering_roi->size(),
                cluster_pre_voxel_size,
                cluster_cloud->size()
            );
            return cloud;
        }

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cluster_cloud);

        constexpr int kUnassigned = -1;
        constexpr int kNoise = -2;
        std::vector<int> labels(cluster_cloud->size(), kUnassigned);
        std::vector<std::vector<int>> clusters;

        auto expand_cluster = [&](int seed_index, int cluster_id) {
            std::queue<int> frontier;
            frontier.push(seed_index);
            labels[seed_index] = cluster_id;
            clusters[cluster_id].push_back(seed_index);

            while (!frontier.empty()) {
                int current = frontier.front();
                frontier.pop();

                const auto& point = cluster_cloud->points[current];
                DbscanParams params = getDbscanParamsForRange(std::hypot(point.x, point.y));

                std::vector<int> neighbor_indices;
                std::vector<float> neighbor_distances;
                kdtree.radiusSearch(point, params.epsilon, neighbor_indices, neighbor_distances);

                if (static_cast<int>(neighbor_indices.size()) < params.min_points) {
                    continue;
                }

                for (int neighbor_idx : neighbor_indices) {
                    if (labels[neighbor_idx] == kNoise) {
                        labels[neighbor_idx] = cluster_id;
                        clusters[cluster_id].push_back(neighbor_idx);
                    }

                    if (labels[neighbor_idx] != kUnassigned) {
                        continue;
                    }

                    labels[neighbor_idx] = cluster_id;
                    clusters[cluster_id].push_back(neighbor_idx);
                    frontier.push(neighbor_idx);
                }
            }
        };

        for (std::size_t i = 0; i < cluster_cloud->size(); ++i) {
            if (labels[i] != kUnassigned) {
                continue;
            }

            const auto& point = cluster_cloud->points[i];
            DbscanParams params = getDbscanParamsForRange(std::hypot(point.x, point.y));

            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;
            kdtree.radiusSearch(point, params.epsilon, neighbor_indices, neighbor_distances);

            if (static_cast<int>(neighbor_indices.size()) < params.min_points) {
                labels[i] = kNoise;
                continue;
            }

            clusters.emplace_back();
            int cluster_id = static_cast<int>(clusters.size()) - 1;
            expand_cluster(static_cast<int>(i), cluster_id);
        }

        std::vector<bool> remove_mask(cloud->size(), false);
        std::size_t removed_points = 0;
        std::size_t dynamic_cluster_count = 0;
        std::vector<bool> dynamic_cluster_mask(clusters.size(), false);
        std::unordered_set<std::size_t> dynamic_cluster_voxels;

        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
            const auto& cluster_indices = clusters[cluster_id];
            if (cluster_indices.empty()) {
                continue;
            }

            ClusterMetrics metrics = computeClusterMetrics(cluster_cloud, cluster_indices);
            if (!isDynamicObstacleCluster(metrics)) {
                continue;
            }

            ++dynamic_cluster_count;
            dynamic_cluster_mask[cluster_id] = true;
            for (int idx : cluster_indices) {
                dynamic_cluster_voxels.insert(makeVoxelKey(cluster_cloud->points[idx], cluster_pre_voxel_size));
            }
        }

        publishClusterDebugVisualization(cluster_cloud, clusters, dynamic_cluster_mask);

        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto& point = cloud->points[i];
            if (std::hypot(point.x, point.y) > dynamic_max_range) {
                continue;
            }

            if (dynamic_cluster_voxels.count(makeVoxelKey(point, cluster_pre_voxel_size)) > 0) {
                remove_mask[i] = true;
                ++removed_points;
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(cloud->size() - std::min(removed_points, cloud->size()));

        for (std::size_t i = 0; i < cloud->size(); ++i) {
            if (!remove_mask[i]) {
                filtered->push_back(cloud->points[i]);
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "   [%s] Multi-radius DBSCAN: full=%zu, roi=%zu, cluster_downsampled=%zu(voxel=%.2f) -> %zu, clusters=%zu, dynamic_clusters=%zu, removed=%zu",
            stage_name.c_str(),
            cloud->size(),
            clustering_roi->size(),
            cluster_cloud->size(),
            cluster_pre_voxel_size,
            filtered->size(),
            clusters.size(),
            dynamic_cluster_count,
            removed_points
        );

        return filtered;
    }
    
    // ==================== Yaw 多假设验证 (Yaw Seed Search) ====================
    /**
     * @brief 在 [yaw0 - Δ, yaw0 + Δ] 范围内搜索最优 yaw 角
     * 
     * 方法: 对每个候选 yaw，用粗分辨率 NDT 跑少量迭代，再结合 trimmed distance、
     * inlier ratio 与 NDT score 进行组合评分，选最优 yaw。
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
        double eval_max_corr_dist = get_parameter("yaw_seed_eval_max_correspondence_dist").as_double();
        double eval_trim_ratio = get_parameter("yaw_seed_eval_trim_ratio").as_double();
        double inlier_weight = get_parameter("yaw_seed_inlier_ratio_weight").as_double();
        double ndt_weight = get_parameter("yaw_seed_ndt_score_weight").as_double();
        
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

        pcl::KdTreeFLANN<pcl::PointXYZI> map_kdtree;
        map_kdtree.setInputCloud(map_cloud_);
        
        double best_score = std::numeric_limits<double>::max();
        double best_yaw = center_yaw;
        int best_idx = -1;
        
        // 存储所有候选的分数用于日志
        struct YawCandidate {
            double yaw_rad;
            double ndt_score;
            double trimmed_distance;
            double inlier_ratio;
            double combined_score;
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
            double ndt_score = ndt_seed->getFitnessScore();
            AlignmentQualityMetrics quality = evaluateAlignmentQuality(
                scan_cloud, candidate_pose, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
            double score = quality.trimmed_distance
                         - inlier_weight * quality.inlier_ratio
                         + ndt_weight * ndt_score;
            
            candidates.push_back({yaw, ndt_score, quality.trimmed_distance, quality.inlier_ratio, score});
            
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
                         [&](size_t a, size_t b) { return candidates[a].combined_score < candidates[b].combined_score; });
        
        RCLCPP_INFO(get_logger(), "   📊 Top-5 候选:");
        for (size_t i = 0; i < std::min<size_t>(5, sorted_indices.size()); ++i) {
            auto& c = candidates[sorted_indices[i]];
            RCLCPP_INFO(get_logger(), "      %s yaw=%+.1f° score=%.4f trim=%.4f inlier=%.3f ndt=%.4f",
                       (sorted_indices[i] == static_cast<size_t>(best_idx)) ? "★" : " ",
                       c.yaw_rad * 180.0 / M_PI,
                       c.combined_score, c.trimmed_distance, c.inlier_ratio, c.ndt_score);
        }
        
        double yaw_shift = (best_yaw - center_yaw) * 180.0 / M_PI;
        const auto& best_candidate = candidates[static_cast<std::size_t>(best_idx)];
        RCLCPP_INFO(get_logger(), "   ✅ 最优 yaw=%.1f° (偏移 %+.1f°), score=%.4f, trim=%.4f, inlier=%.3f, ndt=%.4f, 耗时=%.0fms",
                   best_yaw * 180.0 / M_PI, yaw_shift, best_score,
                   best_candidate.trimmed_distance, best_candidate.inlier_ratio,
                   best_candidate.ndt_score, search_ms);
        RCLCPP_INFO(get_logger(), "🔄 ========================================");
        
        // ===== 置信度检查：trimmed distance 太差或 inlier ratio 太低时回退 =====
        double fallback_threshold = get_parameter("yaw_seed_fallback_threshold").as_double();
        double min_inlier_ratio = get_parameter("yaw_seed_min_inlier_ratio").as_double();
        double max_trimmed_distance = get_parameter("yaw_seed_max_trimmed_distance").as_double();
        if (best_candidate.trimmed_distance > max_trimmed_distance ||
            best_candidate.inlier_ratio < min_inlier_ratio ||
            best_candidate.ndt_score > fallback_threshold) {
            RCLCPP_WARN(get_logger(), 
                "⚠️  Yaw-seed 评分不可信 (trim=%.4f, inlier=%.3f, ndt=%.4f)，回退到用户原始 yaw=%.1f°",
                best_candidate.trimmed_distance, best_candidate.inlier_ratio,
                best_candidate.ndt_score, center_yaw * 180.0 / M_PI);
            RCLCPP_WARN(get_logger(),
                "   阈值: trim<=%.3f, inlier>=%.3f, ndt<=%.3f",
                max_trimmed_distance, min_inlier_ratio, fallback_threshold);
            RCLCPP_WARN(get_logger(),
                "   可能原因: 点云质量差 / 车位置偏差过大 / seed_ndt_resolution 过粗");
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
        pcl::KdTreeFLANN<pcl::PointXYZI> map_kdtree;
        map_kdtree.setInputCloud(map_cloud_);
        bool use_last_ndt_result = get_parameter("use_last_ndt_result").as_bool();
        double eval_max_corr_dist = get_parameter("ndt_eval_max_correspondence_dist").as_double();
        double eval_trim_ratio = get_parameter("ndt_eval_trim_ratio").as_double();
        double eval_trimmed_weight = get_parameter("ndt_eval_trimmed_distance_weight").as_double();
        double eval_inlier_weight = get_parameter("ndt_eval_inlier_ratio_weight").as_double();
        double eval_ndt_weight = get_parameter("ndt_eval_ndt_score_weight").as_double();
        
        // ===== 阶段 0: 评估初始位姿质量 =====
        AlignmentQualityMetrics initial_quality = evaluateAlignmentQuality(
            scan_cloud, initial_guess, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
        double initial_combined_score = computeCombinedAlignmentScore(
            initial_quality, 0.0, eval_trimmed_weight, eval_inlier_weight, eval_ndt_weight);
        RCLCPP_INFO(get_logger(), "📊 初始位姿质量: score=%.4f trim=%.4f inlier=%.3f",
                   initial_combined_score, initial_quality.trimmed_distance, initial_quality.inlier_ratio);
        
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
        AlignmentQualityMetrics stage1_quality = evaluateAlignmentQuality(
            scan_cloud, stage1_pose, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
        double stage1_combined_score = computeCombinedAlignmentScore(
            stage1_quality, stage1_score, eval_trimmed_weight, eval_inlier_weight, eval_ndt_weight);
        RCLCPP_INFO(get_logger(), "   📊 阶段1后质量: score=%.4f trim=%.4f inlier=%.3f ndt=%.3f (初始 score=%.4f)",
                   stage1_combined_score, stage1_quality.trimmed_distance, stage1_quality.inlier_ratio,
                   stage1_score, initial_combined_score);
        
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
        AlignmentQualityMetrics stage2_quality = evaluateAlignmentQuality(
            scan_cloud, stage2_pose, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
        double stage2_combined_score = computeCombinedAlignmentScore(
            stage2_quality, stage2_score, eval_trimmed_weight, eval_inlier_weight, eval_ndt_weight);
        RCLCPP_INFO(get_logger(), "   📊 阶段2后质量: score=%.4f trim=%.4f inlier=%.3f ndt=%.3f (阶段1 score=%.4f)",
                   stage2_combined_score, stage2_quality.trimmed_distance, stage2_quality.inlier_ratio,
                   stage2_score, stage1_combined_score);
        
        // 🔥 强制采用阶段2结果
        if (use_last_ndt_result || stage2_combined_score < stage1_combined_score) {
            final_pose = stage2_pose;
        } else {
            final_pose = stage1_pose;
        }
        if (stage2_combined_score < stage1_combined_score) {
            RCLCPP_INFO(get_logger(), "   ✅ 阶段2改善了结果 (Δ=%.4f)", stage1_combined_score - stage2_combined_score);
        } else {
            if (use_last_ndt_result) {
                RCLCPP_WARN(get_logger(), "   ⚠️ 阶段2质量变差 (Δ=+%.4f)，但 use_last_ndt_result=true，仍采用阶段2", stage2_combined_score - stage1_combined_score);
            } else {
                RCLCPP_WARN(get_logger(), "   ⚠️ 阶段2质量变差 (Δ=+%.4f)，use_last_ndt_result=false，保留阶段1", stage2_combined_score - stage1_combined_score);
            }
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
            AlignmentQualityMetrics stage3_quality = evaluateAlignmentQuality(
                scan_cloud, stage3_pose, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
            double stage3_combined_score = computeCombinedAlignmentScore(
                stage3_quality, stage3_score, eval_trimmed_weight, eval_inlier_weight, eval_ndt_weight);
            double current_best_score = use_last_ndt_result ? stage2_combined_score
                                                            : std::min(stage1_combined_score, stage2_combined_score);
            Eigen::Matrix4f current_best_pose = final_pose;
            RCLCPP_INFO(get_logger(), "   📊 阶段3后质量: score=%.4f trim=%.4f inlier=%.3f ndt=%.3f (上一最佳 score=%.4f)",
                       stage3_combined_score, stage3_quality.trimmed_distance, stage3_quality.inlier_ratio,
                       stage3_score, current_best_score);
            
            if (use_last_ndt_result || stage3_combined_score < current_best_score) {
                final_pose = stage3_pose;
            } else {
                final_pose = current_best_pose;
            }
            if (stage3_combined_score < current_best_score) {
                RCLCPP_INFO(get_logger(), "   ✅ 阶段3改善了结果 (Δ=%.4f)", current_best_score - stage3_combined_score);
            } else {
                if (use_last_ndt_result) {
                    RCLCPP_WARN(get_logger(), "   ⚠️ 阶段3质量变差 (Δ=+%.4f)，但 use_last_ndt_result=true，仍采用阶段3", stage3_combined_score - current_best_score);
                } else {
                    RCLCPP_WARN(get_logger(), "   ⚠️ 阶段3质量变差 (Δ=+%.4f)，use_last_ndt_result=false，保留前面更优结果", stage3_combined_score - current_best_score);
                }
            }
            
            // 直接输出 NDT 最终结果
            AlignmentQualityMetrics final_quality = evaluateAlignmentQuality(
                scan_cloud, final_pose, map_kdtree, eval_max_corr_dist, eval_trim_ratio);
            double final_ndt_score = (final_pose.isApprox(stage3_pose, 1e-5f)) ? stage3_score
                                  : (final_pose.isApprox(stage2_pose, 1e-5f) ? stage2_score : stage1_score);
            double final_combined_score = computeCombinedAlignmentScore(
                final_quality, final_ndt_score, eval_trimmed_weight, eval_inlier_weight, eval_ndt_weight);
            RCLCPP_INFO(get_logger(), "📊 三阶段纯 NDT 最终质量: score=%.4f trim=%.4f inlier=%.3f ndt=%.3f",
                       final_combined_score, final_quality.trimmed_distance,
                       final_quality.inlier_ratio, final_ndt_score);

            if (final_quality.valid_points == 0 ||
                !std::isfinite(final_quality.trimmed_distance) ||
                !std::isfinite(final_ndt_score)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "❌ 配准质量不可信，拒绝发布 TF: valid=%d/%zu, inlier=%.3f, "
                    "trim=%.4f, ndt=%.3f",
                    final_quality.valid_points,
                    scan_cloud->size(),
                    final_quality.inlier_ratio,
                    final_quality.trimmed_distance,
                    final_ndt_score
                );
                return false;
            }
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
        current_map_to_odom_tf_.header.frame_id = "map";
        current_map_to_odom_tf_.child_frame_id = "odom";
        
        current_map_to_odom_tf_.transform.translation.x = T_map_to_odom(0, 3);
        current_map_to_odom_tf_.transform.translation.y = T_map_to_odom(1, 3);
        current_map_to_odom_tf_.transform.translation.z = T_map_to_odom(2, 3);
        
        Eigen::Quaternionf q(T_map_to_odom.block<3,3>(0,0));
        current_map_to_odom_tf_.transform.rotation.x = q.x();
        current_map_to_odom_tf_.transform.rotation.y = q.y();
        current_map_to_odom_tf_.transform.rotation.z = q.z();
        current_map_to_odom_tf_.transform.rotation.w = q.w();
        has_map_to_odom_tf_ = true;

        publishCurrentMapToOdomTF();
        
        RCLCPP_INFO(get_logger(), "🔗 map → odom TF 已更新 (动态)");
    }

    void publishCurrentMapToOdomTF() {
        if (!has_map_to_odom_tf_) {
            return;
        }

        current_map_to_odom_tf_.header.stamp = now();
        tf_broadcaster_->sendTransform(current_map_to_odom_tf_);
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
        if (!feature_cloud) {
            return;
        }

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

    Eigen::Matrix4f transformToMatrix(const geometry_msgs::msg::Transform& transform) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

        mat(0, 3) = transform.translation.x;
        mat(1, 3) = transform.translation.y;
        mat(2, 3) = transform.translation.z;

        Eigen::Quaternionf q(transform.rotation.w, transform.rotation.x,
                            transform.rotation.y, transform.rotation.z);
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_debug_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_label_pub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped current_map_to_odom_tf_;
    bool has_map_to_odom_tf_ = false;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;  // 定时发布地图点云
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
