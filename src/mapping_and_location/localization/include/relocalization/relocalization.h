#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

//small-gicp库
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp" 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/LU>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

const std::string RESET  = "\033[0m";
const std::string RED    = "\033[31m";
const std::string GREEN  = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE   = "\033[34m";
const std::string PURPLE = "\033[35m";

class Relocalization:public rclcpp::Node
{
public:

    Relocalization();
    
private:
    std::mutex mtx_;
    // TF相关
    std::vector<double> init_pos_;        //重定位用初始位姿
    std::vector<double> init_rot_;

    std::vector<double> lidar_pos_;        //雷达到车体中心的外参
    std::vector<double> lidar_rot_;

    geometry_msgs::msg::TransformStamped odom2init_tf_;
    geometry_msgs::msg::TransformStamped lidar_init2map_tf_; 
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> lidar_init_tf_broadcaster_;
    

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 点云相关
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_cloud_sub_; // lio点云订阅
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr priori_cloud_pub_;
    std::string pcd_path_; 
    std::string file_name_; //点云文件名
    PointCloudXYZI::Ptr priori_cloud_;  //map坐标系下的先验点云
    sensor_msgs::msg::PointCloud2 priori_cloud_msg_; //用于在ros中发布的先验点云
    PointCloudXYZI::Ptr scan_cloud_;  // 来自lio的点云，odom坐标系下
    
    //配准
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_srv_; // 重定位服务
    rclcpp::TimerBase::SharedPtr relocalization_timer_;
    Eigen::Affine3f odom2init_transform_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> ransac_;

    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp_;

    std::deque<PointCloudXYZI::Ptr> cloud_buffer_;
    int max_cloud_num_ = 5;
    std::mutex buffer_mutex_;

    bool use_relocalization_;
    bool converge_once_;
    bool converged_;
    bool is_registration_running_; // 标志位
    bool init_pose_get_;

    private:
    bool auto_relocalization_enabled_;
    bool test_mode_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_relocalization_sub_;


    // 测试用
    PointCloudXYZI::Ptr input_cloud_; //测试配准用的点云
    sensor_msgs::msg::PointCloud2 input_cloud_msg_; //用于在ros中发布的测试配准用点云
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_pub_;
    pcl::VoxelGrid<pcl::PointXYZI> sor_;
    pcl::PassThrough<pcl::PointXYZI> pass_;

    
    bool handle_relocalization_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);  // 服务回调函数
    void auto_relocalization_cb(const std_msgs::msg::Bool::SharedPtr msg);

    void mergeRecentClouds(PointCloudXYZI::Ptr&);
    void cloudRegistration(PointCloudXYZI::Ptr&,PointCloudXYZI::Ptr&);
    void setInitTF(void);
    
    void initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
    void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr);
    void tf_timer_cb(void);
    void relocalization_timer_cb();

    //测试用
    void inputCloudProcess(PointCloudXYZI::Ptr&);
};