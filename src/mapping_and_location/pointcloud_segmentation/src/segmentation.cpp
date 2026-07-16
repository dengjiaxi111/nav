/* 
    PointCloud Segmentation包，负责进行点云分割障碍物提取
*/
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
// PCL specific includes
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;


class Segmentation: public rclcpp::Node
{
public:
    Segmentation()
    : rclcpp::Node("Segmentation")
    {
        this->declare_parameter<double>("sor1_k", 5.0);
        this->declare_parameter<double>("sor1_s", 1.0);
        this->declare_parameter<double>("sor2_k", 10);
        this->declare_parameter<double>("sor2_s", 0.2);
        this->declare_parameter<uint8_t>("segementation_type", 0);

        sor1_k = this->get_parameter("sor1_k").as_double();
        sor1_s = this->get_parameter("sor1_s").as_double();
        sor2_k = this->get_parameter("sor2_k").as_double();
        sor2_s = this->get_parameter("sor2_s").as_double();
        this->get_parameter<uint8_t>("segementation_type",segementation_type_);

        RCLCPP_INFO(this->get_logger(), "sor1_k: %f", sor1_k);
        RCLCPP_INFO(this->get_logger(), "sor1_s: %f", sor1_s);
        RCLCPP_INFO(this->get_logger(), "sor2_k: %f", sor2_k);
        RCLCPP_INFO(this->get_logger(), "sor2_s: %f", sor2_s);
        RCLCPP_INFO(this->get_logger(), "segmentation_type: %d", segementation_type_);
        
        ori_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_registered", 10,
            std::bind(&Segmentation::cloud_cb, this, std::placeholders::_1));

        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 1);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        cloud_frame_1_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_frame_2_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        while (true){
            try{
                livox_right_trans_ = tf_buffer_->lookupTransform("livox_left","base_link",tf2::TimePointZero);
            }
            catch (tf2::TransformException & ex) {
                sleep(0.5);
                continue;
            }
            break;
        }

        if(segementation_type_ == 1) // 使用点云差分法(未测试)
        {
            // 读取先验点云
            if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *priori_cloud_) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "get priori pcd failed, path: %s " , pcd_path_.c_str());
            }
            //源点云降采样
            pcl::VoxelGrid<pcl::PointXYZI> sor_;
            sor_.setInputCloud(priori_cloud_);
            sor_.setLeafSize(0.01f,0.01f,0.01f);
            sor_.filter(*priori_cloud_);

            // 先把点云旋转正
            // extrin_tf_ = tf_buffer_->lookupTransform("map", "base_lidar", tf2::TimePointZero);
            // Eigen::Affine3f ex_trans = Eigen::Affine3f::Identity();
            // Eigen::Quaternionf q(extrin_tf_.transform.rotation.w, extrin_tf_.transform.rotation.x, extrin_tf_.transform.rotation.y, extrin_tf_.transform.rotation.z);
            // ex_trans.rotate(q);
            // ex_trans.pretranslate(Eigen::Vector3f(extrin_tf_.transform.translation.x, extrin_tf_.transform.translation.y, extrin_tf_.transform.translation.z ));

            // pcl::transformPointCloud(*priori_cloud_,*priori_cloud_,ex_trans);

            
            pcl::copyPointCloud(*priori_cloud_, *priori_cloud_xyz_);
            kdtree_.setInputCloud(priori_cloud_xyz_);
            RCLCPP_INFO(this->get_logger(), "get priori pcd success!");
            RCLCPP_INFO(this->get_logger(), "point num: %ld" , priori_cloud_->points.size());
        }
    }
private:

    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud);
    void comparePointcloud(PointCloudXYZ::Ptr filtered_cloud ,PointCloudXYZ::Ptr output_cloud);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ori_cloud_sub_;
    double sor1_k, sor1_s, sor2_k, sor2_s; 

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame_1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame_2_;
    geometry_msgs::msg::TransformStamped livox_right_trans_;

    uint8_t segementation_type_;  // 0:传统; 1：点云差分

    // 先验点云相关
    std::string pcd_path_; 
    PointCloudXYZI::Ptr priori_cloud_;  //map坐标系下的先验点云
    PointCloudXYZ::Ptr priori_cloud_xyz_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    geometry_msgs::msg::TransformStamped extrin_tf_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Segmentation>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}



void Segmentation::cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud)
{   
    // 每两帧点云融合做一次处理
    if(cloud_frame_1_->empty())
    {
        pcl::fromROSMsg(*in_cloud, *cloud_frame_1_);
        return;
    }
    else
    {
        auto start_time = this->get_clock()->now();
        pcl::fromROSMsg(*in_cloud, *cloud_frame_2_);
        *cloud_frame_2_ += *cloud_frame_1_;

        

        if(segementation_type_ == 0){
            //传统方法(点云分割)
            // 先把点云旋转到车体坐标系
            auto base_link_trans = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
            Eigen::Affine3f ex_trans = Eigen::Affine3f::Identity();
            Eigen::Quaternionf q(base_link_trans.transform.rotation.w, base_link_trans.transform.rotation.x, base_link_trans.transform.rotation.y, base_link_trans.transform.rotation.z);
            ex_trans.rotate(q);
            ex_trans.pretranslate(Eigen::Vector3f(base_link_trans.transform.translation.x, base_link_trans.transform.translation.y, base_link_trans.transform.translation.z ));

            pcl::transformPointCloud(*cloud_frame_2_,*cloud_frame_2_,ex_trans);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud_frame_2_);
            pass.setFilterFieldName("z");
            
            pass.setFilterLimits(-0.1, 1.5);  // 保留 z 值在范围内的点
            pass.filter(*cloud_frame_2_);

            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            ne_.setNumberOfThreads(2);
            ne_.setInputCloud(cloud_frame_2_);
            //创建一个空的kdtree对象，并把它传递给法线估计对象
            //基于给出的输入数据集，kdtree将被建立
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne_.setSearchMethod(tree);
            //使用半径在查询点周围20厘米范围内的所有邻元素
            ne_.setRadiusSearch(0.5);
            //计算特征值
            ne_.compute(*normals);

            int j = 0;
            pcl::PointIndices::Ptr Indices(new pcl::PointIndices);
            for (auto i : *normals)
            {
                if (i.normal_z < 0.8 && i.normal_z > -0.8)
                {
                    Indices->indices.push_back(j);
                }
                j++;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_frame_2_); 
            extract.setIndices(Indices);
            extract.setNegative(false); // 设置提取反操作以删除indices中的点
            extract.filter(*output); // 最后得到的就是删除梯度过小的点的点云数据

            // 删除离群点
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
            sor1.setInputCloud(output);
            sor1.setMeanK(sor1_k);
            sor1.setStddevMulThresh(sor1_s);
            sor1.setNegative(false);
            sor1.filter(*output_filtered);

            // 再转回odom坐标系
            pcl::transformPointCloud(*output_filtered,*output_filtered,ex_trans.inverse());

            sensor_msgs::msg::PointCloud2 gradient_msg;
            pcl::toROSMsg(*output_filtered, gradient_msg);
            gradient_msg.header.frame_id = "odom";
            filtered_cloud_pub_->publish(gradient_msg);
            rclcpp::Time end_time = this->get_clock()->now();
           // RCLCPP_INFO(this->get_logger(),"used_time: %f", (end_time-start_time).seconds());

        }
        else {// 点云差分法，来自华农
            rclcpp::Time start_time = this->get_clock()->now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);

            // 立方体边界
            float minX = -0.3, maxX = 0.3;
            float minY = -0.3, maxY = 0.3;
            float minZ = -1.0, maxZ = 1.0;

            // 获取当前车辆位置和odom到map的变换
            geometry_msgs::msg::TransformStamped base_link_trans = tf_buffer_->lookupTransform("map","base_link",tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped odom_trans = tf_buffer_->lookupTransform("map","odom",tf2::TimePointZero);

            // 将扫描的点云从odom系转移到map系
            Eigen::Affine3f odom2map_matrix = Eigen::Affine3f::Identity();
            Eigen::Quaternionf q(odom_trans.transform.rotation.w, odom_trans.transform.rotation.x, odom_trans.transform.rotation.y, odom_trans.transform.rotation.z);
            odom2map_matrix.rotate(q);
            odom2map_matrix.pretranslate(Eigen::Vector3f(odom_trans.transform.translation.x, odom_trans.transform.translation.y, odom_trans.transform.translation.z ));
            pcl::transformPointCloud(*cloud_frame_2_,*cloud_frame_2_,odom2map_matrix);

            auto robot_pos = base_link_trans.transform.translation;

            // 遍历点云
            for (const auto &point : *cloud_frame_2_){
                if (!((point.x - robot_pos.x) >= minX && (point.x - robot_pos.x) <= maxX &&
                    (point.y - robot_pos.y) >= minY && (point.y - robot_pos.y) <= maxY &&
                    (point.z - robot_pos.z) >= minZ && (point.z - robot_pos.z) <= maxZ) && 
                    (point.z - robot_pos.z) < 3.0){
                    // 如果点不在立方体内，且高度低于三米，添加到过滤后的点云中
                    cloud_processed->points.push_back(point);
                }
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            // 现在我们得到了map系下的先验点云和当前扫描点云，接下来进行对比和删除
            comparePointcloud(cloud_processed, cloud_filtered);

            sensor_msgs::msg::PointCloud2 cloud_output;
            pcl::toROSMsg(*cloud_filtered, cloud_output);
            cloud_output.header.frame_id = "map";
            cloud_output.header.stamp = this->now();
            filtered_cloud_pub_->publish(cloud_output);

            rclcpp::Time end_time = this->get_clock()->now();
            //RCLCPP_INFO(this->get_logger(),"used_time: %f", (end_time-start_time).seconds());
        }
        
        // 储存的点云清空
        cloud_frame_1_->clear();
        cloud_frame_2_->clear();
    }   
    
}

void Segmentation::comparePointcloud(PointCloudXYZ::Ptr filtered_cloud, PointCloudXYZ::Ptr output_cloud)
{
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    for(const auto &point_xyz: filtered_cloud->points){
        if (kdtree_.nearestKSearch(point_xyz, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
        float z_diff = point_xyz.z - priori_cloud_xyz_->points[pointIdxNKNSearch[0]].z;
        if (z_diff >= 0.1){
            // RCLCPP_INFO(this->get_logger(),"dis_diff_:%f",dis_diff_);
            output_cloud->points.push_back(pcl::PointXYZ(point_xyz.x, point_xyz.y, point_xyz.z));
        }
        }

    }
    
}