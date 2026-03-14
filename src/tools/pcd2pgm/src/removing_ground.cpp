#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "string.h"

class RemoveGround:public rclcpp::Node
{
private:
    std::string pcd_path_;
    std::string output_path_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
    pcl::PointIndicesPtr ground_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
    pcl::PointIndices::Ptr Indices_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_filtered_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    pcl::PCDWriter writer;

public:
    RemoveGround(): rclcpp::Node("RemoveGround")
    {
        cloud_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud1_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ground_  = pcl::make_shared<pcl::PointIndices>();

        normals_ = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
        Indices_ = pcl::make_shared<pcl::PointIndices>();
        output_  = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_filtered_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        tree_    = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

        this->declare_parameter<std::string>("pcd_path", "/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/rmul_2025.pcd");
        this->declare_parameter<std::string>("output_path", "/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/object.pcd");
        this->get_parameter("pcd_path", pcd_path_);
        this->get_parameter("output_path", output_path_);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == -1) 
        {
            PCL_ERROR("Couldn't read file \n");
            return ;
        }
        RCLCPP_INFO(this->get_logger(), "file read");
        //先进行坐标变换，再过滤

        /* //先构造车体中心在地面的投影到雷达坐标系的变换，再求逆
        // 注意，此处的变换需要与livox_left到base_link的变换一致
        Eigen::Vector3d translation(0.045, 0.123, 0.20);
        // 旋转矩阵（通过欧拉角构造）
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitX())  // Roll
                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())  // Pitch
                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); //Yaw
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translate(translation);
        transform.rotate(rotation);
        RCLCPP_INFO(this->get_logger(), "start trans");
        pcl::transformPointCloud(*cloud_, *cloud1_, transform);

        RCLCPP_INFO(this->get_logger(), "end trans"); */

        ne_.setInputCloud(cloud_); 
        ne_.setSearchMethod(tree_);
        ne_.setRadiusSearch(0.1);
        ne_.compute(*normals_);
        RCLCPP_INFO(this->get_logger(), "normals computed");

        int j = 0;
        for (auto i : *normals_)
        {
            if (i.normal_z < 0.84 && i.normal_z > -0.84)
            {
                Indices_->indices.push_back(j);
            }
            j++;
        }

        extract_.setInputCloud(cloud_); 
        extract_.setIndices(Indices_);
        extract_.setNegative(false); // 设置提取反操作以删除indices中的点
        extract_.filter(*output_); // 最后得到的就是删除梯度过小的点的点云数据
        RCLCPP_INFO(this->get_logger(), "ground removed");

        // sor1.setInputCloud(output_);
        // sor1.setMeanK(10);
        // sor1.setStddevMulThresh(0.5);
        // sor1.filter(*output_filtered_);
        // RCLCPP_INFO(this->get_logger(), "statistical outlier removed");
        if (writer.write<pcl::PointXYZ>(output_path_, *output_, false) < 0) 
        {
            std::cerr << "Failed to write point cloud data to " << output_path_ << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "pcd write complete");
    }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoveGround>();

    rclcpp::shutdown();
    return 0;
}
