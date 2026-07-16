#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <iostream>

//small-gicp库
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>

using PointT = pcl::PointXYZ;

Eigen::Matrix4f register_to_map(const std::string& source_file, const std::string& map_file, const Eigen::Matrix4f& init_guess)
{
    pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(source_file, *source);
    pcl::io::loadPCDFile(map_file, *target);

    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp;

    //pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
            //源点云降采样
    pcl::VoxelGrid<pcl::PointXYZ> sor_;
    sor_.setInputCloud(target);
    sor_.setLeafSize(0.05f,0.05f,0.05f);
    sor_.filter(*target);

    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    gicp.setNumThreads(6);
    gicp.setCorrespondenceRandomness(10);
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setMaximumIterations(1000);
    gicp.setVoxelResolution(1.0);
    gicp.setRegistrationType("GICP");  

    pcl::PointCloud<PointT> aligned;
    gicp.align(aligned, init_guess);

    if (!gicp.hasConverged())
    {
        std::cerr << "GICP failed to converge!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Fitness score: " << gicp.getFitnessScore() << std::endl;
    return gicp.getFinalTransformation();
}

void printTransform(const Eigen::Matrix4f& tf, const std::string& name)
{
    std::cout << name << ":\n" << tf << "\n\n";

    Eigen::Vector3f t = tf.block<3,1>(0,3);
    Eigen::Matrix3f R = tf.block<3,3>(0,0);

    Eigen::Vector3f euler = R.eulerAngles(2, 1, 0); // ZYX = yaw-pitch-roll
    std::cout << name << " (x,y,z,yaw,pitch,roll): "
              << t[0] << ", " << t[1] << ", " << t[2] << ", "
              << euler[0] << ", " << euler[1] << ", " << euler[2] << "\n\n";
}

int main()
{
    std::string map_file = "/home/nuc/navigationros2/ros2-humble/src/mapping_and_location/lio_3se/PCD/Changzhou.pcd";
    std::string left_file = "/home/nuc/navigationros2/ros2-humble/src/mapping_and_location/lio_3se/PCD/left.pcd";
    std::string right_file = "/home/nuc/navigationros2/ros2-humble/src/mapping_and_location/lio_3se/PCD/right.pcd";

    // 你可以根据现场经验指定粗略初始位姿
    Eigen::Matrix4f init_left = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_right = Eigen::Matrix4f::Identity();

    // std::cout << "Registering right lidar to map..." << std::endl;
    // Eigen::Matrix4f T_map_right = register_to_map(right_file, map_file, init_right);
    // printTransform(T_map_right, "T_map_right");

    std::cout << "Registering left lidar to map..." << std::endl;
    Eigen::Matrix4f T_map_left = register_to_map(left_file, right_file, init_left);
    printTransform(T_map_left, "T_map_left");

    // Eigen::Matrix4f T_left_to_right = T_map_left.inverse() * T_map_right;
    // printTransform(T_left_to_right, "T_left_to_right");

    return 0;
}
