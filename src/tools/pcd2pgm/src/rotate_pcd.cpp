#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <string>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (argc < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("rotate_pcd"), 
            "Usage: ros2 run your_package rotate_pcd <input.pcd> <output.pcd> <yaw_in_radians>");
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    double yaw = std::stod(argv[3]);  // 旋转角度（弧度）

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rotate_pcd"), "Couldn't read input file %s", input_file.c_str());
        return 1;
    }

    // 构建旋转矩阵（绕 Z 轴）
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    // 变换点云
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);

    // 保存
    if (pcl::io::savePCDFileBinary(output_file, transformed_cloud) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rotate_pcd"), "Couldn't save output file %s", output_file.c_str());
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("rotate_pcd"), 
        "Successfully rotated point cloud by %.3f rad and saved to %s", yaw, output_file.c_str());

    rclcpp::shutdown();
    return 0;
}
