#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // 空间变换的核心头文件

int main()
{
    // 1. 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("your_cloud.pcd", *source_cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    // 2. 初始化一个 4x4 的变换矩阵 (使用 Eigen 库)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // 【第一步：平移】 
    // 把点 (100, 200, 50) 移到 (0,0,0)，相当于所有点减去这个坐标
    transform.translation() << -100.0, -200.0, -50.0;

    // 【第二步：旋转】 
    // 绕 Z 轴旋转 90 度 (在 PCL 中必须使用弧度制)
    float theta = M_PI / 2.0; // M_PI 是 180度，除以2就是 90度
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // 打印出最终的变换矩阵看看
    std::cout << "变换矩阵:\n" << transform.matrix() << std::endl;

    // 3. 执行点云变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 参数：输入点云, 输出点云, 变换矩阵
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

    // 4. 保存修改原点和朝向后的点云
    pcl::io::savePCDFileBinary("transformed_cloud.pcd", *transformed_cloud);
    std::cout << "原点及朝向修改成功，已保存！" << std::endl;

    return 0;
}