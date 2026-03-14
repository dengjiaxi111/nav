#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class MapPreprocessor : public rclcpp::Node
{
public:
    MapPreprocessor()
    : Node("map_preprocessor_node")
    {
        declare_parameter<std::string>("map_path", "map_raw.pcd");
        declare_parameter<std::string>("save_path", "map_filtered.pcd");
        declare_parameter<double>("voxel_leaf", 0.1);
        declare_parameter<double>("z_min", -1.0);
        declare_parameter<double>("z_max", 10.0);
        declare_parameter<std::vector<double>>("lidar_pos",std::vector<double>{0.1, -0.1499, 0.3});
        declare_parameter<std::vector<double>>("lidar_rot", std::vector<double>{0.326,0,0,0.94});  //X Y Z W

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_map", 10);

        std::string map_path, save_path;
        double voxel_leaf, z_min, z_max;
        get_parameter("map_path", map_path);
        get_parameter("save_path", save_path);
        get_parameter("voxel_leaf", voxel_leaf);
        get_parameter("z_min", z_min);
        get_parameter("z_max", z_max);
        this->get_parameter<std::vector<double>>("lidar_pos", lidar_pos_);
        this->get_parameter<std::vector<double>>("lidar_rot", lidar_rot_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *input_cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "无法读取点云文件：%s", map_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "成功加载地图，共 %zu 点", input_cloud->points.size());

        // 先把点云旋转正
        Eigen::Affine3f fixed_transform = Eigen::Affine3f::Identity();
        Eigen::Quaternionf q1(lidar_rot_[3],lidar_rot_[0],lidar_rot_[1],lidar_rot_[2]);
        fixed_transform.rotate(q1);
        fixed_transform.pretranslate(Eigen::Vector3f(lidar_pos_[0] ,lidar_pos_[1] ,lidar_pos_[2]));
        pcl::transformPointCloud(*input_cloud,*input_cloud,fixed_transform);

        // fixed_transform = Eigen::Affine3f::Identity();
        // Eigen::Quaternionf q2(1.0,0.0,0.022,0.0);
        // fixed_transform.rotate(q2);
        // fixed_transform.pretranslate(Eigen::Vector3f(0.0,0.0,0.0));
        // pcl::transformPointCloud(*input_cloud,*input_cloud,fixed_transform);

        // 1. 降采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
        vg.filter(*downsampled);

        // 2. 裁剪高度
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(downsampled);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*clipped);

        // 3. 去除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud(clipped);
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered);

        

        // 4. 保存为 PCD 文件（binary 格式）
        if (pcl::io::savePCDFileBinary(save_path, *filtered) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法保存处理后点云到文件：%s", save_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "处理后地图保存至：%s", save_path.c_str());
        }

        // 5. 发布点云消息
        // sensor_msgs::msg::PointCloud2 output;
        // pcl::toROSMsg(*filtered, output);
        // output.header.frame_id = "map";
        // publisher_->publish(output);
        // RCLCPP_INFO(this->get_logger(), "已发布处理后地图");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::vector<double> lidar_pos_;        //雷达到车体中心的外参
    std::vector<double> lidar_rot_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
