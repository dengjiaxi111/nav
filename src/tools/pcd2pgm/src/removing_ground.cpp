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
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "string.h"

class RemoveGround:public rclcpp::Node
{
private:
    std::string pcd_path_;
    std::string output_path_;
    std::string leveled_full_output_path_;
    bool auto_level_;
    bool require_auto_level_;
    bool translate_ground_to_zero_;
    double level_distance_threshold_;
    int level_max_iterations_;
    int level_candidate_planes_;
    int level_min_plane_inliers_;
    double ground_normal_threshold_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_;
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

    bool levelPointCloudWithGroundPlane()
    {
        if (!auto_level_) {
            RCLCPP_INFO(this->get_logger(), "auto_level disabled, keeping original PCD frame");
            return true;
        }

        if (!cloud_ || cloud_->empty()) {
            RCLCPP_ERROR(this->get_logger(), "auto_level failed: input cloud is empty");
            return false;
        }

        struct PlaneCandidate {
            Eigen::Vector3f normal;
            Eigen::Quaternionf rotation;
            double leveled_z = 0.0;
            std::size_t inliers = 0;
            int plane_index = 0;
        };

        std::vector<PlaneCandidate> candidates;
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(
            new pcl::PointCloud<pcl::PointXYZ>(*cloud_));

        for (int plane_idx = 0; plane_idx < level_candidate_planes_; ++plane_idx) {
            if (remaining->size() < static_cast<std::size_t>(level_min_plane_inliers_)) {
                break;
            }

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(level_max_iterations_);
            seg.setDistanceThreshold(level_distance_threshold_);
            seg.setInputCloud(remaining);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < static_cast<std::size_t>(level_min_plane_inliers_) ||
                coefficients->values.size() < 4) {
                break;
            }

            Eigen::Vector3f normal(
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2]);
            const float normal_norm = normal.norm();
            if (!std::isfinite(normal_norm) || normal_norm < 1e-6f) {
                break;
            }

            normal.normalize();
            if (normal.z() < 0.0f) {
                normal = -normal;
            }

            if (normal.z() >= ground_normal_threshold_) {
                Eigen::Quaternionf rotation =
                    Eigen::Quaternionf::FromTwoVectors(normal, Eigen::Vector3f::UnitZ());

                double leveled_z_sum = 0.0;
                int leveled_z_count = 0;
                for (const int idx : inliers->indices) {
                    if (idx < 0 || idx >= static_cast<int>(remaining->points.size())) {
                        continue;
                    }
                    const auto& point = remaining->points[static_cast<std::size_t>(idx)];
                    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                        continue;
                    }
                    Eigen::Vector3f rotated = rotation * point.getVector3fMap();
                    leveled_z_sum += rotated.z();
                    leveled_z_count++;
                }

                if (leveled_z_count > 0) {
                    PlaneCandidate candidate;
                    candidate.normal = normal;
                    candidate.rotation = rotation.normalized();
                    candidate.leveled_z = leveled_z_sum / static_cast<double>(leveled_z_count);
                    candidate.inliers = inliers->indices.size();
                    candidate.plane_index = plane_idx;
                    candidates.push_back(candidate);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "auto_level candidate %d: normal=[%.5f, %.5f, %.5f], z=%.4f, inliers=%zu",
                        plane_idx, normal.x(), normal.y(), normal.z(),
                        candidate.leveled_z, candidate.inliers);
                }
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "auto_level candidate %d ignored: normal z %.4f < threshold %.4f",
                    plane_idx, normal.z(), ground_normal_threshold_);
            }

            pcl::ExtractIndices<pcl::PointXYZ> remove_plane;
            remove_plane.setInputCloud(remaining);
            remove_plane.setIndices(inliers);
            remove_plane.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr without_plane(new pcl::PointCloud<pcl::PointXYZ>);
            remove_plane.filter(*without_plane);
            remaining = without_plane;
        }

        if (candidates.empty()) {
            RCLCPP_ERROR(this->get_logger(), "auto_level failed: no horizontal floor candidates found");
            return false;
        }

        auto best_it = std::min_element(
            candidates.begin(), candidates.end(),
            [](const PlaneCandidate& a, const PlaneCandidate& b) {
                constexpr double kSameHeightEpsilon = 0.05;
                if (std::abs(a.leveled_z - b.leveled_z) > kSameHeightEpsilon) {
                    return a.leveled_z < b.leveled_z;
                }
                return a.inliers > b.inliers;
            });

        const PlaneCandidate& selected = *best_it;
        Eigen::Vector3f normal = selected.normal;
        Eigen::Quaternionf rotation = selected.rotation;

        Eigen::Affine3f level_transform = Eigen::Affine3f::Identity();
        level_transform.rotate(rotation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr leveled(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_, *leveled, level_transform);
        double ground_z = selected.leveled_z;

        if (translate_ground_to_zero_) {
            for (auto& point : leveled->points) {
                point.z -= static_cast<float>(ground_z);
            }
        }

        Eigen::Vector3f aligned_normal = rotation * normal;
        const double inlier_ratio =
            static_cast<double>(selected.inliers) /
            static_cast<double>(std::max<std::size_t>(1, cloud_->points.size()));

        RCLCPP_INFO(
            this->get_logger(),
            "auto_level: selected plane %d as floor, normal [%.5f, %.5f, %.5f], inliers=%zu/%zu (%.2f%%)",
            selected.plane_index,
            normal.x(), normal.y(), normal.z(),
            selected.inliers, cloud_->points.size(), inlier_ratio * 100.0);
        RCLCPP_INFO(
            this->get_logger(),
            "auto_level: aligned normal [%.5f, %.5f, %.5f], ground_z=%.4f%s",
            aligned_normal.x(), aligned_normal.y(), aligned_normal.z(), ground_z,
            translate_ground_to_zero_ ? " -> shifted to z=0" : "");

        *cloud_ = *leveled;

        if (!leveled_full_output_path_.empty()) {
            if (writer.write<pcl::PointXYZ>(leveled_full_output_path_, *cloud_, false) < 0) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "failed to write leveled full PCD: %s",
                    leveled_full_output_path_.c_str());
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "leveled full PCD written: %s",
                    leveled_full_output_path_.c_str());
            }
        }

        return true;
    }

public:
    RemoveGround(): rclcpp::Node("RemoveGround")
    {
        cloud_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud1_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud2_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud3_   = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ground_  = pcl::make_shared<pcl::PointIndices>();

        normals_ = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
        Indices_ = pcl::make_shared<pcl::PointIndices>();
        output_  = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_filtered_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        tree_    = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

        this->declare_parameter<std::string>("pcd_path", "/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/rmul_2025.pcd");
        this->declare_parameter<std::string>("output_path", "/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/object.pcd");
        this->declare_parameter<std::string>("leveled_full_output_path", "");
        this->declare_parameter<bool>("auto_level", true);
        this->declare_parameter<bool>("require_auto_level", false);
        this->declare_parameter<bool>("translate_ground_to_zero", true);
        this->declare_parameter<double>("level_distance_threshold", 0.08);
        this->declare_parameter<int>("level_max_iterations", 1000);
        this->declare_parameter<int>("level_candidate_planes", 6);
        this->declare_parameter<int>("level_min_plane_inliers", 1000);
        this->declare_parameter<double>("ground_normal_threshold", 0.70);
        this->get_parameter("pcd_path", pcd_path_);
        this->get_parameter("output_path", output_path_);
        this->get_parameter("leveled_full_output_path", leveled_full_output_path_);
        this->get_parameter("auto_level", auto_level_);
        this->get_parameter("require_auto_level", require_auto_level_);
        this->get_parameter("translate_ground_to_zero", translate_ground_to_zero_);
        this->get_parameter("level_distance_threshold", level_distance_threshold_);
        this->get_parameter("level_max_iterations", level_max_iterations_);
        this->get_parameter("level_candidate_planes", level_candidate_planes_);
        this->get_parameter("level_min_plane_inliers", level_min_plane_inliers_);
        this->get_parameter("ground_normal_threshold", ground_normal_threshold_);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == -1) 
        {
            PCL_ERROR("Couldn't read file \n");
            return ;
        }
        RCLCPP_INFO(this->get_logger(), "file read");

        if (!levelPointCloudWithGroundPlane()) {
            if (require_auto_level_) {
                RCLCPP_ERROR(this->get_logger(), "auto_level is required; aborting");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "auto_level failed; continuing with original PCD");
        }

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
