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
    std::string level_method_;
    std::string level_height_origin_mode_;
    bool auto_level_;
    bool require_auto_level_;
    bool translate_ground_to_zero_;
    double level_distance_threshold_;
    int level_max_iterations_;
    int level_candidate_planes_;
    int level_min_plane_inliers_;
    double level_early_stop_below_first_m_;
    double level_ground_percentile_;
    double ground_normal_threshold_;
    double manual_level_roll1_;
    double manual_level_pitch1_;
    double manual_level_yaw1_;
    double manual_level_roll2_;
    double manual_level_pitch2_;
    double manual_level_yaw2_;
    double manual_level_x_;
    double manual_level_y_;
    double manual_level_z_;

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

    bool applyManualExtrinsicTransform()
    {
        if (!cloud_ || cloud_->empty()) {
            RCLCPP_ERROR(this->get_logger(), "manual_extrinsic failed: input cloud is empty");
            return false;
        }

        Eigen::Affine3d transform1 = Eigen::Affine3d::Identity();
        transform1.rotate(
            Eigen::AngleAxisd(manual_level_roll1_, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(manual_level_pitch1_, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(manual_level_yaw1_, Eigen::Vector3d::UnitZ()));

        Eigen::Affine3d transform2 = Eigen::Affine3d::Identity();
        transform2.rotate(
            Eigen::AngleAxisd(manual_level_roll2_, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(manual_level_pitch2_, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(manual_level_yaw2_, Eigen::Vector3d::UnitZ()));

        Eigen::Affine3d transform3 = Eigen::Affine3d::Identity();
        transform3.translate(Eigen::Vector3d(manual_level_x_, manual_level_y_, manual_level_z_));

        // Keep the old behavior: cloud -> transform1 -> transform2 -> transform3.
        Eigen::Affine3d combined = transform3 * transform2 * transform1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_, *transformed, combined);
        *cloud_ = *transformed;

        RCLCPP_INFO(
            this->get_logger(),
            "manual_extrinsic applied: rpy1=[%.4f, %.4f, %.4f], rpy2=[%.4f, %.4f, %.4f], xyz=[%.4f, %.4f, %.4f]",
            manual_level_roll1_, manual_level_pitch1_, manual_level_yaw1_,
            manual_level_roll2_, manual_level_pitch2_, manual_level_yaw2_,
            manual_level_x_, manual_level_y_, manual_level_z_);

        if (!leveled_full_output_path_.empty()) {
            if (writer.write<pcl::PointXYZ>(leveled_full_output_path_, *cloud_, false) < 0) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "failed to write manual-transformed full PCD: %s",
                    leveled_full_output_path_.c_str());
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "manual-transformed full PCD written: %s",
                    leveled_full_output_path_.c_str());
            }
        }

        return true;
    }

    bool preprocessPointCloudFrame()
    {
        std::string method = level_method_;
        if (method.empty()) {
            method = auto_level_ ? "auto" : "none";
        }

        if (method == "none") {
            RCLCPP_INFO(this->get_logger(), "level_method=none, keeping original PCD frame");
            return true;
        }
        if (method == "auto") {
            return levelPointCloudWithGroundPlane();
        }
        if (method == "manual_extrinsic") {
            return applyManualExtrinsicTransform();
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "unknown level_method '%s' (expected: none, auto, manual_extrinsic)",
            method.c_str());
        return false;
    }

    bool levelPointCloudWithGroundPlane()
    {
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

                    if (plane_idx > 0 && !candidates.empty() &&
                        level_early_stop_below_first_m_ > 0.0 &&
                        candidate.leveled_z <
                            candidates.front().leveled_z - level_early_stop_below_first_m_) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "auto_level: early stop at candidate %d because z %.4f is %.3fm below first plane z %.4f",
                            plane_idx, candidate.leveled_z, level_early_stop_below_first_m_,
                            candidates.front().leveled_z);
                        break;
                    }
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

        if (level_height_origin_mode_ == "low_percentile") {
            std::vector<float> finite_z;
            finite_z.reserve(leveled->points.size());
            for (const auto& point : leveled->points) {
                if (std::isfinite(point.z)) {
                    finite_z.push_back(point.z);
                }
            }
            if (!finite_z.empty()) {
                const double clamped_percentile =
                    std::min(0.50, std::max(0.0, level_ground_percentile_));
                std::size_t kth = static_cast<std::size_t>(
                    std::floor(clamped_percentile * static_cast<double>(finite_z.size() - 1)));
                kth = std::min(kth, finite_z.size() - 1);
                std::nth_element(finite_z.begin(), finite_z.begin() + kth, finite_z.end());
                ground_z = finite_z[kth];
                RCLCPP_INFO(
                    this->get_logger(),
                    "auto_level: using low_percentile height origin p=%.3f, z=%.4f",
                    clamped_percentile, ground_z);
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "auto_level: low_percentile requested but no finite z values found; "
                    "falling back to selected plane z");
            }
        } else if (level_height_origin_mode_ != "selected_plane") {
            RCLCPP_WARN(
                this->get_logger(),
                "unknown level_height_origin_mode '%s', using selected_plane",
                level_height_origin_mode_.c_str());
        }

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

        this->declare_parameter<std::string>("pcd_path", "/home/super259/nav/src/tools/pcd2pgm/save_pcd/rmul_2025.pcd");
        this->declare_parameter<std::string>("output_path", "/home/super259/nav/src/tools/pcd2pgm/save_pcd/object.pcd");
        this->declare_parameter<std::string>("leveled_full_output_path", "");
        this->declare_parameter<std::string>("level_method", "");
        this->declare_parameter<std::string>("level_height_origin_mode", "selected_plane");
        this->declare_parameter<bool>("auto_level", true);
        this->declare_parameter<bool>("require_auto_level", false);
        this->declare_parameter<bool>("translate_ground_to_zero", true);
        this->declare_parameter<double>("level_distance_threshold", 0.08);
        this->declare_parameter<int>("level_max_iterations", 1000);
        this->declare_parameter<int>("level_candidate_planes", 4);
        this->declare_parameter<int>("level_min_plane_inliers", 1000);
        this->declare_parameter<double>("level_early_stop_below_first_m", 0.10);
        this->declare_parameter<double>("level_ground_percentile", 0.02);
        this->declare_parameter<double>("ground_normal_threshold", 0.70);
        this->declare_parameter<double>("manual_level_roll1", 0.5);
        this->declare_parameter<double>("manual_level_pitch1", 0.0);
        this->declare_parameter<double>("manual_level_yaw1", 0.0);
        this->declare_parameter<double>("manual_level_roll2", 0.0);
        this->declare_parameter<double>("manual_level_pitch2", 0.0);
        this->declare_parameter<double>("manual_level_yaw2", -1.5708);
        this->declare_parameter<double>("manual_level_x", 0.2);
        this->declare_parameter<double>("manual_level_y", 0.0);
        this->declare_parameter<double>("manual_level_z", 0.05);
        this->get_parameter("pcd_path", pcd_path_);
        this->get_parameter("output_path", output_path_);
        this->get_parameter("leveled_full_output_path", leveled_full_output_path_);
        this->get_parameter("level_method", level_method_);
        this->get_parameter("level_height_origin_mode", level_height_origin_mode_);
        this->get_parameter("auto_level", auto_level_);
        this->get_parameter("require_auto_level", require_auto_level_);
        this->get_parameter("translate_ground_to_zero", translate_ground_to_zero_);
        this->get_parameter("level_distance_threshold", level_distance_threshold_);
        this->get_parameter("level_max_iterations", level_max_iterations_);
        this->get_parameter("level_candidate_planes", level_candidate_planes_);
        this->get_parameter("level_min_plane_inliers", level_min_plane_inliers_);
        this->get_parameter("level_early_stop_below_first_m", level_early_stop_below_first_m_);
        this->get_parameter("level_ground_percentile", level_ground_percentile_);
        this->get_parameter("ground_normal_threshold", ground_normal_threshold_);
        this->get_parameter("manual_level_roll1", manual_level_roll1_);
        this->get_parameter("manual_level_pitch1", manual_level_pitch1_);
        this->get_parameter("manual_level_yaw1", manual_level_yaw1_);
        this->get_parameter("manual_level_roll2", manual_level_roll2_);
        this->get_parameter("manual_level_pitch2", manual_level_pitch2_);
        this->get_parameter("manual_level_yaw2", manual_level_yaw2_);
        this->get_parameter("manual_level_x", manual_level_x_);
        this->get_parameter("manual_level_y", manual_level_y_);
        this->get_parameter("manual_level_z", manual_level_z_);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == -1) 
        {
            PCL_ERROR("Couldn't read file \n");
            return ;
        }
        RCLCPP_INFO(this->get_logger(), "file read");

        if (!preprocessPointCloudFrame()) {
            if (require_auto_level_) {
                RCLCPP_ERROR(this->get_logger(), "level preprocessing is required; aborting");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "level preprocessing failed; continuing with original PCD");
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
