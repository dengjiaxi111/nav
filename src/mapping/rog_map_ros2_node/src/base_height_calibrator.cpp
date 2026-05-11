#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw) {
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

}  // namespace

class BaseHeightCalibrator : public rclcpp::Node {
public:
    BaseHeightCalibrator() : Node("base_height_calibrator") {
        pointcloud_type_ = declare_parameter<std::string>("pointcloud_type", "livox_custom");
        livox_topic_ = declare_parameter<std::string>("livox_topic", "/livox/lidar_192_168_1_199");
        pointcloud2_topic_ = declare_parameter<std::string>("pointcloud2_topic", "/livox/lidar");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");

        current_roll_ = declare_parameter<double>("current_roll", 0.52);
        current_pitch_ = declare_parameter<double>("current_pitch", 0.0);
        current_yaw_ = declare_parameter<double>("current_yaw", 1.5708);
        lidar_xyz_ = Eigen::Vector3d(
            declare_parameter<double>("lidar_x", 0.2),
            declare_parameter<double>("lidar_y", 0.0),
            declare_parameter<double>("lidar_z", 0.05));

        collect_duration_sec_ = declare_parameter<double>("collect_duration_sec", 3.0);
        min_candidates_ = declare_parameter<int>("min_candidates", 3000);
        max_candidates_ = declare_parameter<int>("max_candidates", 200000);
        voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);
        plane_distance_threshold_ = declare_parameter<double>("plane_distance_threshold", 0.04);
        plane_eps_angle_deg_ = declare_parameter<double>("plane_eps_angle_deg", 35.0);
        min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.35);

        min_range_ = declare_parameter<double>("min_range", 0.5);
        max_range_ = declare_parameter<double>("max_range", 12.0);
        candidate_base_z_min_ = declare_parameter<double>("candidate_base_z_min", -1.5);
        candidate_base_z_max_ = declare_parameter<double>("candidate_base_z_max", 0.4);
        candidate_xy_abs_max_ = declare_parameter<double>("candidate_xy_abs_max", 8.0);
        max_stationary_linear_speed_ = declare_parameter<double>("max_stationary_linear_speed", 0.08);
        max_stationary_angular_speed_ = declare_parameter<double>("max_stationary_angular_speed", 0.08);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(20),
            std::bind(&BaseHeightCalibrator::odomCallback, this, std::placeholders::_1));

        if (pointcloud_type_ == "pointcloud2") {
            pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud2_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BaseHeightCalibrator::pointCloud2Callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing PointCloud2: %s", pointcloud2_topic_.c_str());
        } else {
            livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                livox_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BaseHeightCalibrator::livoxCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing Livox CustomMsg: %s", livox_topic_.c_str());
        }

        RCLCPP_INFO(
            get_logger(),
            "Using base_link->livox_frame xyz=[%.3f, %.3f, %.3f], rpy=[%.6f, %.6f, %.6f]",
            lidar_xyz_.x(), lidar_xyz_.y(), lidar_xyz_.z(), current_roll_, current_pitch_,
            current_yaw_);
        RCLCPP_INFO(get_logger(), "Stand the robot still on flat ground and wait for result.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        have_odom_ = true;
    }

    bool odomStationary() const {
        if (!have_odom_) {
            return false;
        }
        const auto& t = latest_odom_.twist.twist;
        const double linear = std::hypot(t.linear.x, t.linear.y, t.linear.z);
        const double angular = std::hypot(t.angular.x, t.angular.y, t.angular.z);
        return linear <= max_stationary_linear_speed_ &&
               angular <= max_stationary_angular_speed_;
    }

    Eigen::Vector3d upInBase() const {
        const auto& q_msg = latest_odom_.pose.pose.orientation;
        Eigen::Quaterniond q_odom_base(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        if (q_odom_base.norm() < 1e-6) {
            return Eigen::Vector3d::UnitZ();
        }
        q_odom_base.normalize();
        return q_odom_base.inverse() * Eigen::Vector3d::UnitZ();
    }

    bool acceptPoint(const Eigen::Vector3d& p_lidar) const {
        const double range = p_lidar.norm();
        if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
            return false;
        }

        const Eigen::Matrix3d r_base_lidar =
            rpyToMatrix(current_roll_, current_pitch_, current_yaw_);
        const Eigen::Vector3d p_base = r_base_lidar * p_lidar + lidar_xyz_;
        if (std::abs(p_base.x()) > candidate_xy_abs_max_ ||
            std::abs(p_base.y()) > candidate_xy_abs_max_) {
            return false;
        }
        return p_base.z() >= candidate_base_z_min_ &&
               p_base.z() <= candidate_base_z_max_;
    }

    void addPoint(const Eigen::Vector3d& p_lidar) {
        if (done_ || !have_odom_ || !odomStationary() || !acceptPoint(p_lidar)) {
            return;
        }
        if (!collecting_) {
            collecting_ = true;
            collect_start_ = now();
            RCLCPP_INFO(get_logger(), "Started collecting ground candidates.");
        }
        if (static_cast<int>(cloud_->size()) >= max_candidates_) {
            return;
        }

        pcl::PointXYZI p;
        p.x = static_cast<float>(p_lidar.x());
        p.y = static_cast<float>(p_lidar.y());
        p.z = static_cast<float>(p_lidar.z());
        p.intensity = 0.0f;
        cloud_->push_back(p);
        up_base_sum_ += upInBase();
        up_base_count_++;
    }

    void maybeCalibrate() {
        if (done_ || !collecting_) {
            return;
        }
        const double elapsed = (now() - collect_start_).seconds();
        if (elapsed < collect_duration_sec_ ||
            static_cast<int>(cloud_->size()) < min_candidates_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Collecting candidates: %zu/%d, %.1f/%.1fs",
                cloud_->size(), min_candidates_, elapsed, collect_duration_sec_);
            return;
        }
        calibrate();
    }

    void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        for (const auto& p : msg->points) {
            addPoint(Eigen::Vector3d(p.x, p.y, p.z));
        }
        maybeCalibrate();
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromROSMsg(*msg, pc);
        for (const auto& p : pc.points) {
            addPoint(Eigen::Vector3d(p.x, p.y, p.z));
        }
        maybeCalibrate();
    }

    void calibrate() {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        if (voxel_leaf_size_ > 1e-4) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(cloud_);
            voxel.setLeafSize(
                static_cast<float>(voxel_leaf_size_),
                static_cast<float>(voxel_leaf_size_),
                static_cast<float>(voxel_leaf_size_));
            voxel.filter(*filtered);
        } else {
            filtered = cloud_;
        }

        if (static_cast<int>(filtered->size()) < min_candidates_ / 3) {
            RCLCPP_WARN(
                get_logger(), "Not enough points after voxel filtering: %zu", filtered->size());
            resetCollection();
            return;
        }

        Eigen::Vector3d up_base = Eigen::Vector3d::UnitZ();
        if (up_base_count_ > 0 && up_base_sum_.norm() > 1e-6) {
            up_base = (up_base_sum_ / static_cast<double>(up_base_count_)).normalized();
        }

        const Eigen::Matrix3d r_base_lidar =
            rpyToMatrix(current_roll_, current_pitch_, current_yaw_);
        const Eigen::Vector3d up_lidar = (r_base_lidar.transpose() * up_base).normalized();

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);
        seg.setAxis(Eigen::Vector3f(up_lidar.x(), up_lidar.y(), up_lidar.z()));
        seg.setEpsAngle(static_cast<float>(degToRad(plane_eps_angle_deg_)));
        seg.setMaxIterations(300);
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coefficients);

        if (coefficients->values.size() < 4 || inliers->indices.empty()) {
            RCLCPP_WARN(get_logger(), "Ground plane fitting failed; collecting again.");
            resetCollection();
            return;
        }

        Eigen::Vector3d normal_lidar(
            coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        const double normal_norm = normal_lidar.norm();
        if (normal_norm < 1e-6) {
            RCLCPP_WARN(get_logger(), "Fitted plane normal is invalid; collecting again.");
            resetCollection();
            return;
        }
        normal_lidar /= normal_norm;
        double plane_d = coefficients->values[3] / normal_norm;
        if (normal_lidar.dot(up_lidar) < 0.0) {
            normal_lidar = -normal_lidar;
            plane_d = -plane_d;
        }

        const double inlier_ratio =
            static_cast<double>(inliers->indices.size()) / static_cast<double>(filtered->size());
        if (inlier_ratio < min_inlier_ratio_) {
            RCLCPP_WARN(
                get_logger(),
                "Ground inlier ratio %.2f is below threshold %.2f; collecting again.",
                inlier_ratio, min_inlier_ratio_);
            resetCollection();
            return;
        }

        const Eigen::Vector3d base_origin_lidar = -r_base_lidar.transpose() * lidar_xyz_;
        const double denom = normal_lidar.dot(up_lidar);
        if (std::abs(denom) < 1e-3) {
            RCLCPP_WARN(get_logger(), "Ground normal is nearly perpendicular to up direction.");
            resetCollection();
            return;
        }

        const double signed_distance_to_plane =
            normal_lidar.dot(base_origin_lidar) + plane_d;
        const double base_height = signed_distance_to_plane / denom;
        const double lidar_height =
            (normal_lidar.dot(Eigen::Vector3d::Zero()) + plane_d) / denom;

        RCLCPP_INFO(get_logger(), "========== Base Height Calibration Result ==========");
        RCLCPP_INFO(
            get_logger(), "Collected raw candidates: %zu, voxel points: %zu, inliers: %zu (%.1f%%)",
            cloud_->size(), filtered->size(), inliers->indices.size(), inlier_ratio * 100.0);
        RCLCPP_INFO(
            get_logger(), "Ground normal in livox_frame: [%.6f, %.6f, %.6f], d=%.6f",
            normal_lidar.x(), normal_lidar.y(), normal_lidar.z(), plane_d);
        RCLCPP_INFO(
            get_logger(), "base_link height above ground: %.4f m", base_height);
        RCLCPP_INFO(
            get_logger(), "livox_frame origin height above ground: %.4f m", lidar_height);
        RCLCPP_INFO(
            get_logger(),
            "If you use ROG-Map virtual ground relative to odom/base startup height, ground is about %.4f m below base_link.",
            -base_height);
        RCLCPP_INFO(get_logger(), "===================================================");
        done_ = true;
    }

    void resetCollection() {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        collecting_ = false;
        up_base_sum_.setZero();
        up_base_count_ = 0;
    }

    std::string pointcloud_type_;
    std::string livox_topic_;
    std::string pointcloud2_topic_;
    std::string odom_topic_;

    double current_roll_{0.52};
    double current_pitch_{0.0};
    double current_yaw_{1.5708};
    Eigen::Vector3d lidar_xyz_{0.2, 0.0, 0.05};

    double collect_duration_sec_{3.0};
    int min_candidates_{3000};
    int max_candidates_{200000};
    double voxel_leaf_size_{0.05};
    double plane_distance_threshold_{0.04};
    double plane_eps_angle_deg_{35.0};
    double min_inlier_ratio_{0.35};

    double min_range_{0.5};
    double max_range_{12.0};
    double candidate_base_z_min_{-1.5};
    double candidate_base_z_max_{0.4};
    double candidate_xy_abs_max_{8.0};
    double max_stationary_linear_speed_{0.08};
    double max_stationary_angular_speed_{0.08};

    bool have_odom_{false};
    bool collecting_{false};
    bool done_{false};
    nav_msgs::msg::Odometry latest_odom_;
    rclcpp::Time collect_start_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZI>};
    Eigen::Vector3d up_base_sum_{Eigen::Vector3d::Zero()};
    int up_base_count_{0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseHeightCalibrator>());
    rclcpp::shutdown();
    return 0;
}
