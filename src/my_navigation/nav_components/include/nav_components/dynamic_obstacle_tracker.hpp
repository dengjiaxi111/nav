#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <vector>

namespace nav_components {

class DynamicObstacleTracker {
public:
    struct Params {
        bool enabled{false};
        int8_t occupied_threshold{80};
        int min_cluster_size{3};
        double association_distance_m{0.8};
        int confirm_hits{2};
        int max_missed_frames{5};
        double velocity_smooth_alpha{0.6};
        double obstacle_radius_m{0.35};
        bool publish_tentative_tracks{true};
    };

    DynamicObstacleTracker();
    explicit DynamicObstacleTracker(const Params& params);

    void setParams(const Params& params);
    const Params& getParams() const { return params_; }

    void reset();

    nav_msgs::msg::OccupancyGrid::SharedPtr process(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& input_map);

private:
    struct Observation {
        double x{0.0};
        double y{0.0};
        int count{0};
    };

    struct Track {
        int id{0};
        double x{0.0};
        double y{0.0};
        double vx{0.0};
        double vy{0.0};
        int hits{0};
        int missed{0};
        bool confirmed{false};
    };

    std::vector<Observation> clusterOccupiedCells(const nav_msgs::msg::OccupancyGrid& map) const;

    void updateTracks(const std::vector<Observation>& observations, double dt);

    nav_msgs::msg::OccupancyGrid::SharedPtr buildTrackedDynamicMap(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& input_map) const;

    static double sqrDist(double x0, double y0, double x1, double y1);

private:
    Params params_;

    std::vector<Track> tracks_;
    int next_track_id_{1};

    bool has_last_stamp_{false};
    rclcpp::Time last_stamp_{};
};

}  // namespace nav_components
