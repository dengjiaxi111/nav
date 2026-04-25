#include "nav_components/dynamic_obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace nav_components {

DynamicObstacleTracker::DynamicObstacleTracker() { setParams(Params{}); }

DynamicObstacleTracker::DynamicObstacleTracker(const Params& params) { setParams(params); }

void DynamicObstacleTracker::setParams(const Params& params) {
    params_ = params;
    if (params_.confirm_hits < 1) {
        params_.confirm_hits = 1;
    }
    if (params_.max_missed_frames < 1) {
        params_.max_missed_frames = 1;
    }
    if (params_.min_cluster_size < 1) {
        params_.min_cluster_size = 1;
    }
    params_.velocity_smooth_alpha = std::clamp(params_.velocity_smooth_alpha, 0.0, 1.0);
    params_.obstacle_radius_m = std::max(0.0, params_.obstacle_radius_m);
    params_.association_distance_m = std::max(0.05, params_.association_distance_m);
}

void DynamicObstacleTracker::reset() {
    tracks_.clear();
    next_track_id_ = 1;
    has_last_stamp_ = false;
}

nav_msgs::msg::OccupancyGrid::SharedPtr DynamicObstacleTracker::process(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& input_map) {
    if (!input_map) {
        return nullptr;
    }
    if (!params_.enabled) {
        return input_map;
    }

    double dt = 0.1;
    if (has_last_stamp_) {
        const double dt_raw = (rclcpp::Time(input_map->header.stamp) - last_stamp_).seconds();
        if (std::isfinite(dt_raw) && dt_raw > 1e-4) {
            dt = std::clamp(dt_raw, 0.01, 0.5);
        }
    }
    last_stamp_ = rclcpp::Time(input_map->header.stamp);
    has_last_stamp_ = true;

    const auto observations = clusterOccupiedCells(*input_map);
    updateTracks(observations, dt);
    return buildTrackedDynamicMap(input_map);
}

std::vector<DynamicObstacleTracker::Observation> DynamicObstacleTracker::clusterOccupiedCells(
    const nav_msgs::msg::OccupancyGrid& map) const {
    std::vector<Observation> observations;

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0) {
        return observations;
    }

    std::vector<uint8_t> visited(static_cast<size_t>(width * height), 0);

    auto isOccupied = [&](int idx) {
        return idx >= 0 && idx < static_cast<int>(map.data.size()) &&
               map.data[static_cast<size_t>(idx)] >= params_.occupied_threshold;
    };

    const int nx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int ny[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int start_idx = y * width + x;
            if (visited[static_cast<size_t>(start_idx)] || !isOccupied(start_idx)) {
                continue;
            }

            std::queue<int> q;
            q.push(start_idx);
            visited[static_cast<size_t>(start_idx)] = 1;

            double sum_x = 0.0;
            double sum_y = 0.0;
            int count = 0;

            while (!q.empty()) {
                const int idx = q.front();
                q.pop();

                const int cx = idx % width;
                const int cy = idx / width;
                const double wx = map.info.origin.position.x + (static_cast<double>(cx) + 0.5) * map.info.resolution;
                const double wy = map.info.origin.position.y + (static_cast<double>(cy) + 0.5) * map.info.resolution;

                sum_x += wx;
                sum_y += wy;
                ++count;

                for (int k = 0; k < 8; ++k) {
                    const int xx = cx + nx[k];
                    const int yy = cy + ny[k];
                    if (xx < 0 || xx >= width || yy < 0 || yy >= height) {
                        continue;
                    }
                    const int nidx = yy * width + xx;
                    if (visited[static_cast<size_t>(nidx)] || !isOccupied(nidx)) {
                        continue;
                    }
                    visited[static_cast<size_t>(nidx)] = 1;
                    q.push(nidx);
                }
            }

            if (count >= params_.min_cluster_size) {
                Observation obs;
                obs.x = sum_x / static_cast<double>(count);
                obs.y = sum_y / static_cast<double>(count);
                obs.count = count;
                observations.push_back(obs);
            }
        }
    }

    return observations;
}

void DynamicObstacleTracker::updateTracks(const std::vector<Observation>& observations, double dt) {
    const size_t existing_track_count = tracks_.size();

    // 预测
    for (auto& tr : tracks_) {
        tr.x += tr.vx * dt;
        tr.y += tr.vy * dt;
    }

    std::vector<int> obs_match(observations.size(), -1);
    std::vector<uint8_t> track_used(existing_track_count, 0);

    // 贪心关联
    for (size_t oi = 0; oi < observations.size(); ++oi) {
        double best_d2 = params_.association_distance_m * params_.association_distance_m;
        int best_t = -1;

        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
            if (track_used[ti]) {
                continue;
            }
            const double d2 = sqrDist(observations[oi].x, observations[oi].y, tracks_[ti].x, tracks_[ti].y);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_t = static_cast<int>(ti);
            }
        }

        if (best_t >= 0) {
            obs_match[oi] = best_t;
            track_used[static_cast<size_t>(best_t)] = 1;
        }
    }

    // 更新匹配轨迹
    for (size_t oi = 0; oi < observations.size(); ++oi) {
        const int ti = obs_match[oi];
        if (ti < 0) {
            continue;
        }

        auto& tr = tracks_[static_cast<size_t>(ti)];
        const double meas_x = observations[oi].x;
        const double meas_y = observations[oi].y;

        const double inst_vx = (meas_x - tr.x) / std::max(1e-3, dt);
        const double inst_vy = (meas_y - tr.y) / std::max(1e-3, dt);

        const double alpha = params_.velocity_smooth_alpha;
        tr.vx = (1.0 - alpha) * tr.vx + alpha * inst_vx;
        tr.vy = (1.0 - alpha) * tr.vy + alpha * inst_vy;

        tr.x = meas_x;
        tr.y = meas_y;
        tr.hits += 1;
        tr.missed = 0;
        if (tr.hits >= params_.confirm_hits) {
            tr.confirmed = true;
        }
    }

    // 标记未匹配轨迹
    for (size_t ti = 0; ti < existing_track_count; ++ti) {
        if (track_used[ti]) {
            continue;
        }
        tracks_[ti].missed += 1;
    }

    // 新建未匹配观测
    for (size_t oi = 0; oi < observations.size(); ++oi) {
        if (obs_match[oi] >= 0) {
            continue;
        }

        Track tr;
        tr.id = next_track_id_++;
        tr.x = observations[oi].x;
        tr.y = observations[oi].y;
        tr.vx = 0.0;
        tr.vy = 0.0;
        tr.hits = 1;
        tr.missed = 0;
        tr.confirmed = (params_.confirm_hits <= 1);
        tracks_.push_back(tr);
    }

    // 清理丢失轨迹
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [&](const Track& tr) { return tr.missed > params_.max_missed_frames; }),
        tracks_.end());
}

nav_msgs::msg::OccupancyGrid::SharedPtr DynamicObstacleTracker::buildTrackedDynamicMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& input_map) const {
    auto output = std::make_shared<nav_msgs::msg::OccupancyGrid>(*input_map);

    const int width = static_cast<int>(output->info.width);
    const int height = static_cast<int>(output->info.height);
    if (width <= 0 || height <= 0 || output->info.resolution <= 0.0f) {
        return output;
    }

    const int r_cells = std::max(1, static_cast<int>(std::ceil(params_.obstacle_radius_m / output->info.resolution)));

    for (const auto& tr : tracks_) {
        if (!tr.confirmed && !params_.publish_tentative_tracks) {
            continue;
        }

        const int8_t track_occupancy = tr.confirmed ? 100 : 90;

        const int cx = static_cast<int>(std::floor((tr.x - output->info.origin.position.x) / output->info.resolution));
        const int cy = static_cast<int>(std::floor((tr.y - output->info.origin.position.y) / output->info.resolution));

        for (int dy = -r_cells; dy <= r_cells; ++dy) {
            for (int dx = -r_cells; dx <= r_cells; ++dx) {
                if (dx * dx + dy * dy > r_cells * r_cells) {
                    continue;
                }
                const int x = cx + dx;
                const int y = cy + dy;
                if (x < 0 || x >= width || y < 0 || y >= height) {
                    continue;
                }
                const int idx = y * width + x;
                auto& cell = output->data[static_cast<size_t>(idx)];
                if (cell < 0) {
                    continue;
                }
                cell = std::max(cell, track_occupancy);
            }
        }
    }

    return output;
}

double DynamicObstacleTracker::sqrDist(double x0, double y0, double x1, double y1) {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return dx * dx + dy * dy;
}

}  // namespace nav_components
