// nav_components/src/layered_map_manager.cpp
// 分层地图管理器实现

#include "nav_components/layered_map_manager.hpp"
#include "nav_components/map_image_io.hpp"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <unordered_set>
#include <Eigen/Eigenvalues>

namespace nav_components {

LayeredMapManager::LayeredMapManager()
    : logger_(rclcpp::get_logger("layered_map_manager")) {}

void LayeredMapManager::initialize(rclcpp::Node* node,
                                    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    node_ = node;
    tf_buffer_ = tf_buffer;
    logger_ = node_->get_logger();
    
    // 读取坐标系参数
    node_->get_parameter_or<std::string>("map_frame", map_frame_, map_frame_);
    node_->get_parameter_or<std::string>("odom_frame", odom_frame_, odom_frame_);

    // 读取性能日志开关（默认 false）
    node_->get_parameter_or<bool>("enable_performance_logging", enable_performance_logging_, enable_performance_logging_);
    
    RCLCPP_INFO(logger_, "LayeredMapManager initialized (map: %s, odom: %s, perf_log: %s)",
                map_frame_.c_str(), odom_frame_.c_str(), 
                enable_performance_logging_ ? "enabled" : "disabled");
}

void LayeredMapManager::setStairLayerConfig(const StairLayerConfig& cfg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    stair_layer_cfg_ = cfg;

    if (!stair_layer_cfg_.enable) {
        stair_clear_indices_.clear();
        stair_forbidden_transitions_.clear();
        stair_normal_x_.clear();
        stair_normal_y_.clear();
        stair_normal_valid_.clear();
        stair_primitives_.clear();
        stair_primitive_id_map_.clear();
        stair_mask_loaded_ = false;
        return;
    }

    if (stair_layer_cfg_.clear_perp_dist_m < 0.0) {
        RCLCPP_WARN(logger_, "stair_layer clear_perp_dist_m<0, 自动修正为0.0m");
        stair_layer_cfg_.clear_perp_dist_m = 0.0;
    }
    if (stair_layer_cfg_.clear_perp_high_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "stair_layer clear_perp_high_dist_m<0, 采用 clear_perp_dist_m");
        stair_layer_cfg_.clear_perp_high_dist_m = -1.0;
    }
    if (stair_layer_cfg_.clear_perp_low_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "stair_layer clear_perp_low_dist_m<0, 采用 clear_perp_dist_m");
        stair_layer_cfg_.clear_perp_low_dist_m = -1.0;
    }

    if (stair_layer_cfg_.stair_level2_clear_perp_dist_m < 0.0) {
        RCLCPP_WARN(logger_, "stair_layer stair_level2_clear_perp_dist_m<0, 自动修正为 clear_perp_dist_m 或者 0.0m");
        stair_layer_cfg_.stair_level2_clear_perp_dist_m = stair_layer_cfg_.clear_perp_dist_m;
    }
    if (stair_layer_cfg_.stair_level2_clear_perp_high_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "stair_layer stair_level2_clear_perp_high_dist_m<0, 采用 stair_level2_clear_perp_dist_m");
        stair_layer_cfg_.stair_level2_clear_perp_high_dist_m = -1.0;
    }
    if (stair_layer_cfg_.stair_level2_clear_perp_low_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "stair_layer stair_level2_clear_perp_low_dist_m<0, 采用 stair_level2_clear_perp_dist_m");
        stair_layer_cfg_.stair_level2_clear_perp_low_dist_m = -1.0;
    }

    rebuildStairLayerCache();
}

void LayeredMapManager::setFlySlopeLayerConfig(const FlySlopeLayerConfig& cfg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    fly_slope_layer_cfg_ = cfg;

    if (!fly_slope_layer_cfg_.enable) {
        fly_slope_clear_indices_.clear();
        fly_slope_forbidden_transitions_.clear();
        fly_slope_normal_x_.clear();
        fly_slope_normal_y_.clear();
        fly_slope_normal_valid_.clear();
        fly_slope_primitives_.clear();
        fly_slope_primitive_id_map_.clear();
        fly_slope_mask_loaded_ = false;
        return;
    }

    if (fly_slope_layer_cfg_.clear_perp_dist_m < 0.0) {
        RCLCPP_WARN(logger_, "fly_slope_layer clear_perp_dist_m<0, 自动修正为0.0m");
        fly_slope_layer_cfg_.clear_perp_dist_m = 0.0;
    }
    if (fly_slope_layer_cfg_.clear_perp_high_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "fly_slope_layer clear_perp_high_dist_m<0, 采用 clear_perp_dist_m");
        fly_slope_layer_cfg_.clear_perp_high_dist_m = -1.0;
    }
    if (fly_slope_layer_cfg_.clear_perp_low_dist_m < -1e-9) {
        RCLCPP_WARN(logger_, "fly_slope_layer clear_perp_low_dist_m<0, 采用 clear_perp_dist_m");
        fly_slope_layer_cfg_.clear_perp_low_dist_m = -1.0;
    }

    rebuildFlySlopeLayerCache();
}

void LayeredMapManager::setRuntimeBlockedStairUphillIds(
    const std::unordered_set<int>& stair_ids) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    runtime_blocked_stair_uphill_ids_ = stair_ids;
}

void LayeredMapManager::clearRuntimeBlockedStairUphillIds() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    runtime_blocked_stair_uphill_ids_.clear();
}

bool LayeredMapManager::loadStairMaskFromYaml(const std::string& yaml_path) {
    MapImageData mask_data;
    if (!MapImageIO::loadYamlAndPGM(yaml_path, mask_data, logger_)) {
        return false;
    }

    stair_mask_width_ = mask_data.width;
    stair_mask_height_ = mask_data.height;
    stair_mask_max_val_ = mask_data.max_val;
    stair_mask_resolution_ = mask_data.resolution;
    stair_mask_origin_x_ = mask_data.origin_x;
    stair_mask_origin_y_ = mask_data.origin_y;
    stair_mask_pixels_ = std::move(mask_data.pixels);
    stair_mask_loaded_ = true;
    loaded_stair_mask_yaml_ = yaml_path;

    RCLCPP_INFO(logger_, "stair mask 加载成功: %dx%d @ %.3f (%s)",
                stair_mask_width_, stair_mask_height_, stair_mask_resolution_,
                mask_data.image_path.c_str());
    return true;
}

bool LayeredMapManager::loadFlySlopeMaskFromYaml(const std::string& yaml_path) {
    MapImageData mask_data;
    if (!MapImageIO::loadYamlAndPGM(yaml_path, mask_data, logger_)) {
        return false;
    }

    fly_slope_mask_width_ = mask_data.width;
    fly_slope_mask_height_ = mask_data.height;
    fly_slope_mask_max_val_ = mask_data.max_val;
    fly_slope_mask_resolution_ = mask_data.resolution;
    fly_slope_mask_origin_x_ = mask_data.origin_x;
    fly_slope_mask_origin_y_ = mask_data.origin_y;
    fly_slope_mask_pixels_ = std::move(mask_data.pixels);
    fly_slope_mask_loaded_ = true;
    loaded_fly_slope_mask_yaml_ = yaml_path;

    RCLCPP_INFO(logger_, "fly_slope mask 加载成功: %dx%d @ %.3f (%s)",
                fly_slope_mask_width_, fly_slope_mask_height_, fly_slope_mask_resolution_,
                mask_data.image_path.c_str());
    return true;
}

bool LayeredMapManager::worldToGlobalIndex(double wx, double wy, int& idx) const {
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
        return false;
    }
    int gx = static_cast<int>((wx - origin_x_) / resolution_);
    int gy = static_cast<int>((wy - origin_y_) / resolution_);
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
        return false;
    }
    idx = gy * width_ + gx;
    return true;
}

uint64_t LayeredMapManager::encodeDirectedTransition(int from_idx, int to_idx) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(from_idx)) << 32) |
           static_cast<uint32_t>(to_idx);
}

void LayeredMapManager::addForbiddenDirectedTransitions(
    int from_idx, int to_idx, std::unordered_set<uint64_t>& out) {
    if (from_idx < 0 || to_idx < 0 || from_idx == to_idx || width_ <= 0 || height_ <= 0) {
        return;
    }

    int from_x = from_idx % width_;
    int from_y = from_idx / width_;
    int to_x = to_idx % width_;
    int to_y = to_idx / width_;

    int dx = to_x - from_x;
    int dy = to_y - from_y;
    int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps <= 0) {
        return;
    }

    int prev_idx = from_idx;
    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(steps);
        int x = static_cast<int>(std::round(from_x + dx * t));
        int y = static_cast<int>(std::round(from_y + dy * t));
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
            continue;
        }

        int cur_idx = y * width_ + x;
        if (cur_idx != prev_idx) {
            out.insert(encodeDirectedTransition(prev_idx, cur_idx));
            prev_idx = cur_idx;
        }
    }
}

bool LayeredMapManager::isTransitionAllowed(int from_x, int from_y, int to_x, int to_y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (from_x < 0 || from_x >= width_ || from_y < 0 || from_y >= height_ ||
        to_x < 0 || to_x >= width_ || to_y < 0 || to_y >= height_) {
        return true;
    }

    int from_idx = from_y * width_ + from_x;
    int to_idx = to_y * width_ + to_x;

    // 同栅格不应被视为方向转移，直接放行。
    if (from_idx == to_idx) {
        return true;
    }

    if (stair_layer_cfg_.enable && stair_layer_cfg_.enable_oneway_stair_down &&
        !stair_forbidden_transitions_.empty()) {
        if (stair_forbidden_transitions_.find(
                encodeDirectedTransition(from_idx, to_idx)) != stair_forbidden_transitions_.end()) {
            return false;
        }
    }

    if (fly_slope_layer_cfg_.enable && fly_slope_layer_cfg_.enable_oneway_low_to_high &&
        !fly_slope_forbidden_transitions_.empty()) {
        if (fly_slope_forbidden_transitions_.find(
                encodeDirectedTransition(from_idx, to_idx)) != fly_slope_forbidden_transitions_.end()) {
            return false;
        }
    }

    if (stair_layer_cfg_.enable && stair_layer_cfg_.block_level2_down &&
        !stair_primitive_id_map_.empty() && !stair_primitives_.empty()) {
        if (from_idx >= 0 && from_idx < static_cast<int>(stair_primitive_id_map_.size()) &&
            to_idx >= 0 && to_idx < static_cast<int>(stair_primitive_id_map_.size())) {
            const int from_stair_id = stair_primitive_id_map_[from_idx];
            const int to_stair_id = stair_primitive_id_map_[to_idx];
            if (from_stair_id >= 0 && from_stair_id == to_stair_id &&
                from_stair_id < static_cast<int>(stair_primitives_.size())) {
                const auto& primitive = stair_primitives_[from_stair_id];
                const Eigen::Vector2d normal = primitive.normal;
                if (primitive.is_level2 && normal.squaredNorm() >= 1e-9) {
                    const double from_wx = origin_x_ + (from_x + 0.5) * resolution_;
                    const double from_wy = origin_y_ + (from_y + 0.5) * resolution_;
                    const double to_wx = origin_x_ + (to_x + 0.5) * resolution_;
                    const double to_wy = origin_y_ + (to_y + 0.5) * resolution_;
                    const Eigen::Vector2d delta(to_wx - from_wx, to_wy - from_wy);
                    const double down_projection = delta.dot(normal);
                    const double down_eps = std::max(1e-6, resolution_ * 0.05);
                    if (down_projection < -down_eps) {
                        return false;
                    }
                }
            }
        }
    }

    if (runtime_blocked_stair_uphill_ids_.empty() || stair_primitive_id_map_.empty() ||
        stair_primitives_.empty()) {
        return true;
    }

    if (from_idx < 0 || from_idx >= static_cast<int>(stair_primitive_id_map_.size()) ||
        to_idx < 0 || to_idx >= static_cast<int>(stair_primitive_id_map_.size())) {
        return true;
    }

    const int from_stair_id = stair_primitive_id_map_[from_idx];
    const int to_stair_id = stair_primitive_id_map_[to_idx];
    if (from_stair_id < 0 || to_stair_id < 0 || from_stair_id != to_stair_id) {
        return true;
    }

    if (runtime_blocked_stair_uphill_ids_.find(from_stair_id) ==
        runtime_blocked_stair_uphill_ids_.end()) {
        return true;
    }

    if (from_stair_id >= static_cast<int>(stair_primitives_.size())) {
        return true;
    }

    const auto& primitive = stair_primitives_[from_stair_id];
    const Eigen::Vector2d normal = primitive.normal;
    if (normal.squaredNorm() < 1e-9) {
        return true;
    }

    const double from_wx = origin_x_ + (from_x + 0.5) * resolution_;
    const double from_wy = origin_y_ + (from_y + 0.5) * resolution_;
    const double to_wx = origin_x_ + (to_x + 0.5) * resolution_;
    const double to_wy = origin_y_ + (to_y + 0.5) * resolution_;
    const Eigen::Vector2d delta(to_wx - from_wx, to_wy - from_wy);
    const double uphill_projection = delta.dot(normal);

    const double uphill_eps = std::max(1e-6, resolution_ * 0.05);
    if (uphill_projection > uphill_eps) {
        return false;
    }

    return true;
}

void LayeredMapManager::getForbiddenTransitionSegments(
    std::vector<std::array<double, 4>>& segments) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    segments.clear();
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0 ||
        (stair_forbidden_transitions_.empty() && fly_slope_forbidden_transitions_.empty())) {
        return;
    }

    segments.reserve(stair_forbidden_transitions_.size() + fly_slope_forbidden_transitions_.size());
    auto append_segments = [&](const std::unordered_set<uint64_t>& transitions) {
        for (const auto key : transitions) {
            int from_idx = static_cast<int>(static_cast<uint32_t>(key >> 32));
            int to_idx = static_cast<int>(static_cast<uint32_t>(key & 0xFFFFFFFFULL));
            if (from_idx < 0 || to_idx < 0 || from_idx >= width_ * height_ || to_idx >= width_ * height_) {
                continue;
            }

            int from_x = from_idx % width_;
            int from_y = from_idx / width_;
            int to_x = to_idx % width_;
            int to_y = to_idx / width_;

            double from_wx = origin_x_ + (from_x + 0.5) * resolution_;
            double from_wy = origin_y_ + (from_y + 0.5) * resolution_;
            double to_wx = origin_x_ + (to_x + 0.5) * resolution_;
            double to_wy = origin_y_ + (to_y + 0.5) * resolution_;

            segments.push_back({from_wx, from_wy, to_wx, to_wy});
        }
    };

    append_segments(stair_forbidden_transitions_);
    append_segments(fly_slope_forbidden_transitions_);
}

bool LayeredMapManager::getStairTraverseNormal(double wx, double wy, double& nx, double& ny) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!stair_layer_cfg_.enable || stair_normal_valid_.empty()) {
        return false;
    }

    int idx = -1;
    if (!worldToGlobalIndex(wx, wy, idx)) {
        return false;
    }

    if (idx < 0 || idx >= static_cast<int>(stair_normal_valid_.size()) ||
        stair_normal_valid_[idx] == 0) {
        return false;
    }

    nx = stair_normal_x_[idx];
    ny = stair_normal_y_[idx];
    return true;
}

bool LayeredMapManager::getStairPrimitiveAt(
    double wx, double wy, StairPrimitive& primitive) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!stair_layer_cfg_.enable || stair_primitives_.empty() || stair_primitive_id_map_.empty()) {
        return false;
    }

    int idx = -1;
    if (!worldToGlobalIndex(wx, wy, idx)) {
        return false;
    }
    if (idx < 0 || idx >= static_cast<int>(stair_primitive_id_map_.size())) {
        return false;
    }

    const int stair_id = stair_primitive_id_map_[idx];
    if (stair_id < 0 || stair_id >= static_cast<int>(stair_primitives_.size())) {
        return false;
    }

    primitive = stair_primitives_[stair_id];
    return true;
}

bool LayeredMapManager::getStairPrimitiveById(
    int stair_id, StairPrimitive& primitive) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (stair_id < 0 || stair_id >= static_cast<int>(stair_primitives_.size())) {
        return false;
    }
    primitive = stair_primitives_[stair_id];
    return true;
}

std::vector<LayeredMapManager::StairPrimitive> LayeredMapManager::getStairPrimitives() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return stair_primitives_;
}

bool LayeredMapManager::getFlySlopeTraverseNormal(
    double wx, double wy, double& nx, double& ny) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!fly_slope_layer_cfg_.enable || fly_slope_normal_valid_.empty()) {
        return false;
    }

    int idx = -1;
    if (!worldToGlobalIndex(wx, wy, idx)) {
        return false;
    }

    if (idx < 0 || idx >= static_cast<int>(fly_slope_normal_valid_.size()) ||
        fly_slope_normal_valid_[idx] == 0) {
        return false;
    }

    nx = fly_slope_normal_x_[idx];
    ny = fly_slope_normal_y_[idx];
    return true;
}

bool LayeredMapManager::getFlySlopePrimitiveAt(
    double wx, double wy, FlySlopePrimitive& primitive) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!fly_slope_layer_cfg_.enable || fly_slope_primitives_.empty() ||
        fly_slope_primitive_id_map_.empty()) {
        return false;
    }

    int idx = -1;
    if (!worldToGlobalIndex(wx, wy, idx)) {
        return false;
    }
    if (idx < 0 || idx >= static_cast<int>(fly_slope_primitive_id_map_.size())) {
        return false;
    }

    const int fly_slope_id = fly_slope_primitive_id_map_[idx];
    if (fly_slope_id < 0 || fly_slope_id >= static_cast<int>(fly_slope_primitives_.size())) {
        return false;
    }

    primitive = fly_slope_primitives_[fly_slope_id];
    return true;
}

bool LayeredMapManager::getFlySlopePrimitiveById(
    int fly_slope_id, FlySlopePrimitive& primitive) const {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (fly_slope_id < 0 || fly_slope_id >= static_cast<int>(fly_slope_primitives_.size())) {
        return false;
    }

    primitive = fly_slope_primitives_[fly_slope_id];
    return true;
}

std::vector<LayeredMapManager::FlySlopePrimitive> LayeredMapManager::getFlySlopePrimitives() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return fly_slope_primitives_;
}

void LayeredMapManager::rebuildStairLayerCache() {
    stair_clear_indices_.clear();
    stair_forbidden_transitions_.clear();
    stair_primitives_.clear();

    if (width_ > 0 && height_ > 0) {
        const size_t n = static_cast<size_t>(width_ * height_);
        stair_normal_x_.assign(n, 0.0f);
        stair_normal_y_.assign(n, 0.0f);
        stair_normal_valid_.assign(n, 0);
        stair_primitive_id_map_.assign(n, -1);
    } else {
        stair_normal_x_.clear();
        stair_normal_y_.clear();
        stair_normal_valid_.clear();
        stair_primitive_id_map_.clear();
    }

    if (!stair_layer_cfg_.enable || stair_layer_cfg_.mask_yaml_path.empty()) {
        return;
    }

    if (!static_map_) {
        return;
    }

    if (!stair_mask_loaded_ || loaded_stair_mask_yaml_ != stair_layer_cfg_.mask_yaml_path) {
        if (!loadStairMaskFromYaml(stair_layer_cfg_.mask_yaml_path)) {
            RCLCPP_WARN(logger_, "stair_layer 已启用但 mask 加载失败，跳过覆盖");
            return;
        }
    }

    if (stair_mask_width_ <= 0 || stair_mask_height_ <= 0 || stair_mask_pixels_.empty()) {
        return;
    }

    std::vector<uint8_t> class_map(static_cast<size_t>(stair_mask_width_ * stair_mask_height_), 0);
    std::vector<std::pair<int, int>> bidir_black_cells;
    std::vector<std::pair<int, int>> level2_black_cells;
    std::vector<std::pair<int, int>> oneway_black_cells;
    bidir_black_cells.reserve(1024);
    level2_black_cells.reserve(1024);
    oneway_black_cells.reserve(1024);

    for (int my = 0; my < stair_mask_height_; ++my) {
        for (int mx = 0; mx < stair_mask_width_; ++mx) {
            int pgm_idx = (stair_mask_height_ - 1 - my) * stair_mask_width_ + mx;
            int mask_idx = my * stair_mask_width_ + mx;
            int v = stair_mask_pixels_[pgm_idx];

            if (v >= stair_layer_cfg_.black_min && v <= stair_layer_cfg_.black_max) {
                class_map[mask_idx] = 1;
                bidir_black_cells.emplace_back(mx, my);
            } else if (v >= stair_layer_cfg_.gray_min && v <= stair_layer_cfg_.gray_max) {
                class_map[mask_idx] = 2;
            } else if (v >= stair_layer_cfg_.stair_level2_black_min &&
                       v <= stair_layer_cfg_.stair_level2_black_max) {
                class_map[mask_idx] = 5;
                level2_black_cells.emplace_back(mx, my);
            } else if (v >= stair_layer_cfg_.stair_level2_gray_min &&
                       v <= stair_layer_cfg_.stair_level2_gray_max) {
                class_map[mask_idx] = 6;
            } else if (stair_layer_cfg_.enable_oneway_stair_down &&
                       v >= stair_layer_cfg_.oneway_black_min &&
                       v <= stair_layer_cfg_.oneway_black_max) {
                class_map[mask_idx] = 3;
                oneway_black_cells.emplace_back(mx, my);
            } else if (stair_layer_cfg_.enable_oneway_stair_down &&
                       v >= stair_layer_cfg_.oneway_gray_min &&
                       v <= stair_layer_cfg_.oneway_gray_max) {
                class_map[mask_idx] = 4;
            }
        }
    }

    if (bidir_black_cells.empty() && level2_black_cells.empty() && oneway_black_cells.empty()) {
        RCLCPP_WARN(logger_, "stair_layer: mask 中未找到任何台阶黑线像素");
        return;
    }

    struct PrimitiveAccumulator {
        bool is_oneway_down{false};
        bool is_level2{false};
        std::vector<std::pair<int, int>> black_cells;
        Eigen::Vector2d normal_sum = Eigen::Vector2d::Zero();
        int normal_count{0};
    };
    std::vector<PrimitiveAccumulator> primitive_acc;
    std::vector<int> mask_primitive_id(
        static_cast<size_t>(stair_mask_width_ * stair_mask_height_), -1);

    auto is_black_cell = [&](int cls) {
        return (cls == 1) || (cls == 5) ||
               (stair_layer_cfg_.enable_oneway_stair_down && cls == 3);
    };

    for (int my = 0; my < stair_mask_height_; ++my) {
        for (int mx = 0; mx < stair_mask_width_; ++mx) {
            const int idx = my * stair_mask_width_ + mx;
            if (!is_black_cell(class_map[idx]) || mask_primitive_id[idx] >= 0) {
                continue;
            }

            PrimitiveAccumulator acc;
            std::vector<int> stack;
            stack.push_back(idx);
            mask_primitive_id[idx] = static_cast<int>(primitive_acc.size());

            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                const int cx = cur % stair_mask_width_;
                const int cy = cur / stair_mask_width_;

                const int cls = class_map[cur];
                if (cls == 3) {
                    acc.is_oneway_down = true;
                } else if (cls == 5) {
                    acc.is_level2 = true;
                }
                acc.black_cells.emplace_back(cx, cy);

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        const int nx = cx + dx;
                        const int ny = cy + dy;
                        if (nx < 0 || nx >= stair_mask_width_ || ny < 0 || ny >= stair_mask_height_) {
                            continue;
                        }
                        const int nidx = ny * stair_mask_width_ + nx;
                        if (!is_black_cell(class_map[nidx]) || mask_primitive_id[nidx] >= 0) {
                            continue;
                        }
                        mask_primitive_id[nidx] = static_cast<int>(primitive_acc.size());
                        stack.push_back(nidx);
                    }
                }
            }

            if (!acc.black_cells.empty()) {
                primitive_acc.push_back(std::move(acc));
            }
        }
    }

    std::unordered_set<int> clear_idx_set;
    clear_idx_set.reserve(
        (bidir_black_cells.size() + level2_black_cells.size() + oneway_black_cells.size()) * 8);

    const int search_r = std::max(1, stair_layer_cfg_.pair_search_radius_cells);
    const double clear_high_m = std::max(
        0.0,
        (stair_layer_cfg_.clear_perp_high_dist_m >= 0.0)
            ? stair_layer_cfg_.clear_perp_high_dist_m
            : stair_layer_cfg_.clear_perp_dist_m);
    const double clear_low_m = std::max(
        0.0,
        (stair_layer_cfg_.clear_perp_low_dist_m >= 0.0)
            ? stair_layer_cfg_.clear_perp_low_dist_m
            : stair_layer_cfg_.clear_perp_dist_m);
    const int clear_high_steps =
        std::max(0, static_cast<int>(std::ceil(clear_high_m / resolution_)));
    const int clear_low_steps =
        std::max(0, static_cast<int>(std::ceil(clear_low_m / resolution_)));

    const double level2_clear_high_m = std::max(
        0.0,
        (stair_layer_cfg_.stair_level2_clear_perp_high_dist_m >= 0.0)
            ? stair_layer_cfg_.stair_level2_clear_perp_high_dist_m
            : stair_layer_cfg_.stair_level2_clear_perp_dist_m);
    const double level2_clear_low_m = std::max(
        0.0,
        (stair_layer_cfg_.stair_level2_clear_perp_low_dist_m >= 0.0)
            ? stair_layer_cfg_.stair_level2_clear_perp_low_dist_m
            : stair_layer_cfg_.stair_level2_clear_perp_dist_m);
    const int level2_clear_high_steps =
        std::max(0, static_cast<int>(std::ceil(level2_clear_high_m / resolution_)));
    const int level2_clear_low_steps =
        std::max(0, static_cast<int>(std::ceil(level2_clear_low_m / resolution_)));

    auto add_clear_sample = [&](double wx, double wy, double nx, double ny, int stair_id) {
        int global_idx = -1;
        if (worldToGlobalIndex(wx, wy, global_idx)) {
            clear_idx_set.insert(global_idx);
            if (global_idx >= 0 && global_idx < static_cast<int>(stair_normal_valid_.size())) {
                stair_normal_x_[global_idx] += static_cast<float>(nx);
                stair_normal_y_[global_idx] += static_cast<float>(ny);
                stair_normal_valid_[global_idx] = 1;
                if (stair_id >= 0 &&
                    stair_id < static_cast<int>(primitive_acc.size()) &&
                    global_idx < static_cast<int>(stair_primitive_id_map_.size()) &&
                    stair_primitive_id_map_[global_idx] < 0) {
                    stair_primitive_id_map_[global_idx] = stair_id;
                }
            }
        }
    };

    auto process_stair_type = [&](const std::vector<std::pair<int, int>>& black_cells,
                                  uint8_t gray_class,
                                  bool build_oneway_forbidden,
                                  int cl_high_steps, int cl_low_steps) {
        for (size_t i = 0; i < black_cells.size(); ++i) {
            int bx = black_cells[i].first;
            int by = black_cells[i].second;
            int stair_id = -1;
            const int mask_idx = by * stair_mask_width_ + bx;
            if (mask_idx >= 0 && mask_idx < static_cast<int>(mask_primitive_id.size())) {
                stair_id = mask_primitive_id[mask_idx];
            }

            double best_dist2 = std::numeric_limits<double>::max();
            int gx_best = -1;
            int gy_best = -1;

            for (int dy = -search_r; dy <= search_r; ++dy) {
                for (int dx = -search_r; dx <= search_r; ++dx) {
                    int gx = bx + dx;
                    int gy = by + dy;
                    if (gx < 0 || gx >= stair_mask_width_ || gy < 0 || gy >= stair_mask_height_) {
                        continue;
                    }
                    int idx = gy * stair_mask_width_ + gx;
                    if (class_map[idx] != gray_class) {
                        continue;
                    }
                    double d2 = static_cast<double>(dx * dx + dy * dy);
                    if (d2 < best_dist2) {
                        best_dist2 = d2;
                        gx_best = gx;
                        gy_best = gy;
                    }
                }
            }

            if (gx_best < 0 || gy_best < 0) {
                continue;
            }

            double bwx = stair_mask_origin_x_ + (bx + 0.5) * stair_mask_resolution_;
            double bwy = stair_mask_origin_y_ + (by + 0.5) * stair_mask_resolution_;
            double gwx = stair_mask_origin_x_ + (gx_best + 0.5) * stair_mask_resolution_;
            double gwy = stair_mask_origin_y_ + (gy_best + 0.5) * stair_mask_resolution_;

            double nx = bwx - gwx;
            double ny = bwy - gwy;
            double norm = std::hypot(nx, ny);
            if (norm < 1e-5) {
                continue;
            }
            nx /= norm;
            ny /= norm;
            if (stair_id >= 0 && stair_id < static_cast<int>(primitive_acc.size())) {
                primitive_acc[stair_id].normal_sum += Eigen::Vector2d(nx, ny);
                primitive_acc[stair_id].normal_count++;
            }

            // 1) 完整清除已匹配的高低像素对：灰侧->黑侧连线（包含两端）
            const double pair_dist = std::hypot(bwx - gwx, bwy - gwy);
            const int pair_steps = std::max(1, static_cast<int>(std::ceil(pair_dist / resolution_)));
            for (int s = 0; s <= pair_steps; ++s) {
                const double t = static_cast<double>(s) / static_cast<double>(pair_steps);
                const double wx = gwx + (bwx - gwx) * t;
                const double wy = gwy + (bwy - gwy) * t;
                add_clear_sample(wx, wy, nx, ny, stair_id);
            }

            // 2) 沿法向扩展清除（高侧 +n，低侧 -n），距离可调
            for (int s = 1; s <= cl_high_steps; ++s) {
                const double wx = bwx + nx * (s * resolution_);
                const double wy = bwy + ny * (s * resolution_);
                add_clear_sample(wx, wy, nx, ny, stair_id);
            }
            for (int s = 1; s <= cl_low_steps; ++s) {
                const double wx = gwx - nx * (s * resolution_);
                const double wy = gwy - ny * (s * resolution_);
                add_clear_sample(wx, wy, nx, ny, stair_id);
            }

            if (build_oneway_forbidden) {
                int low_idx = -1;
                int high_idx = -1;
                if (worldToGlobalIndex(gwx, gwy, low_idx) && worldToGlobalIndex(bwx, bwy, high_idx)) {
                    // 注册主路径 low → high（low 格到 high 格逐步禁止）
                    addForbiddenDirectedTransitions(
                        low_idx, high_idx, stair_forbidden_transitions_);

                    // 修复：A* 8方向扩展时，对角移动可以从 low 的相邻格直接跳到
                    // high_idx，绕过仅注册了 low→high 直线路径的屏障。
                    // 对策：把 low_idx 3×3 邻域内所有能一步到达 high_idx 的格子
                    // 也全部注册为禁止出发点，堵住对角入口。
                    const int hx = high_idx % width_;
                    const int hy = high_idx / width_;
                    const int lx = low_idx % width_;
                    const int ly = low_idx / width_;
                    for (int ddy = -1; ddy <= 1; ++ddy) {
                        for (int ddx = -1; ddx <= 1; ++ddx) {
                            if (ddx == 0 && ddy == 0) {
                                continue;
                            }
                            int nx2 = lx + ddx;
                            int ny2 = ly + ddy;
                            if (nx2 < 0 || nx2 >= width_ || ny2 < 0 || ny2 >= height_) {
                                continue;
                            }
                            // 只在该邻居能一步到达 high_idx 时注册（即 Chebyshev 距离 ≤ 1）
                            if (std::abs(nx2 - hx) <= 1 && std::abs(ny2 - hy) <= 1) {
                                stair_forbidden_transitions_.insert(
                                    encodeDirectedTransition(ny2 * width_ + nx2, high_idx));
                            }
                        }
                    }
                }
            }
        }
    };

    process_stair_type(bidir_black_cells, 2, false, clear_high_steps, clear_low_steps);
    process_stair_type(level2_black_cells, 6, false, level2_clear_high_steps, level2_clear_low_steps);
    if (stair_layer_cfg_.enable_oneway_stair_down) {
        process_stair_type(oneway_black_cells, 4, true, clear_high_steps, clear_low_steps);
    }

    for (int idx : clear_idx_set) {
        if (idx < 0 || idx >= static_cast<int>(stair_normal_valid_.size()) ||
            stair_normal_valid_[idx] == 0) {
            continue;
        }

        double nx = stair_normal_x_[idx];
        double ny = stair_normal_y_[idx];
        double norm = std::hypot(nx, ny);
        if (norm < 1e-6) {
            stair_normal_valid_[idx] = 0;
            stair_normal_x_[idx] = 0.0f;
            stair_normal_y_[idx] = 0.0f;
            continue;
        }

        stair_normal_x_[idx] = static_cast<float>(nx / norm);
        stair_normal_y_[idx] = static_cast<float>(ny / norm);
    }

    stair_clear_indices_.assign(clear_idx_set.begin(), clear_idx_set.end());

    stair_primitives_.reserve(primitive_acc.size());
    for (size_t pid = 0; pid < primitive_acc.size(); ++pid) {
        const auto& acc = primitive_acc[pid];
        if (acc.black_cells.empty()) {
            continue;
        }

        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        std::vector<Eigen::Vector2d> pts;
        pts.reserve(acc.black_cells.size());
        for (const auto& c : acc.black_cells) {
            const double wx = stair_mask_origin_x_ + (c.first + 0.5) * stair_mask_resolution_;
            const double wy = stair_mask_origin_y_ + (c.second + 0.5) * stair_mask_resolution_;
            Eigen::Vector2d p(wx, wy);
            pts.push_back(p);
            center += p;
        }
        center /= static_cast<double>(pts.size());

        Eigen::Vector2d normal = Eigen::Vector2d::UnitX();
        if (acc.normal_count > 0 && acc.normal_sum.norm() > 1e-6) {
            normal = acc.normal_sum.normalized();
        }

        Eigen::Vector2d tangent(-normal.y(), normal.x());
        if (pts.size() >= 2) {
            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (const auto& p : pts) {
                Eigen::Vector2d d = p - center;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
            if (solver.info() == Eigen::Success) {
                tangent = solver.eigenvectors().col(1);
            }
        }

        tangent -= tangent.dot(normal) * normal;
        if (tangent.norm() < 1e-6) {
            tangent = Eigen::Vector2d(-normal.y(), normal.x());
        }
        tangent.normalize();

        if (normal.norm() < 1e-6) {
            normal = Eigen::Vector2d(-tangent.y(), tangent.x());
        } else {
            normal.normalize();
        }

        double half_length = 0.5 * stair_mask_resolution_;
        for (const auto& p : pts) {
            half_length = std::max(half_length, std::abs((p - center).dot(tangent)));
        }

        StairPrimitive primitive;
        primitive.stair_id = static_cast<int>(pid);
        primitive.center = center;
        primitive.normal = normal;
        primitive.tangent = tangent;
        primitive.half_length = half_length;
        primitive.crossing_band.low_side_dist_m = clear_low_m;
        primitive.crossing_band.high_side_dist_m = clear_high_m;
        primitive.crossing_band.tangent_half_width_m = half_length;
        primitive.is_oneway_down = acc.is_oneway_down;
        primitive.is_level2 = acc.is_level2;
        stair_primitives_.push_back(primitive);
    }

    RCLCPP_INFO(logger_,
                "stair_layer: 双向黑线=%zu, 二级黑线=%zu, 单向黑线=%zu, 对象=%zu, 清除栅格=%zu, 禁止边=%zu",
                bidir_black_cells.size(),
                level2_black_cells.size(),
                oneway_black_cells.size(),
                stair_primitives_.size(),
                stair_clear_indices_.size(),
                stair_forbidden_transitions_.size());
}

void LayeredMapManager::rebuildFlySlopeLayerCache() {
    fly_slope_clear_indices_.clear();
    fly_slope_forbidden_transitions_.clear();
    fly_slope_primitives_.clear();

    if (width_ > 0 && height_ > 0) {
        const size_t n = static_cast<size_t>(width_ * height_);
        fly_slope_normal_x_.assign(n, 0.0f);
        fly_slope_normal_y_.assign(n, 0.0f);
        fly_slope_normal_valid_.assign(n, 0);
        fly_slope_primitive_id_map_.assign(n, -1);
    } else {
        fly_slope_normal_x_.clear();
        fly_slope_normal_y_.clear();
        fly_slope_normal_valid_.clear();
        fly_slope_primitive_id_map_.clear();
    }

    if (!fly_slope_layer_cfg_.enable || fly_slope_layer_cfg_.mask_yaml_path.empty()) {
        return;
    }

    if (!static_map_) {
        return;
    }

    if (!fly_slope_mask_loaded_ ||
        loaded_fly_slope_mask_yaml_ != fly_slope_layer_cfg_.mask_yaml_path) {
        if (!loadFlySlopeMaskFromYaml(fly_slope_layer_cfg_.mask_yaml_path)) {
            RCLCPP_WARN(logger_, "fly_slope_layer 已启用但 mask 加载失败，跳过覆盖");
            return;
        }
    }

    if (fly_slope_mask_width_ <= 0 || fly_slope_mask_height_ <= 0 ||
        fly_slope_mask_pixels_.empty()) {
        return;
    }

    std::vector<uint8_t> class_map(
        static_cast<size_t>(fly_slope_mask_width_ * fly_slope_mask_height_), 0);
    std::vector<std::pair<int, int>> low_cells;
    low_cells.reserve(1024);

    for (int my = 0; my < fly_slope_mask_height_; ++my) {
        for (int mx = 0; mx < fly_slope_mask_width_; ++mx) {
            int pgm_idx = (fly_slope_mask_height_ - 1 - my) * fly_slope_mask_width_ + mx;
            int mask_idx = my * fly_slope_mask_width_ + mx;
            int v = fly_slope_mask_pixels_[pgm_idx];

            if (v >= fly_slope_layer_cfg_.low_min && v <= fly_slope_layer_cfg_.low_max) {
                class_map[mask_idx] = 1;
                low_cells.emplace_back(mx, my);
            } else if (v >= fly_slope_layer_cfg_.high_min &&
                       v <= fly_slope_layer_cfg_.high_max) {
                class_map[mask_idx] = 2;
            }
        }
    }

    if (low_cells.empty()) {
        RCLCPP_WARN(logger_, "fly_slope_layer: mask 中未找到任何飞坡低侧线像素");
        return;
    }

    struct PrimitiveAccumulator {
        std::vector<std::pair<int, int>> low_side_cells;
        Eigen::Vector2d normal_sum = Eigen::Vector2d::Zero();
        int normal_count{0};
    };

    std::vector<PrimitiveAccumulator> primitive_acc;
    std::vector<int> mask_primitive_id(
        static_cast<size_t>(fly_slope_mask_width_ * fly_slope_mask_height_), -1);

    for (int my = 0; my < fly_slope_mask_height_; ++my) {
        for (int mx = 0; mx < fly_slope_mask_width_; ++mx) {
            const int idx = my * fly_slope_mask_width_ + mx;
            if (class_map[idx] != 1 || mask_primitive_id[idx] >= 0) {
                continue;
            }

            PrimitiveAccumulator acc;
            std::vector<int> stack;
            stack.push_back(idx);
            mask_primitive_id[idx] = static_cast<int>(primitive_acc.size());

            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                const int cx = cur % fly_slope_mask_width_;
                const int cy = cur / fly_slope_mask_width_;
                acc.low_side_cells.emplace_back(cx, cy);

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        const int nx = cx + dx;
                        const int ny = cy + dy;
                        if (nx < 0 || nx >= fly_slope_mask_width_ || ny < 0 ||
                            ny >= fly_slope_mask_height_) {
                            continue;
                        }
                        const int nidx = ny * fly_slope_mask_width_ + nx;
                        if (class_map[nidx] != 1 || mask_primitive_id[nidx] >= 0) {
                            continue;
                        }
                        mask_primitive_id[nidx] = static_cast<int>(primitive_acc.size());
                        stack.push_back(nidx);
                    }
                }
            }

            if (!acc.low_side_cells.empty()) {
                primitive_acc.push_back(std::move(acc));
            }
        }
    }

    std::unordered_set<int> clear_idx_set;
    clear_idx_set.reserve(low_cells.size() * 8);

    const int search_r = std::max(1, fly_slope_layer_cfg_.pair_search_radius_cells);
    const double clear_high_m = std::max(
        0.0,
        (fly_slope_layer_cfg_.clear_perp_high_dist_m >= 0.0)
            ? fly_slope_layer_cfg_.clear_perp_high_dist_m
            : fly_slope_layer_cfg_.clear_perp_dist_m);
    const double clear_low_m = std::max(
        0.0,
        (fly_slope_layer_cfg_.clear_perp_low_dist_m >= 0.0)
            ? fly_slope_layer_cfg_.clear_perp_low_dist_m
            : fly_slope_layer_cfg_.clear_perp_dist_m);
    const int clear_high_steps =
        std::max(0, static_cast<int>(std::ceil(clear_high_m / resolution_)));
    const int clear_low_steps =
        std::max(0, static_cast<int>(std::ceil(clear_low_m / resolution_)));

    auto add_clear_sample = [&](double wx, double wy, double nx, double ny, int slope_id) {
        int global_idx = -1;
        if (worldToGlobalIndex(wx, wy, global_idx)) {
            clear_idx_set.insert(global_idx);
            if (global_idx >= 0 && global_idx < static_cast<int>(fly_slope_normal_valid_.size())) {
                fly_slope_normal_x_[global_idx] += static_cast<float>(nx);
                fly_slope_normal_y_[global_idx] += static_cast<float>(ny);
                fly_slope_normal_valid_[global_idx] = 1;
                if (slope_id >= 0 && slope_id < static_cast<int>(primitive_acc.size()) &&
                    global_idx < static_cast<int>(fly_slope_primitive_id_map_.size()) &&
                    fly_slope_primitive_id_map_[global_idx] < 0) {
                    fly_slope_primitive_id_map_[global_idx] = slope_id;
                }
            }
        }
    };

    for (const auto& low_cell : low_cells) {
        int lx = low_cell.first;
        int ly = low_cell.second;
        int slope_id = -1;
        const int mask_idx = ly * fly_slope_mask_width_ + lx;
        if (mask_idx >= 0 && mask_idx < static_cast<int>(mask_primitive_id.size())) {
            slope_id = mask_primitive_id[mask_idx];
        }

        double best_dist2 = std::numeric_limits<double>::max();
        int hx_best = -1;
        int hy_best = -1;

        for (int dy = -search_r; dy <= search_r; ++dy) {
            for (int dx = -search_r; dx <= search_r; ++dx) {
                int hx = lx + dx;
                int hy = ly + dy;
                if (hx < 0 || hx >= fly_slope_mask_width_ || hy < 0 ||
                    hy >= fly_slope_mask_height_) {
                    continue;
                }
                int idx = hy * fly_slope_mask_width_ + hx;
                if (class_map[idx] != 2) {
                    continue;
                }
                double d2 = static_cast<double>(dx * dx + dy * dy);
                if (d2 < best_dist2) {
                    best_dist2 = d2;
                    hx_best = hx;
                    hy_best = hy;
                }
            }
        }

        if (hx_best < 0 || hy_best < 0) {
            continue;
        }

        const double low_wx = fly_slope_mask_origin_x_ + (lx + 0.5) * fly_slope_mask_resolution_;
        const double low_wy = fly_slope_mask_origin_y_ + (ly + 0.5) * fly_slope_mask_resolution_;
        const double high_wx = fly_slope_mask_origin_x_ + (hx_best + 0.5) * fly_slope_mask_resolution_;
        const double high_wy = fly_slope_mask_origin_y_ + (hy_best + 0.5) * fly_slope_mask_resolution_;

        double nx = high_wx - low_wx;  // low -> high
        double ny = high_wy - low_wy;
        double norm = std::hypot(nx, ny);
        if (norm < 1e-5) {
            continue;
        }
        nx /= norm;
        ny /= norm;
        if (slope_id >= 0 && slope_id < static_cast<int>(primitive_acc.size())) {
            primitive_acc[slope_id].normal_sum += Eigen::Vector2d(nx, ny);
            primitive_acc[slope_id].normal_count++;
        }

        const double pair_dist = std::hypot(high_wx - low_wx, high_wy - low_wy);
        const int pair_steps = std::max(1, static_cast<int>(std::ceil(pair_dist / resolution_)));
        for (int s = 0; s <= pair_steps; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(pair_steps);
            const double wx = low_wx + (high_wx - low_wx) * t;
            const double wy = low_wy + (high_wy - low_wy) * t;
            add_clear_sample(wx, wy, nx, ny, slope_id);
        }

        for (int s = 1; s <= clear_high_steps; ++s) {
            const double wx = high_wx + nx * (s * resolution_);
            const double wy = high_wy + ny * (s * resolution_);
            add_clear_sample(wx, wy, nx, ny, slope_id);
        }
        for (int s = 1; s <= clear_low_steps; ++s) {
            const double wx = low_wx - nx * (s * resolution_);
            const double wy = low_wy - ny * (s * resolution_);
            add_clear_sample(wx, wy, nx, ny, slope_id);
        }

        if (fly_slope_layer_cfg_.enable_oneway_low_to_high) {
            int low_idx = -1;
            int high_idx = -1;
            if (worldToGlobalIndex(low_wx, low_wy, low_idx) &&
                worldToGlobalIndex(high_wx, high_wy, high_idx)) {
                // 仅允许 low->high，因此禁止 high->low
                addForbiddenDirectedTransitions(high_idx, low_idx, fly_slope_forbidden_transitions_);

                // 对角补洞：阻止 high 邻域一步跳到 low
                const int hx = high_idx % width_;
                const int hy = high_idx / width_;
                const int lxg = low_idx % width_;
                const int lyg = low_idx / width_;
                for (int ddy = -1; ddy <= 1; ++ddy) {
                    for (int ddx = -1; ddx <= 1; ++ddx) {
                        if (ddx == 0 && ddy == 0) {
                            continue;
                        }
                        int nx2 = hx + ddx;
                        int ny2 = hy + ddy;
                        if (nx2 < 0 || nx2 >= width_ || ny2 < 0 || ny2 >= height_) {
                            continue;
                        }
                        if (std::abs(nx2 - lxg) <= 1 && std::abs(ny2 - lyg) <= 1) {
                            fly_slope_forbidden_transitions_.insert(
                                encodeDirectedTransition(ny2 * width_ + nx2, low_idx));
                        }
                    }
                }
            }
        }
    }

    for (int idx : clear_idx_set) {
        if (idx < 0 || idx >= static_cast<int>(fly_slope_normal_valid_.size()) ||
            fly_slope_normal_valid_[idx] == 0) {
            continue;
        }

        double nx = fly_slope_normal_x_[idx];
        double ny = fly_slope_normal_y_[idx];
        double norm = std::hypot(nx, ny);
        if (norm < 1e-6) {
            fly_slope_normal_valid_[idx] = 0;
            fly_slope_normal_x_[idx] = 0.0f;
            fly_slope_normal_y_[idx] = 0.0f;
            continue;
        }

        fly_slope_normal_x_[idx] = static_cast<float>(nx / norm);
        fly_slope_normal_y_[idx] = static_cast<float>(ny / norm);
    }

    fly_slope_clear_indices_.assign(clear_idx_set.begin(), clear_idx_set.end());

    fly_slope_primitives_.reserve(primitive_acc.size());
    for (size_t pid = 0; pid < primitive_acc.size(); ++pid) {
        const auto& acc = primitive_acc[pid];
        if (acc.low_side_cells.empty()) {
            continue;
        }

        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        std::vector<Eigen::Vector2d> pts;
        pts.reserve(acc.low_side_cells.size());
        for (const auto& c : acc.low_side_cells) {
            const double wx = fly_slope_mask_origin_x_ + (c.first + 0.5) * fly_slope_mask_resolution_;
            const double wy = fly_slope_mask_origin_y_ + (c.second + 0.5) * fly_slope_mask_resolution_;
            Eigen::Vector2d p(wx, wy);
            pts.push_back(p);
            center += p;
        }
        center /= static_cast<double>(pts.size());

        Eigen::Vector2d normal = Eigen::Vector2d::UnitX();
        if (acc.normal_count > 0 && acc.normal_sum.norm() > 1e-6) {
            normal = acc.normal_sum.normalized();
        }

        Eigen::Vector2d tangent(-normal.y(), normal.x());
        if (pts.size() >= 2) {
            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (const auto& p : pts) {
                Eigen::Vector2d d = p - center;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
            if (solver.info() == Eigen::Success) {
                tangent = solver.eigenvectors().col(1);
            }
        }

        tangent -= tangent.dot(normal) * normal;
        if (tangent.norm() < 1e-6) {
            tangent = Eigen::Vector2d(-normal.y(), normal.x());
        }
        tangent.normalize();

        if (normal.norm() < 1e-6) {
            normal = Eigen::Vector2d(-tangent.y(), tangent.x());
        } else {
            normal.normalize();
        }

        double half_length = 0.5 * fly_slope_mask_resolution_;
        for (const auto& p : pts) {
            half_length = std::max(half_length, std::abs((p - center).dot(tangent)));
        }

        FlySlopePrimitive primitive;
        primitive.fly_slope_id = static_cast<int>(pid);
        primitive.center = center;
        primitive.normal = normal;
        primitive.tangent = tangent;
        primitive.half_length = half_length;
        primitive.crossing_band.low_side_dist_m = clear_low_m;
        primitive.crossing_band.high_side_dist_m = clear_high_m;
        primitive.crossing_band.tangent_half_width_m = half_length;
        primitive.is_oneway_low_to_high = fly_slope_layer_cfg_.enable_oneway_low_to_high;
        fly_slope_primitives_.push_back(primitive);
    }

    RCLCPP_INFO(logger_,
                "fly_slope_layer: low线=%zu, 对象=%zu, 清除栅格=%zu, 禁止边=%zu",
                low_cells.size(),
                fly_slope_primitives_.size(),
                fly_slope_clear_indices_.size(),
                fly_slope_forbidden_transitions_.size());
}

void LayeredMapManager::applyStairLayerPolicy() {
    if (fused_map_) {
        if (stair_layer_cfg_.enable && !stair_clear_indices_.empty()) {
            for (int idx : stair_clear_indices_) {
                if (idx >= 0 && idx < static_cast<int>(fused_map_->data.size())) {
                    fused_map_->data[idx] = 0;
                }
            }
        }

        if (fly_slope_layer_cfg_.enable && !fly_slope_clear_indices_.empty()) {
            for (int idx : fly_slope_clear_indices_) {
                if (idx >= 0 && idx < static_cast<int>(fused_map_->data.size())) {
                    fused_map_->data[idx] = 0;
                }
            }
        }
    }
}

bool LayeredMapManager::loadStaticMap(const std::string& yaml_path,
                                       const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    static_map_ = StaticMapLoader::load(yaml_path, logger_);
    if (!static_map_) {
        RCLCPP_ERROR(logger_, "Failed to load static map from: %s", yaml_path.c_str());
        return false;
    }
    
    inflation_params_ = params;
    
    // 提取地图元数据
    resolution_ = static_map_->info.resolution;
    origin_x_ = static_map_->info.origin.position.x;
    origin_y_ = static_map_->info.origin.position.y;
    width_ = static_map_->info.width;
    height_ = static_map_->info.height;
    map_frame_ = static_map_->header.frame_id.empty() ? "map" : static_map_->header.frame_id;
    
    // 初始化融合地图（与静态地图相同大小）
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;  // 初始时等于静态地图

    rebuildStairLayerCache();
    rebuildFlySlopeLayerCache();
    applyStairLayerPolicy();
    
    // 构建costmap
    rebuildCostmap();
    
    // 如果启用ESDF，构建初始ESDF
    if (esdf_enabled_) {
        rebuildEsdf();
    }
    
    RCLCPP_INFO(logger_, "Static map loaded: %dx%d @ %.3f m/cell, inflation_radius=%.2f",
                width_, height_, resolution_, params.inflation_radius);
    return true;
}

void LayeredMapManager::setStaticMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    if (!map) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    static_map_ = map;
    
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;
    width_ = map->info.width;
    height_ = map->info.height;
    map_frame_ = map->header.frame_id.empty() ? "map" : map->header.frame_id;
    
    // 初始化融合地图
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;

    rebuildStairLayerCache();
    rebuildFlySlopeLayerCache();
    applyStairLayerPolicy();
    
    rebuildCostmap();
    if (esdf_enabled_) {
        rebuildEsdf();
    }
}

void LayeredMapManager::setInflationParams(const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    inflation_params_ = params;
    if (fused_map_) {
        rebuildCostmap();
    }
}

void LayeredMapManager::createBlankStaticMap(double width_m, double height_m,
                                              double resolution, const InflationParams& params) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 计算栅格尺寸
    int width = static_cast<int>(std::ceil(width_m / resolution));
    int height = static_cast<int>(std::ceil(height_m / resolution));
    
    // 创建空白地图
    static_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    static_map_->header.frame_id = map_frame_;
    static_map_->header.stamp = node_->get_clock()->now();
    
    static_map_->info.resolution = resolution;
    static_map_->info.width = width;
    static_map_->info.height = height;
    
    // 地图中心在原点
    static_map_->info.origin.position.x = -width_m / 2.0;
    static_map_->info.origin.position.y = -height_m / 2.0;
    static_map_->info.origin.position.z = 0.0;
    static_map_->info.origin.orientation.w = 1.0;
    
    // 全部初始化为自由空间 (0)
    static_map_->data.resize(width * height, 0);
    
    // 更新成员变量
    resolution_ = resolution;
    origin_x_ = static_map_->info.origin.position.x;
    origin_y_ = static_map_->info.origin.position.y;
    width_ = width;
    height_ = height;
    
    // 初始化融合地图
    fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    *fused_map_ = *static_map_;

    rebuildStairLayerCache();
    rebuildFlySlopeLayerCache();
    applyStairLayerPolicy();
    
    // 设置膨胀参数并构建Costmap
    inflation_params_ = params;
    rebuildCostmap();
    
    if (esdf_enabled_) {
        rebuildEsdf();
    }
    
    RCLCPP_INFO(logger_, "创建空白静态地图: %dx%d (%.1fm x %.1fm) @ %.3f m/cell",
                width, height, width_m, height_m, resolution);
}

void LayeredMapManager::updateDynamicLayer(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map) {
    
    if (!local_map || !static_map_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 1. 查询 odom -> map 变换
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            map_frame_, odom_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
            "Cannot get %s->%s transform: %s", odom_frame_.c_str(), map_frame_.c_str(), ex.what());
        return;
    }
    
    // 2. 重置融合地图
    if (!fused_map_) {
        fused_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    }
    
    if (static_layer_enabled_) {
        // 静态层启用：融合地图初始化为静态地图
        *fused_map_ = *static_map_;
    } else {
        // 静态层禁用：融合地图初始化为空白（仅保留动态层）
        *fused_map_ = *static_map_;  // 复用结构
        std::fill(fused_map_->data.begin(), fused_map_->data.end(), 0);  // 清空为free space
    }
    fused_map_->header.stamp = node_->get_clock()->now();
    
    // 3. 变换参数
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double yaw = tf2::getYaw(transform.transform.rotation);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    // 局部地图参数
    double local_res = local_map->info.resolution;
    double local_ox = local_map->info.origin.position.x;
    double local_oy = local_map->info.origin.position.y;
    int local_w = local_map->info.width;
    int local_h = local_map->info.height;
    
    // 4. 记录变化区域用于增量ESDF更新
    std::vector<int> changed_cells;
    changed_cells.reserve(local_w * local_h / 4);  // 预估25%变化
    
    DynamicLayerBounds new_bounds;
    new_bounds.min_x = width_;
    new_bounds.min_y = height_;
    new_bounds.max_x = 0;
    new_bounds.max_y = 0;
    
    // 5. 遍历局部地图，投影到全局地图
    for (int ly = 0; ly < local_h; ++ly) {
        for (int lx = 0; lx < local_w; ++lx) {
            int local_idx = ly * local_w + lx;
            int8_t local_val = local_map->data[local_idx];
            
            // 跳过unknown和free（只叠加障碍物）
            if (local_val <= 0) {
                continue;
            }
            
            // 局部栅格中心的odom坐标
            double odom_x = local_ox + (lx + 0.5) * local_res;
            double odom_y = local_oy + (ly + 0.5) * local_res;
            
            // 变换到map坐标系
            double map_x = cos_yaw * odom_x - sin_yaw * odom_y + tx;
            double map_y = sin_yaw * odom_x + cos_yaw * odom_y + ty;
            
            // 计算全局地图索引
            int gx = static_cast<int>((map_x - origin_x_) / resolution_);
            int gy = static_cast<int>((map_y - origin_y_) / resolution_);
            
            // 边界检查
            if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
                continue;
            }
            
            int global_idx = gy * width_ + gx;
            
            // 融合策略
            int8_t fused_val;
            if (static_layer_enabled_) {
                // 静态层启用：取max（保守，障碍物优先）
                int8_t static_val = static_map_->data[global_idx];
                fused_val = std::max(static_val, local_val);
            } else {
                // 静态层禁用：直接使用动态层的值
                fused_val = local_val;
            }
            
            // 记录变化
            if (fused_map_->data[global_idx] != fused_val) {
                fused_map_->data[global_idx] = fused_val;
                changed_cells.push_back(global_idx);
                
                // 更新边界
                new_bounds.min_x = std::min(new_bounds.min_x, gx);
                new_bounds.max_x = std::max(new_bounds.max_x, gx);
                new_bounds.min_y = std::min(new_bounds.min_y, gy);
                new_bounds.max_y = std::max(new_bounds.max_y, gy);
            }
        }
    }
    
    new_bounds.valid = !changed_cells.empty();
    
    // 6. 清除上一次动态层的残留（在新边界外的部分）
    // 保存旧边界用于后续计算
    DynamicLayerBounds old_bounds = last_dynamic_bounds_;
    
    if (old_bounds.valid) {
        for (int gy = old_bounds.min_y; gy <= old_bounds.max_y; ++gy) {
            for (int gx = old_bounds.min_x; gx <= old_bounds.max_x; ++gx) {
                // 跳过当前边界内的栅格（已经更新过了）
                if (new_bounds.valid &&
                    gx >= new_bounds.min_x && gx <= new_bounds.max_x &&
                    gy >= new_bounds.min_y && gy <= new_bounds.max_y) {
                    continue;
                }
                
                int idx = gy * width_ + gx;
                if (idx >= 0 && idx < width_ * height_) {
                    // 恢复基础值
                    int8_t base_val = static_layer_enabled_ ? static_map_->data[idx] : 0;
                    if (fused_map_->data[idx] != base_val) {
                        fused_map_->data[idx] = base_val;
                        changed_cells.push_back(idx);
                    }
                }
            }
        }
    }
    
    // 更新边界记录
    last_dynamic_bounds_ = new_bounds;

    applyStairLayerPolicy();
    
    // 7. 共享EDT计算优化：ESDF和Costmap复用同一次距离场计算
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // 构建ESDF（含EDT计算）
    if (esdf_enabled_) {
        esdf_ = std::make_shared<EsdfMap>();
        esdf_->buildFromOccupancy(fused_map_, obstacle_threshold_);
    }
    auto t_esdf = std::chrono::high_resolution_clock::now();
    
    // 从ESDF复用距离场生成Costmap
    if (esdf_ && esdf_->hasDistance()) {
        rebuildCostmapFromEsdf();
    } else {
        rebuildCostmap();  // Fallback
    }
    auto t_costmap = std::chrono::high_resolution_clock::now();
    
    // 生成ESDF可视化
    if (esdf_enabled_ && esdf_) {
        esdf_vis_ = esdf_->toOccupancyGrid(esdf_vis_max_dist_);
        if (esdf_vis_) {
            esdf_vis_->header.frame_id = map_frame_;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    
    // 性能日志（根据开关决定是否输出）
    if (enable_performance_logging_) {
        auto dur_esdf = std::chrono::duration_cast<std::chrono::microseconds>(t_esdf - t_start);
        auto dur_costmap = std::chrono::duration_cast<std::chrono::microseconds>(t_costmap - t_esdf);
        auto dur_vis = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_costmap);
        auto dur_total = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
        
        RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
            "[共享EDT] ESDF: %.2f ms | Costmap: %.2f ms | Vis: %.2f ms | Total: %.2f ms (节省: %.0f%%)",
            dur_esdf.count() / 1000.0, dur_costmap.count() / 1000.0,
            dur_vis.count() / 1000.0, dur_total.count() / 1000.0,
            100.0 * (103.0 - dur_total.count() / 1000.0) / 103.0);
    }
}

void LayeredMapManager::rebuildFusedMap() {
    // 当前实现：在updateDynamicLayer中直接完成
    // 此函数保留用于未来扩展（如多动态层融合）
}

void LayeredMapManager::rebuildCostmap() {
    if (!fused_map_) return;
    costmap_ = CostmapInflater::inflate(fused_map_, inflation_params_, obstacle_threshold_);
    if (costmap_) {
        costmap_->header.frame_id = map_frame_;
    }
}

void LayeredMapManager::rebuildCostmapFromEsdf() {
    if (!fused_map_ || !esdf_) return;
    
    // 从ESDF获取距离场数据
    std::vector<float> distances(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            distances[idx] = esdf_->getDistanceFast(x, y);
        }
    }
    
    // 使用距离场直接生成costmap（无需重复EDT计算）
    costmap_ = CostmapInflater::fromEsdf(
        distances, width_, height_, resolution_,
        origin_x_, origin_y_, inflation_params_);
    
    if (costmap_) {
        costmap_->header.frame_id = map_frame_;
        costmap_->header.stamp = node_->get_clock()->now();
    }
}

void LayeredMapManager::rebuildEsdf() {
    // 仅用于初始化时的ESDF构建（loadStaticMap/setStaticMap调用）
    // 运行时的ESDF在updateDynamicLayer中与costmap共享EDT构建
    if (!fused_map_) return;
    
    esdf_ = std::make_shared<EsdfMap>();
    esdf_->buildFromOccupancy(fused_map_, obstacle_threshold_);
    esdf_vis_ = esdf_->toOccupancyGrid(esdf_vis_max_dist_);
    if (esdf_vis_) {
        esdf_vis_->header.frame_id = map_frame_;
    }
}

bool LayeredMapManager::transformOdomToMap(double ox, double oy,
                                            double& mx, double& my) const {
    if (!tf_buffer_) {
        return false;
    }
    
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
        
        double tx = transform.transform.translation.x;
        double ty = transform.transform.translation.y;
        double yaw = tf2::getYaw(transform.transform.rotation);
        
        mx = std::cos(yaw) * ox - std::sin(yaw) * oy + tx;
        my = std::sin(yaw) * ox + std::cos(yaw) * oy + ty;
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

// ============ 获取地图 ============

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getStaticMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return static_map_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getFusedMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return fused_map_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getCostmap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return costmap_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr LayeredMapManager::getEsdfVis() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return esdf_vis_;
}

EsdfMap::Ptr LayeredMapManager::getEsdf() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return esdf_;
}

// ============ MapInterface实现 ============

void LayeredMapManager::getBounds(double& min_x, double& min_y,
                                   double& max_x, double& max_y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    min_x = origin_x_;
    min_y = origin_y_;
    max_x = origin_x_ + width_ * resolution_;
    max_y = origin_y_ + height_ * resolution_;
}

nav_core::MapQuery LayeredMapManager::query(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    nav_core::MapQuery result;
    if (!costmap_) return result;
    
    int mx = static_cast<int>((x - origin_x_) / resolution_);
    int my = static_cast<int>((y - origin_y_) / resolution_);
    
    if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
        return result;
    }
    
    result.valid = true;
    int8_t val = costmap_->data[my * width_ + mx];
    result.occupied = (val >= obstacle_threshold_);
    result.cost = (val >= 0) ? val / 100.0 : 1.0;
    
    // 如果有ESDF，填充距离信息
    if (esdf_) {
        result.distance = esdf_->getDistanceFast(mx, my);
    }
    return result;
}

double LayeredMapManager::getEsdfDistanceWithGradient(double x, double y,
                                                       double* gx, double* gy) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!esdf_) {
        if (gx) *gx = 0;
        if (gy) *gy = 0;
        return 1e6;
    }
    
    double dist = esdf_->getDistanceInterp(x, y);
    
    if (gx && gy) {
        esdf_->getGradient(x, y, *gx, *gy);
    }
    
    return dist;
}

}  // namespace nav_components
