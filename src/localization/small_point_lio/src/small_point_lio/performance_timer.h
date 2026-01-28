/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace small_point_lio {

/**
 * @brief 轻量级RAII风格的计时器
 * 使用方法：
 *   if (parameters->enable_performance_debug) {
 *       ScopedTimer timer("function_name");
 *       // ... 你的代码 ...
 *   } // 析构时自动打印耗时
 */
class ScopedTimer {
private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
    bool enabled_;
    
public:
    explicit ScopedTimer(const std::string &name, bool enabled = true)
        : name_(name), enabled_(enabled) {
        if (enabled_) {
            start_ = std::chrono::steady_clock::now();
        }
    }
    
    ~ScopedTimer() {
        if (enabled_) {
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
            RCLCPP_INFO(rclcpp::get_logger("perf"), "[⏱️ %s] %.3f ms", 
                        name_.c_str(), duration / 1000.0);
        }
    }
    
    // 禁止拷贝
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

/**
 * @brief 累积计时器 - 用于统计多次调用的总耗时和平均值
 */
class AccumulativeTimer {
private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
    double total_time_ms_ = 0.0;
    int count_ = 0;
    int report_interval_;
    bool enabled_;
    
public:
    explicit AccumulativeTimer(const std::string &name, int report_interval = 100, bool enabled = true)
        : name_(name), report_interval_(report_interval), enabled_(enabled) {}
    
    void start() {
        if (enabled_) {
            start_ = std::chrono::steady_clock::now();
        }
    }
    
    void stop() {
        if (enabled_) {
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
            total_time_ms_ += duration / 1000.0;
            count_++;
            
            if (count_ % report_interval_ == 0) {
                double avg_time = total_time_ms_ / count_;
                RCLCPP_INFO(rclcpp::get_logger("perf"), 
                            "[📊 %s] Avg: %.3f ms, Total: %d calls, Sum: %.1f ms", 
                            name_.c_str(), avg_time, count_, total_time_ms_);
                // 重置统计
                total_time_ms_ = 0.0;
                count_ = 0;
            }
        }
    }
};

} // namespace small_point_lio
