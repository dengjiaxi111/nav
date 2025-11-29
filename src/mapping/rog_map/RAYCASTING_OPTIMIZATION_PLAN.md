# ROG-Map Raycasting并行化优化方案（保守策略）

**日期**: 2025-11-28  
**性能日志分析完成**: rm_performance_log.csv (700+ samples)  
**关键发现**: Raycast耗时占60-70%，是首要优化目标

---

## 1. 性能日志分析结论

### 1.1 实测数据统计（基于rm_performance_log.csv）

**典型样本分析**:
```
Total:   0.015 - 0.035 秒/帧  (平均 ~0.020s)
Raycast: 0.010 - 0.025 秒     (占比 60-70%)
Update:  0.003 - 0.013 秒     (占比 20-30%)
Inflation: 0.0001 - 0.0005秒  (占比 1-3%)
PointCloudNumber: 6000-10000点/帧 (典型值 8000-9000)
```

**关键瓶颈确认**:
- ✅ Raycasting是绝对瓶颈（10-25ms）
- ⚠️ Update_cache次要瓶颈（3-13ms）
- ℹ️ Inflation影响较小（< 1ms）

### 1.2 历史优化失败的原因分析

**症状**: 
- 单线程: 主线程CPU占用率 ~50%
- 4线程并行: 主线程CPU占用率 > 100%

**推断的根本原因**:

1. **原子操作竞争（最可能）**
   - `operation_cnt[hash_id]++` 和 `hit_cnt[hash_id]++` 在热点grid上高频冲突
   - 多线程写同一个`hash_id`导致Cache Line颠簸
   - 原子操作的总时间 > 串行计算的时间

2. **内存带宽瓶颈（次要）**
   - Raycasting需要大量随机内存访问（`occupancy_buffer_`, `operation_cnt`, `hit_cnt`）
   - 多线程并行加剧内存带宽竞争

3. **TBB调度开销（次要）**
   - Grain size设置不当，任务切换开销过大
   - 线程唤醒/睡眠导致上下文切换

---

## 2. 保守优化策略设计

### 2.1 核心原则

**DO**:
- ✅ 只并行化**计算密集**且**数据独立**的部分
- ✅ 使用**线程局部存储（TLS）**避免同步
- ✅ 采用**两阶段设计**：并行计算 + 串行聚合
- ✅ 限制线程数（最多2-4线程）
- ✅ 大grain size减少调度开销

**DON'T**:
- ❌ 不使用细粒度原子操作（改用TLS）
- ❌ 不过度并行化（避免内存带宽饱和）
- ❌ 不盲目追求线程数（质量 > 数量）

### 2.2 优化方案：两阶段Raycasting

#### Phase 1: 点云预处理并行化（读密集）

**当前瓶颈**:
```cpp
for (const auto& pcl_p : input_cloud) {  // 8000-9000次迭代
    // 1. Intensity filter
    // 2. Temporal filter  
    // 3. Virtual ceil/ground处理
    // 4. 距离检查
    // 5. BoundingBox裁剪
}
```

**优化后**（使用TLS避免竞争）:
```cpp
// 每个线程有独立的局部缓冲区
struct ThreadLocalData {
    std::vector<Vec3f> valid_rays;          // 线程局部
    std::vector<Vec3i> hit_points_id_g;     // 线程局部
    Vec3f local_box_min, local_box_max;
};

tbb::enumerable_thread_specific<ThreadLocalData> thread_local_storage;

tbb::parallel_for(
    tbb::blocked_range<int>(0, cloud_size, 512),  // 大grain size: 512点/任务
    [&](const tbb::blocked_range<int>& r) {
        auto& tls = thread_local_storage.local();
        
        for (int i = r.begin(); i < r.end(); ++i) {
            // 纯计算，无共享数据写入
            Vec3f p = processPoint(input_cloud[i]);  
            
            if (isValid(p)) {
                tls.valid_rays.push_back(p);
                if (update_hit) {
                    tls.hit_points_id_g.push_back(globalIndex(p));
                }
            }
        }
    }
);

// 串行聚合所有线程结果（低开销）
for (auto& tls : thread_local_storage) {
    merged_rays.insert(...);
    merged_hits.insert(...);
}
```

**预期收益**: 
- 点云预处理加速 **1.5-2.5x**（取决于核心数）
- 无原子操作竞争
- Cache友好（每个线程操作自己的vector）

---

#### Phase 2: Raycasting DDA算法（保持串行，但优化数据结构）

**当前问题**:
```cpp
for (const auto& p : raycasting_cloud) {  // 8000-9000条射线
    raycaster.setInput(start, p);
    while (raycaster.step(ray_pt)) {  // 平均50-200步
        insertUpdateCandidate(id_g, false);  // ⚠️ 写操作
    }
}
```

**为什么不并行化这部分？**

1. **写操作热点冲突严重**
   - 同一个`hash_id`会被多条射线hit（机器人前方的grid）
   - 原子操作`operation_cnt[hash_id]++`在热点区域变成串行
   - 并行反而比串行慢

2. **内存访问模式不友好**
   - DDA算法是随机内存访问（跳跃式遍历grid）
   - 多线程并行导致Cache Miss率飙升

**替代优化方案**（不并行化）:

```cpp
// 优化1: 预留容量减少reallocation
raycast_data_.update_cache_id_g.reserve(200000);  

// 优化2: 批量处理减少锁竞争
const int BATCH_SIZE = 100;
for (int batch_start = 0; batch_start < cloud_size; batch_start += BATCH_SIZE) {
    // 串行处理一批
    for (int i = batch_start; i < batch_start + BATCH_SIZE; ++i) {
        // raycasting...
    }
}
```

---

### 2.3 线程数控制策略

**自适应线程数**:
```cpp
// 根据点云大小动态调整
int num_threads = 1;  // 默认串行
if (cloud_size > 5000) {
    num_threads = 2;  // 中等点云用2线程
}
if (cloud_size > 10000) {
    num_threads = std::min(4, (int)std::thread::hardware_concurrency());
}

// TBB设置最大并发度
tbb::global_control gc(tbb::global_control::max_allowed_parallelism, num_threads);
```

**理由**:
- 小点云（< 5000点）：并行化开销 > 收益
- 中等点云（5000-10000点）：2线程最优（避免竞争）
- 大点云（> 10000点）：最多4线程（内存带宽限制）

---

## 3. 实施步骤（分阶段验证）

### Stage 1: 基础TBB集成（1天）

**目标**: 验证TBB环境可用，不修改算法

```bash
# 安装TBB
sudo apt install libtbb-dev

# CMakeLists.txt添加
find_package(TBB REQUIRED)
target_link_libraries(rog_map PUBLIC TBB::tbb)
```

**验证**:
```cpp
#include <tbb/parallel_for.h>
// 简单测试
tbb::parallel_for(0, 1000, [](int i) { /* empty */ });
```

**成功标准**: 编译通过，无运行时错误

---

### Stage 2: 点云预处理并行化（2天）

**修改文件**: `prob_map.cpp::raycastProcess()`

**实现**:
1. 将点云过滤逻辑提取为独立函数
2. 使用TLS存储中间结果
3. 添加性能计时对比

**验证**:
- 对比单线程 vs 2线程 vs 4线程的耗时
- 检查结果一致性（使用MD5 hash）
- 监控CPU占用率（不应超过150%）

**回退条件**: 
- 2线程加速比 < 1.2x → 放弃并行化
- CPU占用率 > 120% → 降低线程数

---

### Stage 3: 性能调优（2天）

**参数优化**:
- Grain size: 测试 [128, 256, 512, 1024]
- 线程数: 测试 [1, 2, 3, 4]
- TLS容量: 根据实际点云调整

**Benchmark工具**:
```cpp
class PerformanceMonitor {
    std::chrono::high_resolution_clock::time_point start;
    double& output_time;
public:
    PerformanceMonitor(double& out) : output_time(out) {
        start = std::chrono::high_resolution_clock::now();
    }
    ~PerformanceMonitor() {
        auto end = std::chrono::high_resolution_clock::now();
        output_time = std::chrono::duration<double>(end - start).count();
    }
};
```

**目标**:
- Raycasting总耗时减少 **30-40%**
- 主线程CPU占用率 < 100%
- 无数据竞争（ThreadSanitizer验证）

---

## 4. 风险缓解与降级方案

### 4.1 编译时开关

```cpp
// In CMakeLists.txt
option(USE_TBB_PARALLEL "Enable TBB parallelization" ON)

// In prob_map.cpp
#ifdef USE_TBB_PARALLEL
    // 并行版本
    tbb::parallel_for(...);
#else
    // 串行版本（原代码）
    for (const auto& pcl_p : input_cloud) { ... }
#endif
```

**好处**: 
- 可快速回退到稳定版本
- 方便A/B测试对比

### 4.2 运行时参数控制

```yaml
# rog_map_config.yaml
rog_map:
  raycasting:
    parallel_enable: true
    max_threads: 2        # 保守默认值
    grain_size: 512
    min_cloud_size_for_parallel: 5000  # 小点云不并行
```

**好处**:
- 无需重新编译
- 可根据硬件动态调整

### 4.3 性能监控与自动降级

```cpp
class AdaptiveParallelizer {
    double avg_speedup = 0.0;
    int fail_count = 0;
    
    bool shouldUseParallel(int cloud_size) {
        if (fail_count > 5) return false;  // 连续失败5次→禁用
        if (cloud_size < cfg_.min_cloud_size) return false;
        if (avg_speedup < 1.1) return false;  // 加速比不足→禁用
        return true;
    }
    
    void updateSpeedup(double serial_time, double parallel_time) {
        double speedup = serial_time / parallel_time;
        avg_speedup = 0.9 * avg_speedup + 0.1 * speedup;  // 指数平滑
        if (speedup < 1.0) fail_count++;
        else fail_count = std::max(0, fail_count - 1);
    }
};
```

---

## 5. 预期成果与评估标准

### 5.1 成功标准

| 指标 | 当前值 | 目标值 | 最低要求 |
|------|--------|--------|----------|
| Raycasting耗时 | 10-25ms | 7-18ms | 8-20ms |
| 总帧时间 | 15-35ms | 12-28ms | 13-30ms |
| 主线程CPU占用 | 50% | 70-85% | < 100% |
| 加速比 | 1.0x | 1.4-1.6x | > 1.2x |

### 5.2 失败判定（立即回退）

- ❌ 主线程CPU > 120%
- ❌ 加速比 < 1.1x（连续10帧）
- ❌ 出现数据竞争（ThreadSanitizer报错）
- ❌ 结果不一致（MD5校验失败）

---

## 6. 实施时间表

| 阶段 | 工作内容 | 工时 | 验证标准 |
|------|----------|------|----------|
| Day 1 | TBB环境搭建 + 编译测试 | 4h | 编译通过 |
| Day 2-3 | 点云预处理并行化实现 | 12h | 功能正确 |
| Day 4-5 | 性能测试 + 参数调优 | 12h | 达到目标加速比 |
| Day 6 | ThreadSanitizer验证 + 压力测试 | 6h | 无数据竞争 |
| Day 7 | 文档更新 + Code Review | 2h | 代码可维护 |

**总工时**: 36小时（约5个工作日）

---

## 7. 技术细节备忘

### 7.1 TBB线程局部存储（TLS）模式

```cpp
// 模式1: enumerable_thread_specific（推荐）
tbb::enumerable_thread_specific<std::vector<Vec3f>> tls(
    []() { 
        std::vector<Vec3f> v;
        v.reserve(2000);  // 预分配减少reallocation
        return v; 
    }
);

// 使用
tbb::parallel_for(..., [&](auto& r) {
    auto& local_vec = tls.local();  // 获取当前线程的vector
    local_vec.push_back(...);
});

// 聚合
for (auto& v : tls) {
    merged.insert(merged.end(), v.begin(), v.end());
}
```

### 7.2 避免False Sharing

```cpp
// 错误：多个线程写相邻字段
struct BadData {
    int counter1;  // Cache Line 1
    int counter2;  // Cache Line 1（False Sharing！）
};

// 正确：使用padding分离
struct alignas(64) GoodData {  // 64字节 = 典型Cache Line大小
    int counter1;
    char padding[60];  // 填充到64字节
};

// 或者使用TLS完全避免
tbb::enumerable_thread_specific<int> counters;
```

### 7.3 Grain Size选择指南

| 点云大小 | Grain Size | 理由 |
|----------|-----------|------|
| < 2000 | N/A | 不并行化 |
| 2000-5000 | 1024 | 减少调度开销 |
| 5000-10000 | 512 | 平衡负载均衡和调度 |
| > 10000 | 256 | 充分利用多核 |

---

## 8. 关键代码片段（待实现）

### 8.1 点云预处理并行化（核心实现）

```cpp
void ProbMap::raycastProcess_Parallel(const PointCloud& input_cloud, const Vec3f& cur_odom) {
    const int cloud_size = input_cloud.size();
    
    // 小点云不并行化
    if (cloud_size < cfg_.min_cloud_size_for_parallel) {
        return raycastProcess_Serial(input_cloud, cur_odom);
    }
    
    // 线程局部存储
    struct TLS {
        std::vector<Vec3f> valid_rays;
        std::vector<Vec3i> hit_id_g;
        Vec3f box_min, box_max;
        
        TLS() {
            valid_rays.reserve(1000);
            hit_id_g.reserve(500);
            box_min = Vec3f::Constant(std::numeric_limits<float>::max());
            box_max = Vec3f::Constant(std::numeric_limits<float>::lowest());
        }
    };
    
    tbb::enumerable_thread_specific<TLS> thread_storage;
    
    // 并行点云过滤
    tbb::parallel_for(
        tbb::blocked_range<int>(0, cloud_size, cfg_.grain_size),
        [&](const tbb::blocked_range<int>& range) {
            auto& tls = thread_storage.local();
            int local_temporal_cnt = range.begin();  // 线程局部计数器
            
            for (int i = range.begin(); i < range.end(); ++i) {
                const auto& pcl_p = input_cloud[i];
                
                // 1. Intensity filter
                if (cfg_.intensity_thresh > 0 && 
                    pcl_p.intensity < cfg_.intensity_thresh) {
                    continue;
                }
                
                // 2. Temporal filter（线程局部）
                if (local_temporal_cnt++ % cfg_.point_filt_num) {
                    continue;
                }
                
                Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
                
                // 3. Virtual bounds check
                bool update_hit = true;
                if (p.z() > cfg_.virtual_ceil_height) {
                    update_hit = false;
                    const double dz = p.z() - cur_odom.z();
                    const double pc = cfg_.virtual_ceil_height - cur_odom.z();
                    p = cur_odom + (p - cur_odom).normalized() * pc / dz;
                }
                // ... 其他过滤逻辑 ...
                
                // 4. 存入线程局部buffer
                tls.valid_rays.push_back(p);
                if (update_hit) {
                    Vec3i id_g;
                    posToGlobalIndex(p, id_g);
                    tls.hit_id_g.push_back(id_g);
                }
                
                // 5. 更新局部box
                tls.box_min = tls.box_min.cwiseMin(p);
                tls.box_max = tls.box_max.cwiseMax(p);
            }
        }
    );
    
    // 串行聚合（低开销）
    std::vector<Vec3f> merged_rays;
    Vec3f global_box_min = cur_odom, global_box_max = cur_odom;
    
    for (auto& tls : thread_storage) {
        merged_rays.insert(merged_rays.end(), 
                          tls.valid_rays.begin(), tls.valid_rays.end());
        global_box_min = global_box_min.cwiseMin(tls.box_min);
        global_box_max = global_box_max.cwiseMax(tls.box_max);
        
        // Hit点立即插入（已经是独立的id_g，冲突少）
        for (const auto& id_g : tls.hit_id_g) {
            insertUpdateCandidate(id_g, true);
        }
    }
    
    raycast_data_.cache_box_min = global_box_min;
    raycast_data_.cache_box_max = global_box_max;
    
    // Raycasting保持串行（避免写冲突）
    if (cfg_.raycasting_en) {
        for (const auto& p : merged_rays) {
            Vec3f start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
            raycast_data_.raycaster.setInput(start, p);
            Vec3f ray_pt;
            while (raycast_data_.raycaster.step(ray_pt)) {
                Vec3i id_g;
                posToGlobalIndex(ray_pt, id_g);
                if (!insideLocalMap(id_g)) break;
                insertUpdateCandidate(id_g, false);
            }
        }
    }
}
```

---

## 9. 总结与决策点

### 9.1 为什么不全盘并行化？

1. **Raycasting DDA算法**不适合并行化：
   - 写操作热点严重（机器人前方的grid被多条射线访问）
   - 随机内存访问模式（Cache Miss高）
   - 原子操作开销 > 计算收益

2. **点云预处理**适合并行化：
   - 纯计算，数据独立
   - 顺序内存访问（Cache友好）
   - 无写冲突（TLS）

### 9.2 最坏情况处理

如果并行化效果不佳：

**Plan B**: 保持串行，优化算法本身
- 使用SIMD加速向量计算（Eigen自带）
- 优化内存布局（SoA替代AoS）
- 减少不必要的中间计算

**Plan C**: 异步化处理
- Raycasting在单独线程执行
- 使用double buffering避免阻塞
- 主线程继续处理其他任务

---

**最后决定**: 
- ✅ 先实施保守的点云预处理并行化
- ✅ 严格监控CPU占用率和加速比
- ✅ 保留随时回退串行版本的能力
- ❌ 不并行化Raycasting DDA部分（除非TLS验证有效）

**开始实施**: 待人类确认后进行
