# ROG-Map TBB并行化优化方案（Raycasting专注版）

**创建时间**: 2025-11-28  
**性能日志分析完成**: 2025-11-28 21:xx  
**优化目标**: 专注优化Raycasting瓶颈（占总耗时60-70%），采用保守策略避免CPU过载  
**理论依据**: 导航系统为定时触发的流水线，各模块顺序执行，并行化可有效减少单模块CPU占用时间

**⚠️ 关键约束（基于历史教训）**:
- 之前4线程并行导致主线程CPU > 100%（单线程仅50%）
- 说明存在**竞争锁开销**或**内存带宽瓶颈**或**Cache颠簸**
- 本次优化必须采用**保守策略**，避免过度并行化

---

## 1. 代码理解总结

### 1.1 ROG-Map核心原理
- **概率占据栅格地图**: 使用log-odds概率更新模型 (`l_hit`, `l_miss`)
- **分层设计**: 
  - `ProbMap`: 基础概率地图 (resolution: 0.1m)
  - `InfMap`: Inflation膨胀层 (inflation_resolution: 0.1-0.3m)
  - `ESDFMap`: 欧氏距离场 (可选)
  - `FreeCntMap`: Frontier计数器 (可选)
- **滑动窗口**: Robocentric设计，地图跟随机器人移动

### 1.2 计算瓶颈识别

#### 瓶颈1: Raycasting处理 (占总时间40-60%)
**位置**: `prob_map.cpp::raycastProcess()` (L760-L880)

**现状**:
```cpp
// 串行处理点云
for (const auto& pcl_p : input_cloud) {
    // 1. 点预处理 (intensity/temporal filter)
    // 2. 虚拟天花板/地板裁剪
    // 3. 距离范围检查
}

// 串行执行raycasting
for (const auto& p : raycasting_cloud) {
    raycast_data_.raycaster.setInput(raycast_start, p);
    while (raycast_data_.raycaster.step(ray_pt)) {
        // DDA算法逐步遍历射线
        insertUpdateCandidate(cur_ray_id_g, false);
    }
}
```

**问题**:
1. 点云规模大 (Livox Mid-360: ~20,000 points/frame @ 10Hz)
2. 每个点需执行raycasting (平均50-200步DDA迭代)
3. 写操作通过`insertUpdateCandidate`已有互斥保护 (操作`operation_cnt`/`hit_cnt` vector)
4. 现有代码单线程顺序执行

**数据依赖分析**:
- ✅ **可并行**: 不同点的raycasting过程独立 (读操作无冲突)
- ⚠️ **需同步**: 共享写入`raycast_data_.operation_cnt[hash_id]`和`hit_cnt[hash_id]`
- 🔒 **已有互斥**: `raycast_data_.raycast_range_mtx` (仅保护box范围)

---

#### 瓶颈2: Inflation更新 (占总时间20-30%)
**位置**: `inf_map.cpp::updateInflation()` (L201-L228)

**现状**:
```cpp
void InfMap::updateInflation(const Vec3i& id_g, const bool is_hit) {
    for (const auto& nei : cfg_.inf_spherical_neighbor) {
        const Vec3i& id_shift = id_g + nei;
        const int& addr = getHashIndexFromGlobalIndex(id_shift);
        if (is_hit) {
            imd_.occ_inflate_cnt[addr]++;
        } else {
            imd_.occ_inflate_cnt[addr]--;
        }
    }
}
```

**触发点**: 每次grid状态跳变 (UNKNOWN→OCCUPIED, FREE→OCCUPIED等)调用

**问题**:
1. 每次调用需遍历球形邻域 (inflation_step=2时: ~125个邻居)
2. 批量更新场景下被频繁调用 (每帧数千次)
3. 邻域计数器更新存在原子性需求

**数据依赖分析**:
- ✅ **可并行**: 不同中心点的inflation操作可并行
- ⚠️ **需同步**: 邻域可能重叠，需原子操作保护`occ_inflate_cnt[addr]`

---

#### 瓶颈3: 概率更新批处理
**位置**: `prob_map.cpp::probabilisticMapFromCache()` (L632-L653)

**现状**:
```cpp
void ProbMap::probabilisticMapFromCache() {
    while (!raycast_data_.update_cache_id_g.empty()) {
        Vec3i id_g = raycast_data_.update_cache_id_g.front();
        raycast_data_.update_cache_id_g.pop();
        // ...
        if (raycast_data_.hit_cnt[hash_id] > 0) {
            hitPointUpdate(pos, hash_id, raycast_data_.hit_cnt[hash_id]);
        } else {
            missPointUpdate(pos, hash_id, ...);
        }
    }
}
```

**问题**:
1. Queue顺序出队 (FIFO)，难以批量并行
2. 每次更新可能触发inflation (间接调用`triggerJumpingEdge`)
3. 状态跳变检测需读取`occupancy_buffer_`当前值

**数据依赖分析**:
- ❌ **较难并行**: Queue结构限制，需转换为vector批处理
- ⚠️ **复杂依赖**: 状态跳变触发的inflation更新形成二级依赖

---

#### 瓶颈4: Box搜索操作 (查询密集场景)
**位置**: `prob_map.cpp::boxSearch()` (L510-L578)

**现状**: 三重循环遍历box范围内所有grid

**数据依赖分析**:
- ✅ **可并行**: 纯查询操作，无写入冲突
- 📊 **收益取决于**: box大小 (小box并行开销 > 收益)

---

## 2. TBB并行化方案设计

### 2.1 技术选型: Intel TBB
**理由**:
- ✅ C++17兼容，ROG-Map已用C++17
- ✅ Header-only可选 (TBB 2021.x OneAPI版本)
- ✅ 提供`parallel_for`、`parallel_reduce`、`concurrent_vector`
- ✅ 任务窃取调度器 (自动负载均衡)
- ✅ Cache-friendly分区策略 (`blocked_range`)

### 2.2 依赖管理
**安装方式** (Ubuntu 22.04 ROS2 Humble):
```bash
# 方式1: 系统包管理器
sudo apt install libtbb-dev

# 方式2: 手动指定版本 (推荐)
wget https://github.com/oneapi-src/oneTBB/archive/v2021.11.0.tar.gz
tar -xzf v2021.11.0.tar.gz
cd oneTBB-2021.11.0
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
```

**CMakeLists.txt修改**:
```cmake
# 在 find_package(PCL REQUIRED) 之后添加
find_package(TBB REQUIRED)

# 在 target_link_libraries 中添加
target_link_libraries(rog_map PUBLIC
    ${third_party_libs}
    TBB::tbb  # 新增
)
```

---

## 3. 并行化实施计划

### Phase 1: Raycasting并行化 (优先级: ⭐⭐⭐⭐⭐)

#### 3.1 修改文件
- `src/mapping/rog_map/src/rog_map/prob_map.cpp`
- `include/rog_map/prob_map.h`

#### 3.2 实现策略

##### 策略A: 点云分块并行 (推荐)
**原理**: 将点云按数量分块，每个线程处理独立的点子集

```cpp
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/spin_mutex.h>  // 轻量级互斥锁

void ProbMap::raycastProcess(const PointCloud& input_cloud, const Vec3f& cur_odom) {
    const int cloud_size = input_cloud.size();
    
    // 预分配线程局部存储
    struct ThreadLocalData {
        std::vector<Vec3f> raycasting_cloud;
        Vec3f cache_box_min, cache_box_max;
    };
    tbb::enumerable_thread_specific<ThreadLocalData> tls(
        [&]() { 
            ThreadLocalData data;
            data.raycasting_cloud.reserve(cloud_size / tbb::task_arena::max_concurrency());
            data.cache_box_min = cur_odom;
            data.cache_box_max = cur_odom;
            return data;
        }
    );
    
    // Step 1: 并行预处理点云
    tbb::parallel_for(
        tbb::blocked_range<int>(0, cloud_size, 256),  // grain_size=256
        [&](const tbb::blocked_range<int>& r) {
            auto& local = tls.local();
            int temperol_cnt = r.begin();  // 线程局部计数
            
            for (int i = r.begin(); i < r.end(); ++i) {
                const auto& pcl_p = input_cloud[i];
                
                // 1.1) intensity filter (无依赖)
                if (cfg_.intensity_thresh > 0 && 
                    pcl_p.intensity < cfg_.intensity_thresh) {
                    continue;
                }
                
                // 1.2) temporal filter (线程局部)
                if (temperol_cnt++ % cfg_.point_filt_num) {
                    continue;
                }
                
                Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
                
                // ... 虚拟天花板/地板处理 (纯计算) ...
                
                // 添加到线程局部缓冲区
                local.raycasting_cloud.push_back(p);
                local.cache_box_min = local.cache_box_min.cwiseMin(p);
                local.cache_box_max = local.cache_box_max.cwiseMax(p);
                
                // Hit点立即标记 (需原子操作)
                if (update_hit) {
                    Vec3i pt_id_g;
                    posToGlobalIndex(p, pt_id_g);
                    insertUpdateCandidateAtomic(pt_id_g, true);
                }
            }
        }
    );
    
    // Step 2: 合并线程局部缓冲区
    std::vector<Vec3f> merged_raycasting_cloud;
    Vec3f global_box_min = cur_odom, global_box_max = cur_odom;
    for (auto& local : tls) {
        merged_raycasting_cloud.insert(
            merged_raycasting_cloud.end(),
            local.raycasting_cloud.begin(),
            local.raycasting_cloud.end()
        );
        global_box_min = global_box_min.cwiseMin(local.cache_box_min);
        global_box_max = global_box_max.cwiseMax(local.cache_box_max);
    }
    raycast_data_.cache_box_min = global_box_min;
    raycast_data_.cache_box_max = global_box_max;
    
    // Step 3: 并行执行raycasting
    if (cfg_.raycasting_en) {
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, merged_raycasting_cloud.size(), 64),
            [&](const tbb::blocked_range<size_t>& r) {
                raycaster::RayCaster local_raycaster(cfg_.resolution);
                
                for (size_t i = r.begin(); i < r.end(); ++i) {
                    const Vec3f& p = merged_raycasting_cloud[i];
                    Vec3f raycast_start = (p - cur_odom).normalized() * 
                                         cfg_.raycast_range_min + cur_odom;
                    
                    local_raycaster.setInput(raycast_start, p);
                    Vec3f ray_pt;
                    while (local_raycaster.step(ray_pt)) {
                        Vec3i cur_ray_id_g;
                        posToGlobalIndex(ray_pt, cur_ray_id_g);
                        if (!insideLocalMap(cur_ray_id_g)) break;
                        
                        insertUpdateCandidateAtomic(cur_ray_id_g, false);
                    }
                }
            }
        );
    }
}
```

##### 关键修改: 原子操作保护
```cpp
// 新增成员变量 (prob_map.h)
#include <atomic>
struct RaycastData {
    // ... 原有成员 ...
    std::vector<std::atomic<uint16_t>> operation_cnt_atomic;
    std::vector<std::atomic<uint16_t>> hit_cnt_atomic;
    tbb::spin_mutex update_queue_mutex;  // 保护queue写入
};

// 原子版本插入函数
void ProbMap::insertUpdateCandidateAtomic(const Vec3i& id_g, bool is_hit) {
    const int hash_id = getHashIndexFromGlobalIndex(id_g);
    
    uint16_t old_cnt = raycast_data_.operation_cnt_atomic[hash_id].fetch_add(
        1, std::memory_order_relaxed
    );
    
    if (old_cnt == 0) {  // 首次访问
        tbb::spin_mutex::scoped_lock lock(raycast_data_.update_queue_mutex);
        raycast_data_.update_cache_id_g.push(id_g);
    }
    
    if (is_hit) {
        raycast_data_.hit_cnt_atomic[hash_id].fetch_add(
            1, std::memory_order_relaxed
        );
    }
}
```

#### 3.3 预期收益
- **理论加速比**: 4-8x (取决于CPU核心数)
- **实际加速比**: 3-6x (考虑同步开销和内存带宽)
- **适用场景**: 点云规模 > 5000点

#### 3.4 风险与缓解
| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 原子操作开销 | 并行效率下降10-15% | 使用`memory_order_relaxed`减少内存屏障 |
| Cache颠簸 | 多线程写同一cacheline | 分块大小调优 (grain_size=64-256) |
| 负载不均 | 部分线程空闲 | TBB自动任务窃取 |

---

### Phase 2: Inflation并行化 (优先级: ⭐⭐⭐⭐)

#### 3.5 修改文件
- `src/mapping/rog_map/src/rog_map/inf_map.cpp`
- `include/rog_map/inf_map.h`

#### 3.6 实现策略

##### 策略B: 批量Inflation并行
**原理**: 收集状态跳变的所有grid，批量并行执行inflation

```cpp
#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>

// 新增成员 (inf_map.h)
struct InflationBatch {
    tbb::concurrent_vector<std::pair<Vec3i, bool>> pending_ops;  // (id_g, is_hit)
    tbb::spin_mutex batch_mutex;
};

void InfMap::batchUpdateInflation() {
    if (inflation_batch_.pending_ops.empty()) return;
    
    // 并行处理所有待更新的grid
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, inflation_batch_.pending_ops.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                const auto& [id_g, is_hit] = inflation_batch_.pending_ops[i];
                
                for (const auto& nei : cfg_.inf_spherical_neighbor) {
                    const Vec3i id_shift = id_g + nei;
                    const int addr = getHashIndexFromGlobalIndex(id_shift);
                    
                    // 原子操作更新计数器
                    if (is_hit) {
                        imd_.occ_inflate_cnt_atomic[addr].fetch_add(
                            1, std::memory_order_relaxed
                        );
                    } else {
                        imd_.occ_inflate_cnt_atomic[addr].fetch_sub(
                            1, std::memory_order_relaxed
                        );
                    }
                }
            }
        }
    );
    
    inflation_batch_.pending_ops.clear();
}
```

#### 3.7 关键改造点
1. **延迟执行**: `triggerJumpingEdge`不立即调用inflation，而是加入batch
2. **原子计数器**: `imd_.occ_inflate_cnt`改为`std::atomic<int16_t>`
3. **批量触发**: 在`probabilisticMapFromCache`末尾调用`batchUpdateInflation`

#### 3.8 预期收益
- **理论加速比**: 2-4x
- **适用场景**: 批量更新场景 (batch_update_size > 1)

---

### Phase 3: BoxSearch并行化 (优先级: ⭐⭐⭐)

#### 3.9 实现代码
```cpp
void ProbMap::boxSearch(const Vec3f& box_min, const Vec3f& box_max, 
                       const GridType& gt, vec_E<Vec3f>& out_points) const {
    // ... 边界检查代码不变 ...
    
    tbb::concurrent_vector<Vec3f> concurrent_results;
    
    tbb::parallel_for(
        tbb::blocked_range3d<int>(
            box_min_id_g.x(), box_max_id_g.x(),
            box_min_id_g.y(), box_max_id_g.y(),
            box_min_id_g.z(), box_max_id_g.z()
        ),
        [&](const tbb::blocked_range3d<int>& r) {
            for (int i = r.pages().begin(); i < r.pages().end(); ++i) {
                for (int j = r.rows().begin(); j < r.rows().end(); ++j) {
                    for (int k = r.cols().begin(); k < r.cols().end(); ++k) {
                        Vec3i id_g(i, j, k);
                        if (isOccupied(id_g)) {  // 根据gt类型调整
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            concurrent_results.push_back(pos);
                        }
                    }
                }
            }
        }
    );
    
    out_points.assign(concurrent_results.begin(), concurrent_results.end());
}
```

---

## 4. 性能评估与测试策略

### 4.1 性能指标
| 指标 | 测量方法 | 目标值 |
|------|----------|--------|
| Raycasting耗时 | `time_consuming_[1]` | 减少50% |
| Inflation耗时 | `time_consuming_[3]` | 减少40% |
| 总帧时间 | `time_consuming_[0]` | < 50ms @ 10Hz |
| CPU占用率 | `htop`观察 | 多核平衡利用 |

### 4.2 测试数据集
1. **实际rosbag**: `performance_test/driver/` 下录制的LiDAR数据
2. **压力测试**: 高点云密度场景 (20k+ points/frame)
3. **稳定性测试**: 长时运行 (1小时+)

### 4.3 Benchmark对比
```bash
# 编译Baseline版本 (不并行化)
colcon build --packages-select rog_map --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译TBB版本
colcon build --packages-select rog_map --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DUSE_TBB=ON

# 运行性能对比
ros2 run rog_map_ros2_node rog_map_node --benchmark
```

---

## 5. 实施时间表与里程碑

| 阶段 | 任务 | 预计工时 | 验证标准 |
|------|------|----------|----------|
| Week 1 | TBB环境搭建 + CMake配置 | 4h | 编译通过，链接无错误 |
| Week 2 | Phase 1.1: Raycasting点云预处理并行化 | 8h | 单元测试通过，结果一致性 |
| Week 2 | Phase 1.2: Raycasting射线遍历并行化 | 12h | 性能提升30%+ |
| Week 3 | Phase 2: Inflation批量并行化 | 10h | 性能提升20%+ |
| Week 4 | Phase 3: BoxSearch并行化 | 6h | 查询延迟减半 |
| Week 5 | 集成测试 + Rosbag验证 | 8h | 无数据竞争，长时运行稳定 |

---

## 6. 备选方案与降级策略

### 6.1 如果TBB不可用
- **方案A**: 使用C++17 `std::execution::par` (GCC 9+ 支持)
- **方案B**: 手动pthread线程池 (控制粒度更细)

### 6.2 如果并行化收益不明显
- **原因分析**: 
  1. 点云规模太小 (< 2000点) → 保持串行
  2. 内存带宽瓶颈 → 优化数据结构 (SoA布局)
  3. 同步开销过大 → 减少原子操作频率

---

## 7. 参考资料

### 7.1 TBB文档
- [Intel TBB官方文档](https://oneapi-src.github.io/oneTBB/)
- [TBB并行模式](https://www.threadingbuildingblocks.org/docs/help/tbb_userguide/title.html)

### 7.2 相关论文
- ROG-Map原论文: `ROG-Map_An_Efficient_Robocentric_Occupancy_Grid_Map.pdf`
- Raycasting优化: "Fast Voxel Traversal Algorithm" (Amanatides & Woo, 1987)

### 7.3 操作系统/多线程知识点回顾
- **Cache一致性**: MESI协议，避免False Sharing
- **内存顺序**: `memory_order_relaxed` vs `memory_order_seq_cst`
- **任务调度**: 工作窃取 (Work Stealing) vs 固定分区
- **负载均衡**: Grain size对并行效率的影响

---

## 8. 注意事项与陷阱

### 8.1 数据竞争检测
```bash
# 使用ThreadSanitizer编译
colcon build --packages-select rog_map --cmake-args \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -g"

# 运行测试
ros2 run rog_map_ros2_node rog_map_node 2>&1 | grep "WARNING: ThreadSanitizer"
```

### 8.2 常见陷阱
1. ❌ **错误**: 直接并行`std::queue`操作 → 使用`concurrent_queue`或转vector
2. ❌ **错误**: 忽略False Sharing → 手动padding或使用thread_local
3. ❌ **错误**: 过小的grain_size → 调度开销 > 计算收益
4. ❌ **错误**: 忘记考虑NUMA架构 → 绑定线程到CPU亲和性

---

## 9. 人工审查检查点

在实施前，必须人工确认以下事项：

### ✅ 代码理解确认
- [ ] 已完整阅读`prob_map.cpp`的raycasting流程
- [ ] 理解`insertUpdateCandidate`的数据依赖关系
- [ ] 掌握ROG-Map的log-odds概率更新模型

### ✅ 架构设计确认
- [ ] 并行化方案不破坏现有接口
- [ ] 原子操作选择合理 (relaxed vs acquire-release)
- [ ] 考虑了不同点云密度的性能表现

### ✅ 测试策略确认
- [ ] 准备好对比测试的baseline数据
- [ ] 设计了充分的单元测试覆盖边界情况
- [ ] 验证方法包含数值一致性检查

### ✅ 风险评估确认
- [ ] 识别了所有潜在的数据竞争点
- [ ] 有回退到串行版本的开关机制
- [ ] 预留了性能调优的参数接口

---

**最后更新**: 2025-11-28  
**审查状态**: 待人工审核  
**下一步行动**: 请人类开发者审阅本方案，确认后开始Phase 1实施
