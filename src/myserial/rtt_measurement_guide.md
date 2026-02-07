# RTT 往返时间测量使用指南

> 用于测量 PC 与 STM32 之间的通信延迟（Round-Trip Time）

最后更新：2026-02-07

---

## 功能概述

本功能通过复用现有通信帧的字段，实现零侵入式的 RTT 测量，无需修改通信协议结构。

### 实现原理

1. **PC 发送**：在发送帧的 `_speed_w` 字段（float，角速度）中存储**当前时间戳模 10000**（0~9999.999ms，10秒循环）
2. **STM32 回传**：收到完整帧后，将该 float 值**直接复制**到接收帧的 `_target_position_x` 字段并发送回 PC
3. **PC 接收**：接收到回传数据后，用**当前时间戳模 10000** 减去回传值，得到 RTT（处理跨越10秒边界的情况）

### 关键优势

- ✅ **零类型转换**：float 到 float，STM32 端无需任何转换，只需赋值
- ✅ **不影响其他功能**：测试时机器人静止，`_speed_w`（角速度）不需要使用
- ✅ **精度更高**：float 可保留小数（微秒级），比 int16_t 精度高
- ✅ **代码最简**：STM32 端只需一行赋值语句

### 字段选择说明

- **发送帧**：`_buff_yaw_diff_angle` - 原用于打符偏角，测试时可复用
- **接收帧**：`_target_position_x` - 原用于目标位置，测试时可复用
- 这两个字段在正常调试/测试场景下一般不使用，不会影响其他功能

---

## PC 端使用方法

### 1. 启用 RTT 测量

在 launch 文件或参数文件中设置：

```yaml
# myserial/config/serial_params.yaml
serial_node:
  ros__parameters:
    enable_rtt_measure: true  # 开启 RTT 测量
    debug_flag: false          # 可选：开启详细日志
```

或在启动时通过命令行参数：

```bash
ros2 run myserial serial_node --ros-args \
  -p enable_rtt_measure:=true \
  -p debug_flag:=true
```

### 2. 查看 RTT 统计

启用后，节点会每 5 秒自动输出统计信息：

```
[INFO] ========== RTT Statistics (last 5s) ==========
[INFO]   Sent: 250, Received: 248, Loss: 0.80%
[INFO]   RTT - Avg: 2.345 ms, Min: 1.123 ms, Max: 5.678 ms
[INFO] =============================================
```

### 3. 统计指标说明

- **Sent**：发送的帧数量（每 20ms 发送一次，5秒约 250 帧）
- **Received**：收到正确回传的帧数量
- **Loss**：丢包率（%）
- **Avg/Min/Max RTT**：平均/最小/最大往返延迟（毫秒）

### 4. 日志记录

所有 RTT 数据会记录到日志文件（`log_path` 参数指定的路径）：

```
[2026-02-07 10:30:15.123] [info] [RTT_STATS] Sent=250, Recv=248, Loss=0.80%, Avg=2.345ms, Min=1.123ms, Max=5.678ms
```

---

### 关键注意事项

1. **立即回传**：收到完整帧后立即发送，不要有长时间的处理延迟
2. **帧完整性**：确保 `_target_position_x` 字段在发送帧中正确填充
3. **不修改其他字段**：除了 RTT 测量字段，其他数据按正常业务填充
4. **帧头帧尾**：确保帧头（0x77）和帧尾（0x88）正确
5. **字节序**：`float` 和 `int16_t` 都使用小端序（Little-Endian）

---

## 性能分析参考

### 正常通信 RTT 基准值

| 场景 | 预期 RTT | 说明 |
|------|----------|------|
| **理想情况** | 1-3 ms | USB CDC Full-Speed，无其他负载 |
| **正常负载** | 3-10 ms | 有其他 ROS 节点、日志输出 |
| **高负载** | 10-50 ms | CPU 占用高、磁盘 I/O 频繁 |
| **异常情况** | >100 ms 或丢包 >5% | 需要优化或检查硬件 |

---

## 关闭 RTT 测量

测试完成后，记得关闭：

```yaml
enable_rtt_measure: false  # 关闭 RTT 测量
```

或在不需要时，STM32 端可以忽略这个字段，PC 端会自动跳过统计。

---
