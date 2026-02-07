# STM32 端 RTT 测量实现补充

## 核心逻辑（4行代码）

```c
// 在接收完整帧并准备回传时
void PrepareResponseFrame(WholeSendFrame* recv_frame, WholeGetFrame* send_frame) {
    // RTT 测量：将接收到的时间戳（float）直接回传
    send_frame->_target_position_x = recv_frame->_speed_w;
    
    // ... 填充其他字段的正常业务逻辑 ...
}
```

## 工作原理

1. PC 发送 `_speed_w = 1234.567`（毫秒时间戳模 10000）
2. STM32 收到后立即回传 `_target_position_x = 1234.567`
3. PC 计算 RTT = (当前时间模 10000) - 1234.567

## 字段选择说明

- **发送帧**：使用 `_speed_w`（float，角速度）- 测试时机器人静止，角速度可以为 0
- **接收帧**：使用 `_target_position_x`（float，目标位置）- 测试时不需要目标位置
- **优势**：float 到 float，无需任何类型转换！

## 完整示例（带开关）

```c
// config.h
#define ENABLE_RTT_MEASURE  1  // 1=启用, 0=禁用

// main.c
void OnFrameReceived(WholeSendFrame* rx_frame) {
    WholeGetFrame tx_frame = {0};
    
    // 帧头帧尾
    tx_frame._sof = 0x77;
    tx_frame._eof = 0x88;
    
#if ENABLE_RTT_MEASURE
    // RTT 模式：直接回传时间戳（float -> float，零开销）
    tx_frame._target_position_x = rx_frame->_speed_w;
#else
    // 正常模式：填充实际目标位置
    tx_frame._target_position_x = target_x;
    tx_frame._target_position_y = target_y;
#endif
    
    // 填充其他字段（底盘速度、裁判系统数据等）
    tx_frame._speed_x = (int8_t)(chassis_speed_x * 50);
    tx_frame._speed_y = (int8_t)(chassis_speed_y * 50);
    // ...
    
    // 发送
    HAL_UART_Transmit(&huart1, (uint8_t*)&tx_frame, sizeof(tx_frame), 100);
}
```

## 时序要求

- ⏱️ **目标延迟**：从接收完成到发送启动 < 5ms
- ✅ **优先级**：建议将通信任务设为高优先级或在中断中处理
- ⚠️ **避免阻塞**：不要在回传前进行长时间计算

## 调试技巧

### 方法 1：LED 指示

```c
void OnFrameReceived(WholeSendFrame* rx_frame) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 收到帧，翻转 LED
    
    // ... 回传逻辑 ...
}
```

### 方法 2：串口打印（注意：会增加延迟）

```c
#if DEBUG_RTT
    printf("RX timestamp: %d\r\n", rx_frame->_buff_yaw_diff_angle);
#endif
```

### 方法 3：逻辑分析仪

- CH1：RX 数据线
- CH2：TX 数据线  
- CH3：GPIO toggle（标记处理开始/结束）

## 常见问题

### Q: 需要保存历史时间戳吗？

**A**: 不需要！直接回传即可，PC 端负责计算 RTT。

### Q: 如果 `_target_position_x` 正在使用怎么办？

**A**: 不影响，因为：
1. 测试 RTT 时机器人通常静止不动，不需要速度控制
2. 如果确实需要同时使用，可以改用其他未使用字段（如 `our_hero_x` 或 `_sentry_info`）

### Q: float 直接赋值会有精度问题吗？

**A**: 不会。float 有 23 位尾数，精度约 7 位十进制，存储 0~9999.999 完全够用。

### Q: 如果 STM32 时钟不准怎么办？

**A**: 无影响！RTT 测量只依赖 PC 端时钟，STM32 只是原样回传数据。

---

**最小改动原则**：只需在回传函数中添加 1 行代码 `send_frame->_target_position_x = recv_frame->_speed_w;` 即可启用 RTT 测量！
