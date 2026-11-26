# STM32H723VGT6 下位机 —— auto-aim 上位机通信

为 `/root/auto-aim` 工程准备的下位机通信模板，协议 **100% 对齐** 上位机 `RMCVSerial` 库（通过反汇编确认 SOF/Len/CRC 细节）。

## 📁 文件清单

```
tools/stm32_h723/
├── protocol.h       # 协议常量 + 结构体 (与 packet.hpp 对齐)
├── protocol.c       # CRC8/CRC16 + 帧封装
├── uart_driver.h    # UART 驱动接口
├── uart_driver.c    # HAL ReceiveToIdle + 环形缓冲 + 状态机解析
└── main.c           # 主循环模板 (每 50ms 上报 IMU + 接收指令)
```

## 🧩 协议帧格式

```
Offset  Size  Field        内容                Byte Order
──────  ────  ──────       ────────            ──────────
 0      1     SOF          固定 0xA5            -
 1-2    2     Length       payload 字节数       小端
 3      1     Seq          序列号 (随便填)      -
 4      1     CRC8         CRC-8/ITU 覆盖 0..3  -
 5-6    2     CmdID        命令字              小端
 7..N   N-7   Data         payload struct       -
N+1..N+2 2    CRC16       CRC-16/MODBUS 覆盖 3..N  小端
```

| 方向 | CmdID | struct | 大小 | 频率 |
|------|-------|--------|------|------|
| 下→上 | `0x1021` | `pc_mcu_data_t` | 21B | 20-50Hz |
| 下→上 | `0x1022` | `robot_data_t` | 37B | 1-10Hz |
| 上→下 | `0x0503` | `advv_detection_t` | 38B | 200Hz (timed_serial 5ms 周期) |

**CRC 算法**（RoboMaster 标准）：
- CRC8:  CRC-8/ITU, poly `0x07`, init `0x00`, 覆盖前 **5 字节** (SOF+Len+Seq+CRC8)
- CRC16: CRC-16/MODBUS, poly `0xA001`, init `0xFFFF`, 覆盖 offset 3 到帧末尾前 2 字节 (Seq+CmdID+Data)

## 🔌 接线

| STM32H723VGT6 | 上位机 Jetson / 电脑 USB-TTL |
|---------------|-------------------------------|
| PA9 (USART1_TX) | RX (ttyS4 / ttyACM0) |
| PA10 (USART1_RX) | TX |
| GND | GND |

> **必须共地**！不共地的话 UART 电平参考不一致，会读到一堆 0x00 或乱码。

## ⚙️ CubeMX 配置

1. **Connectivity → USART1**
   - Mode: Asynchronous
   - Baud Rate: **921600**
   - Word Length: 8 Bits
   - Parity: None
   - Stop Bits: 1
   - HW Flow Control: None
   - Clock Source: PCLK

2. **NVIC Settings → USART1 global interrupt** → **Enabled**
   - Preemption Priority: 2 （低于主循环，H7 建议 priority group 4）

3. **GPIO** — USART1 默认 PA9/PA10, 无需额外配置

4. **Clock Configuration** — HCLK 至少 200MHz, 建议 550MHz (H723 最高)

5. **Generate Code** — 把 `protocol.{h,c}` / `uart_driver.{h,c}` 拷到 `Core/Src` 和 `Core/Inc`,
   把 `main.c` 的内容合并到 CubeMX 生成的 `Core/Src/main.c` (保留 MX_GPIO_Init / MX_USART1_UART_Init / SystemClock_Config)

## 🏗️ 主循环要点

```c
for (;;) {
    /* ① 接收解析 (不要在 ISR 里做业务!) */
    UartFrame_t frame;
    if (UART_Process(&frame)) {
        if (frame.type == UART_CMD_GIMADVV) {
            memcpy(&g_last_cmd, frame.data, sizeof(advv_detection_t));
            // TODO: 把 g_last_cmd.yaw / .pit / .shoot 丢给云台 PID
        }
    }

    /* ② 每 50ms 上报一次 MCU 姿态 */
    if (HAL_GetTick() - last >= 50) {
        // IMU_Read(&yaw, &pitch, &roll);  ← 替换成真实 IMU
        UART_SendMcuData(yaw, pitch, roll, shoot_speed, autoaim_mode);
        last = HAL_GetTick();
    }
    __WFI();
}
```

## 🔍 调试方法

### 1. 板子侧 LED 指示

```c
/* 收到上位机帧时翻转 LED, 确认链路通了 */
if (UART_Process(&frame)) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
```

### 2. 上位机侧用 xxd 抓原始串口

```bash
timeout 6 cat /dev/ttyS4 | xxd -g 1 | grep "a5" | head -5
# 应该看到大量 a5 开头的帧, 例如:
# a5 15 00 00 [CRC8] 21 10 [21字节payload] [CRC16]
# a5 25 00 00 [CRC8] 22 10 [37字节payload] [CRC16]
```

### 3. 用 Python 脚本单独验证协议

```bash
python3 -m serial.tools.miniterm /dev/ttyS4 921600 --raw
# 应该能看到二进制流, 其中每个帧以 0xA5 开头
```

### 4. 启动上位机

```bash
cd /root/auto-aim/build
./auto-aim                       # timed_serial_log_text=true 时会看到 [TimedSerial] 日志
# 同时 tail 上位机:
timeout 10 cat /dev/ttyS4 > /tmp/up.bin && xxd /tmp/up.bin | head
```

## ⚠️ 常见坑

| 症状 | 原因 | 解决 |
|------|------|------|
| 原始数据全是 0x00 | RX 悬空 / 没共地 / TX 没发 | 先测电平, 再连 GND |
| 有数据但上位机完全没有 `[TimedSerial]` 日志 | 帧没带 0xA5 头 | 确认 MCU 发的是 `protocol_pack` 封装后的完整帧 |
| 有 0xA5 但 CRC 不通过 | CRC 算法错 / 覆盖范围错 | 用本仓库 `protocol.c`, 不要自己写 CRC |
| 921600 下丢字节 | H7 UART FIFO 没关 | `HAL_UARTEx_DisableFifoMode(&huart1)` |
| ISR 里直接 `while` 等 | 把解析放 ISR | **ISR 只 push 到环缓, 主循环解析** |
| USART1_IRQHandler 里自己写 RXNE 处理又调 HAL | 重复消费中断源 | **ISR 里只调 `HAL_UART_IRQHandler`, 不要自己读 DR/RDR** |

## ✅ 协议对齐验证

反汇编 `/usr/local/lib/libRMCVSerial.so` 后确认：
- SOF 硬编码 `cmp w0, #0xa5` ✅
- CRC8 校验前 5 字节 ✅  
- CRC16 覆盖 offset 3..N ✅
- Length 上限 `cmp w0, #0x1f6` (502) ✅

本仓库的 `protocol.{h,c}` 与 `uart_driver.c` 的状态机是完整复刻。
