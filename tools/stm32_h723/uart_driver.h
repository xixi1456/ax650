/**
 * @file uart_driver.h
 * @brief STM32H723VGT6 UART 协议收发驱动 (HAL)
 *
 * 接收: HAL_UARTEx_ReceiveToIdle_IT -> RxEvent 回调 -> 环形缓冲
 * 解析: 主循环状态机 (STEP_HEADER_SOF -> ... -> STEP_DATA_CRC16)
 * 发送: HAL_UART_Transmit (阻塞, 短帧足够)
 *
 * 用法:
 *   UART_Init(&huart1);
 *   // HAL_UARTEx_ReceiveToIdle_IT 需在 CubeMX USART 中开启 global interrupt
 *   while(1) {
 *       UART_Process();  // 处理接收到的帧 (ISR 里不做解析)
 *       UART_SendMcuData(yaw, pitch, roll, speed, mode);
 *       HAL_Delay(5);
 *   }
 */

#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include "stm32h7xx_hal.h"
#include "protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE 256    /* ReceiveToIdle 一次最多收这么多 */
#endif

#ifndef UART_RING_SIZE
#define UART_RING_SIZE   2048   /* 环形缓冲, 足够放好几帧 */
#endif

typedef enum {
    UART_CMD_NONE = 0,
    UART_CMD_GIMADVV = GIMADVV_CMD_ID,   /* 收到云台控制指令 */
    UART_CMD_STS     = STS_CMD_ID        /* 收到心跳包 */
} UartCmdType_t;

/* 解析出来的帧信息 */
typedef struct {
    uint16_t      cmd_id;
    uint8_t       data[256];    /* payload */
    size_t        data_len;
    UartCmdType_t type;
} UartFrame_t;

/**
 * @brief 初始化驱动 (启动 ReceiveToIdle 中断接收)
 * @param huart  CubeMX 配置好的 USART HAL 句柄指针
 */
void UART_Init(UART_HandleTypeDef *huart);

/**
 * @brief 主循环调用: 从环形缓冲里解析帧, 收到完整帧返回 true
 * @param frame 解析成功时填充这里
 * @return 1 = 收到完整有效帧, 0 = 还没收到
 */
uint8_t UART_Process(UartFrame_t *frame);

/**
 * @brief HAL 回调 —— RxEvent (由 stm32h7xx_it.c 转发, 用户不要直接改)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief HAL 回调 —— Rx Cplt (作为 ReceiveToIdle 之外的保底)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* --- 便捷发送 API --- */

/** 发送 MCU 姿态数据帧 (CMD_MCU_DATA 0x1021) */
HAL_StatusTypeDef UART_SendMcuData(float yaw, float pitch, float roll,
                                   float shoot_speed, uint8_t autoaim_mode);

/** 发送裁判系统数据帧 (CMD_ROBOT_DATA 0x1022) */
HAL_StatusTypeDef UART_SendRobotData(const robot_data_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DRIVER_H */
