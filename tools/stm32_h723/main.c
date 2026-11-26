/**
 * @file main.c
 * @brief STM32H723VGT6 下位机 —— auto-aim 上位机通信 Demo
 *
 * CubeMX 配置要求:
 *   - USART1: 921600, 8N1, WordLength=8, Parity=No, StopBits=1
 *             开启 USART1 global interrupt (ReceiveToIdle 需要)
 *   - NVIC:  USART1_IRQn 优先级设为 >= 2 (主循环优先级)
 *   - HCLK:  建议跑满 550MHz (H723 最高主频)
 *
 * 引脚 (常见):
 *   USART1_TX = PA9
 *   USART1_RX = PA10
 *   (接电脑 USB转TTL 或 Jetson ttyS4)
 *
 * 接线注意: TX->RX, RX->TX, GND 共地
 */

#include "stm32h7xx_hal.h"
#include "uart_driver.h"
#include "protocol.h"
#include <string.h>

/* ========================= HAL 句柄 ========================= */
UART_HandleTypeDef huart1;

/* ========================= 用户数据 (Demo) ========================= */

/* 模拟的云台姿态 —— 实际工程里应该从 IMU 读取 */
typedef struct {
    float yaw;
    float pitch;
    float roll;
} SimImu_t;

static SimImu_t       g_imu        = {0.0f, 0.0f, 0.0f};
static float          g_shoot_speed = 30.0f;   /* m/s */
static volatile uint8_t g_autoaim_mode = 1;    /* 默认开启自瞄 */

/* 最近一次收到的云台指令 (供调试 / LED 指示) */
static advv_detection_t g_last_cmd = {0};

/* ========================= CubeMX 生成的初始化 (占位) ========================= */

/* CubeMX 会生成以下函数, 这里给出空实现让 IDE 不报红:
   在 STM32CubeMX 里 pinout & configure 后重新 Generate 覆盖这些占位 */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* 如果你用 CubeMX, 就把它生成的 SystemClock_Config / MX_GPIO_Init / MX_USART1_UART_Init
   粘到 stm32h7xx_hal_msp.c / main.c 底部即可; 下面 main() 里的调用保持不变 */

/* ========================= 主函数 ========================= */

int main(void)
{
    /* ---- HAL + Cache ---- */
    HAL_Init();
    SystemClock_Config();
    /* H723 建议开 I-Cache/D-Cache 提升性能 */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* ---- CubeMX 初始化 ---- */
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* ---- 上位机协议驱动 ---- */
    UART_Init(&huart1);

    /* ---- 节拍 ---- */
    uint32_t last_send_tick    = HAL_GetTick();
    uint32_t last_process_tick = HAL_GetTick();

    /* ---- 主循环 ---- */
    for (;;) {
        uint32_t now = HAL_GetTick();

        /* ① 每 5ms 喂一次接收解析 (够及时, 也不频繁) */
        if ((now - last_process_tick) >= 5) {
            last_process_tick = now;

            UartFrame_t frame;
            if (UART_Process(&frame)) {
                switch (frame.type) {
                case UART_CMD_GIMADVV: {
                    memcpy(&g_last_cmd, frame.data, sizeof(advv_detection_t));
                    /* ===== 在这里把指令丢给云台 PID / 电机 ===== */
                    /* g_last_cmd.yaw, g_last_cmd.pit, g_last_cmd.shoot 等 */
                    break;
                }
                case UART_CMD_STS: {
                    /* 心跳包 payload 只有 1 字节 (mode), 实际工程里决定要不要 */
                    break;
                }
                default:
                    /* 未知 cmd_id, 忽略 (可能是上位机其他扩展) */
                    break;
                }
            }
        }

        /* ② 每 50ms (20Hz) 上报一次 MCU 姿态 */
        /* 上位机 auto-aim 的 imu 频率就是 ~20-50Hz, 保持一致 */
        if ((now - last_send_tick) >= 50) {
            last_send_tick = now;

            /* ---- Demo: 让 yaw / pitch 缓慢摆动, 方便调试 ---- */
            static float t = 0.0f;
            t += 0.05f;
            g_imu.yaw   =  sinf(t) * 30.0f;   /* ±30° */
            g_imu.pitch = -sinf(t * 0.7f) * 15.0f;
            g_imu.roll  =  cosf(t * 0.3f) *  3.0f;

            /* ---- 工程里应该调用 IMU 读取函数, 例如: ----
               IMU_Read(&g_imu.yaw, &g_imu.pitch, &g_imu.roll);
            */

            UART_SendMcuData(g_imu.yaw, g_imu.pitch, g_imu.roll,
                             g_shoot_speed, g_autoaim_mode);
        }

        /* 让出 CPU, 避免空转 */
        __WFI();
    }
}

/* ========================= USART1_IRQHandler =========================
 * CubeMX 生成的 stm32h7xx_it.c 里会有 USART1_IRQHandler,
 * 里面已经调用 HAL_UART_IRQHandler(&huart1), 而 HAL_UART_IRQHandler
 * 内部会调用 HAL_UARTEx_RxEventCallback (定义在 uart_driver.c).
 * 所以用户**不要**自己在 USART1_IRQHandler 里直接处理 RXNE,
 * 否则会和 HAL 状态机冲突.
 */

/* ========================= CubeMX 初始化占位 =========================
 * ====== 实际项目里用 CubeMX 生成的代码替换下面三个空函数 ======
 * 这里给出一个 USART1 921600 的最小参考:
 *
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate       = 921600;
    huart1.Init.WordLength     = UART_WORDLENGTH_8B;
    huart1.Init.StopBits       = UART_STOPBITS_1;
    huart1.Init.Parity         = UART_PARITY_NONE;
    huart1.Init.Mode           = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
    // 921600 波特率不能开 FIFO TCB, 否则可能丢字节
    HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart1);
}
*/

#if 0   /* ---- CubeMX 会覆盖, 这里仅作参考 ---- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc  = {0};
    RCC_ClkInitTypeDef clk  = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 4;
    osc.PLL.PLLN            = 275;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 4;
    osc.PLL.PLLR            = 2;
    HAL_RCC_OscConfig(&osc);
    clk.ClockType        = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                         | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                         | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource     = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider    = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider    = RCC_HCLK_DIV1;
    clk.APB1CLKDivider   = RCC_HCLK_DIV2;
    clk.APB2CLKDivider   = RCC_HCLK_DIV2;
    clk.APB3CLKDivider   = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

#endif
