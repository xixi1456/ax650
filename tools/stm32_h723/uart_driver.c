/**
 * @file uart_driver.c
 * @brief STM32H723VGT6 UART 驱动实现
 */

#include "uart_driver.h"
#include <string.h>

/* ========================= 模块内状态 ========================= */

static UART_HandleTypeDef  *s_huart      = NULL;
static uint8_t              s_rx_buf     [UART_RX_BUF_SIZE];
static uint8_t              s_ring_buf   [UART_RING_SIZE];
static volatile uint16_t    s_ring_head  = 0;   /* ISR 写 */
static volatile uint16_t    s_ring_tail  = 0;   /* 主循环读 */

/* 解析状态机 (与上位机 RMCVSerial unpack_process 完全对齐) */
typedef enum {
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

static unpack_step_e s_step = STEP_HEADER_SOF;
static uint16_t      s_data_length = 0;      /* payload 长度 */
static uint16_t      s_frame_len   = 0;      /* 完整帧总长 = s_data_length + FRAME_OVERHEAD */
static uint8_t       s_work_buf    [PROTOCAL_FRAME_MAX_SIZE];
static uint16_t      s_work_idx    = 0;      /* 当前已缓存到 s_work_buf 的字节数 */

/* ========================= 环形缓冲 ========================= */

static inline uint16_t ring_available(void)
{
    /* tail 指向"下一个要读"的位置, head 指向"下一个要写"的位置 */
    int16_t d = (int16_t)(s_ring_head - s_ring_tail);
    if (d < 0) d += UART_RING_SIZE;
    return (uint16_t)d;
}

static inline void ring_push(const uint8_t *data, uint16_t len)
{
    /* ISR 上下文, 关中断保护写 head */
    __disable_irq();
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (s_ring_head + 1) % UART_RING_SIZE;
        if (next == s_ring_tail) {
            /* 溢出: 丢弃, 主循环跟不上就丢 */
            break;
        }
        s_ring_buf[s_ring_head] = data[i];
        s_ring_head = next;
    }
    __enable_irq();
}

static inline uint8_t ring_pop_one(void)
{
    uint8_t b = 0;
    if (s_ring_head == s_ring_tail) return 0;
    __disable_irq();
    b = s_ring_buf[s_ring_tail];
    s_ring_tail = (s_ring_tail + 1) % UART_RING_SIZE;
    __enable_irq();
    return b;
}

/* ========================= HAL 回调 ========================= */

/** ReceiveToIdle 回调: 收完一段立即进环缓, 立即重新投递 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != s_huart) return;
    if (Size > 0) {
        ring_push(s_rx_buf, Size);
    }
    /* 立即重新投递, 避免丢字节 */
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx_buf, UART_RX_BUF_SIZE);
}

/** 作为 ReceiveToIdle 之外的保底 (有些情况下会走这个) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) return;
    ring_push(s_rx_buf, 1);
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx_buf, UART_RX_BUF_SIZE);
}

/* ========================= 初始化 ========================= */

void UART_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_step  = STEP_HEADER_SOF;
    s_work_idx = 0;
    s_ring_head = s_ring_tail = 0;
    memset(s_work_buf, 0, sizeof(s_work_buf));

    /* 启动 ReceiveToIdle 中断接收 */
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx_buf, UART_RX_BUF_SIZE);
}

/* ========================= 帧解析状态机 ========================= */

/**
 * @brief 从环形缓冲里尝试吃一个字节, 解析状态机前进
 * @return 1 = 完整帧成功解析, 0 = 还没凑齐
 */
static uint8_t parse_one_byte(uint8_t byte, UartFrame_t *out)
{
    switch (s_step) {

    case STEP_HEADER_SOF:
        if (byte == SOF_BYTE) {
            s_work_buf[0] = byte;
            s_work_idx = 1;
            s_step = STEP_LENGTH_LOW;
        }
        /* 不是 0xA5 就跳过 */
        break;

    case STEP_LENGTH_LOW:
        s_work_buf[1] = byte;
        s_work_idx = 2;
        s_step = STEP_LENGTH_HIGH;
        break;

    case STEP_LENGTH_HIGH:
        s_work_buf[2] = byte;
        s_work_idx = 3;
        s_data_length = (uint16_t)s_work_buf[1] | ((uint16_t)s_work_buf[2] << 8);
        s_frame_len = s_data_length + FRAME_OVERHEAD;

        /* 防溢出: 与上位机 unpack_process 里 cmp w0, #0x1f6 (502) 对齐 */
        if (s_data_length > 502) {
            s_step = STEP_HEADER_SOF;
        } else {
            s_step = STEP_FRAME_SEQ;
        }
        break;

    case STEP_FRAME_SEQ:
        s_work_buf[3] = byte;
        s_work_idx = 4;
        s_step = STEP_HEADER_CRC8;
        break;

    case STEP_HEADER_CRC8:
        s_work_buf[4] = byte;
        s_work_idx = 5;

        /* CRC8 校验: 覆盖 SOF+Len+Seq+CRC8 自己, 即前 5 字节 */
        if (protocol_crc8(s_work_buf, 5) != 0) {
            /* CRC8 不通过, 重新等 SOF */
            s_step = STEP_HEADER_SOF;
        } else {
            s_step = STEP_DATA_CRC16;
        }
        break;

    case STEP_DATA_CRC16:
        /* 持续收字节直到凑够 s_frame_len */
        s_work_buf[s_work_idx++] = byte;
        if (s_work_idx >= s_frame_len) {
            /* 完整帧到了, 校验 CRC16 */
            /* CRC16 覆盖 Seq..Data (offset 3 到 frame_len-3), 结果存最后 2 字节 */
            uint16_t calc_crc = protocol_crc16(&s_work_buf[3], (uint16_t)(s_frame_len - 3));
            uint16_t recv_crc = (uint16_t)s_work_buf[s_frame_len - 2]
                               | ((uint16_t)s_work_buf[s_frame_len - 1] << 8);

            if (calc_crc == recv_crc) {
                /* 帧有效, 解析到 UartFrame_t */
                out->cmd_id   = (uint16_t)s_work_buf[5] | ((uint16_t)s_work_buf[6] << 8);
                out->data_len = s_data_length;
                if (s_data_len > 0) {
                    memcpy(out->data, &s_work_buf[7], s_data_length);
                }
                switch (out->cmd_id) {
                    case GIMADVV_CMD_ID: out->type = UART_CMD_GIMADVV; break;
                    case STS_CMD_ID:     out->type = UART_CMD_STS;     break;
                    default:             out->type = UART_CMD_NONE;    break;
                }
                s_step = STEP_HEADER_SOF;
                s_work_idx = 0;
                return 1;   /* ✅ 成功 */
            } else {
                /* CRC16 失败, 丢弃 */
                s_step = STEP_HEADER_SOF;
                s_work_idx = 0;
            }
        }
        break;

    default:
        s_step = STEP_HEADER_SOF;
        break;
    }
    return 0;
}

/* ========================= 对外 Process API ========================= */

uint8_t UART_Process(UartFrame_t *frame)
{
    /* 每次 Process 最多吃 N 字节, 避免长时间阻塞 */
    static const uint16_t MAX_EAT = 1024;
    uint16_t eaten = 0;

    while (ring_available() > 0 && eaten < MAX_EAT) {
        uint8_t byte = ring_pop_one();
        eaten++;
        if (parse_one_byte(byte, frame)) {
            return 1;
        }
    }
    return 0;
}

/* ========================= 发送 API ========================= */

HAL_StatusTypeDef UART_SendMcuData(float yaw, float pitch, float roll,
                                   float shoot_speed, uint8_t autoaim_mode)
{
    pc_mcu_data_t payload = {
        .curr_yaw     = yaw,
        .curr_pitch   = pitch,
        .curr_roll    = roll,
        .shoot_speed  = shoot_speed,
        .autoaim_mode = autoaim_mode
    };
    uint8_t frame[sizeof(payload) + FRAME_OVERHEAD];
    size_t  flen = protocol_pack(CMD_MCU_DATA,
                                 (const uint8_t *)&payload, sizeof(payload),
                                 frame);
    if (flen == 0) return HAL_ERROR;
    return HAL_UART_Transmit(s_huart, frame, (uint16_t)flen, 100);
}

HAL_StatusTypeDef UART_SendRobotData(const robot_data_t *rb)
{
    if (rb == NULL) return HAL_ERROR;
    uint8_t frame[sizeof(robot_data_t) + FRAME_OVERHEAD];
    size_t  flen = protocol_pack(CMD_ROBOT_DATA,
                                 (const uint8_t *)rb, sizeof(*rb),
                                 frame);
    if (flen == 0) return HAL_ERROR;
    return HAL_UART_Transmit(s_huart, frame, (uint16_t)flen, 100);
}
