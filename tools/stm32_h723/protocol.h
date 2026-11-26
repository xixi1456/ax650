/**
 * @file protocol.h
 * @brief RoboMaster RMCVSerial 协议定义 (STM32H723VGT6 下位机)
 *
 * 与上位机 /usr/local/include/RMCVSerial/protocol.hpp 完全对齐
 * CRC 表从 /usr/local/lib/libRMCVSerial.so 反汇编提取, 保证 100% 兼容
 *
 * 帧格式:  SOF(0xA5) | Length(2B小端) | Seq | CRC8 | CmdID(2B小端) | Data | CRC16(2B小端)
 * CRC8:    覆盖 SOF+Length+Seq (前4字节), init=0xFF, 库内查表
 * CRC16:   覆盖 SOF~Payload (整个帧除最后2字节), init=0xFFFF, 库内查表
 *
 * 上位机->下位机 CmdID: 0x0503 (GIMAdvv_CMD_ID)  advv_detection_t (28B)
 * 下位机->上位机 CmdID: 0x1021 (CMD_MCU_DATA)    pc_mcu_data_t   (17B)
 * 下位机->上位机 CmdID: 0x1022 (CMD_ROBOT_DATA)  robot_data_t    (33B)
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 协议常量 ========================= */

#define SOF_BYTE                0xA5
#define PROTOCAL_FRAME_MAX_SIZE 512

#define CRC8_LEN                1
#define CRC16_LEN               2
#define FRAME_HEADER_LEN        5   /* SOF + Length(2) + Seq + CRC8 */
#define CMD_ID_LEN              2
#define FRAME_OVERHEAD          (FRAME_HEADER_LEN + CMD_ID_LEN + CRC16_LEN)  /* 9 字节 */

/* ========================= 命令字 ========================= */

/* 上位机 -> 下位机 (下行) */
#define GIMADVV_CMD_ID          0x0503   /* 云台控制指令 */
#define STS_CMD_ID              0x0500   /* 下行心跳包 */

/* 下位机 -> 上位机 (上行) */
#define CMD_MCU_DATA            0x1021   /* MCU 姿态 + 射速 + 自瞄模式 */
#define CMD_ROBOT_DATA          0x1022   /* 裁判系统血量 */

/* ========================= 数据结构 (与上位机 packet.hpp 对齐) ========================= */

/* 上位机 -> 下位机: 云台控制指令 */
#pragma pack(push, 1)
typedef struct {
    float    yaw;          /* 目标 yaw 角 */
    float    pit;          /* 目标 pitch 角 */
    float    yaw_spd;      /* yaw 角速度指令 */
    float    pitch_spd;    /* pitch 角速度指令 */
    float    yaw_acc;      /* yaw 加速度指令 */
    float    pitch_acc;    /* pitch 加速度指令 */
    float    dist;         /* 目标距离 (m) */
    uint8_t  shoot;        /* 0=停火, 1=连发, 2=自行判断, 3=单发 */
    uint8_t  target_id;    /* 1-7=机器人, 8=前哨, 9=基地, 0=无目标 */
} advv_detection_t;
#pragma pack(pop)

/* 下位机 -> 上位机: MCU 姿态数据 */
#pragma pack(push, 1)
typedef struct {
    float    curr_yaw;       /* 绝对 yaw, 顺时针为正 */
    float    curr_pitch;     /* 水平为0, 向上为负 */
    float    curr_roll;
    float    shoot_speed;    /* 子弹初速 (m/s), 下位机 MCU 上报 */
    uint8_t  autoaim_mode;   /* 1=自瞄模式开启, 0=关闭 */
} pc_mcu_data_t;
#pragma pack(pop)

/* 下位机 -> 上位机: 裁判系统赛场信息 */
#pragma pack(push, 1)
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
    uint8_t  robot_id;
} robot_data_t;
#pragma pack(pop)

/* ========================= 帧封装 API ========================= */

/**
 * @brief 将 payload 封装成完整协议帧 (含 SOF/Len/Seq/CRC8/CmdID/CRC16)
 * @param cmd_id    命令字 (小端序在内部处理)
 * @param data      payload 数据指针
 * @param length    payload 字节数 (不含帧头/CRC)
 * @param out_buf   输出缓冲区 (长度必须 >= length + FRAME_OVERHEAD)
 * @return          完整帧总字节数, 失败返回 0
 */
size_t protocol_pack(uint16_t cmd_id, const uint8_t *data, size_t length, uint8_t *out_buf);

/**
 * @brief CRC8 计算 (与 RMCVSerial generate_crc8_checksum 完全一致)
 *        库内查表, init=0xFF
 * @param pchMessage 数据起始地址
 * @param dwLength   参与 CRC8 的字节数
 */
uint8_t protocol_crc8(const uint8_t *pchMessage, uint16_t dwLength);

/**
 * @brief CRC16 计算 (与 RMCVSerial generate_crc16_checksum 完全一致)
 *        库内查表 (poly 0x8005 反射版), init=0xFFFF
 * @param pchMessage 数据起始地址
 * @param dwLength   参与 CRC16 的字节数
 */
uint16_t protocol_crc16(const uint8_t *pchMessage, uint16_t dwLength);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
