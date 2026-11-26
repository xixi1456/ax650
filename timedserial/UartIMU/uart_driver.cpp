//
// UART Driver Implementation - Stateless event-driven driver
//

#include "uart_driver.hpp"
#include "packet.hpp"
#include <functional>
#include <chrono>

using namespace std::chrono;

// 裁判系统规定的最低子弹初速（单位：米/秒）
constexpr float MIN_BULLET_SPEED_MPS = 10.0f;

UartDriver::UartDriver(const std::string device_name) 
    : m_device_name(device_name), m_serial()
{
    // 注册数据包处理函数
    m_serial.register_handler(CMD_MCU_DATA, 
        std::bind(&UartDriver::on_receive_imu, this, std::placeholders::_1, std::placeholders::_2));
    m_serial.register_handler(CMD_ROBOT_DATA, 
        std::bind(&UartDriver::on_receive_sts, this, std::placeholders::_1, std::placeholders::_2));
}

bool UartDriver::init()
{
    LOGM_S("[UART] Opening %s @921600 8N1", m_device_name.c_str());
    // ⚠️ 必须显式指定 parity::none！
    // RMCVSerial::open() 默认 parity=even, STM32 USART1 是 8N1
    // parity 不匹配会导致 boost::asio 读串口报错, async_receive_handler 自动 close()
    m_serial.open(m_device_name,
                  921600,
                  drivers::flowControlType::none,
                  8,
                  drivers::parityType::none,
                  drivers::stopBitsType::one);
    LOGM_S("[UART] port is_open()=%d", m_serial.is_open());
    return m_serial.is_open();
}

void UartDriver::start()
{
    m_serial.start_async_receive();
}

void UartDriver::close()
{
    m_serial.stop_async_receive();
}

/**
 * @brief 处理IMU姿态数据包
 * 
 * 重构要点：
 * - 不再保存状态到成员变量 m_attitude
 * - 构造临时 Attitude 对象
 * - 立即通过回调传递给业务层
 * - shoot_speed 包含在此包中，作为 RobotStatus 的一部分单独触发回调
 * - 实机路径使用 MCU 上报的真实 shoot_speed，下限 10 m/s
 */
void UartDriver::on_receive_imu(drivers::packet_data_t* packet_ptr, drivers::packet_length_t len)
{
    LOGM_S("[UART] ⚡ on_receive_imu! len=%zu sizeof(pc_mcu_data_t)=%zu",
           len, sizeof(pc_mcu_data_t));

    pc_mcu_data_t* _tmp_ptr = (pc_mcu_data_t*)packet_ptr;
    LOGM_S("[UART]   yaw=%.2f pitch=%.2f roll=%.2f shoot_speed=%.1f autoaim_mode=%d",
           _tmp_ptr->curr_yaw, _tmp_ptr->curr_pitch, _tmp_ptr->curr_roll,
           _tmp_ptr->shoot_speed, (int)_tmp_ptr->autoaim_mode);

    // 1. 构造临时的 Attitude 对象并触发回调
    Attitude temp_att{_tmp_ptr->curr_yaw, _tmp_ptr->curr_pitch, _tmp_ptr->curr_roll};
    
    if (this->attitude_cb_) {
        this->attitude_cb_(temp_att);
    }
    
    // 2. shoot_speed 属于 RobotStatus，单独触发状态回调
    // 注意：这里只更新射速，其他字段由 on_receive_sts 填充
    if (this->status_cb_) {
        RobotStatus temp_status;
        temp_status.robot_speed_mps = _tmp_ptr->shoot_speed;

        // 确保射速不低于最小值
        if (temp_status.robot_speed_mps < MIN_BULLET_SPEED_MPS) {
            temp_status.robot_speed_mps = MIN_BULLET_SPEED_MPS;
        }
        temp_status.program_mode = (ProgramMode)_tmp_ptr->autoaim_mode;

        // std::cout << "Autoaim Mode: " << (int)_tmp_ptr->autoaim_mode << std::endl;

        this->status_cb_(temp_status);
    }
}

/**
 * @brief 处理裁判系统数据包
 * 
 * 重构要点：
 * - 不再保存状态到成员变量 m_robotstatus
 * - 构造临时 RobotStatus 对象
 * - 解析敌方颜色和血量信息
 * - 立即通过回调传递给业务层
 */
void UartDriver::on_receive_sts(drivers::packet_data_t* packet_ptr, drivers::packet_length_t len)
{
    LOGM_S("[UART] ⚡ on_receive_sts! len=%zu sizeof(robot_data_t)=%zu",
           len, sizeof(robot_data_t));
    // 校验数据包长度
    if (len != sizeof(robot_data_t)) {
        LOGW_S("[UART][ERROR] ❌ invalid robot status data length: got=%zu expected=%zu",
               len, sizeof(robot_data_t));
        return;
    }
    
    robot_data_t* state_ptr = (robot_data_t*)packet_ptr;
    LOGM_S("[UART] robot_id=%d red_base=%d blue_base=%d",
           state_ptr->robot_id, state_ptr->red_base_HP, state_ptr->blue_base_HP);

    // 构造临时的 RobotStatus 对象
    RobotStatus temp_status;
    
    // 根据 robot_id 判断己方颜色，从而确定敌方颜色
    // 1-20: 红方机器人，敌方是蓝色
    // 100+: 蓝方机器人，敌方是红色
    if (0 < state_ptr->robot_id && state_ptr->robot_id < 20) {
        temp_status.enemy_color = EnemyColor::BLUE;
        temp_status.enemy[0] = state_ptr->blue_7_robot_HP;  // 哨兵
        temp_status.enemy[1] = state_ptr->blue_1_robot_HP;  // 英雄
        temp_status.enemy[2] = state_ptr->blue_2_robot_HP;  // 工程
        temp_status.enemy[3] = state_ptr->blue_3_robot_HP;  // 步兵3
        temp_status.enemy[4] = state_ptr->blue_4_robot_HP;  // 步兵4
        temp_status.enemy[5] = state_ptr->blue_5_robot_HP;  // 步兵5
    }
    else if (state_ptr->robot_id >= 100) {
        temp_status.enemy_color = EnemyColor::RED;
        temp_status.enemy[0] = state_ptr->red_7_robot_HP;   // 哨兵
        temp_status.enemy[1] = state_ptr->red_1_robot_HP;   // 英雄
        temp_status.enemy[2] = state_ptr->red_2_robot_HP;   // 工程
        temp_status.enemy[3] = state_ptr->red_3_robot_HP;   // 步兵3
        temp_status.enemy[4] = state_ptr->red_4_robot_HP;   // 步兵4
        temp_status.enemy[5] = state_ptr->red_5_robot_HP;   // 步兵5
    }
    else {
        temp_status.enemy_color = EnemyColor::GRAY;
    }

    // std::cout << "Enemy Color: " << (int)temp_status.enemy_color << std::endl;

    // 触发回调
    if (this->status_cb_) {
        this->status_cb_(temp_status);
    }
}

/**
 * @brief 发送控制指令到下位机
 * 
 * 保持原有逻辑不变
 */
void UartDriver::transmit_cmd(float yaw, float pitch, float yaw_spd, float pitch_spd, float yaw_acc, float pitch_acc, float dist, uint8_t shoot, uint8_t target_id)
{
    advv_detection_t data_to_send;
    data_to_send.yaw = yaw;
    data_to_send.yaw_spd = yaw_spd;
    data_to_send.pit = pitch;
    data_to_send.pitch_spd = pitch_spd;
    data_to_send.yaw_acc = yaw_acc;
    data_to_send.pitch_acc = pitch_acc;
    data_to_send.dist = dist;
    data_to_send.shoot = shoot;
    data_to_send.target_id = target_id;
    
    m_serial.send(GIMAdvv_CMD_ID, (drivers::packet_data_t*)&data_to_send, sizeof(data_to_send));
}
