#include "gimbal_driver.h"
#include <string.h>

// 发送缓冲区
// 根据手册，位置控制指令最长：地址+功能码+方向速度+4字节脉冲+校验 = 8字节
#define TX_BUF_SIZE 10 
uint8_t gimbal_tx_buf[TX_BUF_SIZE];

/**
 * @brief 计算校验码 (Check Sum)
 * 算法：(地址 + 功能码 + 数据) & 0xFF
 */
static uint8_t Calc_CheckSum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

/**
 * @brief 通过 DMA 发送数据包
 */
static void Gimbal_SendPacket(uint8_t *packet, uint8_t len) {
    // 检查串口是否忙碌（防止上一次 DMA 还没发完就覆盖数据）
    // 如果想要完全非阻塞，这里可以使用队列，但简单应用中等待 Ready 即可
    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
    
    // 复制数据到全局缓冲区（因为 DMA 发送是异步的，不能发送栈上的局部变量）
    memcpy(gimbal_tx_buf, packet, len);
    
    // 启动 DMA 发送
    HAL_UART_Transmit_DMA(&huart2, gimbal_tx_buf, len);
}

/**
 * @brief 速度模式控制
 * 指令格式: Address + 0xF6 + [Dir(1bit)|Speed(7bit)] + CRC
 * 参考手册第 18-19 页
 */
void Gimbal_RunSpeed(uint8_t addr, uint8_t direction, uint8_t speed) {
    uint8_t packet[4];
    uint8_t dir_speed_byte = 0;

    // 限制速度范围 0-127
    if (speed > 127) speed = 127;

    // 组合方向和速度字节 (最高位是方向)
    if (direction == DIR_CCW) {
        dir_speed_byte = 0x80 | speed; // 置最高位为1
    } else {
        dir_speed_byte = speed;        // 最高位为0
    }

    packet[0] = addr;
    packet[1] = 0xF6; // 功能码
    packet[2] = dir_speed_byte;
    packet[3] = Calc_CheckSum(packet, 3); // 计算前3个字节的校验和

    Gimbal_SendPacket(packet, 4);
}

/**
 * @brief 停止电机
 * 指令: Address + 0xF7 + CRC
 */
void Gimbal_Stop(uint8_t addr) {
    uint8_t packet[3];
    
    packet[0] = addr;
    packet[1] = 0xF7;
    packet[2] = Calc_CheckSum(packet, 2);
    
    Gimbal_SendPacket(packet, 3);
}

//初始化 PID 参数
PID_Controller pan_pid = {
    0.15f,  // Kp
    0.005f, // Ki
    0.0f,   // Kd
    0,      // prev_error
    0,      // integral
    127.0f, // output_limit
    50.0f   // integral_limit
};
PID_Controller tilt_pid = {
    0.20f,  // Kp
    0.008f, // Ki
    0.0f,   // Kd
    0,      // prev_error
    0,      // integral
    127.0f, // output_limit
    50.0f   // integral_limit
};
/**
 * @brief 计算 PID 输出
 * @param pid: 指向 PID_Controller 结构体的指针
 * @param error: 当前误差值
 * @return: 计算得到的控制输出，范围 -127 到 127
 */
// error = 画面中心坐标 - 目标中心坐标
int8_t PID_Compute(PID_Controller *pid, float error) {
    float p_out, i_out, d_out, total_out;

    // 1. 死区处理（防止在目标静止时云台抖动）
    if (error > -10 && error < 10) { // 假设误差 10 像素以内忽略
        pid->prev_error = error; // 更新但不积分
        return 0;
    }

    // 2. 比例项
    p_out = pid->Kp * error;

    // 3. 积分项
    pid->integral += error;
    // 积分限幅
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    i_out = pid->Ki * pid->integral;

    // 4. 微分项
    d_out = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    // 5. 总输出
    total_out = p_out + i_out + d_out;

    // 6. 输出限幅 (限制在 -127 到 127 之间，因为电机串口协议只支持 7位速度)
    if (total_out > pid->output_limit) total_out = pid->output_limit;
    if (total_out < -pid->output_limit) total_out = -pid->output_limit;

    return (int8_t)total_out;
}
