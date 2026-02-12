#ifndef __GIMBAL_DRIVER_H
#define __GIMBAL_DRIVER_H

#include "main.h" // 包含 HAL 库定义
#include "usart.h" // 包含 huart3 定义

// 定义电机地址
#define GIMBAL_LOWER_ADDR  0xE0 // 下云台 (Pan)
#define GIMBAL_UPPER_ADDR  0xE1 // 上云台 (Tilt)

// 定义方向
#define DIR_CW  0  // 顺时针
#define DIR_CCW 1  // 逆时针

/* 功能函数声明 */

// 速度控制模式 (对应手册 5.6 节)
// speed: 0-127 (实际转速需根据手册公式换算)
void Gimbal_RunSpeed(uint8_t addr, uint8_t direction, uint8_t speed);

// 相对位置控制模式 (对应手册 5.6 节 串口直接位置控制)
// speed: 0-127
// pulses: 脉冲数 (需根据细分设置换算角度)
void Gimbal_MoveRelative(uint8_t addr, uint8_t direction, uint8_t speed, uint32_t pulses);
void Gimbal_Run_Signed(uint8_t addr, int16_t signed_speed) ;
// 停止 (对应手册 e0 f7)
void Gimbal_Stop(uint8_t addr);

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float output_limit; // 限制最大输出 (对应电机最大速度档位 127)
    float integral_limit;
} PID_Controller;

int8_t PID_Compute(PID_Controller *pid, float error);

extern PID_Controller pan_pid;
extern PID_Controller tilt_pid;
#endif
