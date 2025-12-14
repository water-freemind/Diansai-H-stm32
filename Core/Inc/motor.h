#ifndef MOTOR_H
#define MOTOR_H

/************************ 依赖头文件包含 ************************/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


/************************ 结构体定义 ************************/
// PID控制器结构体
typedef struct {
    float kp, ki, kd;          // PID比例/积分/微分参数
    float integral;            // 积分项
    float last_error;          // 上次误差值
    float output;              // PID输出值
    float output_limit;        // 输出限幅
    float integral_limit;      // 积分限幅
    uint32_t last_time;        // 上次计算时间（ms）
    float dt;                  // 采样时间（s）
    float Kff;                 // 前馈补偿系数
} PID;

// LED状态枚举
typedef enum {
    LED_OFF = 0,        // 常灭
    LED_ON = 1,         // 常亮  
    LED_SLOW_BLINK = 2, // 慢闪
    LED_FAST_BLINK = 3  // 快闪
} LED_State_t;

/************************ 外部变量声明 ************************/
// 按键相关
extern uint16_t Key_flag;
extern uint16_t Key_status;

// 编码器相关
extern int16_t EncoderL;
extern int16_t EncoderR;
extern volatile int32_t total_pulse_L;
extern volatile int32_t total_pulse_R;

// PID参数实例（可外部修改参数）
extern PID Vertical_pid_L;  // 左轮速度环PID
extern PID Vertical_pid_R;  // 右轮速度环PID
extern PID Angel_pid;       // 角度环PID
extern PID Loc_pid;         // 位置环PID（直行）
extern PID Loc_turn_pid;    // 位置环PID（转弯）

// 全局控制变量
extern int pwmL, pwmR;
extern int speedL, speedR;
extern int base_speed, base_pwm;
extern int feedforward, diff;
extern volatile float yaw_angle;  // 陀螺仪偏航角（JY62输出）
extern int distance;              // 累计行走距离（兼容用）

// LED相关
extern volatile uint32_t blink_counter;

/************************ 函数声明 ************************/
// 速度环PID计算（返回PWM值）
int VelocityL(int SpeedL, int Speed);  // 左轮速度环
int VelocityR(int SpeedR, int Speed);  // 右轮速度环

// 角度环PID计算（返回速度修正量）
int angle_pid(float aim_angle);

// 位置环PID计算（返回转向速度修正量）
int loc_pid(int dis_err);
int loc_turn_pid(int dis_err);

// PWM控制
void load_pwm(char wheel, int32_t pwm);    // 加载PWM到指定车轮
void pwm_limit(int32_t *load_pwmR, int32_t *load_pwmL); // PWM限幅

// 车轮方向控制
// wheel：'L'左轮/'R'右轮；fangxiang：0停止/1前进/2后退
void wheel_dir(char wheel, uint16_t fangxiang);

// 速度控制
void load_speed(char wheel, int16_t speed); // 加载目标速度到车轮
void speed_limit(char wheel, int16_t speed); // 速度限幅并加载PWM

// 按键检测
void Key_detect(void);

// 角度转向控制
void turn_angle(int aimgle);
void reset_turn_flag(void);
// 定时器中断回调（若需外部重定义可声明）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* MOTOR_H */
