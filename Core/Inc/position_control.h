#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

#include <stdint.h>

// 宏定义
#define PULSES_PER_MM      7.142857f   // 每毫米脉冲数 (根据实际调整)
#define MM_PER_PULSE       0.14f       // 每个脉冲对应的毫米数

// 位置控制状态枚举
typedef enum {
    POSITION_IDLE,      // 空闲状态
    POSITION_RUNNING,   // 正在运行
    POSITION_COMPLETED, // 完成
    POSITION_ERROR      // 错误
} Position_State_t;

// 运动方向枚举
typedef enum {
    DIRECTION_FORWARD,  // 前进
    DIRECTION_BACKWARD, // 后退
    DIRECTION_AUTO      // 自动（根据目标距离正负判断）
} Direction_t;

// 位置环PID结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分项
    float last_error;   // 上次误差
    float output;       // 输出值
    float output_limit; // 输出限幅
    float integral_limit; // 积分限幅
} Position_PID_t;

// 角度环滤波器结构体
typedef struct {
    float filtered_value; // 滤波后的值
} Angle_Filter_t;

// 位置控制配置结构体
typedef struct {
    Direction_t direction;      // 运动方向
    float forward_base_speed;   // 前进基础速度
    float backward_base_speed;  // 后退基础速度
    uint8_t enable_angle_correction; // 启用角度校正
} Position_Config_t;

// 全局变量声明
extern volatile int32_t total_pulse_L, total_pulse_R;
extern volatile Position_State_t position_state;
extern volatile uint8_t position_control_enable;
extern Position_PID_t position_pid;
extern Position_Config_t position_config;

// 函数声明
void position_control_init(void);
void set_target_distance(float distance_mm);
void set_target_distance_with_direction(float distance_mm, Direction_t direction);
void position_control_update(void);
uint8_t is_position_reached(void);
void stop_position_control(void);
void emergency_stop(void);
float get_current_distance_mm(void);
int32_t get_current_pulse(void);
void position_pid_reset(void);
int position_pid_calculate(int32_t target, int32_t current);

// 角度环相关函数
void init_angle_filter(void);
int get_filtered_angle_output(int angle_raw, Direction_t direction);
void reset_angle_filter(void);

// 配置函数
void set_position_direction(Direction_t direction);
void set_base_speeds(float forward_speed, float backward_speed);
void enable_angle_correction(uint8_t enable);
Direction_t get_current_direction(void);
int32_t get_target_pulse(void);

#endif /* __POSITION_CONTROL_H */
