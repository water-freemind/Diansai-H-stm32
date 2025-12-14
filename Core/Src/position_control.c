#include "position_control.h"
#include "motor.h"
#include <math.h>
#include <string.h>

// 全局变量定义
volatile int32_t total_pulse_L = 0, total_pulse_R = 0;
volatile Position_State_t position_state = POSITION_IDLE;
volatile uint8_t position_control_enable = 0;

// 位置环内部变量
static int32_t target_pulse = 0;
static float target_distance_mm = 0.0f;
static Direction_t current_direction = DIRECTION_FORWARD;
extern int line_angle;  // 外部变量，从其他地方获取的角度

// 角度环滤波器实例
static Angle_Filter_t angle_filter;

// 位置控制配置
Position_Config_t position_config = {
    .direction = DIRECTION_AUTO,
    .forward_base_speed = 3000.0f,
    .backward_base_speed = 1300.0f,
    .enable_angle_correction = 1
};

// 位置环PID参数（基于脉冲误差）
Position_PID_t position_pid = {
    .kp = 0.5f,           // 比例系数（需要根据实际调整）
    .ki = 0.02f,          // 积分系数
    .kd = 0.3f,           // 微分系数
    .integral = 0.0f,
    .last_error = 0.0f,
    .output = 0.0f,
    .output_limit = 800.0f,    // PWM输出限幅
    .integral_limit = 5000.0f  // 积分限幅
};

// 外部变量声明
extern int16_t EncoderL, EncoderR;

// 初始化角度环滤波器
void init_angle_filter(void)
{
    angle_filter.filtered_value = 0.0f;
}

// 获取滤波后的角度环输出（根据方向调整）
int get_filtered_angle_output(int angle_raw, Direction_t direction)
{
    // 一阶低通滤波参数
    #define FILTER_ALPHA 0.3f
    
    // 一阶低通滤波
    angle_filter.filtered_value = FILTER_ALPHA * (float)angle_raw + 
                                 (1.0f - FILTER_ALPHA) * angle_filter.filtered_value;
    
    int filtered_output = (int)angle_filter.filtered_value;
    
    // 后退时，角度环输出需要反转方向
    if (direction == DIRECTION_BACKWARD) {
        filtered_output = -filtered_output;
    }
    
    return filtered_output;
}

// 重置角度滤波器
void reset_angle_filter(void)
{
    init_angle_filter();
}

// 初始化位置环
void position_control_init(void)
{
    position_state = POSITION_IDLE;
    position_control_enable = 0;
    target_pulse = 0;
    target_distance_mm = 0.0f;
    current_direction = DIRECTION_FORWARD;
    
    // 初始化角度滤波器
    init_angle_filter();
    
    // 重置PID
    position_pid_reset();
}

// 设置目标距离（单位：mm），正数为前进，负数为后退
void set_target_distance(float distance_mm)
{
    // 根据距离正负自动判断方向
    if (distance_mm >= 0) {
        current_direction = DIRECTION_FORWARD;
        target_pulse = (int32_t)(distance_mm * PULSES_PER_MM);
    } else {
        current_direction = DIRECTION_BACKWARD;
        target_pulse = (int32_t)(fabs(distance_mm) * PULSES_PER_MM);
    }
    
    target_distance_mm = distance_mm;
    
    // 重置累计脉冲数
    total_pulse_L = 0;
    total_pulse_R = 0;
    
    // 重置PID
    position_pid_reset();
    
    // 重置角度滤波器
    reset_angle_filter();
    
    // 设置状态
    position_state = POSITION_RUNNING;
    position_control_enable = 1;
}

// 设置目标距离并指定方向
void set_target_distance_with_direction(float distance_mm, Direction_t direction)
{
    // 设置方向
    current_direction = direction;
    
    // 计算目标脉冲数（始终为正）
    target_pulse = (int32_t)(fabs(distance_mm) * PULSES_PER_MM);
    target_distance_mm = distance_mm;
    
    // 重置累计脉冲数
    total_pulse_L = 0;
    total_pulse_R = 0;
    
    // 重置PID
    position_pid_reset();
    
    // 重置角度滤波器
    reset_angle_filter();
    
    // 设置状态
    position_state = POSITION_RUNNING;
    position_control_enable = 1;
}

// 位置环PID计算
int position_pid_calculate(int32_t target, int32_t current)
{
    // 计算误差
    float error;
    
    if (current_direction == DIRECTION_FORWARD) {
        error = (float)(target - current);
    } else {
        // 后退时，误差计算方式相反
        error = (float)(current - target);
    }
    
    // 死区处理（减少震荡）
    if(fabs(error) < 5.0f) {
        position_pid.integral = 0;
        return 0;
    }
    
    // 积分项
    position_pid.integral += error;
    
    // 积分限幅
    if(position_pid.integral > position_pid.integral_limit) {
        position_pid.integral = position_pid.integral_limit;
    } else if(position_pid.integral < -position_pid.integral_limit) {
        position_pid.integral = -position_pid.integral_limit;
    }
    
    // 微分项
    float derivative = error - position_pid.last_error;
    
    // PID计算
    position_pid.output = position_pid.kp * error + 
                         position_pid.ki * position_pid.integral + 
                         position_pid.kd * derivative;
    
    // 输出限幅
    if(position_pid.output > position_pid.output_limit) {
        position_pid.output = position_pid.output_limit;
    } else if(position_pid.output < -position_pid.output_limit) {
        position_pid.output = -position_pid.output_limit;
    }
    
    // 保存本次误差
    position_pid.last_error = error;
    
    return (int)position_pid.output;
}

// 重置PID参数
void position_pid_reset(void)
{
    position_pid.integral = 0.0f;
    position_pid.last_error = 0.0f;
    position_pid.output = 0.0f;
}

// 位置环控制主函数（在main循环中调用）
void position_control_update(void)
{
    if(!position_control_enable) {
        return;
    }
    
    // 检查是否到达目标
    if(is_position_reached()) {
        stop_position_control();
        return;
    }
    
    // 获取当前平均脉冲数（始终取绝对值，因为我们用方向来区分前进后退）
    int32_t current_pulse = abs((total_pulse_L + total_pulse_R) / 2);
    
    // 计算位置环输出（速度指令）
    int speed_command = position_pid_calculate(target_pulse, current_pulse);
    
    // 动态基础速度：根据剩余距离调整
    int32_t remaining_pulse = target_pulse - current_pulse;
    int32_t abs_remaining = abs(remaining_pulse);
    
    int base_speed;
    int pwm_L, pwm_R;
    int angle_effect = 0;
    
    // 根据方向选择基础速度
    if (current_direction == DIRECTION_FORWARD) {
        // 前进
        if(abs_remaining > target_pulse * 0.3) {
            base_speed = (int)position_config.forward_base_speed;
        } else if(abs_remaining > target_pulse * 0.1) {
            base_speed = (int)(position_config.forward_base_speed * 0.85f);
        } else {
            base_speed = (int)(position_config.forward_base_speed * 0.6f);
        }
    } else {
        // 后退 - 使用负的速度
        if(abs_remaining > target_pulse * 0.3) {
            base_speed = -(int)position_config.backward_base_speed;
        } else if(abs_remaining > target_pulse * 0.1) {
            base_speed = -(int)(position_config.backward_base_speed * 0.85f);
        } else {
            base_speed = -(int)(position_config.backward_base_speed * 0.6f);
        }
    }
    
    // 应用角度校正（如果启用）
    if (position_config.enable_angle_correction) {
        // 获取滤波后的角度环输出，根据方向调整
        angle_effect = get_filtered_angle_output(angle_pid(line_angle), current_direction);
        
        // 在低速阶段减小角度环的影响
        if(abs_remaining <= target_pulse * 0.1) {
            angle_effect = (int)(angle_effect * 0.3f);
        } else if(abs_remaining <= target_pulse * 0.3) {
            angle_effect = (int)(angle_effect * 0.8f);
        }
    }
    
    // 计算PWM值 - 这是关键修改部分！
    // 前进时：左轮 + 角度效应，右轮 - 角度效应
    // 后退时：左轮 - 角度效应，右轮 + 角度效应（因为电机方向相反）
    if (current_direction == DIRECTION_FORWARD) {
        pwm_L = base_speed + speed_command + angle_effect;
        pwm_R = base_speed + speed_command - angle_effect;
    } else {
        // 后退时，角度效应方向需要反转
        pwm_L = base_speed + speed_command - angle_effect;  // 注意这里减去了角度效应
        pwm_R = base_speed + speed_command + angle_effect;  // 这里加上了角度效应
    }
    
    // 限幅处理
    #define PWM_MAX 7199
    #define PWM_MIN -7199
    
    if(pwm_L > PWM_MAX) pwm_L = PWM_MAX;
    if(pwm_L < PWM_MIN) pwm_L = PWM_MIN;
    if(pwm_R > PWM_MAX) pwm_R = PWM_MAX;
    if(pwm_R < PWM_MIN) pwm_R = PWM_MIN;
    
    // 加载PWM（使用您原有的load_pwm函数）
    load_pwm('L', pwm_L);
    load_pwm('R', pwm_R);
}

// 检查是否到达目标位置
uint8_t is_position_reached(void)
{
    // 计算平均脉冲数（取绝对值）
    int32_t avg_pulse = abs((total_pulse_L + total_pulse_R) / 2);
    
    // 允许的误差范围（±N个脉冲，约±0.14*N mm）
    int32_t tolerance = (int32_t)(50.0f * PULSES_PER_MM);
    
    if(abs(target_pulse - avg_pulse) < tolerance) {
        return 1;  // 到达目标
    }
    return 0;      // 未到达
}

// 停止位置控制（平滑停止）
void stop_position_control(void)
{
    // 平滑停止
    static int stop_counter = 0;
    
    if(stop_counter < 5) {  // 50ms内逐渐减速
        // 根据当前方向确定停止的方向
        int pwm_direction = (current_direction == DIRECTION_FORWARD) ? 1 : -1;
        int pwm = 800 * (5 - stop_counter) / 5 * pwm_direction;
        load_pwm('L', pwm);
        load_pwm('R', pwm);
        stop_counter++;
    } else {
        // 完全停止
        load_pwm('L', 0);
        load_pwm('R', 0);
        
        // 重置状态
        position_control_enable = 0;
        position_state = POSITION_COMPLETED;
        stop_counter = 0;
    }
}

// 紧急停止（立即停止）
void emergency_stop(void)
{
    load_pwm('L', 0);
    load_pwm('R', 0);
    
    position_control_enable = 0;
    position_state = POSITION_IDLE;
    
    // 重置滤波器
    reset_angle_filter();
}

// 获取当前距离（单位：mm），正数表示前进距离，负数表示后退距离
float get_current_distance_mm(void)
{
    int32_t avg_pulse = (total_pulse_L + total_pulse_R) / 2;
    float distance = fabs(avg_pulse) * MM_PER_PULSE;
    
    // 根据方向返回带符号的距离
    if (current_direction == DIRECTION_BACKWARD) {
        return -distance;
    }
    return distance;
}

// 获取当前脉冲数（带符号）
int32_t get_current_pulse(void)
{
    int32_t avg_pulse = (total_pulse_L + total_pulse_R) / 2;
    if (current_direction == DIRECTION_BACKWARD) {
        return -abs(avg_pulse);
    }
    return abs(avg_pulse);
}

// 设置运动方向
void set_position_direction(Direction_t direction)
{
    position_config.direction = direction;
    current_direction = direction;
}

// 设置基础速度
void set_base_speeds(float forward_speed, float backward_speed)
{
    position_config.forward_base_speed = forward_speed;
    position_config.backward_base_speed = backward_speed;
}

// 启用/禁用角度校正
void enable_angle_correction(uint8_t enable)
{
    position_config.enable_angle_correction = enable;
}

// 获取当前方向
Direction_t get_current_direction(void)
{
    return current_direction;
}

// 获取目标脉冲数
int32_t get_target_pulse(void)
{
    return target_pulse;
}
