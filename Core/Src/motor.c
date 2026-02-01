#include "OLED.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include "stm32f1xx_it.h"
#include "motor.h"
#include "jy62.h"
#include <stdint.h>
#include "stdlib.h"
#include "k230_uart.h"

extern  uint16_t Key_flag;
extern uint16_t Key_status;
extern int16_t EncoderL,EncoderR;

int32_t last_encoderL = 0, last_encoderR = 0;
int pwmL,pwmR;
int speedL,speedR;
extern int base_speed,base_pwm;
int32_t PWM_MAX = 7199,PWM_MIN = -7199;//pwm最大值

volatile uint8_t turn_success_flag = 0;  // 转向成功标志位
uint32_t turn_success_timer = 0;         // 转向计时器
float last_yaw_angle = 0;               // 上次的yaw角度

//一定要判断  极性！！！！！！！

//速度环PID参数
PID Vertical_pid_L={
    .kp = 80,//80
    .ki = 40,//40
    .kd = 0.0,
    .integral_limit = 105,
    .output_limit = 7199
};
PID Vertical_pid_R={
    .kp = 80,
    .ki = 40,
    .kd = 0.0,
    .integral_limit = 105,
    .output_limit = 7199
};
//角度环PID参数
PID Angel_pid={
    .kp = 120,//120
    .ki = 15,//5
    .kd = 10,//5
    .integral_limit = 800,//600
    .output_limit = 1700//1700
};
//位置环PID
PID Loc_pid = {
    .kp = 12,           // 平衡KP  10
    .ki = 0.38,          // 平衡KI
    .kd = 3.3,          // 强KD
    .integral_limit = 50,
    .output_limit = 600
};

PID Loc_turn_pid = {
    .kp = 20,           // 强KP
    .ki = 0.42,         // 平衡KI
    .kd = 3.9,          // 强KD
    .integral_limit = 42,
    .output_limit = 680
};


//SpeedL/SpeedR为编码器的值，Speed为目标速度
int VelocityL(int SpeedL,int Speed){
    static int PWM_out,Speed_err,Speed_sum;
    Speed_err = Speed - SpeedL;//获取此次误差值
    Speed_sum += Speed_err;
    //积分限幅
    if (Speed_sum>Vertical_pid_L.integral_limit)Speed_sum = Vertical_pid_L.integral_limit;
    else if (Speed_sum<-Vertical_pid_L.integral_limit)Speed_sum = -Vertical_pid_L.integral_limit;

    PWM_out = Vertical_pid_L.kp*Speed_err + Vertical_pid_L.ki*Speed_sum ;//仅仅使用PI速度环
    
    if (PWM_out > Vertical_pid_L.output_limit) PWM_out = Vertical_pid_L.output_limit;
    else if (PWM_out < -Vertical_pid_L.output_limit) PWM_out = -Vertical_pid_L.output_limit;
    return (int)PWM_out;//返回整型值给load_pwm()
}
int VelocityR(int SpeedR,int Speed){
    static int PWM_out,Speed_err,Speed_sum;
    Speed_err = Speed - SpeedR ;//获取此次误差值
    Speed_sum += Speed_err;
    //积分限幅
    if (Speed_sum>Vertical_pid_R.integral_limit)Speed_sum = Vertical_pid_R.integral_limit;
    else if (Speed_sum<-Vertical_pid_R.integral_limit)Speed_sum = -Vertical_pid_R.integral_limit;

    PWM_out = Vertical_pid_R.kp*Speed_err + Vertical_pid_R.ki*Speed_sum ;//仅仅使用PI速度环
    //输出限幅
    if (PWM_out > Vertical_pid_R.output_limit) PWM_out = Vertical_pid_R.output_limit;
    else if (PWM_out < -Vertical_pid_R.output_limit) PWM_out = -Vertical_pid_R.output_limit;
    return (int)PWM_out;//返回整型值给load_pwm()
}

//角度环串级PID
extern volatile float yaw_angle;
int angle_errcounter = 0;
int angle_derivativecounter = 0;
int angle_pid(float aim_angle){//输入执行目标角度
    static float angle_err=0,angle_last_err=0,angle_err_sum=0;
    static float angle_err_history[5] = {0};  // 存储最近几次的角度误差
    static int history_index = 0;
    
    angle_err =  yaw_angle - aim_angle;//获取角度差值
    
    angle_errcounter = angle_err;
    // 存储角度误差历史数据
    angle_err_history[history_index] = angle_err;
    history_index = (history_index + 1) % 5;
    
    if(abs((int)angle_err) < 4 && abs((int)angle_err) > 1){  // 设定一个阈值，比如5度
        angle_err_sum += angle_err;
    }
    else{
        angle_err_sum = 0;
    }
    //积分限幅
    if(angle_err_sum>Angel_pid.integral_limit){angle_err_sum = Angel_pid.integral_limit;}
    if(angle_err_sum<-Angel_pid.integral_limit){angle_err_sum = -Angel_pid.integral_limit;}
    
    // 获取当前时间，计算时间间隔
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // 转换为秒
    if (dt <= 0 || dt > 0.1f) dt = 0.01f; // 防止异常时间间隔
    last_time = current_time;
    
    // 微分项（角度变化率）
    float angle_derivative = (angle_err - angle_last_err) / dt;
    
    angle_derivativecounter = angle_derivative;
    //角度差转角速度
    float speederr = angle_err * Angel_pid.kp + angle_derivative * Angel_pid.kd + angle_err_sum * Angel_pid.ki ;
    if (speederr>Angel_pid.output_limit) {speederr = Angel_pid.output_limit;}
    if (speederr<-Angel_pid.output_limit) {speederr = -Angel_pid.output_limit;}
    
    angle_last_err = angle_err;//储存上一次误差值
    last_yaw_angle = yaw_angle; // 更新上次的yaw角度
    
    return (int)speederr;
}
int feedforward,diff;
int loc_pid(int dis_err){
    static int turnspeedout,dis_cu_err,dis_last_err,dis_err_sum;
    dis_last_err = dis_cu_err;//更新数据
    diff = dis_cu_err - dis_last_err;
    dis_cu_err = dis_err;//0---targetvalue
    dis_err_sum += dis_cu_err ;
    if(abs(dis_cu_err)>30){dis_err_sum = 0;}
    //积分限幅
    if(dis_err_sum>Loc_pid.integral_limit)dis_err_sum = Loc_pid.integral_limit;
    else if (dis_err_sum<-Loc_pid.integral_limit)dis_err_sum = -Loc_pid.integral_limit;
    //前馈补偿
    feedforward = Loc_pid.Kff * (dis_err - dis_last_err);

    turnspeedout = feedforward + dis_cu_err*Loc_pid.kp + (dis_cu_err-dis_last_err)*Loc_pid.kd + Loc_pid.ki*dis_err_sum;//前馈+PID
    //输出限幅
    if(turnspeedout>Loc_pid.output_limit)turnspeedout = Loc_pid.output_limit;
    else if (turnspeedout<-Loc_pid.output_limit)turnspeedout = -Loc_pid.output_limit;
    return turnspeedout;
}
int loc_turn_pid(int dis_err){
    static int turnspeedout,dis_cu_err,dis_last_err,dis_err_sum;
    dis_last_err = dis_cu_err;//更新数据
    diff = dis_cu_err - dis_last_err;
    dis_cu_err = dis_err;//0---targetvalue
    dis_err_sum += dis_cu_err ;
    if(abs(dis_cu_err)>30){dis_err_sum = 0;}
    //积分限幅
    if(dis_err_sum>Loc_pid.integral_limit)dis_err_sum = Loc_pid.integral_limit;
    else if (dis_err_sum<-Loc_pid.integral_limit)dis_err_sum = -Loc_pid.integral_limit;
    //前馈补偿
    feedforward = Loc_pid.Kff * (dis_err - dis_last_err);

    turnspeedout = feedforward + dis_cu_err*Loc_pid.kp + (dis_cu_err-dis_last_err)*Loc_pid.kd + Loc_pid.ki*dis_err_sum;//前馈+PID
    //输出限幅
    if(turnspeedout>Loc_pid.output_limit)turnspeedout = Loc_pid.output_limit;
    else if (turnspeedout<-Loc_pid.output_limit)turnspeedout = -Loc_pid.output_limit;
    return turnspeedout;
}
void load_pwm(char wheel,int32_t pwm)
{
    switch (wheel){
        case 'L' : 
            if(pwm>0){
                wheel_dir('L',1);
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm);
            }else if (pwm<0) {
                wheel_dir('L', 2);
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,-pwm);
            }
        break;
        case 'R':
             if(pwm>0){
                wheel_dir('R',1);
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm);
            }else if (pwm<0) {
                wheel_dir('R', 2);
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,-pwm);
            }
        break;
        default:
            break;
    }
}
void pwm_limit(int32_t *load_pwmR,int32_t *load_pwmL)
{
    if(*load_pwmR >= PWM_MAX)*load_pwmR = PWM_MAX;
    if(*load_pwmR <= PWM_MIN)*load_pwmR = PWM_MIN;
    if(*load_pwmL >= PWM_MAX)*load_pwmL = PWM_MAX;
    if(*load_pwmL <= PWM_MIN)*load_pwmL = PWM_MIN;
}
//wheel_dir得实验车轮的极性及时修改
void wheel_dir(char wheel,uint16_t fangxiang)//wheeldirection   轮子： A-D 方向： 0 停止  方向向前 1 方向向后 2
{
    switch (wheel){
        case 'L' : 
            if(fangxiang == 0){
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            }
            else if(fangxiang == 1){
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            }
            else if(fangxiang == 2){
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            }
        break;
        case 'R':
            if(fangxiang == 0){
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            }
            else if(fangxiang == 1){
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            }
            else if(fangxiang == 2){
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            }
        break;
        default:
            break;
    }
}

void load_speed(char wheel,int16_t speed){
    speed_limit('L',speed);
    speed_limit('R',speed);
    switch (wheel){
        case 'L' : 
            VelocityL(EncoderL,speed);
        break;
        case 'R':
            VelocityR(EncoderR,speed);
        break;
        default:
            break;
    }
}
void speed_limit (char wheel,int16_t speed) //-138<speed<138
{
    //速度限幅
    if (speed>=138){speed = 138;}
    if (speed<=-138){speed = -138;}
    
    switch (wheel){
        case 'L' : 
            pwmL = VelocityL(EncoderL,speed);
            load_pwm('L',pwmL);
        break;
        case 'R':
            pwmR = VelocityR(EncoderR,speed);
            load_pwm('R',pwmR);
        break;
        default:
            break;
    }
}
//两轮差速  +turn_right  -turn_left
//按键检测封装函数
void Key_detect(void){
        if(Key_flag == 0){//第一次判断
          if(HAL_GPIO_ReadPin(Key1_GPIO_Port,Key1_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 1 ;
          }
          if(HAL_GPIO_ReadPin(Key2_GPIO_Port,Key2_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 2 ;
          }
          if(HAL_GPIO_ReadPin(Key3_GPIO_Port,Key3_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 3 ;
          }
        }
        else if (Key_flag == 1) {//第二次判断 软件消抖
          if(HAL_GPIO_ReadPin(Key1_GPIO_Port,Key1_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 0 ;
            Key_status = 1 ;
          }
          else {
          Key_flag = 0;
        }
        }
        else if (Key_flag == 2) {
          if(HAL_GPIO_ReadPin(Key2_GPIO_Port,Key2_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 0 ;
            Key_status = 2 ;
          }
          else {
          Key_flag = 0;
        }
        }
        else if (Key_flag == 3) {
          if(HAL_GPIO_ReadPin(Key3_GPIO_Port,Key3_Pin) == GPIO_PIN_RESET)
          {
            Key_flag = 0 ;
            Key_status = 3 ;
          }
          else {
          Key_flag = 0;
        }
      }
}
void turn_angle(int aimgle){
    // 如果已经转向成功，则逐渐减速停止
    if(turn_success_flag) {
        static int stop_counter = 0;
        if(stop_counter < 10) {  // 100ms内逐渐减速
            int pwm = pwmL * (10 - stop_counter) / 10;
            load_pwm('L', pwm);
            load_pwm('R', -pwm);
            stop_counter++;
        } else {
            load_pwm('L', 0);
            load_pwm('R', 0);
        }
        return;
    }
    
    // 正常转向控制
    pwmL = angle_pid(aimgle);
    pwmR = -angle_pid(aimgle);
    
    // 添加转向速度限制，避免过冲
    #define TURN_PWM_LIMIT 1200
    if(pwmL > TURN_PWM_LIMIT) pwmL = TURN_PWM_LIMIT;
    if(pwmL < -TURN_PWM_LIMIT) pwmL = -TURN_PWM_LIMIT;
    if(pwmR > TURN_PWM_LIMIT) pwmR = TURN_PWM_LIMIT;
    if(pwmR < -TURN_PWM_LIMIT) pwmR = -TURN_PWM_LIMIT;
    
    load_pwm('L', pwmL);
    load_pwm('R', pwmR);
}
// 在motor.c中添加函数
void reset_turn_flag(void) {
    turn_success_flag = 0;
    turn_success_timer = 0;
}
// 在需要开始新转向的地方调用此函数
// 例如在开始转向前：
// reset_turn_flag();
// turn_angle(target_angle);

LED_State_t led_state;
volatile uint32_t blink_counter = 0;
extern int distance;
extern volatile int32_t total_pulse_L , total_pulse_R ;
volatile uint32_t timer_10ms_counter = 0;  // 10ms计数
volatile uint32_t seconds_counter = 0;     // 秒计数
volatile uint32_t timer_flag = 0;      // 15秒标志位
//定时器中断回调函数执行定时器中断任务
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {

        EncoderL = (int16_t)__HAL_TIM_GetCounter(&htim1);
        EncoderR = -(int16_t)__HAL_TIM_GetCounter(&htim2);
        
        // 累计脉冲数（用于位置环）
        total_pulse_L += EncoderL;
        total_pulse_R += EncoderR;
        
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        
        Key_detect();
        
        // 原有的LED控制等代码保持不变...
        switch(led_state) {
            case LED_OFF:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // 高电平熄灭
                break;
                
            case LED_ON:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); // 低电平点亮
                break;
                
            case LED_SLOW_BLINK:
                // 慢闪：500ms周期 (50个10ms中断)
                if(blink_counter % 70 == 0) {
                    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                }
                break;
                
            case LED_FAST_BLINK:
                // 快闪：200ms周期 (20个10ms中断)
                if(blink_counter % 20 == 0) {
                    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                }
                break;
        }
        blink_counter++;
        if(blink_counter >= 1000) blink_counter = 0; // 防止溢出

        // 判断转向是否完成
        static int stable_counter = 0;
        if(abs(angle_errcounter) < 3.0 && abs(angle_derivativecounter) < 0.5) {
            stable_counter++;
            if(stable_counter > 100) {  // 持续1秒稳定才认为转向成功
                turn_success_flag = 1;
                stable_counter = 100;  // 防止溢出
            }
        } else {
            stable_counter = 0;       // 不稳定，重置计数器
            turn_success_flag = 0;    // 清除转向成功标志
        }
        
         // ========== 15秒计时逻辑 ==========
        timer_10ms_counter++;  // 每10ms加1
        
        // 每100次（1秒）更新秒计数器
        if (timer_10ms_counter >= 100) {
            timer_10ms_counter = 0;
            seconds_counter++;
            
            // 检查是否达到15秒
            if (seconds_counter >= 15) {//看是多少秒
                seconds_counter = 0;  // 重置秒计数器
                timer_flag = 1;   // 设置X（15）秒标志位
            }
        }
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);//清除中断标志位
    }
}
