/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "motor.h"
#include "gimbal_driver.h"
#include "k230_uart.h"
#include "jy62.h"
#include "position_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Key_flag;
uint16_t Key_status;
int16_t EncoderL,EncoderR;
extern volatile float yaw_angle;
extern int speedL,speedR;
extern int pwmL,pwmR;
int16_t MV_data_1 = 0;
int16_t MV_data_2 = 0;
int16_t MV_data_3 = 0;
int16_t MV_data_4 = 0;
int16_t MV_data_5 = 0;
int16_t MV_data_6 = 0;
int16_t MV_data_7 = 0;
extern volatile uint32_t timer_flag;
extern volatile uint32_t timer_flag2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int base_pwm = 1100;
int line_angle = 0;
int mission_flag = 0;
extern volatile uint8_t turn_success_flag;
extern LED_State_t led_state;
int16_t turnflag = 0;
typedef enum {
    TASK_IDLE = 0,          // 空闲状态
    TASK_FIRST_MOVE = 1,    // 第一段移动中
    TASK_TURNING = 2,       // 转向中
    TASK_SECOND_MOVE = 3,   // 第二段移动中
    TASK_COMPLETED = 4,      // 全部任务完成
    TASK_THIRD_MOVE = 5,
    TASK_FORTH_MOVE = 6
} Task_Stage_t;
Task_Stage_t task_stage =TASK_IDLE;
int turning_stable_counter = 0;
int target_turn_angle = 0;
#define TURN_STABLE_TIME 5  // N*10ms ms稳定时间

int direction_flag = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  JY62_UART4_Init();//初始化JY62
  OLED_Init();//OLED初始化+清屏幕
  OLED_Clear();
  position_control_init();
  
  HAL_Delay(500);//等待陀螺仪初始化成功
  reset_turn_flag();
  K230_UART_Init(); // 初始化 K230 UART DMA 接收
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    K230_UART_Poll(); // 处理K230数据
    Gimbal_RunSpeed(GIMBAL_LOWER_ADDR, DIR_CW, 40); // 云台低速旋转，防止进入休眠
    if (k230_data.is_updated) {
        // 显示有符号数（适配负数）
        OLED_ShowSignedNum(0, 0, k230_data.pan_error, 6, OLED_8X16);
        OLED_ShowSignedNum(0, 18, k230_data.tilt_error, 6, OLED_8X16);
    }

    // 原有yaw_angle显示保留
    OLED_ShowString(0, 36, "yaw_angle:", OLED_8X16);
    OLED_ShowFloatNum(80, 36, yaw_angle, 3, 2, OLED_8X16);
    OLED_Update();//显示任何东西都需要更新OLED数据，不然无法显示

  //   if(Key_status == 1 && task_stage == TASK_IDLE){
  //       timer_flag = 0;
  //       set_target_distance(1480);  // 第一段移动
  //       task_stage = TASK_FIRST_MOVE;  // 进入第一段移动状态
  //       Key_status = 0;
  //   }
  //   if(timer_flag == 1 && MV_data_1!=0){
  //     if(direction_flag == 0){
  //       direction_flag = MV_data_1;
  //     }
  //     switch(task_stage) {
  //     case TASK_FIRST_MOVE:
  //         led_state = LED_FAST_BLINK;
  //         if(position_state == POSITION_COMPLETED){
  //             // 第一段移动完成，准备转向
  //             task_stage = TASK_TURNING;
  //             reset_turn_flag();  // 重置转向标志
  //             turning_stable_counter = 0;
  //             // 停止位置环控制
  //             position_control_enable = 0;
  //             // 设置转向目标角度
  //             if(direction_flag == 1){target_turn_angle = -60;}//左
  //             if(direction_flag == 2){target_turn_angle = -90;}//中
  //             if(direction_flag == 3){target_turn_angle = -120;}//右
  //             turnflag = 2; 
  //         }
  //         break;
          
  //     case TASK_TURNING:
  //         // 执行转向控制
  //         led_state = LED_ON;
  //         turn_angle(target_turn_angle);
  //         // 检查转向是否成功
  //         if(turn_success_flag == 1) {
  //             // 转向成功，开始计时稳定
  //             turning_stable_counter++;
              
  //             // 稳定期间保持当前角度
  //             if(turning_stable_counter >= TURN_STABLE_TIME) {
  //                 // 稳定时间足够，进入下一阶段
  //                 led_state = LED_SLOW_BLINK;
  //                 switch (turnflag)
  //                 {
  //                     case 2:
  //                       task_stage = TASK_SECOND_MOVE;
  //                       if(direction_flag == 1){
  //                         line_angle = -60;
  //                         set_target_distance(1700);
  //                       }//左
  //                       if(direction_flag == 2){
  //                         line_angle = -90;
  //                         set_target_distance(1420);
  //                       }//中
  //                       if(direction_flag == 3){
  //                         line_angle = -120;
  //                         set_target_distance(1650);
  //                       }//右
  //                       break;
  //                     case 3:
  //                       HAL_Delay(5000);
  //                       task_stage = TASK_THIRD_MOVE;
  //                       if(direction_flag == 1){
  //                         line_angle = 120;
  //                         set_target_distance(1600);
  //                       }//左
  //                       if(direction_flag == 2){
  //                         line_angle = 90;
  //                         set_target_distance(1420);
  //                       }//中
  //                       if(direction_flag == 3){
  //                         line_angle = 60;
  //                         set_target_distance(1650);
  //                       }//右
  //                       break;
  //                     case 4:
  //                       task_stage = TASK_FORTH_MOVE;
  //                       line_angle = 0;
  //                       set_target_distance(-1600);
  //                       break;
  //                     default:
  //                       break;
  //                 }
  //             }
  //         } else {
  //             // 转向未成功，重置稳定计时器
  //             turning_stable_counter = 0;
  //         }
  //         break;
        
  //     case TASK_SECOND_MOVE:
  //         led_state = LED_ON;
  //         // 第二段移动逻辑保持不变
  //         if(position_state == POSITION_COMPLETED){
  //             task_stage = TASK_TURNING;
  //             reset_turn_flag();  // 重置转向标志
  //             turning_stable_counter = 0;
  //             // 停止位置环控制
  //             position_control_enable = 0;
  //             // 设置转向目标角度
  //             if(direction_flag == 1){target_turn_angle = 120;}//左
  //             if(direction_flag == 2){target_turn_angle = 90;}//中
  //             if(direction_flag == 3){target_turn_angle = 60;}//右
  //             turnflag = 3;
  //         }
  //         break;

  //     case TASK_THIRD_MOVE:
  //         if(position_state == POSITION_COMPLETED){
  //             task_stage = TASK_TURNING;
  //             reset_turn_flag();  // 重置转向标志
  //             turning_stable_counter = 0;
  //             // 停止位置环控制
  //             position_control_enable = 0;
  //             target_turn_angle = 0; 
  //             turnflag = 4;
  //         }
  //         break;
  //       case TASK_FORTH_MOVE:
  //         if(position_state == POSITION_COMPLETED){
  //             task_stage = TASK_COMPLETED;
  //             reset_turn_flag();  // 重置转向标志
  //             turning_stable_counter = 0;
  //             // 停止位置环控制
  //             position_control_enable = 0;
  //         }
  //         break;
  //     case TASK_COMPLETED:
  //         // 任务完成
  //         break;
  //     }
  //   }
    
  //   // 位置环控制更新（在主循环中调用）
  //   position_control_update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
