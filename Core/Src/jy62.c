#include "jy62.h"
#include "usart.h"
#include "string.h"
#include <stdint.h>

/* 全局变量定义 */
volatile float roll_angle = 0.0f;     // X轴角度
volatile float pitch_angle = 0.0f;    // Y轴角度
volatile float yaw_angle = 0.0f;      // Z轴角度
volatile float temperature = 0.0f;    // 温度

volatile uint8_t jy62_new_data = 0;   // 新数据标志

volatile uint8_t rx_buffer[11] = {0};   // 接收缓冲区
volatile uint8_t data_index = 0;        // 数据索引
volatile uint8_t frame_started = 0;     // 帧开始标志
volatile uint8_t data=0;

volatile uint8_t MV_data_ready = 0;
extern int16_t MV_data_1;
extern int16_t MV_data_2;
extern int16_t MV_data_3;
extern int16_t MV_data_4;
extern int16_t MV_data_5;
extern int16_t MV_data_6;
extern int16_t MV_data_7;
int16_t pixel_diff = 0;//像素偏差值
int16_t round_flag = 0;//环岛标志位
int16_t startline = 0;//起始点标志位

/**
  * @brief  UART4初始化函数
  * @note   配置UART4用于与JY62通信，波特率115200
  * @param  None
  * @retval None
  */
void JY62_UART4_Init(void)
{
    // 此函数由CubeMX在main.c中生成
    // 配置参数：波特率115200，8数据位，1停止位，无校验

    // 启动串口接收中断  JY62和K230 uart2-uart2
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer, 1);
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer, 1);
    JY62_ConfigMode();

}

/**
  * @brief  JY62配置模式
  * @note   发送配置指令到JY62（如果需要）
  * @param  None
  * @retval None
  */
void JY62_ConfigMode(void)
{
	// Z轴角度归零指令: 0xFF, 0xAA, 0x52
	    uint8_t cmd[] = {0xFF, 0xAA, 0x52};
	    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), 1000);

	    // 可选：延迟一段时间确保指令被处理
	    HAL_Delay(100);

}

/**
  * @brief  处理接收到的数据
  * @note   使用状态机解析JY62数据包，只处理0x55 0x53开头的角度包
  * @param  data: 接收到的字节数据
  * @retval None
  */
void JY62_ProcessData(uint8_t data)
{
    // 检测帧头0x55 0x53
    if (!frame_started)
    {
        if (data_index == 0 && data == 0x55)
        {
            rx_buffer[data_index++] = data;
        }
        else if (data_index == 1 && data == 0x53)
        {
            rx_buffer[data_index++] = data;
            frame_started = 1;
        }
        else
        {
            data_index = 0;
        }
    }
    else
    {
        // 接收后续数据
        rx_buffer[data_index++] = data;

        // 完整帧长度：11字节
        if (data_index >= 11)
        {
            // 计算校验和
            char checksum = 0;
            for(int i = 0; i < 10; i++)
            {
                checksum += rx_buffer[i];
            }
            char checksum1 = 0;
            checksum1=checksum&0xFF;
            // 验证校验和
            if(checksum1!= rx_buffer[10])
            {
                // 校验失败，重置状态
                data_index = 0;
                frame_started = 0;
                return;
            }

            // 提取原始数据（小端格式）
            short roll_raw = ((short)(rx_buffer[3] << 8) | rx_buffer[2]);
            short pitch_raw = ((short)(rx_buffer[5] << 8) | rx_buffer[4]);
            short yaw_raw = ((short)(rx_buffer[7] << 8) | rx_buffer[6]);
            short temp_raw = ((short)(rx_buffer[9] << 8) | rx_buffer[8]);

            // 计算角度值（根据JY62公式）
            roll_angle = ((float)roll_raw / 32768.0f) * 180.0f;      // X轴角度
            pitch_angle = ((float)pitch_raw / 32768.0f) * 180.0f;    // Y轴角度
            yaw_angle = ((float)yaw_raw / 32768.0f) * 180.0f;        // Z轴角度

            // 计算温度值（根据JY62公式）
            temperature = ((float)temp_raw) / 340.0f + 36.53f;

            // 设置新数据标志
            jy62_new_data = 1;

            // 重置状态
            data_index = 0;
            frame_started = 0;
        }
    }
}


/**
  * @brief  串口接收完成中断回调函数
  * @note   当UART4接收到数据时自动调用
  * @param  huart: 串口句柄指针
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 检查是否是UART1的中断（JY62传感器）
    if(huart->Instance == USART1)
    {
        static uint8_t rx_data;
        // 获取接收到的数据
        rx_data = (uint8_t)(huart->Instance->DR & 0xFF); // 直接从数据寄存器读取
        // 处理接收到的字节数据
        JY62_ProcessData(rx_data);
        // 重新启动接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
