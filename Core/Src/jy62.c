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

void MY_uart2_Receive(uint8_t data)
{
    static uint8_t data_cnt = 0;
    static uint8_t data_buf[16];  // 存储16个数据字节（帧头2+14数据字节=7个数据×2字节）
    
    // 状态机处理数据包
    switch(data_cnt) {
        case 0: 
            if(data == 0xFF) {
                data_buf[0] = data;  // 存储帧头1
                data_cnt = 1;
            }
            break;
        case 1:
            if(data == 0x07) {
                data_buf[1] = data;  // 存储帧头2
                data_cnt = 2;
            } else {
                data_cnt = 0; // 帧头2不匹配，重置
            }
            break;
        case 2:  // Data_1低字节
        case 3:  // Data_1高字节
        case 4:  // Data_2低字节
        case 5:  // Data_2高字节
        case 6:  // Data_3低字节
        case 7:  // Data_3高字节
        case 8:  // Data_4低字节
        case 9:  // Data_4高字节
        case 10: // Data_5低字节
        case 11: // Data_5高字节
        case 12: // Data_6低字节
        case 13: // Data_6高字节
        case 14: // Data_7低字节
        case 15: // Data_7高字节
            data_buf[data_cnt] = data;
            data_cnt++;
            break;
        case 16:  // 帧尾1
            if(data == 0x0D) {
                data_cnt = 17;
            } else {
                data_cnt = 0; // 帧尾错误，重置
            }
            break;
        case 17:  // 帧尾2
            if(data == 0x0A) {
                // 帧尾验证通过，重组数据
                MV_data_1 = (int16_t)((data_buf[3] << 8) | data_buf[2]);   // Data_1
                MV_data_2 = (int16_t)((data_buf[5] << 8) | data_buf[4]);   // Data_2
                MV_data_3 = (int16_t)((data_buf[7] << 8) | data_buf[6]);   // Data_3
                MV_data_4 = (int16_t)((data_buf[9] << 8) | data_buf[8]);   // Data_4
                MV_data_5 = (int16_t)((data_buf[11] << 8) | data_buf[10]); // Data_5
                MV_data_6 = (int16_t)((data_buf[13] << 8) | data_buf[12]); // Data_6
                MV_data_7 = (int16_t)((data_buf[15] << 8) | data_buf[14]); // Data_7
                MV_data_ready = 1;
            }
            data_cnt = 0; // 无论是否匹配都重置状态机
            break;
        default:
            data_cnt = 0; // 错误状态复位
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
        // 调用解析函数，传入接收到的字节
        MY_uart2_Receive(rx_data);
        // 处理接收到的字节数据
        JY62_ProcessData(rx_data);
        // 重新启动接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
    if(huart->Instance == USART2)
    {
        static uint8_t rx_data2;
        // 获取接收到的数据
        rx_data2 = (uint8_t)(huart->Instance->DR & 0xFF); // 直接从数据寄存器读取
        // 调用解析函数，传入接收到的字节
        MY_uart2_Receive(rx_data2);
        // 处理接收到的字节数据
        if (MV_data_ready == 1) {
            //分配数据
            
            //清除标志位
            MV_data_ready = 0;
        }
        // 重新启动接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart2, &rx_data2, 1);
    }
}
