#include "k230_uart.h"
#include "usart.h"  // 必须包含此文件以使用 huart2

// --- DMA 配置参数 ---
#define RX_BUF_SIZE 128       // 缓冲区大小，建议 64 或 128
static uint8_t rx_buffer[RX_BUF_SIZE]; // DMA 目标缓冲区
static uint16_t old_pos = 0;  // 记录 CPU 上次读取的位置

// 实例化全局变量
K230_Data_t k230_data = {0};

// 定义解析状态
typedef enum {
    K230_WAIT_HEADER_1 = 0,
    K230_WAIT_HEADER_2,
    K230_READ_PAN_H,
    K230_READ_PAN_L,
    K230_READ_TILT_H,
    K230_READ_TILT_L,
    K230_CHECK_SUM
} K230_RxState_e;

static K230_RxState_e rx_state = K230_WAIT_HEADER_1;
static uint8_t data_buf[4]; 

// 内部使用的字节解析函数 (保持你原来的逻辑，设为 static 仅内部调用)
static void K230_UART_Process_Byte(uint8_t byte) {
    switch (rx_state) {
        case K230_WAIT_HEADER_1:
            if (byte == 0x5A) rx_state = K230_WAIT_HEADER_2;// 找到第一个头字节
            else rx_state = K230_WAIT_HEADER_1;
            break;

        case K230_WAIT_HEADER_2:
            if (byte == 0xA5) rx_state = K230_READ_PAN_H;
            else if (byte == 0x5A) rx_state = K230_WAIT_HEADER_2;
            else rx_state = K230_WAIT_HEADER_1;
            break;

        case K230_READ_PAN_H:
            data_buf[0] = byte;
            rx_state = K230_READ_PAN_L;
            break;

        case K230_READ_PAN_L:
            data_buf[1] = byte;
            rx_state = K230_READ_TILT_H;
            break;

        case K230_READ_TILT_H:
            data_buf[2] = byte;
            rx_state = K230_READ_TILT_L;
            break;

        case K230_READ_TILT_L:
            data_buf[3] = byte;
            rx_state = K230_CHECK_SUM;
            break;

        case K230_CHECK_SUM: { // 加括号以支持局部变量声明
            uint8_t calc_sum = (data_buf[0] + data_buf[1] + data_buf[2] + data_buf[3]) & 0xFF;
            
            if (calc_sum == byte) {
                int16_t p_err = (int16_t)((data_buf[0] << 8) | data_buf[1]);
                int16_t t_err = (int16_t)((data_buf[2] << 8) | data_buf[3]);
                
                k230_data.pan_error = p_err;
                k230_data.tilt_error = t_err;
                k230_data.is_updated = 1;
            }
            rx_state = K230_WAIT_HEADER_1;
            break;
        }

        default:
            rx_state = K230_WAIT_HEADER_1;
            break;
    }
}

// --- 新增：初始化函数 ---
void K230_UART_Init(void) {
    // 开启 DMA 循环接收
    HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
}


void K230_UART_Poll(void) {

    // 计算当前DMA接收位置（取模确保在0~RX_BUF_SIZE-1范围内）
    uint16_t current_pos = (RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx)) % RX_BUF_SIZE;

    // 处理环形缓冲区中的新字节（避免越界/无限循环）
    while (old_pos != current_pos) {
        K230_UART_Process_Byte(rx_buffer[old_pos]);
        old_pos = (old_pos + 1) % RX_BUF_SIZE; // 环形递增
    }
}
