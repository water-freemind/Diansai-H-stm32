#ifndef __K230_UART_H
#define __K230_UART_H

#include "main.h"

// 视觉数据结构体
typedef struct {
    int16_t pan_error;    // 水平误差
    int16_t tilt_error;   // 垂直误差
    uint8_t is_updated;   // 更新标志
} K230_Data_t;

extern K230_Data_t k230_data;

// --- 新增函数声明 ---
void K230_UART_Init(void);  // 初始化开启DMA
void K230_UART_Poll(void);  // 在主循环调用此函数处理数据

#endif
