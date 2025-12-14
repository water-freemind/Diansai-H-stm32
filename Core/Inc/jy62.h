/*
 * jy62.h
 *
 *  Created on: Oct 16, 2025
 *      Author: 秦始皇
 */

#ifndef SRC_JY62_H_
#define SRC_JY62_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"
#include "stdbool.h"

/* 全局变量声明 */
// extern volatile float roll_angle;     // X轴角度
// extern volatile float pitch_angle;    // Y轴角度
// extern volatile float yaw_angle;      // Z轴角度
// extern volatile float temperature;    // 温度

// extern volatile uint8_t jy62_new_data;  // 新数据标志

// extern volatile uint8_t rx_buffer[11];   // 接收缓冲区
// extern volatile uint8_t data_index;        // 数据索引
// extern volatile uint8_t frame_started;     // 帧开始标志

/* 函数声明 */
void JY62_UART4_Init(void);          // UART4初始化
void JY62_ConfigMode(void);          // 配置模式（如果需要）
void JY62_ProcessData(uint8_t data); // 处理接收数据
#ifdef __cplusplus
}
#endif

#endif /* SRC_JY62_H_ */
