#include "upper.h"
#include "main.h"
#include "usart.h"
void UP_SendByte(unsigned char byte){
   // while(HAL_UART_GetState(&huart2)==HAL_UART_STATE_BUSY_TX);
    HAL_UART_Transmit(&huart2, &byte,1,HAL_MAX_DELAY);
}
void UP_SendData(int16_t a,int16_t b,float c){
    uint8_t i=0;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t buf[50]={0};
    union {
        float f;
        uint8_t bytes[4];
    } float_conv;

    buf[0] = 0xAA;    // 帧头
    buf[1] = 0xFF;    // 目标地址
    buf[2] = 0xF1;    // 功能码
    buf[3] = 0x08;    // 数据长度（8字节）

    // 存储 int16_t a（小端模式，低字节在前）
    buf[4] = (uint8_t)a;
    buf[5] = (uint8_t)(a >> 8);
    // 存储 int16_t b
    buf[6] = (uint8_t)b;
    buf[7] = (uint8_t)(b >> 8);
    // 存储 float c（小端模式，4字节）
    float_conv.f = c;
    buf[8]  = float_conv.bytes[0];
    buf[9]  = float_conv.bytes[1];
    buf[10] = float_conv.bytes[2];
    buf[11] = float_conv.bytes[3];

    // 计算和校验、附加校验
    for(i=0;i<(buf[3]+4);i++){ // 覆盖 buf[0]~buf[11] 共12字节
        sumcheck += buf[i];
        addcheck += sumcheck;
    }
    buf[12] = sumcheck; // 存储和校验
    buf[13] = addcheck; // 存储附加校验

    // 发送完整帧（共14字节）
    HAL_UART_Transmit(&huart3, buf, 14, HAL_MAX_DELAY);
}
