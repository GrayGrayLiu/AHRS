//
// Created by Gray on 2025/1/7.
//

#include "Uart.h"
#include "usart.h"

//uint8_t Usart1Buffer;   //串口1接收22字节缓存
//uint8_t Usart3Buffer;       //串口3接收1字节缓存
//uint8_t Usart6Buffer;       //串口6接收1字节缓存

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    static uint8_t buffer[6];
//    static uint8_t cnt = 0;
//    float roll,ax;
//
//    if(huart->Instance == huart1.Instance){
//        if(Usart1Buffer == 170){
//            cnt = 0;
//            buffer[cnt++] = Usart1Buffer;
//        }else if(cnt > 0 && cnt < 6){
//            buffer[cnt++] = Usart1Buffer;
//        }
//        if(cnt == 6 && buffer[5] == 85){
//            ax = (float) ((int16_t) ((buffer[2] << 8u) | buffer[1]));
//            roll = (float) ((int16_t) ((buffer[4] << 8u) | buffer[3]));
//            jy901_seesaw.euler.roll = roll / 32768 * 180.0;
//            jy901_seesaw.av.ax = ax / 32768 * 2000.0;
//        }
//
//        HAL_UART_Receive_IT(&huart1,&Usart1Buffer,1);
//    }
//    if(huart->Instance == huart3.Instance)
//    {
//        MoveCommandReceive(Usart3Buffer);
//        HAL_UART_Receive_IT(&huart3,&Usart3Buffer,1);
//    }
//}
