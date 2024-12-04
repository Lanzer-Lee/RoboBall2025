/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define UART_BUFFER_SIZE   100
#define UART_STATE_IDLE     0
#define UART_STATE_BUSY     255
typedef struct
{
    uint8_t buffer[UART_BUFFER_SIZE];
    uint8_t byte;
    uint8_t state;
    uint8_t pointer;
}UART_BufferTypeDef;

extern UART_BufferTypeDef uart2_buffer;
extern UART_BufferTypeDef uart3_buffer;
extern UART_BufferTypeDef uart6_buffer;
extern UART_BufferTypeDef uart7_buffer;
extern UART_BufferTypeDef uart8_buffer;
/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART_InterruptionInit(void);
void UART_TransmitString(UART_HandleTypeDef* uartHandle, uint8_t* str);
void UART_Service(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

