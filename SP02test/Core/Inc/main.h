/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Butterworth.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define WITH_RATE	10
#define WITH_AVG	20
#define READ	18
#define IRD		12
#define SAMPLE	200
#define USART_DEBUG		huart1
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define READ_PIN_Pin GPIO_PIN_11
#define READ_PIN_GPIO_Port GPIOA
#define IRD_PIN_Pin GPIO_PIN_12
#define IRD_PIN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

typedef struct {
	float x_last;
	float p_last;
	float Q;
	float R;
	float kg;
	float x_mid;
	float x_now;
	float p_mid;
	float p_now;
}_KERMAN;


extern unsigned char READ_N;
extern unsigned char IRD_N;
extern unsigned char stus;//状态标志
extern  int radtime;//规定计数时间
extern  int irdtime;//规定计数时间

void GET_SPO2(void);
int GET_RATE(void);
int Count_Rate(void);
void Kerman_init(void);
float kaermanR(float z_measure);
float kaermanI(float z_measure);
void	MY_LOG(UART_HandleTypeDef USARTx, char *fmt,...);
void 	Set_Led(unsigned char state);
int 	get_radavg(int rat);
int 	get_iravg(int rat);
void get_rate(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
