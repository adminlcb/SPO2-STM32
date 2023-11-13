/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdio.h>


 

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
_KERMAN  KERMANR;
_KERMAN  KERMANI;
unsigned char READ_N = 0,IRD_N = 0;
unsigned char stus= 0;//状态锟斤拷志

 int radtime=0;
 int irdtime=0;//锟芥定锟斤拷锟斤拷时锟斤拷

unsigned char u1tx=0;
//BANK锟斤拷锟斤拷时锟斤拷
int Bradtime=0;
int Birdtime=0;
int Rindex=0;
int radata[WITH_AVG]={0}; 
int ir_buffer[220]={0}; 
int ir_buffer_bf[220]={0}; 
int Rmax=0;//记录窗口内最大值
int Rmin=100;//记录窗口内最小值

	int RadDC=0;
  int IrDC=0;
	int charge_r=0,charge_i=0;
	int rate=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MY_LOG(UART_HandleTypeDef USARTx, char *fmt,...)
{
 
	unsigned char UsartPrintfBuf[500];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap);							//锟斤拷式锟斤拷
	va_end(ap);

	while(*pStr != NULL)
	{
        HAL_UART_Transmit (&USARTx ,(uint8_t *)pStr++,1,HAL_MAX_DELAY );		
	}
 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
int charge_rmax=0,charge_rmin=100;
int charge_imax=0,charge_imin=100;
	float R=0,PI=0;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Kerman_init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  fof_Init(&fof_1,5, 10, 0);	//低通滤波器，截止频率为10Hz 5ms 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(u1tx)
		{
			u1tx=0;
			RadDC = get_radavg(radtime);
      IrDC  = get_iravg(irdtime);
			charge_r = kaermanR(radtime-Bradtime)*15;
			charge_i = kaermanI(irdtime-Birdtime)*15;
			
			MY_LOG(USART_DEBUG, "{r:%d,%d}\r\n",charge_r,charge_i);
			//MY_LOG(USART_DEBUG, "{d:%d,%d}\r\n",RadDC+charge_r,IrDC+charge_i);
			Bradtime = radtime;
			Birdtime = irdtime;
			if(radtime>100){
			if(Rindex<SAMPLE){
				if(charge_r>charge_rmax)
					charge_rmax = charge_r;
				else if(charge_r<charge_rmin)
					charge_rmin = charge_r;
				if(charge_i>charge_imax)
					charge_imax = charge_i;
				else if(charge_i<charge_imin)
					charge_imin = charge_i;
				//GET_SPO2();
				ir_buffer[Rindex]=IrDC+charge_i;
				if(Rindex>(WITH_RATE-1))//大于滑动尺度开始计算
					get_rate();
				Rindex++;	
			}
			else{
				Rindex=0;
				rate = GET_RATE();
				R = ((float)charge_rmax-charge_rmin)/((float)charge_imax-charge_imin)*((float)IrDC/RadDC);
				PI = ((float)charge_rmax-charge_rmin) / (float)charge_rmax;
				MY_LOG(USART_DEBUG, "{d:%d,%f,%f}\r\n",7229/rate,R,PI);
				charge_rmax=0,charge_rmin=100;
				charge_imax=0,charge_imin=100;
			}
		 } 
		}
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
		
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 82;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, READ_PIN_Pin|IRD_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : READ_PIN_Pin IRD_PIN_Pin */
  GPIO_InitStruct.Pin = READ_PIN_Pin|IRD_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */

void GET_SPO2(void)
{
	float R=0;
	//R = ((float)charge_r/(float)charge_i)*((float)IrDC/(float)RadDC);
	//MY_LOG(USART_DEBUG, "{r:%d,%d,%d,%d}\r\n",charge_r,charge_i,IrDC,RadDC);
	//MY_LOG(USART_DEBUG, "{r:%f}\r\n",(float)charge_r/(float)charge_i);
}

int GET_RATE(void)
{
	int index_max[10]={0},j=0,rrate=0;
	int i=0,Rmedia=((Rmax+Rmin)*12)/20;
	Rmax=0;
	Rmin=100;
	for(i=1;i<(SAMPLE-(WITH_RATE-1)-1);i++)
	{
		if(ir_buffer_bf[i]>Rmedia){
			if(ir_buffer_bf[i]>=ir_buffer_bf[i+1] && ir_buffer_bf[i]>=ir_buffer_bf[i-1]){
				index_max[j]=i;
				i+=20;
				j++;
			}
			if(j>10)
				break;
		}
	}
	if(j>1){
		for(i=1;i<j;i++)
			rrate+=(index_max[i]-index_max[i-1]);
		return rrate/(j-1);
	}
	return 0;
}

int Count_Rate(void)
{
	int i=0,index_max[3]={0},temp;
	for(i=0;i<(SAMPLE-(WITH_RATE-1));i++)
	{
		if(ir_buffer_bf[i]>index_max[0])
			index_max[0]=i;
	}
	for(i=0;i<(SAMPLE-(WITH_RATE-1));i++)
	{
		if(i<(index_max[0]-15) || i>(index_max[0]+15))
		{
			if(ir_buffer_bf[i]>index_max[1])
				index_max[1]=i;
		}
	}
	if(index_max[0]>index_max[1])
	{
		temp = index_max[0];
		index_max[0] = index_max[1] = temp;;
		index_max[1] = temp;
	}
	for(i=0;i<(SAMPLE-(WITH_RATE-1));i++)
	{
		if((i<(index_max[0]-15)) || (i>(index_max[0]+15)&&i<(index_max[1]-15)) || (i>(index_max[1]+15)))
		{
			if(ir_buffer_bf[i]>index_max[2])
				index_max[2]=i;
		}
	}
//	MY_LOG(USART_DEBUG, "{r:%d,%d,%d}\r\n",index_max[0],index_max[1],index_max[2]);
	if(index_max[2]<index_max[0])
		return 7288/(index_max[1]-index_max[0]);
	else if(index_max[2]>index_max[0] && index_max[2]<index_max[1])
		return 7288/(index_max[1]-index_max[2]);
	else if(index_max[2]>index_max[1])
		return 7288/(index_max[2]-index_max[1]);
//	if(index_max[2]<index_max[0])
//	{
//		temp = index_max[0];
//		index_max[0] = index_max[1] = temp;;
//		index_max[1] = temp;
//	}
//	else if()
	return 0;
}

void get_rate(void)
{
	int i=Rindex,j=0,sum=0;
		for(j=(i-(WITH_RATE-1));j<=i;j++){
			sum = sum+ir_buffer[j];
		}
		ir_buffer_bf[i-(WITH_RATE-1)] = (sum/WITH_RATE);
		if((sum/WITH_RATE)>Rmax)
			Rmax = sum/WITH_RATE;
		else if((sum/WITH_RATE)<Rmin)
			Rmin = sum/WITH_RATE;
	//MY_LOG(USART_DEBUG, "{r:%d}\r\n",ir_buffer_bf[i-(WITH_RATE-1)]);

}

void Kerman_init(void)
{
	//速度预测 小预测
	KERMANR.x_last=0;
	KERMANR.p_last=0;
	KERMANR.Q=0.0001;
	KERMANR.R=0.01;
	//速度预测 大预测
	KERMANI.Q = 0.0001;
	KERMANI.R = 0.001;
	KERMANI.x_last=0;
	KERMANI.p_last=0;
}

//卡尔曼滤波函数
float kaermanR(float z_measure)
{ 
	KERMANR.x_mid=KERMANR.x_last;
	KERMANR.x_mid=KERMANR.x_last;                                
	KERMANR.p_mid=KERMANR.p_last+KERMANR.Q;                                
	KERMANR.kg=KERMANR.p_mid/(KERMANR.p_mid+KERMANR.R);                               
	KERMANR.x_now=KERMANR.x_mid+KERMANR.kg*(z_measure-KERMANR.x_mid);                 
	KERMANR.p_now=(1-KERMANR.kg)*KERMANR.p_mid;                               
	KERMANR.p_last = KERMANR.p_now;                                     
	KERMANR.x_last = KERMANR.x_now;                                    
	return KERMANR.x_now;
}
//卡尔曼滤波函数
float kaermanI(float z_measure)
{ 
	KERMANI.x_mid=KERMANI.x_last;
	KERMANI.x_mid=KERMANI.x_last;                                
	KERMANI.p_mid=KERMANI.p_last+KERMANI.Q;                                
	KERMANI.kg=KERMANI.p_mid/(KERMANI.p_mid+KERMANI.R);                               
	KERMANI.x_now=KERMANI.x_mid+KERMANI.kg*(z_measure-KERMANI.x_mid);                 
	KERMANI.p_now=(1-KERMANI.kg)*KERMANI.p_mid;                               
	KERMANI.p_last = KERMANI.p_now;                                     
	KERMANI.x_last = KERMANI.x_now;                                    
	return KERMANI.x_now;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_1)
  {
		if(!stus)
		{	
			//锟斤拷始锟斤拷时
			if(!READ_N)
				TIM2->CNT=0;
			
			READ_N++;
			if(READ_N>READ)
			{
				radtime = TIM2->CNT;//锟斤拷1000锟斤拷us
				TIM2->CNT=0;
//				MY_LOG(USART_DEBUG, "radtime:%d\r\n",radtime);
				READ_N=0;
				stus=1;
				Set_Led(stus);
			}
		}
		else
		{
			//锟斤拷始锟斤拷时
			if(!IRD_N)
				TIM2->CNT=0;
			IRD_N++;
			if(IRD_N>IRD)
			{
				irdtime = TIM2->CNT;//锟斤拷1000锟斤拷us
//				MY_LOG(USART_DEBUG, "irdtime:%d\r\n",irdtime);
				IRD_N=0;
				stus=2;
				Set_Led(stus);
			}
		}

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	}
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//    static unsigned char ledState = 0;
    if (htim == (&htim2))
    {
    }
		//锟斤拷时120Hz
		if (htim == (&htim3))
    {
			READ_N=0;
			IRD_N=0;
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			TIM2->CNT=0;
			stus=0;
			Set_Led(stus);
			
//			if(ledState)
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
//			else
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
//			ledState^=1;
		}
}

void Set_Led(unsigned char state)
{
	switch(state)
	{
		case 0:HAL_GPIO_WritePin(GPIOA,READ_PIN_Pin,GPIO_PIN_RESET);
					 HAL_GPIO_WritePin(GPIOA,IRD_PIN_Pin,GPIO_PIN_SET);
			break;
		case 1:HAL_GPIO_WritePin(GPIOA,READ_PIN_Pin,GPIO_PIN_SET);
					 HAL_GPIO_WritePin(GPIOA,IRD_PIN_Pin,GPIO_PIN_RESET);
			break;
		case 2:HAL_GPIO_WritePin(GPIOA,READ_PIN_Pin,GPIO_PIN_SET);
					 HAL_GPIO_WritePin(GPIOA,IRD_PIN_Pin,GPIO_PIN_SET);
					 HAL_NVIC_DisableIRQ(EXTI1_IRQn);//失锟斤拷锟解部锟斤拷锟斤拷
					 u1tx = 1;
			break;
	
	}
}

/*
*压栈式锟斤拷取十锟斤拷锟斤拷值
*/
int get_radavg(int rat)
{
	unsigned char i=0;
	int sum=rat;
	for(i=0;i<(WITH_AVG-1);i++)
	{
		radata[i]=radata[i+1];
		sum+=radata[i];
	}
	radata[i+1]=rat;
	return sum/WITH_AVG;
}

/*
*压栈式锟斤拷取十锟斤拷锟斤拷值
*/
int get_iravg(int rat)
{
	unsigned char i=0;
	int sum=rat;
	for(i=0;i<(WITH_AVG-1);i++)
	{
		radata[i]=radata[i+1];
		sum+=radata[i];
	}
	radata[i+1]=rat;
	return sum/WITH_AVG;
}
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

#ifdef  USE_FULL_ASSERT
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
