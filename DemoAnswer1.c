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
#include "KvSkillsDriver.h"
#include "stdio.h"
#include "string.h"
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t t0,t1,t2,t3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LEDR1(uint8_t x)
{	COM0(1);
	COM1(1);
	LED1(x & (1<<3));
	LED4(x & (1<<2));
	LED7(x & (1<<1));
	LED10(x & (1<<0));
}
void LEDR2(uint8_t x)
{	COM0(1);
	COM1(1);
	LED2(x & (1<<3));
	LED5(x & (1<<2));
	LED8(x & (1<<1));
	LED11(x & (1<<0));
}
void LEDR3(uint8_t x)
{	COM0(1);
	COM1(1);
	LED3(x & (1<<3));
	LED6(x & (1<<2));
	LED9(x & (1<<1));
	LED12(x & (1<<0));
}
void buzz (uint8_t state)
{	if(state==1)
		htim2.Instance->CCR1 = 25;
	else
		htim2.Instance->CCR1 = 0;
}
void bunyi(uint8_t bil,uint8_t masa)
{	for(uint8_t i=0;i<bil;i++)
	{	buzz(1);	HAL_Delay(masa);
		buzz(0);	HAL_Delay(masa);
	}
}
uint8_t getPB(void)
{
	uint8_t x=0;
	if(PB1==0) x|=1<<0;	//1  ==001
	if(PB2==0) x|=1<<1;	//2  ==010
	if(PB3==0) x|=1<<2;	//4  ==100
	return x;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  bunyi(2,50);
  t0=t1=t2=t3=HAL_GetTick();
  LEDR1(0b0000);
  LEDR2(0b0000);
  LEDR3(0b0000);

  while(1)
  {
	  if(getPB()==1)
	  {
		  for(uint8_t i=0;i<3;i++)
		  {
			  LEDR1(0b1010);
			  LEDR2(0b1100);
			  LEDR3(0b1010);  HAL_Delay(1000);

			  LEDR1(0b0001);
			  LEDR2(0b1010);
			  LEDR3(0b0100);  HAL_Delay(1000);
		  }
		  break;
	  }
  }


  LEDR1(0b0000);
  LEDR2(0b0000);
  LEDR3(0b0000);

  uint8_t status=0,pb,flag=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(t0 < HAL_GetTick())
	  {
		  t0 += 100;
		  status =! status;

		  if(flag==0)
		  {
			  if(status==0)
			  {
				  LEDR1(0b1001);
				  LEDR2(0b0000);
				  LEDR3(0b1001);
			  }
			  if(status==1)
			  {
				  LEDR1(0b0000);
				  LEDR2(0b0000);
				  LEDR3(0b0000);
			  }
		  }
	  }

	  if(t1 < HAL_GetTick())
	  {
		  t1 += 10;
		  pb = getPB();

		  if(pb==1)
		  {
			  flag=1;
			  bunyi(1,50);

			  HAL_Delay(300);

			  for(uint8_t i=0;i<3;i++)
			  {
				  LEDR1(0b0000);
				  LEDR2(0b1000);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b1000);
				  LEDR2(0b0100);
				  LEDR3(0b1000);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0100);
				  LEDR2(0b0010);
				  LEDR3(0b0100);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0010);
				  LEDR2(0b0001);
				  LEDR3(0b0010);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0001);
				  LEDR2(0b0000);
				  LEDR3(0b0001);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0000);
				  LEDR2(0b0001);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0001);
				  LEDR2(0b0010);
				  LEDR3(0b0001);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0010);
				  LEDR2(0b0100);
				  LEDR3(0b0010);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0100);
				  LEDR2(0b1000);
				  LEDR3(0b0100);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b1000);
				  LEDR2(0b0000);
				  LEDR3(0b1000);	bunyi(1,50); HAL_Delay(200);
			  }
			  flag=0;
		  }

		  if(pb==2)
		  {
			  flag=1;
			  bunyi(2,50);

			  HAL_Delay(300);

			  for(uint8_t i=0;i<3;i++)
			  {
				  LEDR1(0b1001);
				  LEDR2(0b0000);
				  LEDR3(0b1001);	bunyi(1,50); HAL_Delay(500);

				  LEDR1(0b1111);
				  LEDR2(0b0000);
				  LEDR3(0b1111);	bunyi(1,50); HAL_Delay(500);

				  LEDR1(0b0000);
				  LEDR2(0b1111);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(500);

				  LEDR1(0b0000);
				  LEDR2(0b1001);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(500);

				  LEDR1(0b0000);
				  LEDR2(0b1111);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(500);

				  LEDR1(0b1111);
				  LEDR2(0b0000);
				  LEDR3(0b1111);	bunyi(1,50); HAL_Delay(500);
			  }

			  for(uint8_t i=0;i<3;i++)
			  {
				  LEDR1(0b1111);
				  LEDR2(0b1001);
				  LEDR3(0b1111);	bunyi(1,50); HAL_Delay(200);

				  LEDR1(0b0000);
				  LEDR2(0b0000);
				  LEDR3(0b0000);	bunyi(1,50); HAL_Delay(200);
			  }
			  flag=0;
		  }

		  if(pb==4)
		  {
			  LEDR1(0b0110);
			  LEDR2(0b1111);
			  LEDR3(0b0110);

			  flag=1;
			  bunyi(3,50);

			  HAL_Delay(300);

			  for(uint8_t i=0;i<10;i++)
			  {
				  bunyi(1,250);
			  }
			  for(uint8_t i=0;i<10;i++)
			  {
				  bunyi(1,100);
			  }

			  flag=0;
		  }
	  }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 88;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5
                           PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
