#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>

//===================================================================
#define LOW 		GPIO_PIN_RESET
#define HIGH 		GPIO_PIN_SET

#define LED1(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,x?  HIGH:LOW)
#define LED2(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,x?  HIGH:LOW)
#define LED3(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,x?  HIGH:LOW)
#define LED4(x) 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,x?  HIGH:LOW)
#define LED5(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,x?  HIGH:LOW)
#define LED6(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,x?  HIGH:LOW)
#define LED7(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,x? LOW:HIGH)
#define LED8(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,x?  LOW:HIGH)
#define LED9(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,x?  LOW:HIGH)
#define LED10(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,x?  LOW:HIGH)
#define LED11(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,x? LOW:HIGH)
#define LED12(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,x?  LOW:HIGH)

#define COM0(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,x?  HIGH:LOW)
#define COM1(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,x?  LOW:HIGH)

#define PB1 		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define PB2 		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define PB3 		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)

//=================================================================== Function Prototype

uint16_t i;

void LEDs(uint16_t x)
{
	LED1(x & 1<<0);
	LED2(x & 1<<1);
	LED3(x & 1<<2);
	LED4(x & 1<<3);
	LED5(x & 1<<4);
	LED6(x & 1<<5);
	LED7(x & 1<<6);
	LED8(x & 1<<7);
	LED9(x & 1<<8);
	LED10(x & 1<<9);
	LED11(x & 1<<10);
	LED12(x & 1<<11);
}

void beep(uint16_t j, uint16_t t)
{
	for(i=0;i<j;i++)
	{
//		buzzer(1);
//		HAL_Delay(t);
//		buzzer(0);
//		HAL_Delay(t);
	}
}


