#include "stm32f4_DAL.h"
#include "tm_stm32f4_delay.h"
extern void echo(char* retezec);

/*#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
*/
using namespace std;

long tim2_us = 0;

void delayMs(int ms) {
	if (ms > 0) {
		ms = ms * 1000;
		for (int i = 0; i < ms; i++) {
			asm("nop");	
		}
	}
}

void delayuS(int us) {
		
	//TM_DELAY_Init();
	
	//for (int i = 0; i < us; i++) {
	//		asm("nop");	}
	echo("IRQ started..");
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//	void        RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);


	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 85 - 1; // 84MHz (to 1 uS)
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //84 - 1; // Down to 1 MHz (adjust per your clock) OK
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // OK
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
	int8_t cnt = 0;
	// num of uS * counter overflow
	echo("Timmer inited..");
	while (us > cnt)
	{
		echo("loop");
		for (;;){	
					
			if (TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET){
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	
			break;
			}	
		}
	cnt++;
	}
	
	
	
		
		
}


void setPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin) {
	GPIO_InitTypeDef tempGPIOsetup;
	tempGPIOsetup.Pin = gpioPin;
	tempGPIOsetup.Mode = GPIO_MODE_OUTPUT_PP;
	
	HAL_GPIO_WritePin(gpioGate, gpioPin, GPIO_PIN_SET);
	
}
void resetPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin) {
	GPIO_InitTypeDef tempGPIOsetup;
	tempGPIOsetup.Pin = gpioPin;
	tempGPIOsetup.Mode = GPIO_MODE_OUTPUT_PP;
	
	HAL_GPIO_WritePin(gpioGate, gpioPin, GPIO_PIN_RESET);
}
void setPin(string pinName) {
	//parse name
	//call setPin standard
	
}


uint8_t readPin(string pinName) {
	return -1;
	
}
uint8_t readPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin) {
	uint8_t bitstatus = 0x00;
	GPIO_InitTypeDef tempGPIOsetup;
	tempGPIOsetup.Pin = gpioPin;
	tempGPIOsetup.Mode = GPIO_MODE_INPUT;
			
	HAL_GPIO_Init(gpioGate, &tempGPIOsetup);
  /* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(gpioGate));
	assert_param(IS_GET_GPIO_PIN(gpioPin));
		
		/* PC1 = pin PC1 */

	if ((GPIOC->IDR & (uint16_t)gpioPin) != (uint32_t)0)
	{
		bitstatus = 1;
	}
	else
	{
		bitstatus = 0;
	}
	
	
	
	return bitstatus;
}	