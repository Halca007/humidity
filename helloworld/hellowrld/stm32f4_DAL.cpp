#include "stm32f4_DAL.h"
#include "tm_stm32f4_delay.h"

using namespace std;

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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 100 - 1; // 1 MHz down to 10 KHz (0.1 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // Down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
	
	int8_t cnt = 0;
	while (us > cnt)
	{
		
	
	if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearFlag(TIM2, TIM_IT_Update);
		cnt++;
	}
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