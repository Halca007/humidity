#include "stm32f4_DAL.h"

using namespace std;

void delay(int ms) {
	if (ms > 0) {
		ms = ms * 1000;
		for (int i = 0; i < ms; i++) {
			asm("nop");	
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