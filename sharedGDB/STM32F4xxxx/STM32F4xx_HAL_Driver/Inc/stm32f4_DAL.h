//hardware related includes
#include <stm32f4xx_hal.h>
#include <stm32f407xx.h>
#include <stm32f4xx_it.h>
#include "stm32f4_discovery.h"
//my display driver

//my HAL layer
#include "stm32f4xx_hal_gpio.h"

// c++ language related includes



#include <string>
#include <cstdio>
#include <cstdlib>
#include <csignal>


void delay(int ms);
void setPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin);
void resetPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin);
uint8_t readPin(GPIO_TypeDef* gpioGate, uint16_t gpioPin);


/*
void resetPin(std::string pinName);

uint8_t readPin(std::string pinName);

void setPin(std::string pinName);
*/