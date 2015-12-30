#define BLversion "DHT0.7"
#define HAL_ADC_MODULE_ENABLED

//#include <stm32f4xx_gpio.h>
//hardware related includes
#include <stm32f4xx_hal.h>
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <stm32f407xx.h>
#include <stm32f4xx_it.h>
#include "stm32f4_discovery.h"
//C:\Users\User\AppData\Local\VisualGDB\EmbeddedBSPs\arm-eabi\com.sysprogs.arm.stm32\STM32F4xxxx\STM32F4xx_HAL_Driver\Src

//my display driver
#include "stm32f4_DAL.h"
//#include "tm_stm32f4_hd44780.h"

//my humidity sensor driver
#include "stm32_dht11.h"

//my HAL layer
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"

// c++ standard library related includes
#include <string>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <vector>
#include <map>






extern "C"
{
	
	#include <usbd_desc.h>

	USBD_HandleTypeDef USBD_Device;
	void SysTick_Handler(void);
	void OTG_FS_IRQHandler(void);
	void OTG_HS_IRQHandler(void);
	extern PCD_HandleTypeDef hpcd;
	
	int VCP_read(void *pBuffer, int size);
	int VCP_write(const void *pBuffer, int size);
	extern char g_VCPInitialized;
	

}

using namespace std;

__IO uint32_t ADCTripleConvertedValue[3];
ADC_HandleTypeDef g_AdcHandle;

GPIO_InitTypeDef GPIO_InitStructure2;

//HW related, to be excluded to separate lib
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2); //HW prescale changed from 1 to 2 (ADC support)

	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	
	GPIO_InitStructure2.Pin = GPIO_PIN_5;
	GPIO_InitStructure2.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure2.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure2.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure2);
	
	GPIO_InitStructure.Pin = GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 288;// bylo 336
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
	                               RCC_CLOCKTYPE_HCLK |
	                               RCC_CLOCKTYPE_PCLK1 |
	                               RCC_CLOCKTYPE_PCLK2);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
	
	SystemCoreClockUpdate();
 
	if (HAL_GetREVID() == 0x1001)
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

void ConfigureSWD() {
	GPIO_InitTypeDef swdInit;
	
	swdInit.Pin = GPIO_PIN_3;
	swdInit.Mode = GPIO_MODE_AF_PP;
	swdInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &swdInit);
	
}

void ConfigureADC()
{
	GPIO_InitTypeDef gpioInit, swdInit;
	
	
	__GPIOC_CLK_ENABLE();
	__ADC1_CLK_ENABLE();
 
	gpioInit.Pin = GPIO_PIN_1;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &gpioInit);
 
	swdInit.Pin = GPIO_PIN_3;
	swdInit.Mode = GPIO_MODE_AF_PP;
	swdInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &swdInit);
	
	
			
	
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
 
	
	ADC_ChannelConfTypeDef adcChannel;
 
	g_AdcHandle.Instance = ADC1;
 
	g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	g_AdcHandle.Init.ScanConvMode = DISABLE;
	g_AdcHandle.Init.ContinuousConvMode = ENABLE;
	g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	g_AdcHandle.Init.NbrOfDiscConversion = 0;
	g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	g_AdcHandle.Init.NbrOfConversion = 1;
	g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
	g_AdcHandle.Init.EOCSelection = DISABLE;
 
	HAL_ADC_Init(&g_AdcHandle);
 
	adcChannel.Channel = ADC_CHANNEL_11;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcChannel.Offset = 0;
 
	if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
	{
		asm("bkpt 255");
	}
}



void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

//#ifdef USE_USB_FS
void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}

void echo(char *retezec) {
	
	int lenth = 0;
	while (*(retezec + lenth) != '\0') {
		lenth++;
	}
	
	VCP_write(retezec, lenth);
}

void echo(string retezec) {
	
	VCP_write(&retezec, retezec.length());
}

void echo(int cislo) {
	char a[200];
		
	sprintf(a, "%i", cislo);
	echo(a);	
}

int echoN() {
	
	VCP_write("\r\n", 2);
	return 0;
}	



void initHW() {
	HAL_Init();
	SystemClock_Config();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_hellowrld_fops); //!!! VGC generuje jmeno registrove promenne podle projektu
	USBD_Start(&USBD_Device);
	
} 

void measADC() {

	uint32_t g_ADCValue;
	int g_MeasurementNumber = 0;
	
	ConfigureADC();
    
	HAL_ADC_Start(&g_AdcHandle);
	for (int i = 0; i < 100;i++)
	{
		if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
		{
			g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
			g_MeasurementNumber++;
		}
		echo("aktualni hodnota: ");
		echo(g_ADCValue);
		echo("\r\n");
	}
		
}

int main(void)
{
	//HAL initialization
	initHW();
	
/*	
	HAL_Init();
	SystemClock_Config();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_hellowrld_fops);
	USBD_Start(&USBD_Device);
	
	
	
*/
		
	//input char buffer
	char keyBuff[200];
	uint8_t dht11data = 0;
	uint8_t * pData = &dht11data;
	
	
	for (;;)
	{
		if (VCP_read(keyBuff, 200) != 1) //read last one char from terminal buffer
			continue;
		
		delayMs(500);
		switch (keyBuff[0]) {
		case 'm':
			echo(BLversion); echoN();
			echo("Menu "); echoN();
			echo("m ... Show MENU");echoN();
			echo("v ... Check variables");echoN();
			echo("b ... Show memmory banks");echoN();
			echo("x ... Check RS232");echoN();
			echo("a ... ADC measure");echoN();
			echo("s ... SetPin PC3");echoN();
			echo("r ... ResetPin PC3");echoN();
			echo("i ... read PC3 pin Value");echoN();
			echo("d 1,2,3... read humidity");echoN();
			echo("t ... test precisious us delay");echoN();
			
			break;
		case 'x':
		//	nextState = activFSM[nextState]->startStav();
			break;
			
		case 'i':
			
			
			echo("Na pinu PC5 je uroven: ");
			echo(readPin(GPIOC, GPIO_PIN_5));
			
			break;
		
		case 'a':
			measADC();
			
			break;
		case 's':
			GPIO_InitStructure2.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOC, &GPIO_InitStructure2);
		
			echo("pin set"); echoN();
			
			//setPin(GPIOD, GPIO_PIN_12);
			setPin(GPIOC, GPIO_PIN_5);
			break;
			
		case 'r':
			echo("pin reset"); echoN();
			resetPin(GPIOD, GPIO_PIN_12);
			resetPin(GPIOC, GPIO_PIN_5);
			break;
			
		case 'd':
			echo("entering dht11..\n"); echoN();
			
			DHT11_Read(pData);
			echo(*pData);echoN();
			break;
			
		case '1':
			echo("set dht11 pin..\n"); echoN();
			debugSet();
			break;
		
		case '2':
			echo("read dht11 pin..\n"); echoN();
					
			echo(debugRead());echoN();
			break;
			
		case '3':
			echo("reset dht11 pin..\n"); echoN();
			debugReset();
			break;

		case 't':
			echo("started");echoN();
					
			delayuS(1);
			
			echo("ended in 1 uS");echoN();
			break;
			
		default:
			echo("Unknown cmd");
			echoN();
			echoN();
		}
		
	}	
		
	
}