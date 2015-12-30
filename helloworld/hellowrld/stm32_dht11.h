#ifndef _DHT11_DEF_
#define _DHT11_DEF_
 
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4_DAL.h"
//#include "stm32f4xx_hal_gpio.h"
//#include "timer.h"
 
/* configuration macros: */
/* enable Delay setup */
#define DELAY_INIT          TIM2_Init();
#define DELAY_DEINIT        TIM2_Deinit()
#define DELAY_ENABLE        TIM2_Enable()
#define DELAY_DISABLE       TIM2_Disable()
#define DELAY_US(__time__)  TIM2_DelayUs((__time__))
 
/* GPIO configuration defines for 1-wire data input/output pin*/
#define GPIO_SET_AS_OUTPUT  Gpio_SetPinAsOutput(GPIOA, GPIO_PIN1, GPIO_SPEED_2MHZ, GPIO_GP_PUSH_PULL)
#define GPIO_SET_AS_INPUT   Gpio_SetPinAsInput(GPIOA, GPIO_PIN1, GPIO_FLOATING_IN)
#define GPIO_OUPUT_CLEAR    Gpio_ClearOutputPin(GPIOA, GPIO_PIN1)   /* clear port state */
#define GPIO_OUTPUT_SET     Gpio_SetOutputPin(GPIOA, GPIO_PIN1)     /* set port state to 1 */
#define GPIO_INPUT_GET      Gpio_GetInputPinValue(GPIOA, GPIO_PIN1) /* should return 0 or 1 */
 
/* Optional critical section (because of delays slow as 30us) */
#define CRITICAL_SECTION_INIT
#define CRITICAL_SECTION_DEINIT
#define CRITICAL_SECTION_ENTER
#define CRITICAL_SECTION_LEAVE
 
/* optional timeouts for while() loops (in case of hardware failure) */
#define ENABLE_TIMEOUTS     /* comment to not perform timeout checks */
#define TIMEOUT_VALUE       100000
 
/* Return codes */
typedef enum DHT11_ERROR_CODE_t
{
	DHT11_OK            = 0,
	DHT11_TIMEOUT,
	DHT11_WRONG_CHCKSUM
} DHT11_ERROR_CODE_t;
 
/* Interface function declarations */
void               DHT11_Init(void);
void               DHT11_Denit(void);
DHT11_ERROR_CODE_t DHT11_Read(uint8_t * const pData);
#endif

int debugRead();
void debugSet();
void debugReset();
