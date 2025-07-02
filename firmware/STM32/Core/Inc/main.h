#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

/* Exported functions prototypes */
void Error_Handler(void);
void SystemClock_Config(void);

/* Private defines */
#define CS_LoRa_Pin GPIO_PIN_4
#define CS_LoRa_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

#endif /* __MAIN_H */