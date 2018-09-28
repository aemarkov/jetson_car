#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdint.h>
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

void Gpio_Init(GPIO_TypeDef* gpio, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed);

#endif
