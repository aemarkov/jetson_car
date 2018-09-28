#include "gpio.h"

void Gpio_Init(GPIO_TypeDef* gpio, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed)
{
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = mode;
    gpio_init_structure.GPIO_Pin = pin;
    gpio_init_structure.GPIO_Speed = speed;
    GPIO_Init(gpio,&gpio_init_structure);
}
