#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

#include <config.h>
#include <drivers/uart/uart.h>
#include <drivers/gpio/gpio.h>
#include <drivers/motors/motors.h>
#include <stdio.h>

void UartHandler(int8_t left, int8_t right)
{
    //GPIOC->ODR^=GPIO_Pin_13;
    //printf("L: %d R: %d\n", left, right);
    Motors_Control(left, right);    
}

int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	Gpio_Init(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);	
    Uart_Init(USART_BAUD, UartHandler);
    Motors_Init();	
    
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);

    
    //Motors_Control(0, 0);
    
    //TIM2->CCR1 = 0x10 * MOTOR_MAX / 127;
    //TIM2->CCR2 = 0x10 * MOTOR_MAX / 127;
    
    //Motors_Control(-16, -16);
    //GPIO_WriteBit(GPIOA, GPIO_Pin_2, 0);
    //GPIO_WriteBit(GPIOA, GPIO_Pin_3, 0);
    
    //GPIO_WriteBit(GPIOC, GPIO_Pin_15, 0);
    //GPIO_WriteBit(GPIOC, GPIO_Pin_14, 0);
    
	while(1);
	
	return 0;
}
	