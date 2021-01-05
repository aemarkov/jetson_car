#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

#include <config.h>
#include <drivers/uart/uart.h>
#include <drivers/gpio/gpio.h>
#include <drivers/motors/motors.h>
#include <stdio.h>

#define MOTORS_STOP_TIMEOUT 30 //300 ms

volatile int16_t timeout_counter;

void UartHandler(int8_t left, int8_t right)
{
    Motors_Control(left, right);
    // Reset timeout
    timeout_counter = MOTORS_STOP_TIMEOUT;
}

void stopTimerInit()
{
    /* Motor timeout stop timer. If no commands are received
       in specific time, motors will be stopped for security
       reasons

       SYSCLOCK = 72 MHz
       TIMEOUT = 300ms
       Prescaler = 72
       Interval =  10'000
       T = 72 Mhz / 72 / 10'000 = 100 Hz
    */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef timInit;
    TIM_TimeBaseStructInit(&timInit);
    timInit.TIM_ClockDivision = TIM_CKD_DIV1;
    timInit.TIM_CounterMode = TIM_CounterMode_Up;
    timInit.TIM_Prescaler = 72;
    timInit.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM3, &timInit);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler()
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update)!=RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        timeout_counter--;
        if(timeout_counter<0)
        {
            // Stop motors
            timeout_counter = MOTORS_STOP_TIMEOUT;
            Motors_Control(0, 0);
            GPIOC->ODR^=GPIO_Pin_13;
        }
    }
}

int main()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    Gpio_Init(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);

    timeout_counter = 0;
    stopTimerInit();
    Uart_Init(USART_BAUD, UartHandler);
    Motors_Init();

    while(1);

    return 0;
}
