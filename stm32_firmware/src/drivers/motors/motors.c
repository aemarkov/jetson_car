#include "motors.h"
#include <stdbool.h>
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include <drivers/gpio/gpio.h>
#include <config.h>
#include <stdio.h>

#define MOTOR_MIN               0
#define MOTOR_MAX               3600

void Motors_Init(void)
{   
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOC,
		ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    
    // Setup motor control pins
    Gpio_Init(L_MOTOR_GPIO, L_MOTOR_A_PIN | L_MOTOR_B_PIN, 
              GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    
    Gpio_Init(R_MOTOR_GPIO, R_MOTOR_A_PIN | R_MOTOR_B_PIN, 
              GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

    Gpio_Init(PWM_GPIO, L_MOTOR_PWM_PIN |
                        R_MOTOR_PWM_PIN,
                        GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    
    // Setup PWM
    /*
        SYSCLOCK = 72 MHz
        PWM - 20 kHz
        Tpwm = 1/PWM = 50 uS
        
        F = 72 MHz
        T = 1/f = ...
        Period = Tpwm / T = 3600
    */
    
    TIM_TimeBaseInitTypeDef motorTimerInit;    
    TIM_TimeBaseStructInit(&motorTimerInit);
    motorTimerInit.TIM_ClockDivision = TIM_CKD_DIV1;
    motorTimerInit.TIM_CounterMode = TIM_CounterMode_Up;
    motorTimerInit.TIM_Prescaler = 1 - 1; // Prescaler is "TIM_Prescaler - 1"
    motorTimerInit.TIM_Period = MOTOR_MAX;
    TIM_TimeBaseInit(TIM2, &motorTimerInit);
    
    
    // Setup timer channels
    TIM_OCInitTypeDef channelInit;    
    TIM_OCStructInit(&channelInit);    
    channelInit.TIM_OCMode = TIM_OCMode_PWM1;
    channelInit.TIM_OutputState = TIM_OutputState_Enable;
    channelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // L motor
    channelInit.TIM_Pulse = 0;
    TIM_OC1Init(TIM2, &channelInit);    
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // R motor
    channelInit.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &channelInit);    
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

// Control left motor H-bridge
void L_Motor_SetDirection(bool a, bool b)
{
    //printf("L Dir: %d %d\n", a, b);
    if(a) L_MOTOR_GPIO->ODR |= L_MOTOR_A_PIN;
    else L_MOTOR_GPIO->ODR &= ~L_MOTOR_A_PIN;

    if(b) L_MOTOR_GPIO->ODR |= L_MOTOR_B_PIN;
    else L_MOTOR_GPIO->ODR &= ~L_MOTOR_B_PIN;
}

// Control right motor H-bridge
void R_Motor_SetDirection(bool a, bool b)
{
    if(a) R_MOTOR_GPIO->ODR |= R_MOTOR_A_PIN;
    else R_MOTOR_GPIO->ODR &= ~R_MOTOR_A_PIN;

    if(b) R_MOTOR_GPIO->ODR |= R_MOTOR_B_PIN;
    else R_MOTOR_GPIO->ODR &= ~R_MOTOR_B_PIN;
}

// Control left motor PWM duty cycle
void L_Motor_SetPWM(uint8_t pwm)
{
    TIM2->CCR2 = (uint32_t)pwm * MOTOR_MAX / 127U;
}

// Control right motor PWM duty cycle
void R_Motor_SetPWM(uint8_t pwm)
{
    TIM2->CCR1 = (uint32_t)pwm * MOTOR_MAX / 127U;
}

void Motors_Control(int8_t left, int8_t right)
{
    if(left > 0)
    {
        L_Motor_SetDirection(false, true);
        L_Motor_SetPWM(left);
    }
    else if(left < 0)
    {
        L_Motor_SetDirection(true, false);
        L_Motor_SetPWM(-left);
    }
    else
    {
        L_Motor_SetDirection(false, false);
        L_Motor_SetPWM(0);
    }
    
    if(right > 0)
    {
        R_Motor_SetDirection(false, true);
        R_Motor_SetPWM(right);
    }
    else if(right < 0)
    {
        R_Motor_SetDirection(true, false);
        R_Motor_SetPWM(-right);
    }
    else
    {
        R_Motor_SetDirection(false, false);
        R_Motor_SetPWM(0);
    }
}
