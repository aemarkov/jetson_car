#include "motors.h"
#include <stdbool.h>
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include <drivers/gpio/gpio.h>
#include <config.h>
#include <stdio.h>

void Motors_Init(void)
{   
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOC,
		ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    
    // Настройка выводов управления мостами
    Gpio_Init(L_MOTOR_GPIO, L_MOTOR_A_PIN | L_MOTOR_B_PIN, 
              GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    
    Gpio_Init(R_MOTOR_GPIO, R_MOTOR_A_PIN | R_MOTOR_B_PIN, 
              GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
        
    // Настройка PWM выводов
    Gpio_Init(PWM_GPIO, L_MOTOR_PWM_PIN |
                        R_MOTOR_PWM_PIN,
                        GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    
    // Настройка таймера
    /*
        SYSCLOCK = 72 MHz
        PWM - 50 Hz
        Tpwm = 1/PWM = 20 ms
        
        F = 72 MHz / 72 = 1000 kHz
        T = 1/F =  1 mks
        Period = Tpwm / T = 20000
        
        Pulse min = 1000 
        Pulse max = 2000
		
		1*10^-6 * 10^3
    */
    
    TIM_TimeBaseInitTypeDef motorTimerInit;    
    TIM_TimeBaseStructInit(&motorTimerInit);
    motorTimerInit.TIM_ClockDivision = TIM_CKD_DIV1;
    motorTimerInit.TIM_CounterMode = TIM_CounterMode_Up;
    motorTimerInit.TIM_Prescaler = 72;
    motorTimerInit.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM2, &motorTimerInit);
    
    
    // Настройка каналов таймера
    TIM_OCInitTypeDef channelInit;    
    TIM_OCStructInit(&channelInit);    
    channelInit.TIM_OCMode = TIM_OCMode_PWM1;
    channelInit.TIM_OutputState = TIM_OutputState_Enable;
    channelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // L motor
    channelInit.TIM_Pulse = 0;
    TIM_OC1Init(TIM2, &channelInit);    
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    //GPIO_PinAFConfig(PWM_GPIO, L_MOTOR_PINSOURCE, GPIO_AF_1);    
    
    // R motor
    channelInit.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &channelInit);    
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    //GPIO_PinAFConfig(PWM_GPIO, R_MOTOR_PINSOURCE, GPIO_AF_1);
    
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
	
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);   
    //NVIC_EnableIRQ(TIM2_IRQn);
}

// Управляет ключами H-моста левого мотора
void L_Motor_SetDirection(bool a, bool b)
{
    //printf("L Dir: %d %d\n", a, b);
    if(a) L_MOTOR_GPIO->ODR |= L_MOTOR_A_PIN;
    else L_MOTOR_GPIO->ODR &= ~L_MOTOR_A_PIN;

    if(b) L_MOTOR_GPIO->ODR |= L_MOTOR_B_PIN;
    else L_MOTOR_GPIO->ODR &= ~L_MOTOR_B_PIN;
}

// Управляет ключами H-моста правого мотора
void R_Motor_SetDirection(bool a, bool b)
{
    //printf("R Dir: %d %d\n", a, b);
    if(a) R_MOTOR_GPIO->ODR |= R_MOTOR_A_PIN;
    else R_MOTOR_GPIO->ODR &= ~R_MOTOR_A_PIN;

    if(b) R_MOTOR_GPIO->ODR |= R_MOTOR_B_PIN;
    else R_MOTOR_GPIO->ODR &= ~R_MOTOR_B_PIN;
}

// Устанавлвиает скважность PWM левого мотора
void L_Motor_SetPWM(int8_t pwm)
{
    //printf("L PWM: %d\n", pwm);
    TIM2->CCR2 = pwm * MOTOR_MAX / 127;
    //TIM2->CCR1 = 3000;
}

// Устанавлвиает скважность PWM левого мотора
void R_Motor_SetPWM(int8_t pwm)
{
    //printf("R PWM: %d\n", pwm);
    TIM2->CCR1 = pwm * MOTOR_MAX / 127;
    //TIM2->CCR2 = 3000;
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
