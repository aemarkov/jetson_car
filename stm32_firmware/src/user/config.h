#ifndef __CONFIG_H__
#define __CONFIG_H__

//////////////// UART ///////////////////////////

#define USART_BAUD 115200
#define CMD_USART  USART1
#define USART_TX   GPIO_Pin_9
#define USART_RX   GPIO_Pin_10

//////////////// MOTORS /////////////////////////

#define L_MOTOR_GPIO            GPIOA
#define L_MOTOR_A_PIN           GPIO_Pin_3
#define L_MOTOR_B_PIN           GPIO_Pin_2

#define R_MOTOR_GPIO            GPIOC
#define R_MOTOR_A_PIN           GPIO_Pin_14
#define R_MOTOR_B_PIN           GPIO_Pin_15

#define PWM_GPIO                GPIOA
#define L_MOTOR_PWM_PIN         GPIO_Pin_1
#define R_MOTOR_PWM_PIN         GPIO_Pin_0

#endif
