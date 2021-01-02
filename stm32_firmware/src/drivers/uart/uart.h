#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

/**
    \brief Command received callback function ptr
    \param[in] left motor speed
    \param[in] right motor speed
*/
typedef void (*CommandHandler)(int8_t, int8_t);

/**
    \brief Configure UART
    Callback is called when command is received
    \param[in] handler Callback
*/
void Uart_Init(int baud, CommandHandler handler);

int sendchar(int ch);

#endif
