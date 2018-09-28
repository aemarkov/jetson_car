#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

typedef void (*CommandHandler)(int8_t, int8_t);

void Uart_Init(int baud, CommandHandler handler);

int sendchar(int ch);

#endif
