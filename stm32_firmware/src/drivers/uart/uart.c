#include "uart.h"
#include <config.h>
#include <string.h>
#include <drivers/gpio/gpio.h>
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

/*
Setup USART and receive motor control commands.

Protocol:
|--------|------|-------|
| HEADER | LEFT | RIGHT |
|--------|------|-------|
             1b     1b
             
Calls callback when command is received
*/

#define CMD_SIZE 2

// Handler
CommandHandler _uartCommandHandler;

uint8_t _header[] = {0x34, 0x27, 0x77};
uint8_t _state;
uint8_t _headerCnt;
uint8_t _bufferCnt;
int8_t _buffer[CMD_SIZE];


void Uart_Init(int baud, CommandHandler handler)
{
    _uartCommandHandler = handler;
    _state = 0;
    _headerCnt = 0;
    _bufferCnt = 0;
    memset(_buffer, 0, sizeof(_buffer));
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
                           RCC_APB2Periph_USART1,
                           ENABLE);
	
    // Setup USART pins
    Gpio_Init(GPIOA, USART_TX, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    Gpio_Init(GPIOA, USART_RX, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);

    // Setup USART
    USART_InitTypeDef usartInit;
    usartInit.USART_BaudRate = baud;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_DeInit(CMD_USART);
    USART_Init(CMD_USART, &usartInit);
    USART_Cmd(CMD_USART, ENABLE);

    // Setup USART interrupts
    NVIC_InitTypeDef init;
    init.NVIC_IRQChannel = USART1_IRQn;
    init.NVIC_IRQChannelPreemptionPriority  = 11;
    init.NVIC_IRQChannelSubPriority = 0;
    init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&init);

    USART_ITConfig(CMD_USART, USART_IT_RXNE, ENABLE); 
}

void USART1_IRQHandler()
{   
    if(USART_GetITStatus(CMD_USART, USART_IT_RXNE)!=RESET)
    {
        USART_ClearITPendingBit(CMD_USART, USART_IT_RXNE);
        uint8_t byte = USART_ReceiveData(CMD_USART);              

        if(_state == 0)
        {
			// Receiving header
            if(byte == _header[_headerCnt])
            {
                _headerCnt++;
                if(_headerCnt >= 3)
                    _state = 1;
            }
            else
            {
                _headerCnt = 0;
            }
            
        }
        else if(_state == 1)
        {
            // Receiving payload
            _buffer[_bufferCnt] = byte;
            _bufferCnt++;
            
            // Data received
            if(_bufferCnt >= CMD_SIZE)
            {
                _uartCommandHandler(_buffer[0], _buffer[1]);
                _headerCnt = 0;
                _bufferCnt = 0;
                _state = 0;
            }
        }
    }
}

// Send char synchonically
int sendchar(int ch)
{
    while(!USART_GetFlagStatus(CMD_USART, USART_FLAG_TXE));
    USART_SendData(CMD_USART, ch);
    return ch;
}
