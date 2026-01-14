#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>

void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(char* str);

#endif