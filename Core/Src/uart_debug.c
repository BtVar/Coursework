#include "uart_debug.h"
#include "stm32f411xe.h"

void UART_Init(void)        // инициализация UART2
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    GPIOA->MODER &= ~(3 << 4);
    GPIOA->MODER |= (2 << 4);
    GPIOA->AFR[0] &= ~(0xF << 8);
    GPIOA->AFR[0] |= (7 << 8);
    
   
      USART2->BRR = 417;  // 48000000 / 115200 = 416.67
    
     USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Ждем, пока буфер передачи освободится
    USART2->DR = (uint8_t)c;              // Явное приведение
}

void UART_SendString(char* str) {
 
   while (*str) UART_SendChar(*str++);
}