#include "uart_debug.h"
#include "stm32f411xe.h"

void UART_Init(void)        // инициализация UART2
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        //включаю тактирование порта A
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;       //включаю тактирование USART2

    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);       //очищаю настройку пинов PA2 и PA3
    GPIOA->MODER |=  (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);       // Установка альтернативной функции для PA2 и PA3

    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);     // Устанавливаю альтернативной функции AF7 (USART2) для PA2 и PA3

    USART2->BRR = 48000000 / 115200;        //установка скорости передачи 115200 бод при тактировании 48 МГц

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;     //включение передачи и приема
    USART2->CR1 |= USART_CR1_UE;                    //включение USART2
}

void UART_SendChar(char c)      // отправка одного символа по UART2
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART_SendString(const char *s)     // отправка строки по UART2
{
    while (*s) {
        UART_SendChar(*s++);
    }
}