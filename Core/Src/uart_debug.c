#include "uart_debug.h"
#include "stm32f411xe.h"

void UART_Init(void)
{
    /* GPIOA + USART2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 (TX), PA3 (RX) AF7 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |=  (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);

    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) |
                     (7 << GPIO_AFRL_AFSEL3_Pos);

    /* 115200 baud @ 48 MHz APB1 */
    USART2->BRR = 48000000 / 115200;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

void UART_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART_SendString(const char *s)
{
    while (*s) {
        UART_SendChar(*s++);
    }
}