#ifndef IT_HANDLERS_H
#define IT_HANDLERS_H

#include <stdint.h>

extern volatile uint32_t sys_tick;

void SysTick_Handler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif