#include "it_handlers.h"
#include "gyrocoptercopter.h"

uint32_t sys_tick = 0;
extern uint16_t left_encoder_ticks;
extern uint16_t right_encoder_ticks;

void SysTick_Handler(void)      // прерывание системного таймера
{
    sys_tick++;
    gyrocoptercopter_Update();      // обновление данных гироскопа
}

void EXTI9_5_IRQHandler(void)       //обработчик прерываний левого энкодера
{
    if (EXTI->PR & EXTI_PR_PR8) {
        left_encoder_ticks++;
        EXTI->PR = EXTI_PR_PR8;
    }
}

void EXTI15_10_IRQHandler(void)     //обработчик прерываний правого энкодера
{
    if (EXTI->PR & EXTI_PR_PR12) {
        right_encoder_ticks++;
        EXTI->PR = EXTI_PR_PR12;
    }
}
