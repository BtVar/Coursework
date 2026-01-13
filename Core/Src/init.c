#include "init.h"

void GPIO_Init_Ports(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;      // включаю тактирование портов B и C
    GPIOB->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE6_0;       // левый мотор
    GPIOB->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE4_0;        // правый мотор
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_0;                       // кнопка
}

void SysTick_Init(void)
{
    SysTick->LOAD = 95999;      // 1 мс при частоте 96 МГц
    SysTick->VAL  = 0;          // сброс значения текущего счётчика
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // источник шина, разрешаю прерывания и врубаю таймер
}

void RCC_Init(void)
{
    RCC->CR |= RCC_CR_HSION;        // включаю внутренний генератор (очевидно HSI) 16 МГц
    while (!(RCC->CR & RCC_CR_HSIRDY));     // жду готовности HSI
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_5;   // 16 МГц / 8 * 168 / 2 = 84 МГц
    RCC->CR |= RCC_CR_PLLON;       // включаю PLL
    while (!(RCC->CR & RCC_CR_PLLRDY));     // жду готовности PLL

    RCC->CFGR |= RCC_CFGR_SW_PLL;   // переключаю системный тактовый сигнал на PLL
}

void TIM1_PWM_Init(void)        // инициализация ШИМ на TIM1 CH1 и CH2 (PA8 и PA9)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        // включаю тактирование порта A
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;     // включаю тактирование TIM1

    GPIOA->MODER |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;    // настраиваю PA8 и PA9
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4);       // настраиваю альтернативную функцию PA8 и PA9 на TIM1 CH1 и CH2

    TIM1->PSC = 5;       // предделитель таймера
    TIM1->ARR = 999;        // период таймера

    TIM1->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE | (6 << 12) | TIM_CCMR1_OC2PE;        // режим ШИМ1 для CH1 и CH2, включаю предзагрузку регистра сравнения
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;    // включаю выходы CH1 и CH2    
    TIM1->BDTR |= TIM_BDTR_MOE;     // включаю главный выход для TIM1
    TIM1->CR1 |= TIM_CR1_CEN;       // включаю таймер
}

void ITR_Init(void)     // инициализация прерываний по кнопке на PC8 и PC12
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;       // включаю тактирование SYSCFG

    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PC;       // настраиваю EXTI8 на PC8
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PC;      // настраиваю EXTI12 на PC12

    EXTI->IMR |= EXTI_IMR_MR8 | EXTI_IMR_MR12;          // разрешаю прерывания по EXTI8 и EXTI12
    EXTI->RTSR |= EXTI_RTSR_TR8 | EXTI_RTSR_TR12;       // настраиваю прерывания по Rising edge

    NVIC_EnableIRQ(EXTI9_5_IRQn);                       // разрешаю прерывания в NVIC для EXTI8
    NVIC_EnableIRQ(EXTI15_10_IRQn);                 // разрешаю прерывания в NVIC для EXTI12
}