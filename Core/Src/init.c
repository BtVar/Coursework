#include "init.h"

void GPIO_Init_Ports(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);             // Включение тактирования GPIOB
    
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODE10_0);             // Настройка пина PB10 на выход, регистр GPIOx_MODER
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);            // Установление PB10 в режим pull-push, регистр OTYPER
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10_0);     // Устанавливаем скорость бита PB10 (средняя), регистр OSPEEDR
    CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD10_0);           // Отключаем подтягивающий резистор PB10, регистр PUPDR
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR10);                   // Установление на пине PB10 0, регистр GPIOx_BSRR

    SET_BIT(GPIOB->MODER, GPIO_MODER_MODE6_0);              // Настройка пина PB4 на выход, регистр GPIOx_MODER
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_6);             // Установление PB4 в режим pull-push, регистр OTYPER
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6_0);      // Устанавливаем скорость бита PB4 (средняя), регистр OSPEEDR
    CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD6_0);            // Отключаем подтягивающий резистор PB4, регистр PUPDR
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR6);                    // Установление на пине PB4 0, регистр GPIOx_BSRR
        
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODE5_0);             // Настройка пина PB10 на выход, регистр GPIOx_MODER
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_5);            // Установление PB10 в режим pull-push, регистр OTYPER
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5_0);     // Устанавливаем скорость бита PB10 (средняя), регистр OSPEEDR
    CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD5_0);           // Отключаем подтягивающий резистор PB10, регистр PUPDR
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR5);                   // Установление на пине PB10 0, регистр GPIOx_BSRR

    SET_BIT(GPIOB->MODER, GPIO_MODER_MODE7_0);              // Настройка пина PB4 на выход, регистр GPIOx_MODER
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_7);             // Установление PB4 в режим pull-push, регистр OTYPER
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7_0);      // Устанавливаем скорость бита PB4 (средняя), регистр OSPEEDR
    CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD7_0);            // Отключаем подтягивающий резистор PB4, регистр PUPDR
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);                    // Установление на пине PB4 0, регистр GPIOx_BSRR
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

    // настройка EXTI регистров
    SET_BIT(EXTI->IMR, EXTI_IMR_MR12); //Настройка маскирования 12 линии 
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR12); //Настройка детектирования нарастающего фронта 12 линии 
    //SET_BIT(EXTI->FTSR, EXTI_FTSR_TR12); //Настройка детектирования спадающего фронта 12 линии 
    NVIC_SetPriority(EXTI15_10_IRQn, 
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); //Установка 0 приоритета прерывания для вектора EXTI15_10 
    NVIC_EnableIRQ(EXTI15_10_IRQn); //Включение прерывания по вектору EXTI15_10
    
    //==================================================================================================

    // Настройка прерываний на PC8 (левый энкодер)
    MODIFY_REG(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI8_Msk, 
    SYSCFG_EXTICR3_EXTI8_PC); //Настройка мультиплексора на вывод линии прерывания EXTI8 на PC8 

    // настройка EXTI регистров
    SET_BIT(EXTI->IMR, EXTI_IMR_MR8); //Настройка маскирования 8 линии 
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR8); //Настройка детектирования нарастающего фронта 8 линии 
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR12); //Настройка детектирования спадающего фронта 8 линии 
    NVIC_SetPriority(EXTI9_5_IRQn, 
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); //Установка 0 приоритета прерывания для вектора EXTI9_5 
    NVIC_EnableIRQ(EXTI9_5_IRQn); //Включение прерывания по вектору EXTI15_10

    //==================================================================================================

    // Настройка прерываний на PC13 (кнопка)
    MODIFY_REG(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13_Msk, 
    SYSCFG_EXTICR4_EXTI13_PC); //Настройка мультиплексора на вывод линии прерывания EXTI13 на PC13 

    // настройка EXTI регистров
    SET_BIT(EXTI->IMR, EXTI_IMR_MR13); //Настройка маскирования 13 линии 
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR13); //Настройка детектирования нарастающего фронта 13 линии 
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR13); //Настройка детектирования спадающего фронта 13 линии 
    NVIC_SetPriority(EXTI15_10_IRQn, 
    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); //Установка 0 приоритета прерывания для вектора EXTI15_10 
    NVIC_EnableIRQ(EXTI15_10_IRQn); //Включение прерывания по вектору EXTI15_10
    
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

}

