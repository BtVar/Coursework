
#include "init.h"  // Подключаем заголовочный файл с объявлениями функций
#include "utils.h"

uint8_t pwm_state_pa8 = 0;
uint8_t pwm_state_pa9 = 0;
uint16_t left_encoder_ticks = 0;
uint16_t right_encoder_ticks = 0;
uint32_t left_delay_counter = 0;
uint32_t right_delay_counter = 0;

uint32_t millis_counter = 0;

uint16_t button_delay_counter = 0;
bool status_button = 0;

int main(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    SET_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR12_0);

    GPIO_Init_Ports();

    // Инициализируем прерывания
    ITR_Init();
    // Инициализируем тактирование системы (RCC)
    RCC_Init();            // Настраивает тактовую частоту 96 МГц
    
    // Инициализируем SysTick (опционально, для задержек)
    SysTick_Init();        // Настраивает системный таймер на 1 кГц
    
    // Инициализируем ШИМ
    TIM1_PWM_Init();       // Настраивает TIM1 для ШИМ на PA8 и PA9
    
    
    left_wheel_direction(2);
    right_wheel_direction(2);
    Set_PWM_Left_DutyCycle(65);
    Set_PWM_Right_DutyCycle(65);
    
   // moveForward(12.0); // Двигаемся вперед на 100 см
    while(1)
    {
   
    }
}