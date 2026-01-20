#include "init.h" // Подключаем заголовочный файл с объявлениями функций
#include "utils.h"
#include "mpu_i2c.h"

#define WHO_AM_I_REG 0x75

uint8_t pwm_state_pa8 = 0;
uint8_t pwm_state_pa9 = 0;
volatile uint16_t left_encoder_ticks = 0;
volatile uint16_t right_encoder_ticks = 0;
uint32_t left_delay_counter = 0;
uint32_t right_delay_counter = 0;
volatile uint32_t sys_tick = 0;

uint16_t button_delay_counter = 0;
volatile bool status_button = 0;



int main(void)
{

    RCC_Init();
    GPIO_Init_Ports();
    SysTick_Init();
    TIM1_PWM_Init();
    ITR_Init();
    I2C_Init_HW();

    MPU_Calibrate();

    while (1)
    {
        if (status_button)
        {
            delay_ms(1000);
            Traject();
            status_button = 0;
        }
    }
}
