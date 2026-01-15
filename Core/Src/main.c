#include "init.h" // Подключаем заголовочный файл с объявлениями функций
#include "utils.h"
#include "uart_debug.h"
#include "mpu_i2c.h"

#define WHO_AM_I_REG        0x75

uint8_t pwm_state_pa8 = 0;
uint8_t pwm_state_pa9 = 0;
volatile uint16_t left_encoder_ticks = 0;
volatile uint16_t right_encoder_ticks = 0;
uint32_t left_delay_counter = 0;
uint32_t right_delay_counter = 0;
volatile uint32_t sys_tick = 0;

uint16_t button_delay_counter = 0;
volatile bool status_button = 0;

uint8_t z;

int main(void)
{

    RCC_Init();
    GPIO_Init_Ports();
    SysTick_Init();
    TIM1_PWM_Init();
    ITR_Init();
    I2C_Init_HW();
    UART_Init();

    UART_SendString("UART OK\r\n");
    if (MPU_Init() != 0) {
        UART_SendString("MPU NOT FOUND\r\n");
        while (1);
    }

    UART_SendString("MPU OK\r\n");

    MPU_Calibrate();
    UART_SendString("MPU CALIBRATED\r\n");
    //moveStraight(50.0f);
    //turnByAngle(90.0f);
    //moveStraight(50.0f);
    //turnByAngle(-24.0f);
    
    while (1)
    {
        if (status_button)
        {
            delay_ms(1000);
            MPU_Debug_Print();

            turnByAngle(23.5f);
            MPU_Debug_Print();
            // moveStraight(60.0f);
            delay_ms(2000);
            MPU_Debug_Print();
            // turnByAngle(-120.0f);
            // delay_ms(1000);
            // moveStraight(60.0f);
        
            // delay_ms(1000);
            // turnByAngle(-24.0f);
            // delay_ms(1000);

            // moveStraight(50.0f);
            // delay_ms(1000);
            // turnByAngle(72.0f);
            // delay_ms(1000);

            // moveStraight(50.0f);
            // delay_ms(1000);
            // turnByAngle(72.0f);
            // delay_ms(1000);

            // moveStraight(50.0f);
            // delay_ms(1000);
            // turnByAngle(72.0f);
            // delay_ms(1000);

            // moveStraight(50.0f);
            // delay_ms(1000);
            // turnByAngle(72.0f);
            // delay_ms(1000);

            // moveStraight(50.0f);
            // delay_ms(1000);
            // turnByAngle(-24.0f);
            // delay_ms(1000);

            // moveStraight(60.0f);
            // delay_ms(1000);
            // turnByAngle(-120.0f);
            // delay_ms(1000);
            // delay_ms(2500);
        }
        
    }
}

