
#include "init.h" // Подключаем заголовочный файл с объявлениями функций
#include "utils.h"

uint8_t pwm_state_pa8 = 0;
uint8_t pwm_state_pa9 = 0;
volatile uint16_t left_encoder_ticks = 0;
volatile uint16_t right_encoder_ticks = 0;
uint32_t left_delay_counter = 0;
uint32_t right_delay_counter = 0;
volatile uint32_t sys_tick = 0;

uint32_t millis_counter = 0;

uint16_t button_delay_counter = 0;
bool status_button = 0;

int main(void)
{
    RCC_Init();
    GPIO_Init_Ports();
    SysTick_Init();
    TIM1_PWM_Init();
    ITR_Init();
    // Инициализируем тактирование системы (RCC)
    RCC_Init(); // Настраивает тактовую частоту 96 МГц

    // Инициализируем SysTick (опционально, для задержек)
    SysTick_Init(); // Настраивает системный таймер на 1 кГц

    // Инициализируем ШИМ
    TIM1_PWM_Init(); // Настраивает TIM1 для ШИМ на PA8 и PA9

    /*
    left_wheel_direction(2);
    right_wheel_direction(2);
    Set_PWM_Left_DutyCycle(65);
    Set_PWM_Right_DutyCycle(65);
    */

    // moveForward(49); // Двигаемся вперед на 100 см
    // turnLeft(90);
    // turnRight(90);

    // turnLeft(120);
    while (1)
    {
        if (status_button)
        {
            // // moveForward(30);
            // // delay_ms(1000);
            // moveForward(30);
            // //delay_ms(3000);
            // //turnLeft(120);
            // //delay_ms(3000);
            // break;



            // moveForward(60);
            // delay_ms(1000);

            // turnRight(120);
            // delay_ms(1000);
            // moveForward(60);
            // delay_ms(1000);

            // turnRight(24);
            // delay_ms(1000);
            // moveForward(50);
            // delay_ms(1000);

            // turnLeft(72);
            // delay_ms(1000);
            // moveForward(50);
            // delay_ms(1000);

            // turnLeft(72);
            // delay_ms(1000);
            // moveForward(50);
            // delay_ms(1000);

            // turnLeft(72);
            // delay_ms(1000);
            // moveForward(50);
            // delay_ms(1000);

            // turnLeft(72);
            // delay_ms(1000);
            // moveForward(50);
            // delay_ms(1000);

            // turnRight(24);
            // delay_ms(1000);
            // moveForward(60);
            // delay_ms(1000);

            // turnRight(120);
            // delay_ms(5000);



            // turnLeft(24);
            // delay_ms(5000);

            // moveForward(50);
            // delay_ms(1500);
            // moveForward(60);
            // delay_ms(1500);moveForward(50);
            // delay_ms(1500);
            // moveForward(30);
            // delay_ms(1500);
            // Set_PWM_Left_DutyCycle(65);
            // //Set_PWM_Right_DutyCycle(65);
            // left_wheel_direction(1);
            // //right_wheel_direction(1);
            // delay_ms(500);
           
            //right_wheel_direction(0);
            //Set_PWM_Right_DutyCycle(0);
            //delay_ms(30);
            // left_wheel_direction(0);
            // Set_PWM_Left_DutyCycle(0);
            // delay_ms(3000);
            
            // turnRight(120);
            // delay_ms(3000);
            // turnLeft(120);
            // delay_ms(3000);

            moveForward(20.9);
            delay_ms(3000);
            
        }
    }
}

/*#include "uart_debug.h"
#include "mpu_i2c.h"

int main(void)
{
    UART_Init();

    if (!MPU_Init()) {
        UART_SendString("MPU NOT FOUND\r\n");
        while (1);
    }

    UART_SendString("MPU OK\r\n");

    MPU_Calibrate();
    UART_SendString("MPU CALIBRATED\r\n");

    while (1)
    {
        MPU_Update();
        MPU_Debug_Print();
    }
}
*/
