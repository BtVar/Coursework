#include "init.h"
#include "utils.h"
#include "gyrocopter.h"

int main(void)
{
    RCC_Init();
    GPIO_Init_Ports();
    SysTick_Init();
    TIM1_PWM_Init();
    ITR_Init();

    gyrocopter_Init();
    gyrocopter_Calibrate();

    while (1)
    {
        motionLoop(1, 1, calculatePulsesForDistance(60));
        motionLoop(2, 1, calculatePulsesForAngle(120));
    }
}
