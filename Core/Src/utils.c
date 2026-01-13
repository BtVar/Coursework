#include "utils.h"
#include "gyrocopter.h"
#include <math.h>

#define MAX_PWM 65              //ограничение максимального значение ШИМ в % до 65
#define CONTROL_PERIOD_MS 10    //период управления в миллисекундах (100 раз в секунду)
#define POSITION_EPSILON 1     //погрешность по положению в тиках энкодера (если робот не доехал меньше чем на 1 импульс цель достигнута)

extern uint16_t left_encoder_ticks;     
extern uint16_t right_encoder_ticks;
extern uint32_t sys_tick;

PID_t pidYaw = {3.5f, 0.05f, 0.8f, 0, 0};  //Настройка для курса робота

float PID_Update(PID_t *pid, float error, float dt) //Принимаю указатель на PID структуру, текущую ошибку, шаг времени
{
    pid->integral += error * dt;        //накопление интегральной ошибки
    pid->integral = constrainf(pid->integral, -100, 100);       //ограничиваю интервал чтобы не ыло неустойчиовсти
    float d = (error - pid->lastError) / dt;        // производная ошибки для демпфирования
    pid->lastError = error;         // текущая ошибка для следующего шага
    return pid->kp * error + pid->ki * pid->integral + pid->kd * d; // возвращаю значение ПИД регулятора
}

void motionLoop(uint8_t dirLeft, uint8_t dirRight, uint32_t targetPulses)// Функция движения робота с поддержанием курса
{
    left_encoder_ticks = 0;         // сброс энкодеров
    right_encoder_ticks = 0;
    gyrocopter_ResetYaw();         // сброс угла курса 

    uint32_t lastTime = millis();// момент начала движения для расчета дельты времени

    while (1)
    {
        uint32_t now = millis();    // считываю текущее время
        if (now - lastTime < CONTROL_PERIOD_MS) continue;   // жду пока не пройдет период управления (< 10 мс - скип итерации)

        float dt = (now - lastTime) / 1000.0f;      // шан времени в секундах
        lastTime = now;                             // обновляю момент времени

        int32_t pos = (left_encoder_ticks + right_encoder_ticks) / 2;   // вычисляю среднее пройденное расстояние по энкодерам
        if (abs(targetPulses - pos) < POSITION_EPSILON) break;      //если достигнута цель по положению - выход из цикла

        float yaw_err = -yaw_angle;                     // коррекция по курсу - стремимся к 0 градусам
        float corr = PID_Update(&pidYaw, yaw_err, dt);  // вычисляю коррекцию PID-а по курсу
        corr = constrainf(corr, -15, 15);               // ограничиваю коррекцию чтобы не было резких поворотов

        left_wheel_direction(dirLeft);                  // задаю направление движения колес
        right_wheel_direction(dirRight);                
        Set_PWM_Left_DutyCycle(60 + corr);              // задаю ШИМ с учетом коррекции по курсу
        Set_PWM_Right_DutyCycle(60 - corr);
    }

    stopMotors();                         // остановка моторов по достижении цели
}
// Дальше и так понятно
float constrainf(float x, float min, float max)     // ограничение значения в диапазоне
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

uint32_t millis(void)       // возвращает время в миллисекундах с момента старта системы
{
    return sys_tick;
}

void delay_ms(uint32_t ms)      // задержка в миллисекундах
{
    uint32_t start = sys_tick;
    while (sys_tick - start < ms);
}
