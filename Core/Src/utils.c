#include "init.h"
#include "utils.h"
#include "mpu_i2c.h"
#include <math.h>
#include <stdlib.h>

/* ===================== ПАРАМЕТРЫ ===================== */

#define WHEEL_DIAMETER 66.5         // диаметр колеса в мм
#define PULSES_PER_REVOLUTION 40    // количество импульсов на оборот энкодера
#define WHEEL_BASE 225.0          // расстояние между колесами в мм

#define CONTROL_PERIOD_MS 10        // период обновления системы в мс
#define POSITION_EPSILON_PULSES  2         // допустимая ошибка позиции в импульсах

#define MAX_PWM 65              // максимальное значение скважности ШИМ
#define MAX_TARGET_SPEED 65         // максимальная желаемая скорость в мм/с
#define SPEED_BUFFER_SIZE 10        // размер буфера скорости для усреднения

float wheelCircumference = 3.14159f * WHEEL_DIAMETER;       // длина окружности колеса в мм
float robotCircumference = 3.14159f * WHEEL_BASE;       // длина окружности робота при повороте на 360 градусов в мм

/* ===================== ЭНКОДЕРЫ ===================== */

extern uint16_t left_encoder_ticks;     // количество тиков левого энкодера
extern uint16_t right_encoder_ticks;    // количество тиков правого энкодера
extern uint32_t millis_counter;         // системное время в мс
extern volatile uint32_t sys_tick;      // системный тик

/* ===================== ВРЕМЯ ===================== */

uint32_t millis(void)       // получение времени в мс
{
    return millis_counter;
}

void delay_ms(uint32_t ms)      // задержка в мс
{
    uint32_t start = sys_tick;
    while ((uint32_t)(sys_tick - start) < ms);
}

/* ===================== ВСПОМОГАТЕЛЬНЫЕ ===================== */

float constrainf(float x, float min, float max)     // ограничение значения float
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* ===================== PWM ===================== */

void Set_PWM_Right_DutyCycle(uint8_t percent)   // установка скважности ШИМ для правого мотора    
{
    if (percent > 100) percent = 100;
    MODIFY_REG(TIM1->CCR1, TIM_CCR1_CCR1_Msk,(uint16_t)(TIM1->ARR * percent) / 100);
}

void Set_PWM_Left_DutyCycle(uint8_t percent)    // установка скважности ШИМ для левого мотора
{
    if (percent > 100) percent = 100;
    MODIFY_REG(TIM1->CCR2, TIM_CCR2_CCR2_Msk,(TIM1->ARR * percent) / 100);
}

/* ===================== НАПРАВЛЕНИЕ ===================== */

void left_wheel_direction(uint8_t dir)  // установка направления вращения левого колеса
{
    if (dir == 0) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BR6;     //остановка
    if (dir == 1) GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BR6;     //вперед
    if (dir == 2) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BS6;     //назад
}

void right_wheel_direction(uint8_t dir)
{
    if (dir == 0) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR7;      //остановка
    if (dir == 1) GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR7;      //вперед
    if (dir == 2) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS7;      //назад
}

void stopMotors(void)       // остановка обоих моторов
{
    left_wheel_direction(0);
    right_wheel_direction(0);
    Set_PWM_Left_DutyCycle(0);
    Set_PWM_Right_DutyCycle(0);
}

/* ===================== PID ===================== */

typedef struct {
    float kp, ki, kd;       // PID коэффициенты
    float integral;         // интегральная составляющая
    float lastError;        // последнее значение ошибки
} PID_t;

float PID_Update(PID_t *pid, float error, float dt)     // обновление PID контроллера
{
    if (dt < 0.001f) dt = 0.001f;
    pid->integral += error * dt;        // интегральная часть
    pid->integral = constrainf(pid->integral, -100, 100);   // защита от интегрального насыщения
    float d = (error - pid->lastError) / dt;        // дифференциальная часть (скорость изменения ошибки)
    pid->lastError = error;             // сохраняю текущую ошибку

    return pid->kp * error + pid->ki * pid->integral + pid->kd * d;     // возвращаю управляющее воздействие
}

void reset_PID(PID_t *pidLeft, PID_t *pidRight, PID_t *pidPos, PID_t *pidYaw)  // сброс интегральной и дифференциальной частей PID контроллеров
{
    pidLeft->integral = pidLeft->lastError = 0;
    pidRight->integral = pidRight->lastError = 0;
    pidPos->integral = pidPos->lastError = 0;
    pidYaw->integral = pidYaw->lastError = 0;
}

/* ===================== PID НАСТРОЙКИ ===================== */

PID_t pidPosition   = { 2.0f, 0.0f, 0.0f, 0, 0 };       //PID для управления позицией
PID_t pidSpeedLeft  = { 10.0f, 0.5f, 2.0f, 0, 0 };      //PID для управления скоростью левого колеса
PID_t pidSpeedRight = { 10.0f, 0.5f, 2.0f, 0, 0 };      //PID для управления скоростью правого колеса
PID_t pidYaw        = { 3.0f, 0.0f, 0.8f, 0, 0 };       //PID для управления углом поворота

/* ===================== ДВИЖЕНИЕ ===================== */

typedef struct {        // буфер скорости для усреднения
    float values[SPEED_BUFFER_SIZE];        // значения скорости
    int index;          // текущий индекс
    int count;          // количество реально записанных значений
    float sum;          // сумма значений
} SpeedBuffer;

void SpeedBuffer_Init(SpeedBuffer *b)       // инициализация буфера скорости
{
    b->index = b->count = 0;
    b->sum = 0.0f;
    for (int i = 0; i < SPEED_BUFFER_SIZE; i++)
        b->values[i] = 0.0f;

}

void SpeedBuffer_Add(SpeedBuffer *b, float v)       // добавление значения скорости в буфер
{
    if (b->count >= SPEED_BUFFER_SIZE)
        b->sum -= b->values[b->index];

    b->values[b->index] = v;
    b->sum += v;

    b->index = (b->index + 1) % SPEED_BUFFER_SIZE;
    if (b->count < SPEED_BUFFER_SIZE)
        b->count++;
}

float SpeedBuffer_Get(SpeedBuffer *b)       // получение усредненного значения скорости из буфера
{
    return (b->count == 0) ? 0.0f : b->sum / b->count;
}

/* ===================== ОСНОВНОЙ ЦИКЛ ДВИЖЕНИЯ ===================== */

void motionLoop(uint8_t dirLeft, uint8_t dirRight, uint32_t targetPulses)
{
    left_encoder_ticks = right_encoder_ticks = 0;       // сброс энкодеров

    MPU_Calibrate();        // калибровка гироскопа
    float desiredYaw = MPU_GetYaw();        // сохранение текущего угла

    SpeedBuffer bufL, bufR;     // буферы скорости для левого и правого колеса
    SpeedBuffer_Init(&bufL);    // инициализация буфера левого колеса
    SpeedBuffer_Init(&bufR);    // инициализация буфера правого колеса

    int32_t prevLeft = 0, prevRight = 0;
    uint32_t lastTime = millis();   // время последнего обновления

    while (1)
    {
        uint32_t now = millis();    // текущее время
        if (now - lastTime < CONTROL_PERIOD_MS) // проверка периода обновления
            continue;

        float dt = (now - lastTime) / 1000.0f;  // вычисление дельты времени в секундах
        lastTime = now;     // обновление времени

        MPU_Update();       // обновление данных MPU6050
        float yaw = MPU_GetYaw();   //получение текущего угла

        int32_t leftPath = left_encoder_ticks;      // получение пройденного пути левого колеса
        int32_t rightPath = right_encoder_ticks;    // получение пройденного пути правого колеса

        int32_t avgPath = (leftPath + rightPath) / 2;   // вычисление среднего пройденного пути
        int32_t posError = targetPulses - avgPath;      // вычисление ошибки позиции
        if (labs(posError) <= POSITION_EPSILON_PULSES)  // проверка достижения цели
            break;

        float speedL = (leftPath - prevLeft) / dt;
        float speedR = (rightPath - prevRight) / dt;
        prevLeft = leftPath;
        prevRight = rightPath;

        SpeedBuffer_Add(&bufL, speedL);
        SpeedBuffer_Add(&bufR, speedR);

        speedL = SpeedBuffer_Get(&bufL);
        speedR = SpeedBuffer_Get(&bufR);
        
        float targetSpeed = constrainf(     // вычисление желаемой скорости на основе позиции
            PID_Update(&pidPosition, posError, dt),     //
            -MAX_TARGET_SPEED, MAX_TARGET_SPEED         //
        );

        float pwmL = PID_Update(&pidSpeedLeft,  targetSpeed - speedL, dt);  // вычисление управляющего воздействия для левого колеса
        float pwmR = PID_Update(&pidSpeedRight, targetSpeed - speedR, dt);  // вычисление управляющего воздействия для правого колеса

        float yawError = desiredYaw - yaw;  // вычисление ошибки угла
        // нормализация ошибки угла
        if (yawError > 180) yawError -= 360;    
        if (yawError < -180) yawError += 360;

        float yawCorr = PID_Update(&pidYaw, yawError, dt);  // вычисление коррекции по углу

        pwmL -= yawCorr;    // применение коррекции к левому колесу
        pwmR += yawCorr;    // применению коррекции к правому колесу
        // ограничение значения ШИМ
        pwmL = constrainf(pwmL, 0, MAX_PWM);    
        pwmR = constrainf(pwmR, 0, MAX_PWM);    
        // установка направлений и скважности ШИМ
        left_wheel_direction(dirLeft);
        right_wheel_direction(dirRight);
        Set_PWM_Left_DutyCycle((uint8_t)pwmL);
        Set_PWM_Right_DutyCycle((uint8_t)pwmR);
    }

    reset_PID(&pidSpeedLeft, &pidSpeedRight, &pidPosition, &pidYaw);    // сброс PID контроллеров после завершения движения
    stopMotors();
}

/* ===================== API ===================== */

uint32_t calculatePulsesForDistance(float cm)   // расчет количества импульсов для заданного расстояния в см
{
    return (uint32_t)((cm * 10.0f / wheelCircumference) * PULSES_PER_REVOLUTION);
}

uint32_t calculatePulsesForAngle(float deg) // расчет количества импульсов для заданного угла поворота в градусах
{
    float arc = (deg / 360.0f) * robotCircumference;
    return (uint32_t)((arc / wheelCircumference) * PULSES_PER_REVOLUTION);
}

void moveForward(float cm)      // движение вперед на заданное расстояние в см
{
    motionLoop(1, 1, calculatePulsesForDistance(cm));
}

void turnLeft(float deg)        // поворот влево на заданный угол в градусах
{
    MPU_Calibrate();
    while (fabs(MPU_GetYaw()) < deg) {
        MPU_Update();
        left_wheel_direction(2);
        right_wheel_direction(1);
        Set_PWM_Left_DutyCycle(50);
        Set_PWM_Right_DutyCycle(50);
    }
    stopMotors();
}

void turnRight(float deg)       // поворот вправо на заданный угол в градусах
{
    MPU_Calibrate();
    while (fabs(MPU_GetYaw()) < deg) {
        MPU_Update();
        left_wheel_direction(1);
        right_wheel_direction(2);
        Set_PWM_Left_DutyCycle(50);
        Set_PWM_Right_DutyCycle(50);
    }
    stopMotors();
}
