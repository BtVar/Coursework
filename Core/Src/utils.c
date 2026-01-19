#include "utils.h"
#include "init.h"
#include <math.h>
#include <stdlib.h>
#include "mpu_i2c.h"

/* ===================== КОНСТАНТЫ ===================== */

#define WHEEL_DIAMETER 67.0f
#define PULSES_PER_REV 40
#define WHEEL_BASE 225.0f

#define CONTROL_PERIOD_MS 1

#define POSITION_EPS 0
#define ANGLE_EPS 0.1f

#define PWM_MAX 700 // 650
#define PWM_MIN 620 // 620
#define PWM_DEADZONE 600

// РОграничение для интегральной ошибки
#define I_MAX 2000.0f
#define I_MIN -2000.0f

#define PI 3.14159265358979323846f

/* ===================== ГЛОБАЛЬНЫЕ ===================== */

extern volatile uint16_t left_encoder_ticks;
extern volatile uint16_t right_encoder_ticks;
extern volatile uint32_t sys_tick;

/* ===================== ВРЕМЯ ===================== */

uint16_t MAX_PWM = PWM_MAX;
uint16_t MIN_PWM = PWM_MIN;

uint32_t millis(void)
{
    return sys_tick;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms)
        ;
}

/* ===================== ВСПОМОГАТЕЛЬНЫЕ ===================== */

int constrainf(float v, int min, int max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

float normalizeAngle(float a)
{
    while (a > 180)
        a -= 360;
    while (a < -180)
        a += 360;
    return a;
}

/* ===================== PWM ===================== */

void PWM_Left(uint16_t value)
{
    TIM1->CCR2 = constrainf(value, PWM_MIN, PWM_MAX);
}

void PWM_Right(uint16_t value)
{
    TIM1->CCR1 = constrainf(value, PWM_MIN, PWM_MAX);
}

/* ===================== НАПРАВЛЕНИЕ ===================== */

void left_dir(uint8_t d)
{
    if (d == 0)
        GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BR6;
    if (d == 1)
        GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BR6;
    if (d == 2)
        GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BS6;
}

void right_dir(uint8_t d)
{
    if (d == 0)
        GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR7;
    if (d == 1)
        GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR7;
    if (d == 2)
        GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS7;
}

void stopMotors(void)
{
    left_dir(0);
    right_dir(0);
    PWM_Left(0);
    PWM_Right(0);
}

/* ===================== PID ===================== */

float PID(PID_t *p, float err, float dt)
{
    p->i += (err + p->prev)/2.0 * dt;

    // Anti-windup (Что бы интегральная часть не уходила в бесконечность)
    if (p->i > I_MAX)
        p->i = I_MAX;
    if (p->i < I_MIN)
        p->i = I_MIN;

    float d = (err - p->prev) / dt;
    p->prev = err;

    return p->kp * err + p->ki * p->i + p->kd * d;
}

void PID_Reset(PID_t *p)
{
    p->i = 0;
    p->prev = 0;
}

/* ===================== PID НАСТРОЙКИ ===================== */
// ПИДЫ ДЛЯ ПРЯМОЙ ЕЗДЫ
PID_t pidSpeed = {13.7f, 1.9f, 0.1f}; // Начните с малого Kp
PID_t pidYaw = {250.0f, 0.0f, 8.15f};
PID_t pidSync = {2.0f, 0.0f, 0.0f};

PID_t pidSpeedang = {13.7f, 1.9f, 0.1f}; // Начните с малого Kp
PID_t pidYawang = {27.8f, 0.0f, 1.15f};
PID_t pidSyncang = {10.0f, 1.0f, 0.5f};
/* ===================== ДВИЖЕНИЕ ПРЯМО ===================== */

void moveStraight(float distance_cm)
{
    // Обнуляем ПИД регуляторы
    PID_Reset(&pidSpeed);
    PID_Reset(&pidYaw);
    PID_Reset(&pidSync);

    // Обнуляем энкодеры
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    // Записываем угл в начальный момент времени (для выравнивания в процессе езды)
    MPU_Update();
    float startYaw = MPU_GetYaw();

    // Расчет количества тиков для достижения цели
    uint32_t targetTicks =
        (distance_cm * 10.0f / (2 * PI * WHEEL_DIAMETER)) * PULSES_PER_REV;

    // Запись времени для обновления скорости раз в промежуток времени
    uint32_t lastTime = millis();

    while (1)
    {
        MPU_Debug_Print();
        // Проверяем прошло ли время
        if (millis() - lastTime < CONTROL_PERIOD_MS)
            continue;

        // Записываем сколько времени прошло
        float dt = (millis() - lastTime) / 1000.0f;
        lastTime = millis();

        // Считываем правый и левый энкодер, рассчитываем сколько тиков прошел робот
        uint16_t L = left_encoder_ticks;
        uint16_t R = right_encoder_ticks;
        uint16_t avg = (uint16_t)((L + R) / 2.0f);

        // Считаем ошибку, если она допустима прекращаем выполнение программы
        int32_t distErr = targetTicks - avg;
        if (abs(distErr) <= POSITION_EPS)
            break;

        // Использование ПИД регуляторов для корректировки скорости колес
        // Установление базовой скорости (на основе расстояний, т.е. чем дальше от желаемой точки больше скорость, при приближении уменьшается)
        float basePWM = PID(&pidSpeed, distErr, dt);
        MPU_Update();
        // ПИД настройки движения по заданной прямой, то есть при отклонении по гироскопу выравнивает движение
        float yawCorr = PID(&pidYaw,
                            startYaw - MPU_GetYaw(), dt);
        // ПИД контролирующий одинаковое ли расстояние прошли колеса
        float syncCorr = PID(&pidSync, L - R, dt);

        // Расчет результирующего ШИМ сигнала
        float pwmL = basePWM - yawCorr - syncCorr;
        float pwmR = basePWM + yawCorr + syncCorr;

        // Проверка входит ли он в диапазон возможного ШИМ
        
        pwmL = constrainf(pwmL, PWM_MIN, PWM_MAX);
        pwmR = constrainf(pwmR, PWM_MIN, PWM_MAX);

        // if (avg <= 10)
        // {
        //     pwmL = constrainf(pwmL, PWM_MIN, 670);
        //     pwmR = constrainf(pwmR, PWM_MIN, 670);
        // }
        // Если ШИМ меньше того при котором робот начинает медленно двигаться, то задаем минимальный ШИМ для движения
        // if (pwmL > 0 && pwmL < PWM_DEADZONE)
        //     pwmL = PWM_DEADZONE;
        // if (pwmR > 0 && pwmR < PWM_DEADZONE)
        //     pwmR = PWM_DEADZONE;

        // Задаем направления и скорости моторов

        left_dir(1);
        right_dir(1);
        PWM_Left(pwmL);
        PWM_Right(pwmR);
    }

    // Останавливаем моторы робота
    stopMotors();
    MPU_Update();
    // turnByAngle(startYaw - MPU_GetYaw());
}



void moveStraight2(float distance_cm)
{

    // Обнуляем ПИД регуляторы
    PID_Reset(&pidSpeed);
    PID_Reset(&pidYaw);
    PID_Reset(&pidSync);

    // Обнуляем энкодеры
    __disable_irq();
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    __enable_irq();

    // Записываем угл в начальный момент времени (для выравнивания в процессе езды)
    MPU_Update();
    float startYaw = MPU_GetYaw();

    // Расчет количества тиков для достижения цели
    uint32_t targetTicks =
        (distance_cm * 10.0f / (2 * PI * WHEEL_DIAMETER)) * PULSES_PER_REV;

    // Запись времени для обновления скорости раз в промежуток времени
    uint32_t lastTime = millis();

    while (1)
    {
        MPU_Debug_Print();
        // Проверяем прошло ли время
        if (millis() - lastTime < CONTROL_PERIOD_MS)
            continue;

        // Считываем правый и левый энкодер, рассчитываем сколько тиков прошел робот
        __disable_irq();
        uint16_t L = left_encoder_ticks;
        uint16_t R = right_encoder_ticks;
        __enable_irq();
        
        uint16_t avg = (uint16_t)((L + R) / 2.0f);

        int32_t distErr = targetTicks - avg;

        float basePWM = 630;

        if (distErr < 150)
        {
            basePWM = 620;
        }
        else if (distErr < 50)
        {
            basePWM = 615;
        }

        if (abs(distErr) <= POSITION_EPS)
        {
            MAX_PWM = PWM_MAX;
            break;
        }

        // Записываем сколько времени прошло
        float dt = (millis() - lastTime) / 1000.0f;
        lastTime = millis();

        // Использование ПИД регуляторов для корректировки скорости колес
        // Установление базовой скорости (на основе расстояний, т.е. чем дальше от желаемой точки больше скорость, при приближении уменьшается)
        


        // ПИД настройки движения по заданной прямой, то есть при отклонении по гироскопу выравнивает движение
        MPU_Update();
        float yawCorr = PID(&pidYaw,
                            startYaw - MPU_GetYaw(), dt);
        // ПИД контролирующий одинаковое ли расстояние прошли колеса
        float syncCorr = PID(&pidSync, L - R, dt);

        // Расчет результирующего ШИМ сигнала
        float pwmL = basePWM - yawCorr - syncCorr;
        float pwmR = basePWM + yawCorr + syncCorr;

        // Проверка входит ли он в диапазон возможного ШИМ
        if(pwmL > pwmR)
        {
            if (pwmL > MAX_PWM)
            {
                float k = pwmL/MAX_PWM;
                pwmL = MAX_PWM;
                pwmR = pwmR/k;
            }
        }
        if(pwmR > pwmL)
        {
            if (pwmR > MAX_PWM)
            {
                float k = pwmR/MAX_PWM;
                pwmR = MAX_PWM;
                pwmL = pwmL/k;
            }
        }
        // pwmL = constrainf(pwmL, MIN_PWM, MAX_PWM);
        // pwmR = constrainf(pwmR, MIN_PWM, MAX_PWM);

        left_dir(1);
        right_dir(1);
        PWM_Left(pwmL);
        PWM_Right(pwmR);
    }

    // Останавливаем моторы робота
    stopMotors();
    delay_ms(200);
    MPU_Update();
    turnByAngle(normalizeAngle(startYaw - MPU_GetYaw()));
}

/* ===================== ПОВОРОТ НА МЕСТЕ ===================== */

void turnByAngle(float angle_deg)
{

    // Обнуляем ПИД регуляторы
    PID_Reset(&pidSpeedang);
    PID_Reset(&pidYawang);
    PID_Reset(&pidSyncang);

    // Обнуляем энкодеры
    __disable_irq();
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    __enable_irq();

    // Рассчитываем желаемый угл
    MPU_Update();
    float targetYaw = MPU_GetYaw() + angle_deg;
    // Запись времени для обновления скорости раз в промежуток времени
    uint32_t lastTime = millis();

    while (1)
    {
        // Проверяем прошло ли время
        if (millis() - lastTime < CONTROL_PERIOD_MS)
            continue;

        // Записываем сколько времени прошло
        float dt = (millis() - lastTime) / 1000.0f;
        lastTime = millis();

        MPU_Update();
        // Считаем ошибку, если она допустима прекращаем выполнение программы
        float angleErr = targetYaw - MPU_GetYaw();
        if (fabs(angleErr) <= ANGLE_EPS)
            break;

        // ПИД по углу с помощью гироскопа
        float omega = PID(&pidYawang, angleErr, dt);
        // Задаем скорость через ошибку по углу (положению) (т.е. при приближении к желаемому углу замедляемся)
        float basePWM = PID(&pidSpeedang, fabs(omega), dt);
        // ПИД для прохождения колесами одинакового расстояния (чтобы робот вращался на месте)
        float sync = PID(&pidSyncang,
                         left_encoder_ticks - right_encoder_ticks,
                         dt);

        // Расчет результирующего ШИМ сигнала
        float pwmL = basePWM - sync;
        float pwmR = basePWM + sync;

        // Проверка входит ли он в диапазон возможного ШИМ
        pwmL = constrainf(pwmL, PWM_MIN, 650);
        pwmR = constrainf(pwmR, PWM_MIN, 650);

        // Если ШИМ меньше того при котором робот начинает медленно двигаться, то задаем минимальный ШИМ для движения
        if (pwmL > 0 && pwmL < PWM_DEADZONE)
            pwmL = PWM_DEADZONE;
        if (pwmR > 0 && pwmR < PWM_DEADZONE)
            pwmR = PWM_DEADZONE;

        // Задаем направления и скорости моторов
        if (omega > 0)
        {
            left_dir(2);
            right_dir(1);
        }
        else
        {
            left_dir(1);
            right_dir(2);
        }

        PWM_Left(pwmL);
        PWM_Right(pwmR);
    }
    // Останавливаем моторы робота
    stopMotors();
}



void Traject()
{
    MPU_Calibrate();
    MPU_Update();
    moveStraight2(57.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(-120.0f );

    MPU_Calibrate();
    moveStraight2(59.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(-23.0f);

    MPU_Calibrate();
    moveStraight2(48.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(71.0f);

    MPU_Calibrate();
    moveStraight2(48.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(71.0f);

    MPU_Calibrate();
    moveStraight2(47.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(70.0f);

    MPU_Calibrate();
    moveStraight2(49.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(70.0f);

    MPU_Calibrate();
    moveStraight2(47.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(-26.0f);

    MPU_Calibrate();
    moveStraight2(57.0f);
    delay_ms(200);

    MPU_Update();
    turnByAngle(-120.0f);

    delay_ms(5000);
}