#include "utils.h"
#include "init.h"
#include <math.h>
#include <stdlib.h>

/* ===================== КОНСТАНТЫ ===================== */

#define WHEEL_DIAMETER 66.5f
#define PULSES_PER_REV 40
#define WHEEL_BASE     225.0f

#define CONTROL_PERIOD_MS 10

#define POSITION_EPS 2
#define ANGLE_EPS    1.0f

#define PWM_MAX (TIM1->ARR)
#define PWM_MIN 0
#define PWM_DEADZONE 80

// РОграничение для интегральной ошибки
#define I_MAX  1000.0f
#define I_MIN -1000.0f

#define PI 3.14159265358979323846f

/* ===================== ГЛОБАЛЬНЫЕ ===================== */

extern uint16_t left_encoder_ticks;
extern uint16_t right_encoder_ticks;
extern float yaw_angle;
extern uint32_t millis_counter;

/* ===================== ВРЕМЯ ===================== */

uint32_t millis(void)
{
    return millis_counter;
}

/* ===================== ВСПОМОГАТЕЛЬНЫЕ ===================== */

float constrainf(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

float normalizeAngle(float a)
{
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
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
    if (d == 0) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BR6;
    if (d == 1) GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BR6;
    if (d == 2) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BS6;
}

void right_dir(uint8_t d)
{
    if (d == 0) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR7;
    if (d == 1) GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR7;
    if (d == 2) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS7;
}

void stopMotors(void)
{
    left_dir(0);
    right_dir(0);
    PWM_Left(0);
    PWM_Right(0);
}

/* ===================== PID ===================== */

// typedef struct {
//     float kp, ki, kd;
//     float i;
//     float prev;
// } PID_t;

float PID(PID_t *p, float err, float dt)
{
    p->i += err * dt;

    // Anti-windup (Что бы интегральная часть не уходила в бесконечность)
    if (p->i > I_MAX) p->i = I_MAX;
    if (p->i < I_MIN) p->i = I_MIN;

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

PID_t pidSpeed = { 0.8f, 0.0f, 0.4f };
PID_t pidYaw   = { 4.0f, 0.0f, 0.6f };
PID_t pidSync  = { 2.0f, 0.0f, 0.2f };

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
    float startYaw = yaw_angle;

    // Расчет количества тиков для достижения цели
    uint32_t targetTicks =
        (distance_cm * 10.0f / (2 * PI * WHEEL_DIAMETER)) * PULSES_PER_REV;

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

        // Считываем правый и левый энкодер, рассчитываем сколько тиков прошел робот
        int32_t L = left_encoder_ticks;
        int32_t R = right_encoder_ticks;
        int32_t avg = (L + R) / 2;

        // Считаем ошибку, если она допустима прекращаем выполнение программы
        int32_t distErr = targetTicks - avg;
        if (abs(distErr) <= POSITION_EPS)
            break;

        // Использование ПИД регуляторов для корректировки скорости колес
        // Установление базовой скорости (на основе расстояний, т.е. чем дальше от желаемой точки больше скорость, при приближении уменьшается)
        float basePWM = PID(&pidSpeed, distErr, dt);
        // ПИД настройки движения по заданной прямой, то есть при отклонении по гироскопу выравнивает движение
        float yawCorr = PID(&pidYaw,
                            normalizeAngle(startYaw - yaw_angle), dt);
        // ПИД контролирующий одинаковое ли расстояние прошли колеса
        float syncCorr = PID(&pidSync, L - R, dt);

        // Расчет результирующего ШИМ сигнала
        float pwmL = basePWM - yawCorr - syncCorr;
        float pwmR = basePWM + yawCorr + syncCorr;

        // Проверка входит ли он в диапазон возможного ШИМ
        pwmL = constrainf(pwmL, PWM_MIN, PWM_MAX);
        pwmR = constrainf(pwmR, PWM_MIN, PWM_MAX);

        // Если ШИМ меньше того при котором робот начинает медленно двигаться, то задаем минимальный ШИМ для движения
        if (pwmL > 0 && pwmL < PWM_DEADZONE) pwmL = PWM_DEADZONE;
        if (pwmR > 0 && pwmR < PWM_DEADZONE) pwmR = PWM_DEADZONE;

        // Задаем направления и скорости моторов
        left_dir(1);
        right_dir(1);
        PWM_Left(pwmL);
        PWM_Right(pwmR);
    }

    // Останавливаем моторы робота
    stopMotors();
}

/* ===================== ПОВОРОТ НА МЕСТЕ ===================== */

void turnByAngle(float angle_deg)
{
    // Обнуляем ПИД регуляторы
    PID_Reset(&pidSpeed);
    PID_Reset(&pidYaw);
    PID_Reset(&pidSync);

    // Обнуляем энкодеры
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;

    // Рассчитываем желаемый угл
    float targetYaw = normalizeAngle(yaw_angle + angle_deg);
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

        // Считаем ошибку, если она допустима прекращаем выполнение программы
        float angleErr = normalizeAngle(targetYaw - yaw_angle);
        if (fabs(angleErr) <= ANGLE_EPS)
            break;

        // ПИД по углу с помощью гироскопа
        float omega = PID(&pidYaw, angleErr, dt);
        // Задаем скорость через ошибку по углу (положению) (т.е. при приближении к желаемому углу замедляемся)
        float basePWM = PID(&pidSpeed, fabs(omega), dt);
        // ПИД для прохождения колесами одинакового расстояния (чтобы робот вращался на месте)
        float sync = PID(&pidSync,
                         left_encoder_ticks - right_encoder_ticks,
                         dt);

        // Расчет результирующего ШИМ сигнала
        float pwmL = basePWM - sync;
        float pwmR = basePWM + sync;

        // Проверка входит ли он в диапазон возможного ШИМ
        pwmL = constrainf(pwmL, PWM_MIN, PWM_MAX);
        pwmR = constrainf(pwmR, PWM_MIN, PWM_MAX);

        // Если ШИМ меньше того при котором робот начинает медленно двигаться, то задаем минимальный ШИМ для движения
        if (pwmL > 0 && pwmL < PWM_DEADZONE) pwmL = PWM_DEADZONE;
        if (pwmR > 0 && pwmR < PWM_DEADZONE) pwmR = PWM_DEADZONE;

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
