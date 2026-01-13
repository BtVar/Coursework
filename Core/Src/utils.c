#include "init.h"
#include "utils.h"
#include <math.h>
#include <stdlib.h>

/* ===================== ПАРАМЕТРЫ ===================== */

#define WHEEL_DIAMETER 66.5
#define PULSES_PER_REVOLUTION 40
#define WHEEL_BASE 225

#define CONTROL_PERIOD_MS 10
#define POSITION_EPSILON  1

#define MAX_PWM 65
#define MAX_TARGET_SPEED 65
#define SPEED_BUFFER_SIZE 10  // Размер буфера для усреднения скорости

float wheelCircumference = 3.14159f * WHEEL_DIAMETER;
float robotCircumference = 3.14159f * WHEEL_BASE;

/* ===================== ЭНКОДЕРЫ ===================== */

extern uint16_t left_encoder_ticks;
extern uint16_t right_encoder_ticks;
extern uint32_t millis_counter;
extern volatile uint32_t sys_tick;

/* ===================== ВРЕМЯ ===================== */

uint32_t millis(void)
{
    return millis_counter;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick;
    while ((uint32_t)(sys_tick - start) < ms);
}

/* ===================== ВСПОМОГАТЕЛЬНЫЕ ===================== */

float constrainf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* ===================== PWM ===================== */

void Set_PWM_Right_DutyCycle(uint8_t percent)
{
    if (percent > 100) percent = 100;
    MODIFY_REG(TIM1->CCR1, TIM_CCR1_CCR1_Msk,
              (uint16_t)(TIM1->ARR * percent) / 100);
}

void Set_PWM_Left_DutyCycle(uint8_t percent)
{
    if (percent > 100) percent = 100;
    MODIFY_REG(TIM1->CCR2, TIM_CCR2_CCR2_Msk,
              (TIM1->ARR * percent) / 100);
}

/* ===================== НАПРАВЛЕНИЕ ===================== */

void left_wheel_direction(uint8_t dir)
{
    if (dir == 0) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BR6;
    if (dir == 1) GPIOB->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BR6;
    if (dir == 2) GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BS6;
}

void right_wheel_direction(uint8_t dir)
{
    if (dir == 0) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR7;
    if (dir == 1) GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR7;
    if (dir == 2) GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS7;
}

void stopMotors(void)
{
    left_wheel_direction(0);
    right_wheel_direction(0);
    Set_PWM_Left_DutyCycle(0);
    Set_PWM_Right_DutyCycle(0);
}

/* ===================== PID ===================== */

typedef struct {
    float kp, ki, kd;
    float integral;
    float lastError;
} PID_t;

float PID_Update(PID_t *pid, float error, float dt)
{
    pid->integral += error * dt;
    pid->integral = constrainf(pid->integral, -100, 100);

    float d = (error - pid->lastError) / dt;
    pid->lastError = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * d;
}

void reset_PID(PID_t *pidLeft, PID_t *pidRight, PID_t *pidPos, PID_t *pidSync)
{
    pidLeft -> integral = 0;
    pidLeft -> lastError = 0;
    pidRight -> integral = 0;
    pidRight -> lastError = 0;
    pidPos -> integral = 0;
    pidPos -> lastError = 0;
    pidSync -> integral = 0;
    pidSync -> lastError = 0;
}


/* ===================== PID НАСТРОЙКИ ===================== */

PID_t pidPosition = { 2.0f, 0.0f, 0.0f, 0, 0 };
PID_t pidSpeedLeft = { 10.0f, 0.5f, 2.0f, 0, 0 };
PID_t pidSpeedRight = { 10.0f, 0.5f, 2.0f, 0, 0 };
// PID_t pidSpeedLeft = { 4.0f, 0.5f, 2.0f, 0, 0 };
// PID_t pidSpeedRight = { 7.0f, 0.5f, 2.0f, 0, 0 };
PID_t pidSync = { 5.0f, 0.0f, 0.5f, 0, 0 };

/* ===================== ДВИЖЕНИЕ ===================== */

// Структура для буферизации скорости
typedef struct {
    float values[SPEED_BUFFER_SIZE];
    float dt_values[SPEED_BUFFER_SIZE];  // Временные интервалы для каждого измерения
    int index;
    int count;
    float sum_values;
    float sum_dt;
} SpeedBuffer;

// Инициализация буфера скорости
void SpeedBuffer_Init(SpeedBuffer *buffer) {
    buffer->index = 0;
    buffer->count = 0;
    buffer->sum_values = 0.0f;
    buffer->sum_dt = 0.0f;
    for (int i = 0; i < SPEED_BUFFER_SIZE; i++) {
        buffer->values[i] = 0.0f;
        buffer->dt_values[i] = 0.0f;
    }
}



// Добавление нового значения в буфер
void SpeedBuffer_Add(SpeedBuffer *buffer, float value, float dt) {
    // Вычитаем старое значение из сумм
    if (buffer->count >= SPEED_BUFFER_SIZE) {
        buffer->sum_values -= buffer->values[buffer->index];
        buffer->sum_dt -= buffer->dt_values[buffer->index];
    }
    
    // Добавляем новое значение
    buffer->values[buffer->index] = value;
    buffer->dt_values[buffer->index] = dt;
    buffer->sum_values += value;
    buffer->sum_dt += dt;
    
    // Обновляем индекс
    buffer->index = (buffer->index + 1) % SPEED_BUFFER_SIZE;
    
    // Обновляем счетчик
    if (buffer->count < SPEED_BUFFER_SIZE) {
        buffer->count++;
    }
}

// Получение усредненной скорости
float SpeedBuffer_GetAverage(SpeedBuffer *buffer) {
    if (buffer->count == 0 || buffer->sum_dt == 0.0f) {
        return 0.0f;
    }
    // Средняя скорость = сумма(скорость*dt) / сумма(dt)
    // Но у нас уже суммы посчитаны
    return buffer->sum_values / buffer->sum_dt;
}

// Глобальные переменные для движения
int32_t posError;
uint32_t currentPulses;
uint32_t Tar;
int32_t leftPath;
int32_t rightPath;
int32_t prevLeftPath = 0;
int32_t prevRightPath = 0;


float targetSpeed;
float speedL;
float speedR;
float pwmL;
float pwmR;

float sync;

// Буферы для усреднения скорости
SpeedBuffer speedBufferLeft;
SpeedBuffer speedBufferRight;

// void motionLoop(uint8_t dirLeft, uint8_t dirRight, uint32_t targetPulses)
// {
//     /* ====== ИНИЦИАЛИЗАЦИЯ ====== */
//     left_encoder_ticks  = 0;
//     right_encoder_ticks = 0;
    
//     Tar = targetPulses;
//     leftPath  = 0;
//     rightPath = 0;
//     prevLeftPath = 0;
//     prevRightPath = 0;
    
//     // Инициализация буферов скорости
//     SpeedBuffer_Init(&speedBufferLeft);
//     SpeedBuffer_Init(&speedBufferRight);


//     // Переменные для контроля времени
//     uint32_t lastTime = millis();

//     while (1)
//     {
//         uint32_t now = millis();
//         if (now - lastTime < CONTROL_PERIOD_MS)
//             continue;

//         float dt = (now - lastTime) / 1000.0f;
//         lastTime = now;

//         /* ====== ЧТЕНИЕ ЭНКОДЕРОВ ====== */
//         leftPath  = (int32_t)left_encoder_ticks;
//         rightPath = (int32_t)right_encoder_ticks;

//         /* ====== ВЫЧИСЛЕНИЕ ПРИРАЩЕНИЙ ====== */
//         float dL = (float)(leftPath - prevLeftPath);
//         float dR = (float)(rightPath - prevRightPath);
        
//         prevLeftPath = leftPath;
//         prevRightPath = rightPath;

//         /* ====== УСРЕДНЕНИЕ СКОРОСТИ ====== */
//         // Текущая мгновенная скорость
        
//         // Добавляем в буферы
//         SpeedBuffer_Add(&speedBufferLeft, dL, dt);   // Передаем приращение и dt
//         SpeedBuffer_Add(&speedBufferRight, dR, dt);
        
//         // Получаем усредненную скорость
//         speedL = SpeedBuffer_GetAverage(&speedBufferLeft);
//         speedR = SpeedBuffer_GetAverage(&speedBufferRight);

//         /* ====== СРЕДНЯЯ ПОЗИЦИЯ ====== */
//         currentPulses = (leftPath + rightPath) / 2;

//         posError = targetPulses - currentPulses;
//         if (labs(posError) <= POSITION_EPSILON)
//             break;

//         /* ====== ПИД ПО ПОЗИЦИИ ====== */
//         targetSpeed =
//             constrainf(
//                 PID_Update(&pidPosition, posError, dt),
//                 -MAX_TARGET_SPEED,
//                 MAX_TARGET_SPEED
//             );

//         //targetSpeed = 50.0f;  // Фиксированная скорость для отладки

//         /* ====== ПИД ПО СКОРОСТИ ====== */
//         float speedErrorL = targetSpeed - speedL;
//         float speedErrorR = targetSpeed - speedR;
        
//         pwmL = PID_Update(&pidSpeedLeft, speedErrorL, dt);
//         pwmR = PID_Update(&pidSpeedRight, speedErrorR, dt);

//         /* ====== СИНХРОНИЗАЦИЯ КОЛЁС ====== */
//         sync = PID_Update(&pidSync, leftPath - rightPath, dt);

//         pwmL -= sync;
//         pwmR += sync;

//         // Корректируем пределы PWM (65 слишком высокий минимум!)
//         pwmL = constrainf(pwmL, 60, MAX_PWM);
//         pwmR = constrainf(pwmR, 60, MAX_PWM);

//         left_wheel_direction(dirLeft);
//         right_wheel_direction(dirRight);

//         Set_PWM_Left_DutyCycle((uint8_t)pwmL);
//         Set_PWM_Right_DutyCycle((uint8_t)pwmR);

//     }
//     speedL = 0;
//     pwmL = 0;

//     speedR = 0;
//     pwmR = 0;

//     sync = 0;
//     targetSpeed = 0;
//     stopMotors();
// }
uint32_t n = 0;



void motionLoop(uint8_t dirLeft, uint8_t dirRight, uint32_t targetPulses)
{
    /* ====== ИНИЦИАЛИЗАЦИЯ ====== */
    left_encoder_ticks  = 0;
    right_encoder_ticks = 0;
    
    Tar = targetPulses;
    leftPath  = 0;
    rightPath = 0;
    prevLeftPath = 0;
    prevRightPath = 0;
    
    // Флаг отключения правого мотора
    bool rightMotorDisabled = false;
    
    // Инициализация буферов скорости
    SpeedBuffer_Init(&speedBufferLeft);
    SpeedBuffer_Init(&speedBufferRight);

    // Переменные для контроля времени
    uint32_t lastTime = millis();

    while (1)
    {
        uint32_t now = millis();
        if (now - lastTime < CONTROL_PERIOD_MS)
            continue;

        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        /* ====== ЧТЕНИЕ ЭНКОДЕРОВ ====== */
        leftPath  = (int32_t)left_encoder_ticks;
        rightPath = (int32_t)right_encoder_ticks;

        /* ====== ВЫЧИСЛЕНИЕ ПРИРАЩЕНИЙ ====== */
        float dL = (float)(leftPath - prevLeftPath);
        float dR = (float)(rightPath - prevRightPath);
        
        prevLeftPath = leftPath;
        prevRightPath = rightPath;

        /* ====== УСРЕДНЕНИЕ СКОРОСТИ ====== */
        // Добавляем в буферы
        SpeedBuffer_Add(&speedBufferLeft, dL, dt);
        
        // Правый мотор обрабатываем только если он не отключен
        if (!rightMotorDisabled) {
            SpeedBuffer_Add(&speedBufferRight, dR, dt);
        }
        
        // Получаем усредненную скорость
        speedL = SpeedBuffer_GetAverage(&speedBufferLeft);
        speedR = rightMotorDisabled ? 0.0f : SpeedBuffer_GetAverage(&speedBufferRight);

        /* ====== СРЕДНЯЯ ПОЗИЦИЯ ====== */
        currentPulses = (leftPath + rightPath) / 2;

        posError = targetPulses - currentPulses;
        
        /* ====== ПРОВЕРКА НА ОТКЛЮЧЕНИЕ ПРАВОГО МОТОРА ====== */
        // Если осталось n тиков до цели и правый мотор еще не отключен
        if (!rightMotorDisabled && posError <= (int32_t)n) {
            rightMotorDisabled = true;
            // Отключаем правый мотор
            Set_PWM_Right_DutyCycle(0);
            // Сбрасываем ПИД правого мотора для предотвращения накопления ошибки
            //PID_Reset(&pidSpeedRight);
        }
        
        // Условие выхода из цикла
        if (labs(posError) <= POSITION_EPSILON)
            break;

        /* ====== ПИД ПО ПОЗИЦИИ ====== */
        targetSpeed =
            constrainf(
                PID_Update(&pidPosition, posError, dt),
                -MAX_TARGET_SPEED,
                MAX_TARGET_SPEED
            );

        //targetSpeed = 65;

        /* ====== ПИД ПО СКОРОСТИ ====== */
        float speedErrorL = targetSpeed - speedL;
        pwmL = PID_Update(&pidSpeedLeft, speedErrorL, dt);
        
        float pwmR_temp = 0.0f;
        if (!rightMotorDisabled) {
            float speedErrorR = targetSpeed - speedR;
            pwmR_temp = PID_Update(&pidSpeedRight, speedErrorR, dt);
        }

        /* ====== СИНХРОНИЗАЦИЯ КОЛЁС ====== */
        // Если правый мотор отключен, синхронизацию не применяем
        float sync = 0.0f;
        if (!rightMotorDisabled) {
            sync = PID_Update(&pidSync, leftPath - rightPath, dt);
            pwmR_temp += sync;
            pwmL -= sync;
        }

        // Корректируем пределы PWM
        pwmL = constrainf(pwmL, 59, MAX_PWM);
        
        // Устанавливаем PWM только для левого мотора, если правый отключен
        left_wheel_direction(dirLeft);
        Set_PWM_Left_DutyCycle((uint8_t)pwmL);
        
        // Управляем правым мотором только если он не отключен
        if (!rightMotorDisabled) {
            pwmR = constrainf(pwmR_temp, 59, MAX_PWM);
            right_wheel_direction(dirRight);
            Set_PWM_Right_DutyCycle((uint8_t)pwmR);
        } else {
            pwmR = 0;
        }
    }
    
    // Завершение работы - остановка всех моторов
    speedL = 0;
    pwmL = 0;
    speedR = 0;
    pwmR = 0;
    sync = 0;
    targetSpeed = 0;
    reset_PID(&pidSpeedLeft, &pidSpeedRight, &pidPosition, &pidSync);
    stopMotors();
}

/* ===================== API ===================== */

uint32_t calculatePulsesForDistance(float cm)
{
    return (uint32_t)((cm * 10.0f / wheelCircumference)* PULSES_PER_REVOLUTION);
}

uint32_t calculatePulsesForAngle(float deg)
{
    float arc = (deg / 360.0f) * robotCircumference;
    return (uint32_t)((arc / wheelCircumference)* PULSES_PER_REVOLUTION);
}

void moveForward(float cm)
{
    motionLoop(1, 1, calculatePulsesForDistance(cm));
}

void turnLeft(float deg)
{
    motionLoop(2, 1, calculatePulsesForAngle(deg));
}

void turnRight(float deg)
{
    motionLoop(1, 2, calculatePulsesForAngle(deg));
}