#include "init.h"
#include "utils.h"

#define WHEEL_DIAMETER 65       // Диаметр колеса в мм
#define PULSES_PER_REVOLUTION 20 // Количество импульсов на один оборот колеса
#define WHEEL_BASE 225 // Расстояние между колесами в мм

// Параметры управления поворотом
int turnSpeed = 50;        // Скорость поворота (0-100)
int moveSpeed = 75;        // Скорость движения прямо (0-100)
float wheelCircumference = 3.14159 * WHEEL_DIAMETER;   // Длина окружности колеса
float robotCircumference = 3.14159 * WHEEL_BASE;   // Длина окружности поворота робота
float robotCircumference;   // Длина окружности поворота робота

// Тики энкодеров
extern uint16_t left_encoder_ticks;
extern uint16_t right_encoder_ticks;

// Переменные для движения
int8_t leftSpeed = 0;
int8_t rightSpeed = 0;

// ПИД параметры для синхронизации моторов при повороте
float kp = 3.0;             // Пропорциональный коэффициент
float ki = 0.5;             // Интегральный коэффициент  
float kd = 0.1;             // Дифференциальный коэффициент

// Переменные ПИД регулятора
float lastError = 0;
float integral = 0;
uint32_t lastTime = 0;

// Параметры для ПИД регулятора
float maxSpeed = 100;         // Максимальная скорость
float minSpeed = 0;          // Минимальная скорость

extern uint32_t millis_counter;


// Сброс показаний энкодеров
void resetEncoders() {
  left_encoder_ticks = 0;
  right_encoder_ticks = 0;
}

// Сброс ПИД регулятора
void resetPID() {
  lastError = 0;
  integral = 0;
  lastTime = millis();
}

// Функция ограничения значения в заданном диапазоне
int constrain(int x, int min, int max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// Функция ограничения значения с плавающей точкой в заданном диапазоне
float constrainf(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// Получение текущего времени в миллисекундах
uint32_t millis()
{
    return millis_counter;
}

//правый ШИМ PA8
void Set_PWM_Right_DutyCycle(uint8_t percent)
{
    if (percent > 100)                                      // Ограничение максимального значения
    {
        percent = 100;
    }
    
    uint32_t ccr1_value = (TIM1->ARR * percent) / 100;      // Перевод процентов в значение CCR1
    MODIFY_REG(TIM1->CCR1, TIM_CCR1_CCR1_Msk, ccr1_value);  // Устанавливаем новое значение в регистр CCR1
}

// левый ШИМ PA9
void Set_PWM_Left_DutyCycle(uint8_t percent)
{
    if (percent > 100)                                      // Ограничение максимального значения
    {
        percent = 100;
    }
    
    uint32_t ccr2_value = (TIM1->ARR * percent) / 100;      // Перевод процентов в значение CCR2
    MODIFY_REG(TIM1->CCR2, TIM_CCR2_CCR2_Msk, ccr2_value);  // Устанавливаем новое значение в регистр CCR2
}


// Настройка драйвера (IN3 и IN4)
 void left_wheel_direction(uint8_t direction)
 {
    if (direction == 0)
    {
        // Остановка колеса
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR10);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR6);
    }
    if (direction == 1)
    {
        // Колесо движется вперед
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS10);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR6);
    }
    if (direction == 2)
    {
        // Колесо движется назад
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR10);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS6);
    }
 }

 // Настройка драйвера (IN1 и IN2)
  void right_wheel_direction(uint8_t direction)
 {
    if (direction == 0)
    {
        // Остановка колеса
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR5);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR8);
    }
    if (direction == 1)
    {
        // Колесо движется вперед
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS5);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR8);
    }
    if (direction == 2)
    {
        // Колесо движется назад
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR5);
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS8);
    }
 }

 void stopMotors() 
 {
    left_wheel_direction(0);
    right_wheel_direction(0);
    Set_PWM_Left_DutyCycle(0);
    Set_PWM_Right_DutyCycle(0);
 }

 void calculatePID(uint16_t leftCount, uint16_t rightCount, int baseSpeed) 
 {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Время в секундах
  
  if (deltaTime <= 0) deltaTime = 0.01; // Защита от деления на ноль
  
  // Ошибка - разность показаний энкодеров
  float error = leftCount - rightCount;
  
  // Пропорциональная составляющая
  float proportional = kp * error;
  
  // Интегральная составляющая
  integral += error * deltaTime;
  // Ограничиваем интегральную составляющую для предотвращения насыщения
  integral = constrainf(integral, -100.f, 100.f);
  float integralTerm = ki * integral;
  
  // Дифференциальная составляющая
  float derivative = (error - lastError) / deltaTime;
  float derivativeTerm = kd * derivative;
  
  // Общая коррекция
  float correction = proportional + integralTerm + derivativeTerm;
  
  // Применяем коррекцию к скоростям
  leftSpeed = constrainf(baseSpeed - correction, minSpeed, maxSpeed);
  rightSpeed = constrainf(baseSpeed + correction, minSpeed, maxSpeed);
  
  // Сохраняем значения для следующей итерации
  lastError = error;
  lastTime = currentTime;
}

// Вычисление необходимого количества импульсов для движения на заданное расстояние
uint32_t calculatePulsesForDistance(float distance_cm) {
  // Количество оборотов колеса для преодоления расстояния
  float wheelRotations = (distance_cm * 10) / wheelCircumference; // distance_cm * 10 для перевода в мм
  
  // Количество импульсов энкодера
  uint32_t pulses = (uint32_t)(wheelRotations * PULSES_PER_REVOLUTION);
  
  return pulses;
}

// Движение вперед на заданное расстояние с ПИД регулятором
void moveForward(float distance_cm) {
  resetEncoders();
  resetPID();
  
  long targetPulses = calculatePulsesForDistance(distance_cm);
  if (targetPulses <= 0) return;
  
  uint32_t lastUpdate = millis();
  // Двигаемся до достижения нужного расстояния
  while (1) 
  {
    uint16_t currentPulses;
    if (left_encoder_ticks > right_encoder_ticks)
        currentPulses = left_encoder_ticks;
    else
        currentPulses = right_encoder_ticks;

    if (currentPulses >= targetPulses)
        break;

    uint32_t now = millis();
    if (now - lastUpdate >= 10) // Обновляем каждые 10 мс
    {
        lastUpdate = now;
        // Вычисляем скорости с ПИД коррекцией для синхронизации моторов
        calculatePID(left_encoder_ticks, right_encoder_ticks, moveSpeed);
    
        // Оба мотора вперед
        left_wheel_direction(1);
        right_wheel_direction(1);
    
        // Применяем скорости с ПИД коррекцией
        Set_PWM_Left_DutyCycle(leftSpeed);
        Set_PWM_Right_DutyCycle(rightSpeed);
    }
  }
  stopMotors();
}

