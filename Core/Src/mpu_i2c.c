#include "mpu_i2c.h"
#include "uart_debug.h"
#include "utils.h"
#include <stdio.h>


MPU_t mpu;               // структура данных MPU6050
float gyro_offset_z = 0.0f;      // смещение гироскопа по оси Z
uint32_t last_time = 0;          // время последнего обновления

static float gz_filtered = 0.0f;
static float yaw_angle = 0.0f;
static uint32_t last_update = 0; 
extern bool status_button;

#define I2C_TIMEOUT 10000       // таймаут ожидания флага I2C

int I2C_Wait(volatile uint32_t *reg, uint32_t flag)     
{
    uint32_t timeout = I2C_TIMEOUT;
    while (!(*reg & flag) && timeout--) { __NOP(); }
    return (timeout == 0) ? -1 : 0;
}

void I2C_Init_HW(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;        //включаю тактирование порта B
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;         //включаю тактирование I2C1

    /* PB8, PB9 AF4 */
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);   //очищаю настройку пинов PB8 и PB9
    GPIOB->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);       // Установка альтернативной функции для PB8 и PB9

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;                     // Установка открытого стока для PB8 и PB9
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;      //установка высокой скорости для PB8 и PB9

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);       // Очистка настроек подтягивающих резисторов для PB8 и PB9
    GPIOB->PUPDR |=  GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;     //включение подтягивающих резисторов для PB8 и PB9

    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4)); // очищаем 
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);       // Устанавливаю альтернативной функции AF4 (I2C1) для PB8 и PB9

    // Reset I2C
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // APB1 = 48 MHz
    I2C1->CR2 &= ~I2C_CR2_FREQ;
    I2C1->CR2 |= 48;

    // 100 kHz
    I2C1->CCR = 240;
    I2C1->TRISE = 49;

    I2C1->CR1 |= I2C_CR1_PE;
}

int I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;         //генерация стартового условия
    return I2C_Wait(&I2C1->SR1, I2C_SR1_SB);    //ожидание установки флага SB
}

int I2C_Stop(void)      // генерация стоп условия
{
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

int I2C_Address(uint8_t addr)        //адресация ведомого устройства
{
    I2C1->DR = addr;
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_ADDR)) return -1;
    volatile uint32_t tmp = I2C1->SR1 | I2C1->SR2;
    (void)tmp;
    return 0;
}

int I2C_Write(uint8_t data)    // запись байта данных
{
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_TXE)) return -1;
    I2C1->DR = data;
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_BTF)) return -1;
    return 0;
}

int I2C_Read(uint8_t *buf, uint8_t len)  // чтение принятого байта
{      
    I2C1->CR1 |= I2C_CR1_ACK;
    for (uint8_t i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C_Stop();
        }
        if (I2C_Wait(&I2C1->SR1, I2C_SR1_RXNE)) return -1;
        buf[i] = I2C1->DR;
    }
    return 0;
}

int MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len)     // чтение регистра MPU6050
{
    if (I2C_Start() != 0) return -1;
    if (I2C_Address(MPU_ADDR << 1)) { I2C_Stop(); return -1; }
    if (I2C_Write(reg)) { I2C_Stop(); return -1; }

    // Повторный запуск для чтения
    if (I2C_Start()) return -1;

    if (len == 1)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;      
        if (I2C_Address((MPU_ADDR << 1) | 1)) { I2C_Stop(); return -1; }

        if (I2C_Wait(&I2C1->SR1, I2C_SR1_RXNE)) { I2C_Stop(); return -1; }
        buf[0] = I2C1->DR;
        I2C_Stop();
    }
    else
    {
        I2C1->CR1 |= I2C_CR1_ACK;
        if (I2C_Address((MPU_ADDR << 1) | 1)) { I2C_Stop(); return -1; }

        for (uint8_t i = 0; i < len; i++)
        {
            if (i == len - 1)
            {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C_Stop();
            }
            if (I2C_Wait(&I2C1->SR1, I2C_SR1_RXNE)) return -1;
            buf[i] = I2C1->DR;
        }
    }
    return 0;
}

int MPU_Write(uint8_t reg, uint8_t data) // запись регистра MPU6050
{
    if (I2C_Start()) return -1;
    if (I2C_Address(MPU_ADDR << 1)) { I2C_Stop(); return -1; }
    if (I2C_Write(reg)) { I2C_Stop(); return -1; }
    if (I2C_Write(data)) { I2C_Stop(); return -1; }
    I2C_Stop();
    return 0;
}


bool MPU_Init(void)// инициализация MPU6050
{


    uint8_t who;
    MPU_Read(MPU_WHO_AM_I, &who, 1);

    /* защита от подделки */
    if (who != 0x68 && who != 0x98)
        return 1;


    // Ждём 100 мс
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < 100);


    MPU_Write(MPU_PWR_MGMT_1, 0x00);
    MPU_Write(MPU_SMPLRT_DIV, 0x00);
    MPU_Write(MPU_CONFIG, 0x01);    // посмотреть подробнее что за частота, лучше использовать 44гц
    MPU_Write(MPU_GYRO_CONFIG, 0x00);



    gz_filtered = 0.0f;
    yaw_angle = 0.0f;

    last_time = millis();

    return 0;
}

void MPU_Calibrate(void)        // калибровка гироскопа MPU6050
{


    float sum = 0.0f;
    uint16_t samples = 0;

    for (int i = 0; i < 1000; i++) 
    {

        uint8_t data_z[2];
        MPU_Read(MPU_GYRO_ZOUT_H, data_z, 2);
        int16_t raw = (int16_t)((data_z[0] << 8) | data_z[1]);
        sum += (raw / GYRO_SCALE_250DPS);
        samples++;

        // Небольшая задержка
        uint32_t wait_start = millis();
        while ((uint32_t)(millis() - wait_start) < 5);  // 1 мс

    }

    if (samples > 0) {
        gyro_offset_z = sum / samples;
    }

    // Сброс фильтра
    gz_filtered = 0.0f;

}

void MPU_Update(void)       // обновление данных MPU6050
{
    // Получаем сырые данные
    uint8_t data_z[2];
    MPU_Read(MPU_GYRO_ZOUT_H, data_z, 2);
    int16_t raw = (int16_t)((data_z[0] << 8) | data_z[1]);

    float rate_raw = (float)(raw / GYRO_SCALE_250DPS) - gyro_offset_z;

    mpu.yaw_rate_raw = rate_raw;

    // Простая фильтрация (скользящее среднее)
    static float buffer[3] = {0.0f};  // 3 последних значения
    static uint8_t idx = 0;
    
    buffer[idx] = rate_raw;
    idx = (idx + 1) % 3;
    
    // Среднее из 3 значений
    gz_filtered = (buffer[0] + buffer[1] + buffer[2]) / 3.0f;


    // Сохраняем отфильтрованное
    mpu.yaw_rate = gz_filtered;

    uint32_t now = millis();
    
    // Вычисляем дельту времени (в секундах)
    float dt = (float)(now - last_update) / 1000.0f;
    
    // Защита от слишком больших dt
    if (dt > 0.1f) dt = 0.1f;  // Если пропустили >100мс, считаем как 1мс

    if (dt <= 0.0f) {
        last_update = now;
        return;
    }
    
    // Интегрируем: угол = угол + скорость * время
    yaw_angle += gz_filtered * dt;
    
    // Нормализация угла в диапазон [-180, 180]
    if (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    else if (yaw_angle < -180.0f) yaw_angle += 360.0f;
    
    // Сохраняем
    mpu.yaw = yaw_angle;
    last_update = now;

}

float MPU_GetYaw(void)      // получение интегрированного угла
{
    return mpu.yaw;
}

float MPU_GetYawRate(void)          // получение угловой скорости
{
    return mpu.yaw_rate;
}

void MPU_Debug_Print(void)          // вывод данных MPU6050 в UART
{
    static uint32_t last_print = 0;
    
    // Выводим раз в 500 мс
    if ((uint32_t)(millis() - last_print) > 1000) {
        last_print = millis();

        char buf[80];
        sprintf(buf, "Yaw: %6.1f° | Rate: %d°/s | Offset: %6.3f\r\n",
                mpu.yaw, status_button, gyro_offset_z);
        
        // Предполагаем, что есть UART_SendString
        for(int i = 0; buf[i]; i++) {
            UART_SendChar(buf[i]);  // Или ваша функция отправки
        }
    }
}
