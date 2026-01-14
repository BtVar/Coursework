#include "mpu_i2c.h"
#include "uart_debug.h"
#include <stdio.h>
extern uint32_t millis_counter; // переменная времени

static MPU_t mpu;               // структура данных MPU6050
static float gyro_offset_z = 0.0f;      // смещение гироскопа по оси Z
static uint32_t last_time = 0;          // время последнего обновления

#define I2C_TIMEOUT 10000       // таймаут ожидания флага I2C

static int I2C_Wait(volatile uint32_t *reg, uint32_t flag)     
{
    uint32_t t = I2C_TIMEOUT;  // таймаут
    while(((*reg) & flag) == 0 && --t);    // ожидание флага
    return t ? 0 : -1;          // возврат ошибки при таймауте
}

static void I2C_Init_HW(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;        //включаю тактирование порта B
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;         //включаю тактирование I2C1

    /* PB8, PB9 AF4 */
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);   //очищаю настройку пинов PB8 и PB9
    GPIOB->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);       // Установка альтернативной функции для PB8 и PB9

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;     // Установка открытого стока для PB8 и PB9
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;      //установка высокой скорости для PB8 и PB9

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);       // Очистка настроек подтягивающих резисторов для PB8 и PB9
    GPIOB->PUPDR |=  GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;     //включение подтягивающих резисторов для PB8 и PB9

    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);       // Устанавливаю альтернативной функции AF4 (I2C1) для PB8 и PB9

    /* reset */
    I2C1->CR1 |= I2C_CR1_SWRST;     //сброс I2C1
    I2C1->CR1 &= ~I2C_CR1_SWRST;        //снятие сброса I2C1

    /* APB1 = 48 MHz */
    I2C1->CR2 = 48;               //установка частоты тактирования I2C1
    I2C1->CCR = 240;            //установка скорости 100 кГц
    I2C1->TRISE = 49;           //установка максимального времени подъема

    I2C1->CR1 |= I2C_CR1_PE;            //включение I2C1
}

static int I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;         //генерация стартового условия
    return I2C_Wait(&I2C1->SR1, I2C_SR1_SB);    //ожидание установки флага SB
}

static void I2C_Stop(void)      //генерация стоп условия
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static int I2C_Address(uint8_t addr)        //адресация ведомого устройства
{
    I2C1->DR = addr;
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_ADDR)) return -1;
    (void)(I2C1->SR1 | I2C1->SR2);
    return 0;
}

static int I2C_Write(uint8_t data)    // запись байта данных
{
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_TXE)) return -1;
    I2C1->DR = data;
    if (I2C_Wait(&I2C1->SR1, I2C_SR1_BTF)) return -1;
    return 0;
}

static int I2C_Read(uint8_t *buf, uint8_t len)  // чтение принятого байта
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

static int MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len)     // чтение регистра MPU6050
{
    if (I2C_Start()) return -1;         // генерация стартового условия
    if (I2C_Address(MPU_ADDR << 1)) return -1;// адресация устройства на запись
    if (I2C_Write(reg)) return -1;

    if (I2C_Start()) return -1;
    if (I2C_Address((MPU_ADDR << 1) | 1)) return -1;

    return I2C_Read(buf, len);
}

static int MPU_Write(uint8_t reg, uint8_t data) // запись регистра MPU6050
{
    if (I2C_Start()) return -1;
    if (I2C_Address(MPU_ADDR << 1)) return -1;
    if (I2C_Write(reg)) return -1;
    if (I2C_Write(data)) return -1;
    I2C_Stop();
    return 0;
}

// API для MPU6050
bool MPU_Init(void)// инициализация MPU6050
{
    I2C_Init_HW();

    uint8_t who;
    MPU_Read(MPU_WHO_AM_I, &who, 1);

    /* защита от подделки */
    if (who != 0x68 && who != 0x98)
        return false;

    MPU_Write(MPU_PWR_MGMT_1, 0x00);
    MPU_Write(MPU_SMPLRT_DIV, 0x00);
    MPU_Write(MPU_CONFIG, 0x01);
    MPU_Write(MPU_GYRO_CONFIG, 0x00);
    MPU_Write(MPU_ACCEL_CONFIG, 0x00);

    last_time = millis_counter;
    mpu.yaw = 0;

    return true;
}

void MPU_Calibrate(void)        // калибровка гироскопа MPU6050
{
    float sum = 0;
    for (int i = 0; i < 1000; i++) {
        uint8_t d[2];
        MPU_Read(MPU_GYRO_ZOUT_H, d, 2);
        int16_t raw = (d[0] << 8) | d[1];
        sum += raw / GYRO_SCALE_250DPS;
    }
    gyro_offset_z = sum / 1000.0f;
}

void MPU_Update(void)       // обновление данных MPU6050
{
    uint32_t now = millis_counter;
    float dt = (now - last_time) / 1000.0f;
    last_time = now;

    uint8_t g[2];
    MPU_Read(MPU_GYRO_ZOUT_H, g, 2);

    int16_t raw = (g[0] << 8) | g[1];
    float rate = raw / GYRO_SCALE_250DPS - gyro_offset_z;

    mpu.yaw_rate = rate;
    mpu.yaw += rate * dt;

    if (mpu.yaw > 180) mpu.yaw -= 360;
    if (mpu.yaw < -180) mpu.yaw += 360;
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

    if ((millis_counter - last_print) < 500)
        return;

    last_print = millis_counter;

    char buf[128];
    snprintf(buf, sizeof(buf),"Yaw: %7.2f deg | Rate: %7.2f deg/s\r\n",mpu.yaw, mpu.yaw_rate);

    UART_SendString(buf);
}
