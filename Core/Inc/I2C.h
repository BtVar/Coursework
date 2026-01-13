#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "stm32f411xe.h"

// --- Регистры MPU6050 (только нужные) ---
#define MPU6050_ADDR        0x68
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B      //Выводит датчик из спящего режима (по умолчанию он спит)
#define SMPLRT_DIV_REG      0x19      //(Digital Low Pass Filter): Устанавливает фильтр на 44 Гц. Это отсекает мелкие вибрации моторов робота, чтобы они не портили данные.
#define CONFIG_REG          0x1A      // DLPF фильтр
#define GYRO_CONFIG_REG     0x1B
#define GYRO_ZOUT_H_REG     0x47      // Только Z ось!

// --- Настройки (оптимально для курса) ---
#define DLPF_44HZ           0x01      // Фильтр 44Hz
#define GYRO_RANGE_250DPS   0x00      // ±250°/с (самый точный)
#define GYRO_SCALE          131.0f    // Масштаб для ±250°/с

// --- Конфигурация I2C ---
void I2C_Config(void);

// --- Основные I2C операции ---
// Все функции теперь возвращают int: 0 = OK, -1 = ошибка / таймаут
int I2C_Start(void);
int I2C_Stop(void);
int I2C_Write(uint8_t data);
int I2C_Address(uint8_t address);

// --- Работа с MPU6050 ---
int MPU_Write(uint8_t dev, uint8_t reg, uint8_t data);
int MPU_Read(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
