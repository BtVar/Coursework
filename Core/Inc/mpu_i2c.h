#include <stdint.h>
#include <stdbool.h>
#include "stm32f411xe.h"

#define MPU_ADDR            0x68  //  регистр адреса устройства MPU6050
#define MPU_WHO_AM_I        0x75    //ренистр идентификации устройства
#define MPU_PWR_MGMT_1      0x6B    //регистр управления питанием
#define MPU_SMPLRT_DIV      0x19    // регистр делителя частоты выборки
#define MPU_CONFIG          0x1A    // регистр конфигурации цифрового фильтра
#define MPU_GYRO_CONFIG     0x1B    //регистр конфигурации гироскопа (настройка диапазона гироскопа)
#define MPU_ACCEL_CONFIG    0x1C    // регистр конфигурации акселерометра (настройка диапазона акселерометра)

#define MPU_ACCEL_XOUT_H    0x3B    // регистр старшего байта данных акселерометра по оси X
#define MPU_GYRO_ZOUT_H     0x47    // регистр старшего байта данных гироскопа по оси Z

#define GYRO_SCALE_250DPS   131.0f  //коэффициент перевода сырых данных с гироскопа
#define ACCEL_SCALE_2G      16384.0f    // коэффициент перевода сырых данных с акселерометра

typedef struct {
    float yaw;          //интегрированный угол
    float yaw_rate;     // текущая угловая скорость
    float accel_x;      // текущие ускорения по оси x
    float accel_y;      // текущие ускорения по оси y
    float accel_z;      // текущие ускорения по оси z
} MPU_t;

/* ===== API ===== */
bool MPU_Init(void);
void MPU_Calibrate(void);
void MPU_Update(void);

float MPU_GetYaw(void);
float MPU_GetYawRate(void);