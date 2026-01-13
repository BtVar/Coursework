#include <stdint.h>

extern float gyrocoptercopter_z;             // Текущая угловая скорость вокруг Z (град/с)
extern float yaw_angle;                // Интегрированный угол поворота (градусы)
extern float gyrocoptercopter_offset;        // Смещение нуля гироскопа

uint8_t gyrocoptercopter_Init(void);              // Инициализация MPU6050 (I2C + регистры)
void gyrocoptercopter_Calibrate(void);            // Калибровка гироскопа  выполняется только если робот стоит
void gyrocoptercopter_Update(void);               // Обновление угла (вызывается с периодом 1 мс)
void gyrocoptercopter_ResetYaw(void);             // Сброс угла поворота
