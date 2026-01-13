#include "gyrocopter.h"
#include "I2C.h"

#define MPU_ADDR 0x68               // I2C адрес микросхемы MPU6050
#define gyrocopter_ZOUT_H 0x47      // Адрес регистра старшего байта угловой скорости по оси Z
#define gyrocopter_SCALE 131.0f     // коэффициент пересчёта сырых данных гироскопа в градусы в секунду там формула есть

float gyrocopter_z = 0;
float yaw_angle = 0;
float gyrocopter_offset = 0;
static float gyrocopter_filt = 0;

uint8_t gyrocopter_Init(void)       // Инициализация гироскопа
{
    MPU_Write(MPU_ADDR, 0x6B, 0x00);    // wake up samuray
    MPU_Write(MPU_ADDR, 0x1A, 0x01);    //настройка фильтра частота среза ~44ГЦ
    MPU_Write(MPU_ADDR, 0x1B, 0x00);    //настраиваю диапазон измерения гироскопа ±250 град/с
    return 0;
}

void gyrocopter_Calibrate(void)     // калибровка гироскопа
{
    float sum = 0;          
    for (int i = 0; i < 1000; i++) {    // делаю 1000 выборок для усреднения
        uint8_t d[2];                   //буфер для данных 16 бит
        MPU_Read(MPU_ADDR, gyrocopter_ZOUT_H, d, 2);        // чтение двух байтов угловой скорости
        sum += ((int16_t)(d[0] << 8 | d[1])) / gyrocopter_SCALE;        // объеденяю байты, перевожу в градусы и добавляю сумму
    }
    gyrocopter_offset = sum / 1000.0f;      // среднее значение смещения гироскопа
}

void gyrocopter_Update(void)        // обновление данных гироскопа (1мс)
{
    uint8_t d[2];                   // те же 2 байта
    MPU_Read(MPU_ADDR, gyrocopter_ZOUT_H, d, 2);        // считываю текущее значение угловой скорости
    float gz = ((int16_t)(d[0] << 8 | d[1])) / gyrocopter_SCALE;    // объединяю байты и перевожу в градусы в секунду
    gz -= gyrocopter_offset;    //смещение нуля

    gyrocopter_filt = 0.9f * gyrocopter_filt + 0.1f * gz;   // филтр для сглаживания и подавления вибрации
    gyrocopter_z = gyrocopter_filt;  // сохраняю отфильтрованное значение
    yaw_angle += gyrocopter_z * 0.001f;         // интегрирование угловой скорости для получения угла (1мс)
        // нормализация угла в диапазон -180..180
    if (yaw_angle > 180) yaw_angle -= 360;
    if (yaw_angle < -180) yaw_angle += 360;
}

void gyrocopter_ResetYaw(void)  // сброс угла рыскания
{
    yaw_angle = 0;
}
