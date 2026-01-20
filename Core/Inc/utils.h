#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>


typedef struct {
    float kp, ki, kd;
    float i;
    float prev;
} PID_t;


float PID(PID_t *p, float err, float dt);

/* ===== МОТОРЫ ===== */
void stopMotors(void);

/* ===== ДВИЖЕНИЕ ===== */
void turnByAngle(float angle_deg);
void moveStraight(float distance_cm);
void PID_Reset(PID_t *p);
float PID(PID_t *p, float err, float dt);
void left_dir(uint8_t d);
void right_dir(uint8_t d);
void PWM_Left(uint16_t value);
void PWM_Right(uint16_t value);
float normalizeAngle(float angle);

/* ===== ВРЕМЯ ===== */
uint32_t millis(void);
void delay_ms(uint32_t ms);

int constrainf(float x, int min, int max);


void Traject();
#endif 
