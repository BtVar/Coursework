#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
/* ===== PID ===== */
typedef struct {
    float kp, ki, kd;
    float integral;
    float lastError;
} PID_t;

float PID_Update(PID_t *pid, float error, float dt);

/* ===== МОТОРЫ ===== */
void Set_PWM_Left_DutyCycle(uint8_t percent);
void Set_PWM_Right_DutyCycle(uint8_t percent);
void left_wheel_direction(uint8_t dir);
void right_wheel_direction(uint8_t dir);
void stopMotors(void);

/* ===== ДВИЖЕНИЕ ===== */
void motionLoop(uint8_t dirLeft,uint8_t dirRight,uint32_t targetPulses);

uint32_t calculatePulsesForDistance(float cm);
uint32_t calculatePulsesForAngle(float deg);

/* ===== ВРЕМЯ ===== */
uint32_t millis(void);
void delay_ms(uint32_t ms);

uint32_t millis();
float constrainf(float x, float min, float max);
int constrain(int x, int min, int max);
void resetPID();
void resetEncoders();
void stopMotors();
void calculatePID(uint16_t leftCount, uint16_t rightCount, int baseSpeed);
void moveForward(float distance_cm);
uint32_t calculatePulsesForDistance(float distance_cm);
uint32_t calculatePulsesForAngle(float angle);
void turnLeft(float angle);
void turnRight(float angle);
void delay_ms(uint32_t ms);
void motionLoop(uint8_t dirLeft, uint8_t dirRight, uint32_t targetPulses);

#endif 
