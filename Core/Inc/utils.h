void Set_PWM_Right_DutyCycle(uint8_t percent);
void Set_PWM_Left_DutyCycle(uint8_t percent);
void left_wheel_direction(uint8_t direction);
void right_wheel_direction(uint8_t direction);

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