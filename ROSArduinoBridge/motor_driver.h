/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
#if defined(ESP32)
// Arduino Nano ESP32 Pin Recommendations for L298 Motor Driver:
// - GPIO 1, 3: Reserved for Serial (TX/RX)
// - GPIO 0: Connected to BOOT button, avoid using
// - GPIO 6-11: Connected to internal flash, DO NOT USE
// - GPIO 2: Connected to onboard LED
//
// Good pins for PWM motor control:
// - GPIO 4, 5, 12-19, 21-23, 25-27, 32, 33
//
// Adjust these pin numbers to match your wiring:
#define RIGHT_MOTOR_BACKWARD 8 // Nano D5 / GPIO8 (was D5)
#define LEFT_MOTOR_BACKWARD 7  // Nano D4 / GPIO7 (was D4)
#define RIGHT_MOTOR_FORWARD 10 // Nano D7 / GPIO10 (was D12/GPIO47)
#define LEFT_MOTOR_FORWARD 9   // Nano D6 / GPIO9 (was D13/GPIO48)
#define RIGHT_MOTOR_ENABLE 21  // Nano D10 / GPIO21 (New PWM Channel 1)
#define LEFT_MOTOR_ENABLE 38   // Nano D11 / GPIO38 (New PWM Channel 0)
#else
// Original Arduino Nano pin definitions
#define RIGHT_MOTOR_BACKWARD 5
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 9
#define LEFT_MOTOR_FORWARD 10
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_ENABLE 13
#endif
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
