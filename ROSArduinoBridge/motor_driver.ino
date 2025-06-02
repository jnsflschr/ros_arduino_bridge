/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE

#ifdef POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController()
{
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd)
{
  if (i == LEFT)
    drive.setM1Speed(spd);
  else
    drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController()
{
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd)
{
  if (i == LEFT)
    drive.setM1Speed(spd);
  else
    drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined L298_MOTOR_DRIVER
// Include globals.h for motor direction definitions
#include "globals.h"

// Reference the external motor direction variables
extern MotorDirection leftMotorDirection;
extern MotorDirection rightMotorDirection;

#if defined(ESP32)
// ESP32 specific setup for L298N
// Define PWM channels (0-15) and properties
const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;
const int PWM_FREQ = 5000;    // PWM frequency in Hz
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

void initMotorController()
{
  // Set motor direction pins as OUTPUTS
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Configure PWM channels for motor enable pins
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to motor enable pins
  ledcAttachPin(LEFT_MOTOR_ENABLE, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_ENABLE, RIGHT_PWM_CHANNEL);

  // Ensure motors are initially stopped
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  ledcWrite(LEFT_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);
  leftMotorDirection = STOPPED;
  rightMotorDirection = STOPPED;
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > MAX_PWM) // Use MAX_PWM defined in ROSArduinoBridge.ino
    spd = MAX_PWM;

  if (i == LEFT)
  {
    if (reverse == 0 && spd > 0)
    { // Forward
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      ledcWrite(LEFT_PWM_CHANNEL, spd);
      leftMotorDirection = FORWARD;
    }
    else if (reverse == 1 && spd > 0)
    { // Backward
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
      ledcWrite(LEFT_PWM_CHANNEL, spd);
      leftMotorDirection = BACKWARD;
    }
    else
    { // Stop
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      ledcWrite(LEFT_PWM_CHANNEL, 0);
      leftMotorDirection = STOPPED;
    }
  }
  else /* if (i == RIGHT) */
  {
    if (reverse == 0 && spd > 0)
    { // Forward
      digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      ledcWrite(RIGHT_PWM_CHANNEL, spd);
      rightMotorDirection = FORWARD;
    }
    else if (reverse == 1 && spd > 0)
    { // Backward
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
      ledcWrite(RIGHT_PWM_CHANNEL, spd);
      rightMotorDirection = BACKWARD;
    }
    else
    { // Stop
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      ledcWrite(RIGHT_PWM_CHANNEL, 0);
      rightMotorDirection = STOPPED;
    }
  }
}

#else  // Original AVR code (Non-ESP32)
void initMotorController()
{
  // Set pin modes for AVR
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH); // For L298N, Enable is usually tied to PWM or HIGH
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);  // If not using PWM on enable pins for AVR

  // Ensure motors are initially stopped
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  // For AVR, if Enable pins are not PWM, speed is controlled by PWM on FWD/BCK pins
  // If Enable pins ARE PWM for AVR (less common for basic L298N setups):
  // analogWrite(LEFT_MOTOR_ENABLE, 0);
  // analogWrite(RIGHT_MOTOR_ENABLE, 0);
  leftMotorDirection = STOPPED;
  rightMotorDirection = STOPPED;
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > MAX_PWM) // Use MAX_PWM
    spd = MAX_PWM;

  // Original AVR L298N logic assumes PWM is done on FWD/BCK pins, and Enable is HIGH
  // Or, for some L298N boards, ENABLE pins are PWM inputs.
  // The old code was:
  // if (i == LEFT) {
  //   if (reverse == 0 && spd > 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); leftMotorDirection = FORWARD; }
  //   else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); leftMotorDirection = BACKWARD; }
  //   else { analogWrite(LEFT_MOTOR_FORWARD, 0); analogWrite(LEFT_MOTOR_BACKWARD, 0); leftMotorDirection = STOPPED; }
  // } // ... and similar for RIGHT

  // Assuming standard L298N where IN1/IN2 are direction and ENA is PWM speed for AVR:
  // This part needs to align with how your *specific* AVR L298N setup was intended to work.
  // The old code implies PWM on FORWARD/BACKWARD pins. Let's stick to that for AVR.
  // And ENABLE pins are just kept HIGH.

  if (i == LEFT)
  {
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH); // Ensure enabled if not using PWM on enable
    if (reverse == 0 && spd > 0)
    { // Forward
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW); // Use digitalWrite for 0 PWM
      leftMotorDirection = FORWARD;
    }
    else if (reverse == 1 && spd > 0)
    { // Backward
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      leftMotorDirection = BACKWARD;
    }
    else
    {                                      // Stop
      analogWrite(LEFT_MOTOR_FORWARD, 0);  // Or digitalWrite LOW
      analogWrite(LEFT_MOTOR_BACKWARD, 0); // Or digitalWrite LOW
      leftMotorDirection = STOPPED;
    }
  }
  else /*if (i == RIGHT)*/
  {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH); // Ensure enabled
    if (reverse == 0 && spd > 0)
    { // Forward
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
      rightMotorDirection = FORWARD;
    }
    else if (reverse == 1 && spd > 0)
    { // Backward
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      rightMotorDirection = BACKWARD;
    }
    else
    { // Stop
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
      rightMotorDirection = STOPPED;
    }
  }
}
#endif // ESP32 or AVR

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif
