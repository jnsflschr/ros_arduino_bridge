/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data.

    This version supports both Arduino Nano (AVR) and Arduino Nano ESP32.
    The code automatically detects the platform and uses the appropriate
    implementation.

    When using with Arduino Nano ESP32:
    1. Make sure to adjust pin numbers in encoder_driver.ino and motor_driver.h
    2. Install the ESP32Servo library if using servos
    3. Be aware of the 3.3V logic level (vs 5V on Arduino Nano)

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Define ESP32 to enable ESP32-specific code
#define ESP32

// Define this to enable direct pin reading diagnostics in loop()
// #define DEBUG_ENCODER_PINS // Temporarily disabled

#define USE_BASE // Enable the base controller code
// #undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
// #define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
// #define POLOLU_MC33926

/* The RoboGaia encoder shield */
// #define ROBOGAIA

/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER

/* Transmissive Optical Sensor with Phototransistor Output with external pull down resistor*/
// #define TOSPO_ENCODER

/* L298 Motor driver*/
#define L298_MOTOR_DRIVER
#endif

/* Optional Debug Output Flag */
// #define DEBUG_OUTPUT // Uncomment to enable general debug serial prints

/* Encoder Decoding Mode Selection */
#define ENCODER_MODE_X2 2
#define ENCODER_MODE_X4 4
#define ENCODER_DECODING_MODE ENCODER_MODE_X4 // Default to X4, change to ENCODER_MODE_X2 for X2

// #define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE 57600

/* Maximum PWM signal */
#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "servos.h"
#endif

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE 30 // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */
// Include globals.h and initialize motor direction variables regardless of encoder type
#include "globals.h"
// Initialize global variables with a default direction
MotorDirection leftMotorDirection = STOPPED;
MotorDirection rightMotorDirection = STOPPED;

// A pair of variables to help parse serial commands (thanks Fergs)
int arg = 0;
// Rename 'index' to 'arg_index' to avoid conflict with ESP32 library function
int arg_index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand()
{
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  arg_index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  char response[256];
  response[0] = '\0'; // Initialize to empty string for safety
  bool response_prepared = false;

#ifdef DEBUG_OUTPUT
  Serial.print("DEBUG: runCommand() received cmd: ");
  Serial.write(cmd ? cmd : '0');
  Serial.print(" | argv1: [");
  Serial.print(argv1);
  Serial.print("]");
  Serial.print(" | argv2: [");
  Serial.print(argv2);
  Serial.println("]");
  Serial.flush();
#endif

  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

#ifdef DEBUG_OUTPUT
  Serial.print("DEBUG: atoi results arg1: ");
  Serial.print(arg1);
  Serial.print(" | arg2: ");
  Serial.println(arg2);
  Serial.flush();
#endif

  switch (cmd)
  {
  case GET_BAUDRATE:
    snprintf(response, sizeof(response), "%ld", (long)BAUDRATE);
    response_prepared = true;
    break;
  case ANALOG_READ:
    snprintf(response, sizeof(response), "%d", analogRead(arg1));
    response_prepared = true;
    break;
  case DIGITAL_READ:
    snprintf(response, sizeof(response), "%d", digitalRead(arg1));
    response_prepared = true;
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0)
      digitalWrite(arg1, LOW);
    else if (arg2 == 1)
      digitalWrite(arg1, HIGH);
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case PIN_MODE:
    if (arg2 == 0)
      pinMode(arg1, INPUT);
    else if (arg2 == 1)
      pinMode(arg1, OUTPUT);
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case PING:
    snprintf(response, sizeof(response), "%d", Ping(arg1));
    response_prepared = true;
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case SERVO_READ:
    snprintf(response, sizeof(response), "%d", servos[arg1].getServo().read());
    response_prepared = true;
    break;
#endif

#ifdef USE_BASE
  case READ_ENCODERS:
  {
    long left_val = readEncoder(LEFT);
    long right_val = readEncoder(RIGHT);
    snprintf(response, sizeof(response), "%ld %ld", left_val, right_val);
    response_prepared = true;
  }
  break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0)
    {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else
    { // For any non-zero speed command
      if (moving == 0)
      {             // If we are transitioning from not moving to moving
        resetPID(); // Reset PID states before starting a new movement
      }
      moving = 1;
    }
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    resetPID();
    moving = 0;
    setMotorSpeeds(arg1, arg2);
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  case UPDATE_PID:
  {
    char temp_argv1[16];
    strncpy(temp_argv1, argv1, sizeof(temp_argv1) - 1);
    temp_argv1[sizeof(temp_argv1) - 1] = '\0';
    char *tp = temp_argv1;

    while ((str = strtok_r(tp, ":", &tp)) != NULL)
    {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    strncpy(response, "OK", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
  }
  break;
  case GET_PID_CONSTANTS:
    snprintf(response, sizeof(response), "Kp:%d Kd:%d Ki:%d Ko:%d", Kp, Kd, Ki, Ko);
    response_prepared = true;
    break;
#endif
  default:
    strncpy(response, "Invalid Command", sizeof(response) - 1);
    response[sizeof(response) - 1] = '\0';
    response_prepared = true;
    break;
  } // End of switch

  if (response_prepared)
  {
    Serial.println(response);
  }

#ifdef DEBUG_OUTPUT
  Serial.println("runCommand() finished");
#endif
  return 0;
}

/* Setup function--runs once at startup. */
void setup()
{
  Serial.begin(BAUDRATE);

#if defined(ESP32) && defined(ARDUINO_ENC_COUNTER)
  // For basic GPIO read test, set encoder pins as INPUT_PULLUP
  // This helps if your Hall sensors are open-collector or need pull-ups.
  // Note: The PCNT setup in encoder_driver.ino does not set pull-ups.
  //       If this test works, we might need to add pull-up configuration
  //       more formally or ensure external pull-ups are present.
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP); // Also for right, though test focuses on left
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
#ifdef DEBUG_OUTPUT
  Serial.println("DEBUG: ESP32 Encoder pins set to INPUT_PULLUP for testing.");
#endif
#endif

// Initialize the motor controller if used */
#ifdef USE_BASE
#if defined(ESP32)
// ESP32 specific initialization
#if defined(ARDUINO_ENC_COUNTER) || defined(TOSPO_ENCODER)
  initEncoders(); // Call our new ESP32 encoder initialization function
#endif
#else
// Original AVR-specific code
#ifdef ARDUINO_ENC_COUNTER
  // set as inputs - LEFT_ENC_PIN_A/B are PD2/PD3, RIGHT_ENC_PIN_A/B are PC4/PC5
  DDRD &= ~((1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B));   // Clear bits for PD2, PD3
  DDRC &= ~((1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B)); // Clear bits for PC4, PC5

  // enable pull up resistors
  PORTD |= ((1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B));   // Set bits for PD2, PD3
  PORTC |= ((1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B)); // Set bits for PC4, PC5

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 (PORTC) and PCINT2 (PORTD) interrupts in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#elif defined(TOSPO_ENCODER)
  // set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  // no pull up because of external pull down
  // tell pin change mask to listen to left encoder pin
  PCMSK2 |= (1 << LEFT_ENC_PIN_A);
  // tell pin change mask to listen to right encoder pin
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A);
  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#endif
#endif

  initMotorController(); // Calls ESP32 or AVR version from motor_driver.ino
  resetPID();
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++)
  {
    servos[i].initServo(
        servoPins[i],
        stepDelay[i],
        servoInitPosition[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop()
{

#if defined(DEBUG_OUTPUT) && defined(ARDUINO_ENC_COUNTER) && defined(DEBUG_ENCODER_PINS) // DEBUG_ENCODER_PINS is now legacy, subsumed by DEBUG_OUTPUT for this block
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100)
  { // Print every 100ms
    int laState = digitalRead(LEFT_ENC_PIN_A);
    int lbState = digitalRead(LEFT_ENC_PIN_B);
    // int raState = digitalRead(RIGHT_ENC_PIN_A); // Optionally check right too
    // int rbState = digitalRead(RIGHT_ENC_PIN_B);
    Serial.print("DEBUG L_Enc A: ");
    Serial.print(laState);
    Serial.print(" B: ");
    Serial.print(lbState);
    // Serial.print(" R_Enc A: ");
    // Serial.print(raState);
    // Serial.print(" B: ");
    // Serial.println(rbState);
    Serial.println(); // Newline for left only
    lastPrintTime = millis();
  }
#endif
  while (Serial.available() > 0)
  {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13)
    {
      if (arg == 1)
        argv1[arg_index] = '\0';
      else if (arg == 2)
        argv2[arg_index] = '\0';
      runCommand();
      resetCommand(); // Re-enable this call
#ifdef DEBUG_OUTPUT
      Serial.println("DEBUG: resetCommand was CALLED"); // Update this message
      Serial.flush();                                   // Ensure it prints
#endif
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1)
      {
        argv1[arg_index] = '\0';
        arg = 2;
        arg_index = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[arg_index] = chr;
        arg_index++;
      }
      else if (arg == 2)
      {
        argv2[arg_index] = chr;
        arg_index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    ;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++)
  {
    servos[i].doSweep();
  }
#endif

  yield(); // Allow other tasks to run, prevent tight empty loop issues
}
