# ROSArduinoBridge ESP32 Porting & Debugging Journey

This document outlines the process and key learnings from porting the ROSArduinoBridge project to support the Arduino Nano ESP32 (ESP32-S3) and debugging various issues encountered.

## Initial Goal
Add ESP32 support to the existing Arduino (AVR) `ROSArduinoBridge` project, enabling control of a differential drive robot with motors and encoders.

## Key Systems Involved
-   **Microcontroller:** Arduino Nano ESP32 (ESP32-S3)
-   **Motor Driver:** L298N
-   **Encoders:** Hall effect sensors, intended to be read by ESP32's PCNT (Pulse Counter) peripheral.
-   **Communication:** Serial commands ('e' for encoders, 'm' for motor speeds, 'o' for raw PWM).

## Debugging Chronology & Learnings

### 1. Initial Porting & Motor Control Setup
-   **Challenge:** `analogWrite()` behaves differently on ESP32 (uses LEDC/SigmaDelta peripherals) compared to AVR.
-   **Solution:** Modified `motor_driver.ino` to use the ESP-IDF's `ledc` driver functions (`ledcSetup`, `ledcAttachPin`, `ledcWrite`) for direct PWM control, providing more explicit configuration of frequency and resolution suitable for motors.

### 2. System Reboots with Specific Commands
-   **Symptom:** ESP32 rebooted (serial connection lost, LEDs flashed) when sending 'e' (READ_ENCODERS), 'm' (MOTOR_SPEEDS), and also for invalid serial commands. Raw PWM ('o') worked initially.
-   **Investigation - Buffer Overflow in `runCommand()`:**
    -   Debug prints narrowed the crash point to *within* the `switch` statement in `runCommand()` in `ROSArduinoBridge.ino`.
    -   The crash occurred after `Serial.println` calls within cases (e.g., "Invalid Command") or when printing encoder values.
    -   **Root Cause:** The `char response[64];` buffer used to format command replies was being overflowed by `sprintf` or `strcpy`.
    -   **Fix:**
        -   Increased buffer size: `char response[256];`
        -   Replaced unsafe string functions with `snprintf` and `strncpy` (with manual null termination) for safer buffer handling.

### 3. Unstable Command Loop & Repeating Output
-   **Symptom:** After fixing the buffer overflow, commands would sometimes execute, but then the serial output would show a strange repeating pattern (e.g., "0 \n I am still here \n 0 \n ..."), and `resetCommand()` wasn't reliably called.
-   **Investigation - Missing `return` Statement:**
    -   The `runCommand()` function was declared to return `int` but lacked an explicit `return 0;` statement at the end.
    -   **Root Cause:** This led to undefined behavior on function exit, likely corrupting the stack or program flow, causing the command processing loop in `void loop()` to misbehave and re-enter `runCommand()` with stale data.
    -   **Fix:** Added `return 0;` to the end of `runCommand()`.
-   **Investigation - `Serial.flush()` Impact:**
    -   An unnecessary `Serial.flush()` at the end of `runCommand()` seemed to exacerbate the re-entrancy.
    -   **Fix:** Removed the `Serial.flush()` from `runCommand()`, keeping a strategic one in `loop()` after command processing.

### 4. ESP32-S3 Boot Loops (`TG1WDT_SYS_RST`) & Pin Conflicts
-   **Symptom:** After enabling "legacy GPIO numbering" (direct GPIO usage), the ESP32-S3 entered a continuous boot loop, with the watchdog timer (`TG1WDT_SYS_RST`) triggering the reset.
-   **Pin Mapping (Arduino Nano ESP32 - ESP32-S3):**
    -   D0:GPIO44, D1:GPIO43, D2:GPIO5, D3:GPIO6, D4:GPIO7, D5:GPIO8, D6:GPIO9, D7:GPIO10, D8:GPIO17, D9:GPIO18, D10:GPIO21, D11:GPIO38, D12:GPIO47, D13:GPIO48
    -   A0:GPIO1, A1:GPIO2, A2:GPIO3, A3:GPIO4, A4:GPIO11, A5:GPIO12, A6:GPIO13, A7:GPIO14
-   **Investigation - Isolating the Failing Initialization:**
    -   The boot loop occurred after printing "DEBUG: ESP32 Encoder pins set to INPUT_PULLUP for testing." in `setup()`.
    -   Systematically commented out `resetPID()`, then `initEncoders()`, then parts of `initMotorController()`.
    -   **Initial Suspects (Incorrect Pin Choices):**
        -   Encoders on D2(GPIO5)/D3(GPIO6): GPIO6 is often tied to SPI flash. GPIO5 has boot-time requirements.
        -   Motors on D12(GPIO47)/D13(GPIO48): These are often strapping pins or used for onboard LEDs/JTAG.
    -   **Revised Pin Strategy:** Shifted to using known safer GPIOs based on ESP32-S3 datasheets and community pinout guides.
    -   **Root Cause Identified:** The boot loop was eventually traced to `ledcAttachPin(RIGHT_MOTOR_ENABLE, RIGHT_PWM_CHANNEL);` when `RIGHT_MOTOR_ENABLE` was assigned to GPIO32 (Nano D14).
-   **Solution - Motor Pin Reassignment:**
    -   `LEFT_MOTOR_ENABLE` (PWM CH0) successfully used GPIO33 (Nano D15).
    -   `RIGHT_MOTOR_ENABLE` (PWM CH1) was changed from GPIO32 (Nano D14) to **GPIO21 (Nano D10)**, which resolved the boot loop.
    -   Other motor direction pins were also mapped to safer GPIOs:
        -   L_FWD: GPIO9 (D6), L_BCK: GPIO7 (D4)
        -   R_FWD: GPIO10 (D7), R_BCK: GPIO8 (D5)

### 5. Encoder PCNT Issues
-   **Symptom (Previously):** With motor PWM stable, re-enabling `resetPID()` (which calls `readEncoder()`) without `initEncoders()` active results in "PCNT driver error" messages. This is expected, as `pcnt_get_counter_value` is called on uninitialized hardware.
-   **Symptom (Later):** Motor oscillation with 'm' (MOTOR_SPEEDS) command was observed. This was initially thought to be an encoder or L298N enable pin PWM control issue.
-   **Initial Encoder Pin Choices (Caused Boot Loops):** D2(GPIO5), D3(GPIO6).
-   **Revised Encoder Pin Choices (Safer, for Left Encoder):
    -   `LEFT_ENC_PIN_A`: GPIO17 (Nano D8)
    -   `LEFT_ENC_PIN_B`: GPIO18 (Nano D9)
-   **Pin Conflict Resolution & Further Motor Pin Changes:** `RIGHT_MOTOR_ENABLE` was moved to GPIO21 (D10). `LEFT_MOTOR_ENABLE` was initially on the non-existent GPIO33 (D15) and later corrected to GPIO38 (D11). Placeholders for right encoder pins `RIGHT_ENC_PIN_A` (GPIO1/A0) and `RIGHT_ENC_PIN_B` (GPIO2/A1) were set.
-   **PCNT Configuration (X2 vs X4):** Implemented conditional compilation in `encoder_driver.ino` to switch between X2 and X4 decoding modes using the `ENCODER_DECODING_MODE` flag in `ROSArduinoBridge.ino`. X4 is the default for higher resolution.
-   **Filter Value:** `PCNT_FILTER_VAL` in `encoder_driver.ino` was set to `100` (1.25us filter) after initial testing with `0`.

### 6. Motor Kick with PID Control
-   **Symptom:** When sending a motor speed command (e.g., `m 50 0`), the motor would briefly spin rapidly in the opposite direction before settling to the correct speed and direction.
-   **Investigation:**
    -   The PID implementation in `diff_controller.h` uses `p->output += p->output_from_previous_iteration;` and `p->ITerm += Ki * Perror;`.
    -   The PID constant `Ki` was found to be `0` by default, meaning the `ITerm` itself wouldn't accumulate if not changed by a 'u' command.
    -   The main cause was identified as stale values in the PID state variables (particularly `leftPID.output`, `rightPID.output`, and `PrevInput`) when a new movement command was issued *without* an immediately preceding `m 0 0` command (which calls `resetPID()`).
    -   The `AUTO_STOP_INTERVAL` would set `moving = 0` and `setMotorSpeeds(0,0)`, but `resetPID()` would only occur in the *next* `updatePID()` cycle.
-   **Solution:** Modified the `MOTOR_SPEEDS` case in `runCommand()` in `ROSArduinoBridge.ino` to explicitly call `resetPID()` if the robot was transitioning from a non-moving state (`moving == 0`) to a moving state. This ensures PID internal states are cleared before a new movement.

## Current Stable Pin Configuration (Relevant for ESP32-S3/Nano ESP32)

**Motor Pins (`motor_driver.h`):**
-   `RIGHT_MOTOR_BACKWARD`: GPIO8 (Nano D5)
-   `LEFT_MOTOR_BACKWARD`:  GPIO7 (Nano D4)
-   `RIGHT_MOTOR_FORWARD`:  GPIO10 (Nano D7)
-   `LEFT_MOTOR_FORWARD`:   GPIO9 (Nano D6)
-   `RIGHT_MOTOR_ENABLE`:   GPIO21 (Nano D10) - PWM Channel 1
-   `LEFT_MOTOR_ENABLE`:    GPIO38 (Nano D11) - PWM Channel 0 (Corrected from D15/GPIO33)

**Encoder Pins (`encoder_driver.h`):**
-   `LEFT_ENC_PIN_A`:   GPIO17 (Nano D8)
-   `LEFT_ENC_PIN_B`:   GPIO18 (Nano D9)
-   `RIGHT_ENC_PIN_A`:  GPIO1 (Nano A0)  *(Placeholder - ensure this is suitable if right encoder is used)*
-   `RIGHT_ENC_PIN_B`:  GPIO2 (Nano A1)  *(Placeholder - ensure this is suitable if right encoder is used)*

## General ESP32 Development Learnings
-   Be meticulous with ESP32 (especially ESP32-S3) pin assignments. Strapping pins, flash interface pins, and pins with specific boot-time requirements can easily lead to boot loops or instability if used incorrectly.
-   Direct hardware peripheral control (like `ledc` or `pcnt` drivers) requires careful reading of datasheets and understanding of the specific ESP32 variant.
-   Arduino Core abstractions (`analogWrite`) can simplify PWM but might hide underlying peripheral configurations. Direct driver use offers more control but requires more care.
-   Systematic commenting-out of initialization routines is a powerful technique for isolating sources of boot loops or crashes in `setup()`.
-   Buffer overflows and missing return statements can lead to very confusing runtime behavior that isn't always an immediate crash at the source of the bug.
-   USB-CDC (Serial via USB) debug output can be invaluable but may be lost during rapid boot loops. Hardware debuggers (JTAG) would be the next step for even deeper issues. 