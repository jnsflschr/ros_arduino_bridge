/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#if defined(ESP32) && defined(ARDUINO_ENC_COUNTER)
#include "driver/pcnt.h" // Include PCNT driver only when ARDUINO_ENC_COUNTER is used on ESP32
#endif

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  if (i == LEFT)
    return encoders.YAxisGetCount();
  else
    return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  if (i == LEFT)
    return encoders.YAxisReset();
  else
    return encoders.XAxisReset();
}
#elif defined(ARDUINO_ENC_COUNTER)
#if defined(ESP32)
// ESP32 specific encoder implementation using PCNT peripheral (X2 Decoding)
// Volatile variables are not strictly necessary here as counts are read from PCNT hardware.
// They are kept for structural similarity if any other code were to hypothetically access them.
volatile long esp32_left_enc_pos_shadow = 0L;
volatile long esp32_right_enc_pos_shadow = 0L;

// Software 32-bit accumulation of PCNT counts (file-scope for clarity and single-definition)
static long left_total_counts = 0L;
static long right_total_counts = 0L;

#define PCNT_HIGH_LIMIT 32767
#define PCNT_LOW_LIMIT -32767
#define PCNT_FILTER_VAL 320 // ~4.0us debounce (320 * 12.5ns) for hall sensors at ~12.9k edges/s

pcnt_unit_t left_pcnt_unit = PCNT_UNIT_0;
pcnt_unit_t right_pcnt_unit = PCNT_UNIT_1;

// This function is now EXCLUSIVELY for ESP32 with ARDUINO_ENC_COUNTER
void setupEncoders_esp32()
{
  // Configure Left Encoder PCNT Unit - Channel 0 (Common for X2 and X4)
  pcnt_config_t pcnt_config_left = {
      .pulse_gpio_num = LEFT_ENC_PIN_A,
      .ctrl_gpio_num = LEFT_ENC_PIN_B,
      .lctrl_mode = PCNT_MODE_REVERSE, // Reverse count direction when B is low
      .hctrl_mode = PCNT_MODE_KEEP,    // Keep count direction when B is high
      .pos_mode = PCNT_COUNT_INC,      // Increment on A's positive edge
      .neg_mode = PCNT_COUNT_DEC,      // Decrement on A's negative edge (This combination provides X2 on this channel)
      .counter_h_lim = PCNT_HIGH_LIMIT,
      .counter_l_lim = PCNT_LOW_LIMIT,
      .unit = left_pcnt_unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_left);

  // For X4 decoding with PCNT, channel 0 is configured for signal A (pulse) and B (ctrl),
  // and channel 1 is configured for signal B (pulse) and A (ctrl).
  // The PCNT unit effectively combines counts from both configurations.
#if defined(ENCODER_DECODING_MODE) && (ENCODER_DECODING_MODE == ENCODER_MODE_X4)
  pcnt_config_t pcnt_config_left_ch1 = {
      .pulse_gpio_num = LEFT_ENC_PIN_B, // Pulse on B
      .ctrl_gpio_num = LEFT_ENC_PIN_A,  // Control with A
      .lctrl_mode = PCNT_MODE_KEEP,     // When A is low, B positive edge = INC, B negative edge = DEC
      .hctrl_mode = PCNT_MODE_REVERSE,  // When A is high, B positive edge = DEC, B negative edge = INC (standard quadrature logic)
      .pos_mode = PCNT_COUNT_INC,       // Increment on B's positive edge (relative to A's control)
      .neg_mode = PCNT_COUNT_DEC,       // Decrement on B's negative edge (relative to A's control)
      .counter_h_lim = PCNT_HIGH_LIMIT,
      .counter_l_lim = PCNT_LOW_LIMIT,
      .unit = left_pcnt_unit, // Same unit, different channel
      .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_config_left_ch1);
#endif

  // Configure Right Encoder PCNT Unit - Channel 0 (Common for X2 and X4)
  pcnt_config_t pcnt_config_right = {
      .pulse_gpio_num = RIGHT_ENC_PIN_A,
      .ctrl_gpio_num = RIGHT_ENC_PIN_B,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC, // This combination provides X2 on this channel
      .counter_h_lim = PCNT_HIGH_LIMIT,
      .counter_l_lim = PCNT_LOW_LIMIT,
      .unit = right_pcnt_unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_right);

#if defined(ENCODER_DECODING_MODE) && (ENCODER_DECODING_MODE == ENCODER_MODE_X4)
  pcnt_config_t pcnt_config_right_ch1 = {
      .pulse_gpio_num = RIGHT_ENC_PIN_B, // Pulse on B
      .ctrl_gpio_num = RIGHT_ENC_PIN_A,  // Control with A
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = PCNT_HIGH_LIMIT,
      .counter_l_lim = PCNT_LOW_LIMIT,
      .unit = right_pcnt_unit, // Same unit, different channel
      .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_config_right_ch1);
#endif

  // Initialize and start PCNT units
  pcnt_set_filter_value(left_pcnt_unit, PCNT_FILTER_VAL); // PCNT_FILTER_VAL is currently 0
  pcnt_filter_enable(left_pcnt_unit);
  pcnt_counter_pause(left_pcnt_unit);
  pcnt_counter_clear(left_pcnt_unit);
  pcnt_counter_resume(left_pcnt_unit);

  pcnt_set_filter_value(right_pcnt_unit, PCNT_FILTER_VAL); // PCNT_FILTER_VAL is currently 0
  pcnt_filter_enable(right_pcnt_unit);
  pcnt_counter_pause(right_pcnt_unit);
  pcnt_counter_clear(right_pcnt_unit);
  pcnt_counter_resume(right_pcnt_unit);
}

#else                                                // AVR implementation for ARDUINO_ENC_COUNTER (interrupt based)
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

// AVR Pin Definitions from encoder_driver.h (e.g., LEFT_ENC_PIN_A is PD2)
// These macros help map the port-specific pin names to bit numbers for clarity in ISRs
#define AVR_LEFT_ENC_A_BIT (LEFT_ENC_PIN_A & 0x07)   // e.g., PD2 -> bit 2
#define AVR_LEFT_ENC_B_BIT (LEFT_ENC_PIN_B & 0x07)   // e.g., PD3 -> bit 3
#define AVR_RIGHT_ENC_A_BIT (RIGHT_ENC_PIN_A & 0x07) // e.g., PC4 -> bit 4
#define AVR_RIGHT_ENC_B_BIT (RIGHT_ENC_PIN_B & 0x07) // e.g., PC5 -> bit 5

static const int8_t ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; // Encoder lookup table

// Interrupt routine for LEFT encoder (PORTD pins, e.g., D2, D3)
ISR(PCINT2_vect)
{
  static uint8_t enc_last_left = 0;
  uint8_t current_pins = 0;
  // Read directly from PIND register, assuming LEFT_ENC_PIN_A is PD2 and LEFT_ENC_PIN_B is PD3
  // (PIND & 0b00001100) >> 2 would isolate PD2 and PD3 to bits 0 and 1
  if (PIND & (1 << AVR_LEFT_ENC_A_BIT))
    current_pins |= 1; // LSB for Pin A
  if (PIND & (1 << AVR_LEFT_ENC_B_BIT))
    current_pins |= 2; // Next bit for Pin B

  enc_last_left = (enc_last_left << 2) | current_pins;
  left_enc_pos += ENC_STATES[(enc_last_left & 0x0f)];
}

// Interrupt routine for RIGHT encoder (PORTC pins, e.g., A4, A5)
ISR(PCINT1_vect)
{
  static uint8_t enc_last_right = 0;
  uint8_t current_pins = 0;
  // Read directly from PINC register, assuming RIGHT_ENC_PIN_A is PC4 and RIGHT_ENC_PIN_B is PC5
  // (PINC & 0b00110000) >> 4 would isolate PC4 and PC5 to bits 0 and 1
  if (PINC & (1 << AVR_RIGHT_ENC_A_BIT))
    current_pins |= 1; // LSB for Pin A
  if (PINC & (1 << AVR_RIGHT_ENC_B_BIT))
    current_pins |= 2; // Next bit for Pin B

  enc_last_right = (enc_last_right << 2) | current_pins;
  right_enc_pos += ENC_STATES[(enc_last_right & 0x0f)];
}

#endif // ESP32 or AVR for ARDUINO_ENC_COUNTER

// Common functions for ARDUINO_ENC_COUNTER
long readEncoder(int i)
{
#if defined(ESP32)
  // Accumulate PCNT (16-bit) counts into a software 32-bit total and clear hardware counter
  // to avoid nearing the ±32767 hardware limits which can cause velocity estimation spikes.
  int16_t count = 0;
  if (i == LEFT)
  {
    pcnt_get_counter_value(left_pcnt_unit, &count);
    pcnt_counter_clear(left_pcnt_unit);
    left_total_counts += (long)count;              // accumulate signed delta
    esp32_left_enc_pos_shadow = left_total_counts; // keep shadow in sync for any external use
    return left_total_counts;
  }
  else
  {
    pcnt_get_counter_value(right_pcnt_unit, &count);
    pcnt_counter_clear(right_pcnt_unit);
    right_total_counts += (long)count;
    esp32_right_enc_pos_shadow = right_total_counts;
    return right_total_counts;
  }
#else // AVR
  long val;
  noInterrupts(); // Protect read of volatile variable
  if (i == LEFT)
    val = left_enc_pos;
  else
    val = right_enc_pos;
  interrupts(); // Re-enable
  return val;
#endif
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
#if defined(ESP32)
  if (i == LEFT)
  {
    pcnt_counter_clear(left_pcnt_unit);
    esp32_left_enc_pos_shadow = 0L; // Reset shadow too
    left_total_counts = 0L;         // Reset software total to keep semantics consistent
  }
  else
  {
    pcnt_counter_clear(right_pcnt_unit);
    esp32_right_enc_pos_shadow = 0L; // Reset shadow too
    right_total_counts = 0L;          // Reset software total
  }
#else // AVR
  noInterrupts(); // Protect write of volatile variable
  if (i == LEFT)
  {
    left_enc_pos = 0L;
  }
  else
  {
    right_enc_pos = 0L;
  }
  interrupts(); // Re-enable
#endif
  // No return needed here for void function
}
#elif defined(TOSPO_ENCODER)
#include "globals.h"
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

#if defined(ESP32)
// ESP32 specific encoder implementation for TOSPO
//
// Arduino Nano ESP32 Pin Recommendations:
// - GPIO 1, 3: Reserved for Serial (TX/RX)
// - GPIO 0: Connected to BOOT button, avoid using as input
// - GPIO 6-11: Connected to internal flash, DO NOT USE
// - GPIO 34-39: Input only pins, no internal pull-up resistors
// - GPIO 2: Connected to onboard LED
//
// Good pins for encoders (support interrupts):
// - GPIO 4, 5, 12-19, 21-23, 25-27, 32, 33
//
// Adjust these pin numbers to match your wiring:
const int LEFT_ENC_PIN = 18;  // Good interrupt pin
const int RIGHT_ENC_PIN = 21; // Good interrupt pin

// External variables from globals.h
extern MotorDirection leftMotorDirection;
extern MotorDirection rightMotorDirection;

void IRAM_ATTR leftEncoderISR()
{
  if (leftMotorDirection == FORWARD)
  {
    left_enc_pos++;
  }
  else if (leftMotorDirection == BACKWARD)
  {
    left_enc_pos--;
  }
}

void IRAM_ATTR rightEncoderISR()
{
  if (rightMotorDirection == FORWARD)
  {
    right_enc_pos++;
  }
  else if (rightMotorDirection == BACKWARD)
  {
    right_enc_pos--;
  }
}

void setupEncoders()
{
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, CHANGE);
}
#else
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR(PCINT2_vect)
{
  if (leftMotorDirection == FORWARD)
  {
    left_enc_pos++;
  }
  else if (leftMotorDirection == BACKWARD)
  {
    left_enc_pos--;
  }
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR(PCINT1_vect)
{
  if (rightMotorDirection == FORWARD)
  {
    right_enc_pos++;
  }
  else if (rightMotorDirection == BACKWARD)
  {
    right_enc_pos--;
  }
}
#endif

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  if (i == LEFT)
    return left_enc_pos;
  else
    return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  if (i == LEFT)
  {
    left_enc_pos = 0L;
    return;
  }
  else
  {
    right_enc_pos = 0L;
    return;
  }
}
#else
#error A encoder driver must be selected!
#endif

// This function is common for any encoder type if USE_BASE is defined
void resetEncoders()
{
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

// initEncoders() is called from ROSArduinoBridge.ino setup() for ESP32 with ARDUINO_ENC_COUNTER
#if defined(ESP32) && (defined(ARDUINO_ENC_COUNTER) || defined(TOSPO_ENCODER))
// This function needs to be called in the setup() function of the main sketch
void initEncoders()
{
  setupEncoders_esp32(); // Calls the ESP32 X2 PCNT setup
}
#endif // ESP32 && ARDUINO_ENC_COUNTER

#endif // USE_BASE
