/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#if defined(ESP32)
// ESP32 specific encoder pin implementation
// Define the pins used for encoders
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
#define LEFT_ENC_PIN_A 17 // Was D2/GPIO5, now Nano D8/GPIO17
#define LEFT_ENC_PIN_B 18 // Was D3/GPIO6, now Nano D9/GPIO18
#define RIGHT_ENC_PIN_A 1 // Nano A0/GPIO1 (Placeholder, was conflicting GPIO21)
#define RIGHT_ENC_PIN_B 2 // Nano A1/GPIO2 (Placeholder, was GPIO38)
#else
#ifdef ARDUINO_ENC_COUNTER
// below can be changed, but should be PORTD pins;
// otherwise additional changes in the code are required
#define LEFT_ENC_PIN_A PD2  // pin 2
#define LEFT_ENC_PIN_B PD3  // pin 3

// below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A PC4 // pin A4
#define RIGHT_ENC_PIN_B PC5 // pin A5
#elif defined(TOSPO_ENCODER)
#define LEFT_ENC_PIN_A PD2  // pin 2
// #define LEFT_ENC_PIN_B PD3  //pin 3

// below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A PC4 // pin A4
// #define RIGHT_ENC_PIN_B PC5   //pin A5
#endif
#endif

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#if defined(ESP32) && (defined(ARDUINO_ENC_COUNTER) || defined(TOSPO_ENCODER))
void initEncoders(); // Add declaration for our new function
#endif
