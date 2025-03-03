/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  #if defined(ESP32)
  // ESP32 specific encoder implementation
  // Define the pins used for encoders
  const int LEFT_ENC_A = 2;  // Adjust these pin numbers to match your wiring
  const int LEFT_ENC_B = 3;
  const int RIGHT_ENC_A = 4;
  const int RIGHT_ENC_B = 5;
  
  void IRAM_ATTR leftEncoderISR() {
    static uint8_t enc_last = 0;
    uint8_t enc_current = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
    
    enc_last <<= 2;
    enc_last |= enc_current;
    
    left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  void IRAM_ATTR rightEncoderISR() {
    static uint8_t enc_last = 0;
    uint8_t enc_current = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
    
    enc_last <<= 2;
    enc_last |= enc_current;
    
    right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  void setupEncoders() {
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightEncoderISR, CHANGE);
  }
  #else
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  #endif
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(TOSPO_ENCODER)
  #include "globals.h"
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  #if defined(ESP32)
  // ESP32 specific encoder implementation for TOSPO
  const int LEFT_ENC_PIN = 2;  // Adjust these pin numbers to match your wiring
  const int RIGHT_ENC_PIN = 4;
  
  // External variables from Globals.h
  extern int leftMotorDirection;
  extern int rightMotorDirection;
  extern const int FORWARD;
  extern const int BACKWARD;
  
  void IRAM_ATTR leftEncoderISR() {
    if (leftMotorDirection == FORWARD) { left_enc_pos++; } 
    else if (leftMotorDirection == BACKWARD) { left_enc_pos--; }
  }
  
  void IRAM_ATTR rightEncoderISR() {
    if (rightMotorDirection == FORWARD) { right_enc_pos++; } 
    else if (rightMotorDirection == BACKWARD) { right_enc_pos--; }
  }
  
  void setupEncoders() {
    pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, CHANGE);
  }
  #else
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
    if (leftMotorDirection == FORWARD) { left_enc_pos++; } 
    else if (leftMotorDirection == BACKWARD) { left_enc_pos--; }
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
    if (rightMotorDirection == FORWARD) { right_enc_pos++; } 
    else if (rightMotorDirection == BACKWARD) { right_enc_pos--; }
  }
  #endif
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#if defined(ESP32) && (defined(ARDUINO_ENC_COUNTER) || defined(TOSPO_ENCODER))
// This function needs to be called in the setup() function of the main sketch
void initEncoders() {
  setupEncoders();
}
#endif

#endif

