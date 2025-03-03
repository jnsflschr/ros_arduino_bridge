/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
#if defined(ESP32)
  // ESP32 pin definitions are handled in encoder_driver.ino
  // No need for the AVR-specific pin definitions here
#else
  #ifdef ARDUINO_ENC_COUNTER
    //below can be changed, but should be PORTD pins; 
    //otherwise additional changes in the code are required
    #define LEFT_ENC_PIN_A PD2  //pin 2
    #define LEFT_ENC_PIN_B PD3  //pin 3
    
    //below can be changed, but should be PORTC pins
    #define RIGHT_ENC_PIN_A PC4  //pin A4
    #define RIGHT_ENC_PIN_B PC5   //pin A5
  #elif defined(TOSPO_ENCODER)
    #define LEFT_ENC_PIN_A PD2  //pin 2
    //#define LEFT_ENC_PIN_B PD3  //pin 3
    
    //below can be changed, but should be PORTC pins
    #define RIGHT_ENC_PIN_A PC4  //pin A4
    //#define RIGHT_ENC_PIN_B PC5   //pin A5
  #endif
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#if defined(ESP32) && (defined(ARDUINO_ENC_COUNTER) || defined(TOSPO_ENCODER))
void initEncoders();  // Add declaration for our new function
#endif

