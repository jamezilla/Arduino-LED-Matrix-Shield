#include <pins_arduino.h>
#include <TimerOne.h>
#include "MsTimer2.h"

#define MAX_IN_USE 1    //change this variable to set how many MAX7219's you'll use

#define DIN        4
#define CLK        3
#define LOAD       2

int e = 0;

// define max7219 registers
byte max7219_reg_noop        = 0x00;
byte max7219_reg_digit0      = 0x01;
byte max7219_reg_digit1      = 0x02;
byte max7219_reg_digit2      = 0x03;
byte max7219_reg_digit3      = 0x04;
byte max7219_reg_digit4      = 0x05;
byte max7219_reg_digit5      = 0x06;
byte max7219_reg_digit6      = 0x07;
byte max7219_reg_digit7      = 0x08;
byte max7219_reg_decodeMode  = 0x09;
byte max7219_reg_intensity   = 0x0a;
byte max7219_reg_scanLimit   = 0x0b;
byte max7219_reg_shutdown    = 0x0c;
byte max7219_reg_displayTest = 0x0f;

// Output/input registers and bitmasks
volatile uint8_t *clk_port;
volatile uint8_t clk_bitmask;
volatile uint8_t *din_port;
volatile uint8_t din_bitmask;
volatile uint8_t *load_port;
volatile uint8_t load_bitmask;

volatile uint8_t levels[64];

// generates the output/input registers and bitmasks needed for fastDigtialWrite()
void cacheRegisters()
{
  getRegAndBitmask(&clk_port,  &clk_bitmask,  CLK,  INPUT);
  getRegAndBitmask(&din_port,  &din_bitmask,  DIN,  INPUT);
  getRegAndBitmask(&load_port, &load_bitmask, LOAD, INPUT);
}

// Get output/input register and bitmask for a given pin
void getRegAndBitmask(volatile uint8_t** reg, volatile uint8_t* bitmask, uint8_t pin, uint8_t mode) {
  uint8_t port = digitalPinToPort(pin);
  *reg = (mode == OUTPUT) ? portOutputRegister(port) : portInputRegister(port);
  *bitmask = digitalPinToBitMask(pin);
}

// faster replacement for digitalWrite()
inline void fastDigitalWrite(volatile uint8_t* port, volatile uint8_t mask, boolean value) {
  if (value)
    *port |= mask; 
  else 
    *port &= ~mask;
}

inline void putByte(byte data) {
  byte i = 8;
  byte mask;
  while(i > 0) {
    mask = 0x01 << (i - 1);      // get bitmask
    fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
    if (data & mask){            // choose bit
      fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
    } 
    else {
      fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
    }
    fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock
    --i;                         // move to lesser bit
  }
}

// maxSingle is the "easy"  function to use for a
// single max7219
void maxSingle( byte reg, byte col) {    
  fastDigitalWrite(load_port, load_bitmask, LOW);       // begin     
  putByte(reg);                  // specify register
  putByte(col);                  //((data & 0x01) * 256) + data >> 1); // put data
  fastDigitalWrite(load_port, load_bitmask, HIGH); 
}

void initDisplay(void) {
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);  

  // set the scan limit so it displays all digits
  putByte(max7219_reg_scanLimit);
  putByte(0x07);

  // using an led matrix (not digits)
  putByte(max7219_reg_decodeMode);
  putByte(0x00);  

  // not in shutdown mode
  putByte(max7219_reg_shutdown);
  putByte(0x01);    

  // no display test
  putByte(max7219_reg_displayTest);
  putByte(0x00);   

  clearDisplay();

  // set the brightness, the first 0x0f is the value you can set
  // range: 0x00 to 0x0f
  putByte(max7219_reg_intensity);
  putByte(0x0f & 0x0f);     

}

void clearDisplay(void) {
  fastDigitalWrite(load_port, load_bitmask, LOW);       // LOAD
  // could be a loop, but unrolled here for speed 
  putByte(1);
  putByte(0);
  putByte(2);
  putByte(0);
  putByte(3);
  putByte(0);
  putByte(4);
  putByte(0);
  putByte(5);
  putByte(0);
  putByte(6);
  putByte(0);
  putByte(7);
  putByte(0);
  putByte(8);
  putByte(0);  
  fastDigitalWrite(load_port, load_bitmask, HIGH);
}

// initialize  all  MAX7219's in the system
//void maxAll (byte reg, byte col) {    
//  int c = 0;
//  fastDigitalWrite(load_port, load_bitmask, LOW);       // begin     
//  for (c = 1; c <= MAX_IN_USE; c++) {
//    putByte(reg);                // specify register
//    putByte(col);                //((data & 0x01) * 256) + data >> 1); // put data
//  }
//  fastDigitalWrite(load_port, load_bitmask, HIGH);
//}

#ifdef DEBUG_TIMER

void report() {
  Serial.println(loop_count);
  loop_count = 0;
}

#endif

void foo(){
}

void setup () {
  Serial.begin(9600);

 initialize the dim levels
  for(int i=0; i<64; i++) {
    levels[i] = 0;
  }

  // generate port/pin/mask mappings needed for fastDigitalWrite()
  cacheRegisters();

  pinMode(DIN, OUTPUT);
  pinMode(CLK,  OUTPUT);
  pinMode(LOAD,   OUTPUT);

  initDisplay();

  //#ifdef DEBUG_TIMER


  //  MsTimer2::set(1000, report); 
  MsTimer2::set(1000, foo); 
  MsTimer2::start();
  //Serial.println(MsTimer2::tcnt2);
  //#endif
}  

void loop () {

  clearDisplay();

  // turn on the leds in the 8th row one at a time
  //  uint8_t mask = 1;
  //  uint8_t data = 0;
  //  for(int i=0; i<8; i++) {
  //    data |= mask;
  //    maxSingle(8,data);
  //    mask = mask << 1;
  //    delay(500);
  //  }

  // turn on the diagonal  
  //  maxSingle(1,1);                       //  + - - - - - - -
  //  maxSingle(2,2);                       //  - + - - - - - -
  //  maxSingle(3,4);                       //  - - + - - - - -
  //  maxSingle(4,8);                       //  - - - + - - - -
  //  maxSingle(5,16);                      //  - - - - + - - -
  //  maxSingle(6,32);                      //  - - - - - + - -
  //  maxSingle(7,64);                      //  - - - - - - + -
  //  maxSingle(8,128);                     //  - - - - - - - +

  //  ALL ON
  //  maxSingle(1,255);
  //  maxSingle(2,255);
  //  maxSingle(3,255);
  //  maxSingle(4,255);
  //  maxSingle(5,255);
  //  maxSingle(6,255);
  //  maxSingle(7,255);
  //  maxSingle(8,255);

  // flash 2 corners, then the opposite 2 corners
  //  maxSingle(1,128);
  //  maxSingle(8,1);
  //  delay(500);
  //  clearDisplay();
  //  maxSingle(1,1);
  //  maxSingle(8,128);
  //  delay(500);

#ifdef DEBUG_TIMER 
  loop_count++;
#endif
}




















