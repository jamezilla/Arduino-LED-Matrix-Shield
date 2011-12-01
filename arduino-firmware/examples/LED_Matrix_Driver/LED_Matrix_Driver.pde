#include <pins_arduino.h>
//#include <TimerOne.h>
#include <MsTimer2.h>

#define DEBUG_TIMER 1
#define MAX_IN_USE 1    //change this variable to set how many MAX7219's you'll use

#define DIN        4
#define CLK        3
#define LOAD       2

int e = 0;

// define max7219 registers
#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DIGIT6      0x07
#define MAX7219_REG_DIGIT7      0x08
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0a
#define MAX7219_REG_SCANLIMIT   0x0b
#define MAX7219_REG_SHUTDOWN    0x0c
#define MAX7219_REG_DISPLAYTEST 0x0f

// Output/input registers and bitmasks
volatile uint8_t *clk_port;
volatile uint8_t clk_bitmask;
volatile uint8_t *din_port;
volatile uint8_t din_bitmask;
volatile uint8_t *load_port;
volatile uint8_t load_bitmask;


// volatile uint8_t levels[64];

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

  // == bit 8 (most significant)
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x80)            // choose bit
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 7
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x40)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 6
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x20)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 5
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x10)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 4
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x8)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 3
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x4)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 2
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x2)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 1 (least significant)
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x01)
    fastDigitalWrite(din_port, din_bitmask, HIGH);// send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW); // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

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
  putByte(MAX7219_REG_SCANLIMIT);
  putByte(0x07);

  // using an led matrix (not digits)
  putByte(MAX7219_REG_DECODEMODE);
  putByte(0x00);  

  // not in shutdown mode
  putByte(MAX7219_REG_SHUTDOWN);
  putByte(0x01);    

  // no display test
  putByte(MAX7219_REG_DISPLAYTEST);
  putByte(0x00);   

  clearDisplay();

  // set the brightness, the first 0x0f is the value you can set
  // range: 0x00 to 0x0f
  putByte(MAX7219_REG_INTENSITY);
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

long loop_count = 0;

void report() {
  Serial.println(loop_count);
  loop_count = 0;
}

#endif

void setup () {
  Serial.begin(9600);

   //initialize the dim levels
  // for(int i=0; i<64; i++) {
  //   levels[i] = 0;
  // }

  // generate port/pin/mask mappings needed for fastDigitalWrite()
  cacheRegisters();

  pinMode(DIN, OUTPUT);
  pinMode(CLK,  OUTPUT);
  pinMode(LOAD,   OUTPUT);

  initDisplay();

#ifdef DEBUG_TIMER
  loop_count = 0;
  MsTimer2::set(1000, report); 
  MsTimer2::start();
#endif
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
   maxSingle(1,1);                       //  + - - - - - - -
   maxSingle(2,2);                       //  - + - - - - - -
   maxSingle(3,4);                       //  - - + - - - - -
   maxSingle(4,8);                       //  - - - + - - - -
   maxSingle(5,16);                      //  - - - - + - - -
   maxSingle(6,32);                      //  - - - - - + - -
   maxSingle(7,64);                      //  - - - - - - + -
   maxSingle(8,128);                     //  - - - - - - - +

#ifdef DEBUG_TIMER 
  loop_count++;
#endif
}
