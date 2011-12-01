/******************************************************************************
 * Includes
 ******************************************************************************/

// extern "C" {
//   // AVR LibC Includes
//   #include <inttypes.h>
//   // #include <stdlib.h>

//   // Wiring Core Includes
//   // #undef abs
//   // #include "WConstants.h"
// }

#include <pins_arduino.h>
#include "MatrixDriver.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

// Matrix registers
#define REG_NOOP        0x00
#define REG_DIGIT0      0x01
#define REG_DIGIT1      0x02
#define REG_DIGIT2      0x03
#define REG_DIGIT3      0x04
#define REG_DIGIT4      0x05
#define REG_DIGIT5      0x06
#define REG_DIGIT6      0x07
#define REG_DIGIT7      0x08
#define REG_DECODEMODE  0x09
#define REG_INTENSITY   0x0A
#define REG_SCANLIMIT   0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAYTEST 0x0F

/******************************************************************************
 * Constructors
 ******************************************************************************/

MatrixDriver::MatrixDriver(uint8_t data, uint8_t clock, uint8_t load)
{
  // record pins for sw spi
  _pinData  = data;
  _pinClock = clock;
  _pinLoad  = load;

  // set ddr for sw spi pins
  pinMode(_pinClock, OUTPUT);
  pinMode(_pinData,  OUTPUT);
  pinMode(_pinLoad,  OUTPUT);

  // cache the buffer size
  _buffer_size = sizeof(uint8_t)*8;

  // initialize registers
  cacheRegisters();
  clear();             // clear display
  setScanLimit(0x07);  // use all rows/digits
  setBrightness(0x0E); // maximum brightness
  setRegister(REG_SHUTDOWN, 0x01);    // normal operation
  setRegister(REG_DECODEMODE, 0x00);  // pixels not integers
  setRegister(REG_DISPLAYTEST, 0x00); // not in test mode
}

/******************************************************************************
 * MAX7219 SPI
 ******************************************************************************/

// sends a single byte by sw spi (no latching)
inline void MatrixDriver::putByte(byte data)
{
  // NOTE: this would be a succinct loop, but it's faster unrolled

  // == bit 8 (most significant)
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x80)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 7
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x40)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 6
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x20)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 5
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x10)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 4
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x8)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 3
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x4)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 2
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x2)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock

  // == bit 1 (least significant)
  fastDigitalWrite(clk_port, clk_bitmask, LOW);    // tick
  if (data & 0x01)
    fastDigitalWrite(din_port, din_bitmask, HIGH); // send 1
  else 
    fastDigitalWrite(din_port, din_bitmask, LOW);  // send 0
  fastDigitalWrite(clk_port, clk_bitmask, HIGH);   // tock
}

// sets register to a byte value for all screens
void MatrixDriver::setRegister(uint8_t reg, uint8_t data)
{
  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(reg);
  putByte(data);
  fastDigitalWrite(load_port, load_bitmask, HIGH);
}

// syncs row of display with buffer
// void MatrixDriver::syncRow(uint8_t row)
// {
//   fastDigitalWrite(load_port, load_bitmask, LOW);   // begin
//   putByte(8 - row);                                 // specify register
//   putByte(_buffer[row]);                            // send data
//   fastDigitalWrite(load_port, load_bitmask, HIGH);  // begin
// }

/******************************************************************************
 * MAX7219 Configuration
 ******************************************************************************/

// sets how many digits are displayed
void MatrixDriver::setScanLimit(uint8_t value)
{
  setRegister(REG_SCANLIMIT, value & 0x07);
}

// sets brightness of the display
void MatrixDriver::setBrightness(uint8_t value)
{
  setRegister(REG_INTENSITY, value & 0x0F);
}

/******************************************************************************
 * Helper Functions
 ******************************************************************************/

// generates the output/input registers and bitmasks needed for fastDigtialWrite()
void MatrixDriver::cacheRegisters()
{
  getRegAndBitmask(&clk_port,  &clk_bitmask,  _pinClock, INPUT);
  getRegAndBitmask(&din_port,  &din_bitmask,  _pinData,  INPUT);
  getRegAndBitmask(&load_port, &load_bitmask, _pinLoad,  INPUT);
}

// Get output/input register and bitmask for a given pin
void MatrixDriver::getRegAndBitmask(volatile uint8_t** reg, volatile uint8_t* bitmask, uint8_t pin, uint8_t mode)
{
  uint8_t port = digitalPinToPort(pin);
  *reg = (mode == OUTPUT) ? portOutputRegister(port) : portInputRegister(port);
  *bitmask = digitalPinToBitMask(pin);
}

// faster replacement for digitalWrite()
inline void MatrixDriver::fastDigitalWrite(volatile uint8_t* port, volatile uint8_t mask, boolean value)
{
  if (value)
    *port |= mask; 
  else 
    *port &= ~mask;
}

/******************************************************************************
 * User API
 ******************************************************************************/

// clears screens and buffers
void MatrixDriver::clear(void)
{
  // clear buffer
  // memset(_buffer, 0x00, _buffer_size);
  _buffer[0] = 0x00;
  _buffer[1] = 0x00;
  _buffer[2] = 0x00;
  _buffer[3] = 0x00;
  _buffer[4] = 0x00;
  _buffer[5] = 0x00;
  _buffer[6] = 0x00;
  _buffer[7] = 0x00;
  
  // clear registers
  update();
}

uint8_t* MatrixDriver::getBuffer(void)
{
  return _buffer;
}

void MatrixDriver::update(void)
{
  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(8);
  putByte(_buffer[0]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);
  
  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(7);
  putByte(_buffer[1]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);

  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(6);
  putByte(_buffer[2]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);

  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(5);
  putByte(_buffer[3]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);

  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(4);
  putByte(_buffer[4]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);

  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(3);
  putByte(_buffer[5]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);

  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(2);
  putByte(_buffer[6]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);
  
  fastDigitalWrite(load_port, load_bitmask, LOW);
  putByte(1);
  putByte(_buffer[7]);
  fastDigitalWrite(load_port, load_bitmask, HIGH);
}
