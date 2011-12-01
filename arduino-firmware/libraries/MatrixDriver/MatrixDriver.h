#ifndef MATRIX_DRIVER_H
#define MATRIX_DRIVER_H

#include "WProgram.h"

class MatrixDriver
{
  private:
    uint8_t  _pinData;
    uint8_t  _pinClock;
    uint8_t  _pinLoad;
    uint8_t  _buffer[8];
    int      _buffer_size;
    // uint8_t* _buffer;
    // uint8_t  _screens;
    // uint8_t  _height;
    // uint8_t  _width;
    // uint8_t  _maximumX;

    // Output/input registers and bitmasks
    volatile uint8_t* clk_port;
    volatile uint8_t  clk_bitmask;
    volatile uint8_t* din_port;
    volatile uint8_t  din_bitmask;
    volatile uint8_t* load_port;
    volatile uint8_t  load_bitmask;

    void cacheRegisters();
    void getRegAndBitmask(volatile uint8_t** reg, volatile uint8_t* bitmask, uint8_t pin, uint8_t mode);
    void fastDigitalWrite(volatile uint8_t* port, volatile uint8_t mask, boolean value);
    void putByte(uint8_t);
    void setRegister(uint8_t, uint8_t);
    void setScanLimit(uint8_t);
    // void buffer(uint8_t x, uint8_t y, uint8_t value);
    // void syncRow(uint8_t);

  public:
    MatrixDriver(uint8_t data, uint8_t clock, uint8_t load);
    void     setBrightness(uint8_t value);
    void     clear(void);
    uint8_t* getBuffer(void);
    void     update(void);
    // void     write(uint8_t x, uint8_t y, uint8_t value);
};

#endif

