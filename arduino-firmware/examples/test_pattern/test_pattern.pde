#include <MatrixDriver.h>
#include <MsTimer2.h>

#define DEBUG_TIMER 1

/* create a new Matrix instance
   pin 0: data  (din)
   pin 1: load  (load)
   pin 2: clock (clk)
*/
MatrixDriver matrix = MatrixDriver(4, 3, 2);

// get a pointer to the display buffer so we can update it directly
uint8_t* buffer = matrix.getBuffer();

int swap_count = 0;
uint8_t pattern = 0;
unsigned long timer = micros();

#ifdef DEBUG_TIMER

int loop_count = 0;

void report() {
  Serial.println(loop_count);
  loop_count = 0;
}

#endif

void setup()
{
  Serial.begin(9600);

#ifdef DEBUG_TIMER
  loop_count = 0;
  MsTimer2::set(1000, report); 
  MsTimer2::start();
#endif
}

void loop()
{
  // turn some pixels on
  if(swap_count > 1000) {
    swap_count = 0;
    ++pattern;
    if (pattern > 3)
      pattern = 0;
  }

  switch(pattern) {
    case 0:
      buffer[0] = B11111111;
      buffer[1] = B11111111;
      buffer[2] = B11111111;
      buffer[3] = B11111111;
      buffer[4] = B11111111;
      buffer[5] = B11111111;
      buffer[6] = B11111111;
      buffer[7] = B11111111;
      matrix.update();
      break;
    case 1:
      buffer[0] = B10101010;
      buffer[1] = B01010101;
      buffer[2] = B10101010;
      buffer[3] = B01010101;
      buffer[4] = B10101010;
      buffer[5] = B01010101;
      buffer[6] = B10101010;
      buffer[7] = B01010101;
      matrix.update();
      break;
    case 2:
      buffer[0] = B11001100;
      buffer[1] = B11001100;
      buffer[2] = B00110011;
      buffer[3] = B00110011;
      buffer[4] = B11001100;
      buffer[5] = B11001100;
      buffer[6] = B00110011;
      buffer[7] = B00110011;
      matrix.update();
      break;
    default:
      matrix.clear(); // clear display
      break;
  }

  ++swap_count;
  
#ifdef DEBUG_TIMER 
  loop_count++;
#endif
}

