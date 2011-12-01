#include <FlexiTimer2.h>
#include <MatrixDriver.h>
#include <TimerOne.h>

// Set the delay for the frequency of power (65 for 60Hz, 78 for 50Hz) per step
// (using 128 steps). freqStep may need some adjustment depending on your power.
// the formula you need to use is (500000/AC_freq)/NumSteps = freqStep. You
// could also write a seperate function to determine the freq
#define FREQ_STEP 65

#define NUM_ROW   8
#define NUM_COL   8
#define NUM_LEDS  64

#define DEBUG_TIMER 1
// #define STEP_DEBUGGER 1

// pin 0: data  (din)
// pin 1: load  (load)
// pin 2: clock (clk)
MatrixDriver matrix = MatrixDriver(4, 3, 2);

// get a pointer to the display buffer so we can update it directly
uint8_t*            buffer = matrix.getBuffer();
uint8_t             dim_value[NUM_LEDS];
volatile uint8_t    dim_count[NUM_LEDS];
int                 dim_arr_size = sizeof(uint8_t)*NUM_LEDS;  // cache the size of the dim_* arrays
uint8_t             num_switched;
volatile bool       zero_cross; // Boolean to store a "switch" to tell us if we have crossed zero

// NOTE: could make this 56 bytes (NUM_LEDS*7/8)
uint8_t  serial_buffer[NUM_LEDS];
int      bytes_needed = NUM_LEDS;

#ifdef DEBUG_TIMER
int loop_count = 0;
void report() {
    Serial.println(loop_count);
    loop_count = 0;
}
#endif

void detected_zero_cross() {    
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  cli();
  memset((void*)dim_count, 0x0, dim_arr_size);
  sei();
  matrix.clear();
  num_switched = 0;

#ifdef DEBUG_TIMER
  Serial.println(loop_count, DEC);
  loop_count = 0;
#endif

#ifdef STEP_DEBUGGER
  Serial.println("zero cross detected");
#endif
}

// Function will fire the triac at the proper time
void dim_check() {                   
#ifdef STEP_DEBUGGER
    Serial.println("dim check");
#endif
#ifdef DEBUG_TIMER
    ++loop_count;
#endif
    
    if(zero_cross == true) {              
        // Serial.println("\tupdating");
        // get a pointer into the matrix array
        uint8_t* bufPtr = buffer;
        
        for(uint8_t i=0; i<NUM_LEDS; i++) {
            if(dim_count[i] >= dim_value[i]) {
                // since our dim values can only be 0-128, we reserve 255 as a
                // special way of marking this count as done
                if(dim_count[i] != 255) {
                    // mark it done
                    dim_count[i] = 255;
                    // increase our running total
                    ++num_switched;
                }
                // set its bit in the matrix to off
                (*bufPtr) &= 0x0;
            } else {
                // set its bit in the matrix to on
                (*bufPtr) |= 0x1;
                // increment the count
                dim_count[i] += 1;
            }

            // check to see if we're byte aligned
            if((i + 1) % 8 == 0)
                // move to the next byte
                ++bufPtr;
            else
                // inspect the next bit
                (*bufPtr) <<= 1;
        }

        // have all lights exceeded their dim value?
        if(num_switched >= NUM_LEDS) {
            zero_cross   = false;
            num_switched = 0;
        }
        
        // repaint the matrix
        matrix.update();
    }
}

#ifdef STEP_DEBUGGER
void print_dim_value()
{
    Serial.println("-- dim_value --");
    for(uint8_t i=0; i<NUM_ROW; i++) {    
        int offset = i*NUM_ROW;
        for(int j=0; j<NUM_COL; j++) {
            Serial.print(dim_value[offset+j], DEC);
            Serial.print('\t');
        }
        Serial.println();
    }
    Serial.println('-');
}

void print_dim_count()
{
    Serial.println("-- dim_count --");
    Serial.print("num_switched: ");
    Serial.println(num_switched, DEC);
    for(uint8_t i=0; i<NUM_ROW; i++) {    
        int offset = i*NUM_ROW;
        for(int j=0; j<NUM_COL; j++) {
            Serial.print(dim_count[offset+j], DEC);
            Serial.print('\t');
        }
        Serial.println();
    }
    Serial.println('-');
}

void print_buffer()
{
    Serial.println("-- buffer --");
    for(uint8_t i=0; i<NUM_ROW; i++) {    
        Serial.print(buffer[i], DEC);
        Serial.print('\t');
    }
    Serial.println();
    Serial.println('-');    
}
#endif

void setup()
{
    Serial.begin(57600);
    matrix.setBrightness(0x0D);

    memset(dim_value, 0x0, dim_arr_size);
    for(uint8_t i=0; i<NUM_ROW; i++) {    
        int offset = i*NUM_ROW;
        for(int j=0; j<NUM_COL; j++)
            dim_value[offset+j] = (1 << j);
    }

    cli();
    memset((void*)dim_count, 0x0, dim_arr_size);
    sei();
    num_switched = 0;
    zero_cross   = false;
    
#ifndef STEP_DEBUGGER
    FlexiTimer2::set(1, 1.0/120.0, detected_zero_cross);
    FlexiTimer2::start();
    // attachInterrupt(0, detected_zero_cross, RISING);   // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection

    Timer1.initialize(FREQ_STEP);                      // Initialize TimerOne library for the freq we need
    Timer1.attachInterrupt(dim_check, FREQ_STEP);      
#endif

#ifdef DEBUG_TIMER
    loop_count = 0;
    // FlexiTimer2::set(1000, report); 
    // FlexiTimer2::start();
#endif
}

void loop()
{
    if (Serial.available() > 0) {
        // get incoming byte
        byte inByte = Serial.read();

#ifdef STEP_DEBUGGER
        switch(inByte) {
            case 's':
                Serial.println("============================================");
                dim_check();
                print_dim_value();
                print_dim_count();
                print_buffer();                
                break;
            case 'd':
                dim_check();
                break;
            case 'z':
                detected_zero_cross();
                break;
            case 'v':
                print_dim_value();
                break;
            case 'c':
                print_dim_count();
                break;
            case 'b':
                print_buffer();
                break;                
        }
#endif
    }
// #ifdef DEBUG_TIMER 
//     loop_count++;
// #endif
}

