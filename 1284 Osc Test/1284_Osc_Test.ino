/*
                      ATmega 1284p Development
                      Vernon Billingsley c2021
                      https://github.com/ScottBillingsley

                      Simple Oscilator reading and included sample
                      file containing 2048 8 bit samples from -128 to 127

                      Sample file constants from Mozzi tables
                      https://github.com/sensorium/Mozzi

                      DDS Method and Phase Accumulator idea from
                      http://electronics-diy.com/arduino-sinewave-generator.php


                      24 mhz external clock
                      MightyCore Bootloader
                      https://github.com/MCUdude/MightyCore
                      Standard pinout

                      Pinout
                      Pin       Function
                      1       PB0
                      2       PB1
                      3       PB2
                      4       PB3
                      5       PB4
                      6       PB5
                      7       PB6
                      8       PB7
                      9       Reset 10k resistor to +5
                      10      VCC   +5 vdc regulated
                      11      GND
                      12  ---|
                      13  ---| 24 mHz XTAL
                      14      PD0
                      15      PD1
                      16      PD2
                      17      PD3
                      18      PD4   OC1B
                      19      PD5   OC1A
                      20      PD6
                      21      PD7
                      22      PC0
                      23      PC1
                      24      PC2
                      25      PC3
                      26      PC4
                      27      PC5
                      28      PC6
                      29      PC7
                      30      AVCC    +5 vdc regulated
                      31      GND
                      32      AREF    .1 uF cap to GND
                      33      PA7
                      34      PA6
                      35      PA5
                      36      PA4
                      37      PA3
                      38      PA2
                      39      PA1
                      40      PA0

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission
    notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

/* Include the constants file */
#include "cos2048.h"

/************************* Defines ********************************/
#define DEBUG 0

#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* Sample rate, in powers of two  2^14 = 16384 */
#define SAMPLE_RATE 16384

/* Mid voltage point of the 10 bit PWM */
#define PWM_BIAS 512

/************************** Variables *****************************/

/*The measured referance clock speed */
const double ref_clk = 16381.6;
/* Store the dfreq */
double dFreq;

// variables used inside interrupt service declared as voilatile
volatile uint16_t icnt; // var inside interrupt
volatile unsigned long phaccu; // pahse accumulator
volatile unsigned long tword_m; // dds tuning word m
volatile boolean next_step = false;



/**************************  Functions ****************************/

/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }

  /************************* Setup Pins ***************************/
  /*Set PD5 to OUTPUT */
  DDRD |= _BV (5); // pinMode (5, OUTPUT);
  DDRB |= _BV (6); // pinMode (6, OUTPUT);

  /*************************  Setup Timer1 ************************/
  /* Stop interrupts */
  cli();
  /* Clear the Timer 1 registers */
  TCCR1A = 0;
  TCCR1B = 0;
  /* Initialize the count to Zero */
  TCNT1  = 0;
  /* Set PWM value max 10 bit ie, 0 to 1024 */
  OCR1A = 512;
  /* Set to Fast PWM  10 bit */
  cbi(TCCR1B, WGM13);
  sbi(TCCR1B, WGM12);
  sbi(TCCR1A, WGM11);
  sbi(TCCR1A, WGM10);

  /* Set prescaler */
  /*  CS12  CS11  CS10    prescaler
       0    0     0       No clock source, disabled
       0    0     1       System clock, no prescale
       0    1     0       clk/8
       0    1     1       clk/64
       1    0     0       clk/256
       1    0     1       clk/1024
  */
  cbi(TCCR1B, CS12);
  cbi(TCCR1B, CS11);
  sbi(TCCR1B, CS10);

  /* Enable timer compare interrupt */
  /* This will enable the ISR to be used */
  //sbi(TIMSK1, OCIE1A);

  /*Set OC1A to non-inverting mode  */
  /* Pin PD5 Clear on Compare, Set at BOTTOM */
  sbi(TCCR1A, COM1A1);
  cbi(TCCR1A, COM1A0);
  /* Pin PD4 normal operation */
  cbi(TCCR1A, COM1B1);
  cbi(TCCR1A, COM1B0);

  /*Allow interrupts */
  sei();

  /*Use Timer 3 to control sample time */

  /*************************  Setup Timer3 ************************/
  /* Stop interrupts */
  cli();
  /* Clear the Timer 1 registers */
  TCCR3A = 0;
  TCCR3B = 0;
  /* Initialize the count to Zero */
  TCNT3  = 0;
  /* Set compare match register value */
  /* F_CPU / ( prescaler * time )  */
  OCR3A = F_CPU / SAMPLE_RATE;
  /* Set to Clear Timer on Compare Match */
  sbi(TCCR3B, WGM32);

  /* Set prescaler */
  /*  CS12  CS11  CS10    prescaler
       0    0     0       No clock source, disabled
       0    0     1       System clock, no prescale
       0    1     0       clk/8
       0    1     1       clk/64
       1    0     0       clk/256
       1    0     1       clk/1024
  */
  cbi(TCCR3B, CS32);
  cbi(TCCR3B, CS31);
  sbi(TCCR3B, CS30);

  /* Enable timer compare interrupt */
  /* This will enable the ISR to be used */
  sbi(TIMSK3, OCIE3A);

  /*Toggle OC1A/OC1B on compare match */
  /* Pin PB6 normal operation */
  cbi(TCCR3A, COM3A1);
  cbi(TCCR3A, COM3A0);
  /* Pin PB7 normal operation */
  cbi(TCCR3A, COM3B1);
  cbi(TCCR3A, COM3B0);

  /*Allow interrupts */
  sei();

  /*Initial freq */
  dFreq = 110;    //100 Hz
  /* calulate DDS new tuning word */
  tword_m = pow(2, 32) * dFreq / ref_clk;

}/**************************  End Setup **************************/

/*ISR to handle the Timer 1 interrupt */
ISR(TIMER1_COMPA_vect) {

}

/*ISR to handle the Timer 3 interrupt */
ISR(TIMER3_COMPA_vect) {
  /* Set the next value */
  OCR1A = PWM_BIAS + (COS2048_DATA[icnt] << 2);
  next_step = true;
}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {
  if (next_step) {
    phaccu = phaccu + tword_m; // soft DDS, phase accu with 32 bits
    icnt = phaccu >> 21; // use upper 11 bits for phase accu as frequency information
    next_step = false;
  }



}/*************************** End Loop *****************************/
