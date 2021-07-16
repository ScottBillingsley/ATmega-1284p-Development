/*
                      ATmega 1284p Development

                      Fast PWM with sine output using
                      Timer 1 mode 7 Fast PWM, 10 bit
                      Toggle pin PD5
                      Timer 3 used as system timer running at
                      16384 Hz producing a 128 Hz sine wave using
                      128 sample array..

                      Vernon Billingsley c2021
                      https://github.com/ScottBillingsley

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
volatile int16_t counter = 0;

/* 128 samples at 1020 max value, 10 bit */
volatile uint16_t sine[] = {
  0x1fe, 0x217, 0x230, 0x249, 0x261, 0x27a, 0x292, 0x2aa, 0x2c1, 0x2d8, 0x2ee, 0x304, 0x319, 0x32e, 0x342, 0x354, 0x367, 0x378, 0x388, 0x398, 0x3a6, 0x3b3, 0x3c0, 0x3cb, 0x3d5, 0x3de, 0x3e6, 0x3ed, 0x3f2, 0x3f6, 0x3fa, 0x3fb,
  0x3fc, 0x3fb, 0x3fa, 0x3f6, 0x3f2, 0x3ed, 0x3e6, 0x3de, 0x3d5, 0x3cb, 0x3c0, 0x3b3, 0x3a6, 0x398, 0x388, 0x378, 0x367, 0x354, 0x342, 0x32e, 0x319, 0x304, 0x2ee, 0x2d8, 0x2c1, 0x2aa, 0x292, 0x27a, 0x261, 0x249, 0x230, 0x217,
  0x1fe, 0x1e5, 0x1cc, 0x1b3, 0x19b, 0x182, 0x16a, 0x152, 0x13b, 0x124, 0x10e, 0xf8, 0xe3, 0xce, 0xba, 0xa8, 0x95, 0x84, 0x74, 0x64, 0x56, 0x49, 0x3c, 0x31, 0x27, 0x1e, 0x16, 0xf, 0xa, 0x6, 0x2, 0x1,
  0x0, 0x1, 0x2, 0x6, 0xa, 0xf, 0x16, 0x1e, 0x27, 0x31, 0x3c, 0x49, 0x56, 0x64, 0x74, 0x84, 0x95, 0xa8, 0xba, 0xce, 0xe3, 0xf8, 0x10e, 0x124, 0x13b, 0x152, 0x16a, 0x182, 0x19b, 0x1b3, 0x1cc, 0x1e5,
};

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
  

  /*************************  Setup Timer1 ************************/
  /* Stop interrupts */
  cli();
  /* Clear the Timer 1 registers */
  TCCR1A = 0;
  TCCR1B = 0;
  /* Initialize the count to Zero */
  TCNT1  = 0;
  /* Set PWM value max 10 bit ie, 0 to 1024 */
  OCR1A = sine[counter];
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

}/**************************  End Setup **************************/

/*ISR to handle the Timer 1 interrupt */
ISR(TIMER1_COMPA_vect) {

}

/*ISR to handle the Timer 3 interrupt */
ISR(TIMER3_COMPA_vect) {
  /* Set the next value */
  OCR1A = sine[counter];
  /* Increment the count */
  counter ++;
  /* If greater the 127, reset the counter */
  if (counter > 127) {
    counter = 0;
  }

}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {

}/*************************** End Loop *****************************/
