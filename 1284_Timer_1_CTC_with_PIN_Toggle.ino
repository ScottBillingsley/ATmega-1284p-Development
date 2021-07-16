/*
                      ATmega 1284p Development

                      Timer 1 CTC with Pin 5 Toggle
                      As written, will produce a 6 kHz clock
                      output on pin PD5

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
#define DEBUG 1

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

/************************** Variables *****************************/


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
  /* Set compare match register value */
  /* F_CPU / ( prescaler * time )  */
  /* 24 mHz / ( 1 * 6 kHz) = 2000  */
  OCR1A = 2000; 
  /* Set to Clear Timer on Compare Match */
  sbi(TCCR1B, WGM12);

  /* Set prescaler */
  /*  CS12  CS11  CS10    prescaler
   *   0    0     0       No clock source, disabled
   *   0    0     1       System clock, no prescale
   *   0    1     0       clk/8
   *   0    1     1       clk/64
   *   1    0     0       clk/256
   *   1    0     1       clk/1024
   */
  cbi(TCCR1B, CS12);
  cbi(TCCR1B, CS11);
  sbi(TCCR1B, CS10);

  /* Enable timer compare interrupt */
  /* This will enable the ISR to be used */
  //sbi(TIMSK1, OCIE1A);

  /*Toggle OC1A/OC1B on compare match */
  /* Pin PD5 */
  cbi(TCCR1A, COM1A1);
  sbi(TCCR1A, COM1A0);
  /* Pin PD4 */
  //cbi(TCCR1A, COM1B1);
  //sbi(TCCR1A, COM1B0);


  /*Allow interrupts */
  sei();

}/**************************  End Setup **************************/

/*ISR to handle the Timer 1 interrupt */
ISR(TIMER1_COMPA_vect) {

}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {


}/*************************** End Loop *****************************/
