/*
                      ATmega 1284p Development

                      Timer 3 CTC with Pin 6 Toggle
                      As written, will produce a 6 kHz clock
                      output on pin PB6

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
                      7       PB6   OC3A
                      8       PB7   OC3B
                      9       Reset 10k resistor to +5
                      10      VCC   +5 vdc regulated
                      11      GND
                      12  ---|
                      13  ---| 24 mHz XTAL
                      14      PD0
                      15      PD1
                      16      PD2
                      17      PD3
                      18      PD4  
                      19      PD5  
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
  /*Set PB6 to OUTPUT */
  DDRB |= _BV (6); // pinMode (6, OUTPUT);

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
  /* 24 mHz / ( 1 * 6 kHz) = 2000  */
  OCR3A = 2000; 
  /* Set to Clear Timer on Compare Match */
  sbi(TCCR3B, WGM32);

  /* Set prescaler */
  /*  CS12  CS11  CS10    prescaler
   *   0    0     0       No clock source, disabled
   *   0    0     1       System clock, no prescale
   *   0    1     0       clk/8
   *   0    1     1       clk/64
   *   1    0     0       clk/256
   *   1    0     1       clk/1024
   */
  cbi(TCCR3B, CS32);
  cbi(TCCR3B, CS31);
  sbi(TCCR3B, CS30);

  /* Enable timer compare interrupt */
  /* This will enable the ISR to be used */
  //sbi(TIMSK3, OCIE3A);

  /*Toggle OC1A/OC1B on compare match */
  /* Pin PB6 */
  cbi(TCCR3A, COM3A1);
  sbi(TCCR3A, COM3A0);
  /* Pin PB7 */
  //cbi(TCCR3A, COM3B1);
  //sbi(TCCR3A, COM3B0);


  /*Allow interrupts */
  sei();

}/**************************  End Setup **************************/

/*ISR to handle the Timer 3 interrupt */
ISR(TIMER3_COMPA_vect) {

}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {


}/*************************** End Loop *****************************/
