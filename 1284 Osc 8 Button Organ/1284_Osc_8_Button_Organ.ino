/*
                      ATmega 1284p Development
                      Vernon Billingsley c2021
                      https://github.com/ScottBillingsley

                      Simple Oscilator reading an included sample
                      file containing 2048 8 bit samples from -128 to 127,

                      With AM modulation using an envelope
                      8 button keyboard

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
                      1       PB0   LED
                      2       PB1   Button
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
#include "decay256.h"
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

/* Control the rate of checking controls */
#define CONTROL_RATE 128

/* Mid voltage point of the 10 bit PWM */
#define PWM_BIAS 512

/************************** Variables *****************************/

/*The measured referance clock speed */
const double ref_clk = 16381.6;
/* Store the dfreq */
double dFreq;

// variables used inside interrupt service declared as voilatile
volatile int32_t sample;
volatile uint16_t icnt; // var inside interrupt

volatile unsigned long phaccu; // pahse accumulator

volatile unsigned long tword_m; // dds tuning word m

/*Notes */
const unsigned long c_tword_m = 34345896;
const unsigned long d_tword_m = 38540816;
const unsigned long e_tword_m = 43260096;
const unsigned long f_tword_m = 46144104;
const unsigned long g_tword_m = 51387752;
const unsigned long a_tword_m = 57680132;
const unsigned long b_tword_m = 64759056;
const unsigned long c2_tword_m = 68691792;

volatile boolean next_step = false;

volatile uint16_t control_count;
volatile uint16_t control_match;

/******* About the envelope *************/
volatile uint8_t mod_count = 0;
volatile uint16_t mod_tick;
boolean envelope = false;
boolean att = false;
boolean rel = false;
boolean button_check = false;
boolean octave = false;


/******** About the ADC *****************/
uint16_t adc0 = 0;
uint16_t adc_array[4];
uint16_t old_adc0 = 0;

/*About the filter  */
const float alpha = .25;
/*Store the filtered sample */
uint16_t adc_filtered[4] = {0, 0, 0,};
/*Store the previous filtered sample */
uint32_t f_v[4][3];

/**************************  Functions ****************************/
/* Sample rate tick timer, place any counter to increment here */
void tick_hook() {
  mod_tick ++;
  /* Fixed an anoying over-flow tick */
  if (mod_tick > 16384) {
    mod_tick = 0;
  }
}

void filter(byte ary_num) {
  /*Calculate the new value */
  f_v[ary_num][1] = (float)alpha * adc_array[ary_num] + (1 - alpha) * f_v[ary_num][0];
  /*Store the old value */
  f_v[ary_num][0] = f_v[ary_num][1];

  /*Store the filtered value */
  adc_filtered[ary_num] = f_v[ary_num][1];
}


/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }

  /************************* Setup Pins ***************************/

  for (byte p = 16; p < 24; p ++) {
    pinMode(p, INPUT_PULLUP);
  }

  pinMode(1, INPUT_PULLUP);

  /*Set PD5 to OUTPUT */
  //DDRD |= _BV (5); // pinMode (5, OUTPUT);
  pinMode(13, OUTPUT);

  /* PB0, pin 1 as OUTPUT for LED */
  DDRB |= _BV (0); // pinMode (0, OUTPUT);


  /*************************  Setup ADC ***************************/
  /*Set to Right Adjust for 1024 precision */
  cbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  /*Set to ADC0 to start */
  cbi(ADMUX, MUX3);
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  cbi(ADMUX, MUX0);

  /*Set prescaler to 64 */
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  cbi(ADCSRA, ADEN);

  /*Start the first conversion */
  cbi(ADCSRA, ADSC);

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

  control_match = SAMPLE_RATE / CONTROL_RATE;


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
  /*System rate tick timer */
  tick_hook();
  
  /* Set the next value */
  OCR1A = PWM_BIAS + sample; // (COS2048_DATA[icnt] << 2) ;
  
  /* Get the next tword */  
  phaccu = phaccu + tword_m; // soft DDS, phase accu with 32 bits
  /* Get the next phase position */
  icnt = phaccu >> 21; // use upper 11 bits for phase accu as frequency information

  /* Fixed point math to calculate the envelope */
  int32_t ww = COS2048_DATA[icnt] << 2;
  /* 127 from the decay table is aprox. multiply by 1 */
  int32_t mm = DECAY256_DATA[mod_count];
  /* Same as sample * mod, ie  256 * .5 = 128 */
  sample =  (((ww << 8) * mm) >> 16);

  /*Check the controls at the control rate */
  control_count ++;
  if (control_count >= control_match) {
    control_count = 0;
    next_step = true;
  }
  
}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {

  /*****  Envelope control ****/
  /* ATTACK */
  if (envelope && att) {
    /*Attack speed of the envelope */
    uint16_t env_speed = 4;
    if (mod_tick >= env_speed) {
      /*Increment the mod counter */
      mod_count += 2;
      /*Clear the tick counter */
      mod_tick = 0;
    }
    if (mod_count == 126) {
      att = false;
    }
  }
  /* RELEASE */
  if (!envelope && rel) {
    /*Release speed of the envelope */
    uint16_t env_speed = 64;
    if (mod_tick >= env_speed) {
      /*Increment the mod counter */
      mod_count += 2;
      /*Clear the tick counter */
      mod_tick = 0;
    }
    if (mod_count >= 254) {
      rel = false;
    }
  }

  /*If the button is pressed start the envelope */
  uint8_t p = PINC;

  if (p != 255 && !button_check) {
    /*Determine the key pressed and set the new tword*/

    for (byte k = 0; k < 8; k ++) {
      byte h = p & 0B00000001;
      if (!h) {
        switch (k) {
          case 0:
            if (octave) {
              tword_m = (c_tword_m << 1);
            } else {
              tword_m = c_tword_m;
            }
            break;
          case 1:
            if (octave) {
              tword_m = (d_tword_m << 1);
            } else {
              tword_m = d_tword_m;
            }
            break;
          case 2:
            if (octave) {
              tword_m = (e_tword_m << 1);
            } else {
              tword_m = e_tword_m;
            }
            break;
          case 3:
            if (octave) {
              tword_m = (f_tword_m << 1);
            } else {
              tword_m = f_tword_m;
            }
            break;
          case 4:
            if (octave) {
              tword_m = (g_tword_m << 1);
            } else {
              tword_m = g_tword_m;
            }
            break;
          case 5:
            if (octave) {
              tword_m = (a_tword_m << 1);
            } else {
              tword_m = a_tword_m;
            }
            break;
          case 6:
            if (octave) {
              tword_m = (b_tword_m << 1);
            } else {
              tword_m = b_tword_m;
            }
            break;
          case 7:
            if (octave) {
              tword_m = (c2_tword_m << 1);
            } else {
              tword_m = c2_tword_m;
            }
            break;
        }
      }
      p = p >> 1;
    }

    PORTB |= _BV (0); // digitalWrite (0, HIGH);
    envelope = true;
    att = true;
    button_check = true;
  }

  /*Button check true, hold the envelope */

  /*If the button is released, start the release and turn off the envelope */

  if (p == 255 && button_check) {
    PORTB &= ~_BV (0); // digitalWrite (0, LOW);
    envelope = false;
    rel = true;
    button_check = false;
  }

  /*Octave button */
  if (!(PINB & _BV (1))) {
    octave = true;
  } else {
    octave = false;
  }

}/*************************** End Loop *****************************/
