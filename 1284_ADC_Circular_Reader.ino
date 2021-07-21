/*
                      ATmega 1284p Development
                      Read the ADCs in a circular poll..
                      
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
boolean print_array = false;

/*****************  About the ADC  **********************/
   /*Store the current ADC read */
   unsigned int adc_array[8];
   /*Store the old ADC read for compare */
   unsigned int old_adc_array[8];
   /* The active ADC */
   byte adc_count = 0;
 

/**************************  Functions ****************************/

/* Change the adc channel for next read.. */

void change_adc(byte adc) {

  switch (adc) {
    case 0:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      cbi(ADMUX, MUX2);
      cbi(ADMUX, MUX1);
      cbi(ADMUX, MUX0);
      break;
    case 1:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      cbi(ADMUX, MUX2);
      cbi(ADMUX, MUX1);
      sbi(ADMUX, MUX0);
      break;
    case 2:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      cbi(ADMUX, MUX2);
      sbi(ADMUX, MUX1);
      cbi(ADMUX, MUX0);
      break;
    case 3:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      cbi(ADMUX, MUX2);
      sbi(ADMUX, MUX1);
      sbi(ADMUX, MUX0);
      break;
    case 4:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      sbi(ADMUX, MUX2);
      cbi(ADMUX, MUX1);
      cbi(ADMUX, MUX0);
      break;
    case 5:
      cbi(ADMUX, MUX4);
      cbi(ADMUX, MUX3);
      sbi(ADMUX, MUX2);
      cbi(ADMUX, MUX1);
      sbi(ADMUX, MUX0);
      break;
  }

  /*Start the next conversion */
  sbi(ADCSRA, ADSC);

}

/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }

  /************************* Setup Pins ***************************/
  /*************************  Setup ADC ***************************/
  /*Set to Right Adjust for 1024 precision */
  cbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  /*Set to ADC0 to start */
  //cbi(ADMUX, MUX4);
  //cbi(ADMUX, MUX3);
  //cbi(ADMUX, MUX2);
  //cbi(ADMUX, MUX1);
  //cbi(ADMUX, MUX0);

  /*Set prescaler to 64 */
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  sbi(ADCSRA, ADEN);

  /*Start the circular counter on zero */
  change_adc(adc_count);

  /*Start the first conversion */
  //sbi(ADCSRA, ADSC);


}/**************************  End Setup **************************/


/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {

  /*Check to see if ADC has finished */
  if (!(bitRead(ADCSRA, ADSC))) {
    /*Read and store the results  */
    uint8_t temp_adcl = ADCL;
    uint16_t temp_adc = (ADCH << 8) + temp_adcl;
    /* Store the adc read in the adc array */
    switch(adc_count){
      case 0:
         /*Keep a running average */
         adc_array[0] = (adc_array[0] + temp_adc) / 2;
         
         /*Store the old value */
         old_adc_array[0] = adc_array[0];         
         /* Increment the adc count */
         adc_count = 1;
          /* Change the mux */
          change_adc(adc_count);
          break;         
      case 1:
         /*Keep a running average */
         adc_array[1] = (adc_array[1] + temp_adc) / 2;
         
         /*Store the old value */
         old_adc_array[1] = adc_array[1];         
         /* Increment the adc count */
         adc_count = 2;
          /* Change the mux */
          change_adc(adc_count); 
          break;
      case 2:
         /*Keep a running average */
         adc_array[2] = (adc_array[2] + temp_adc) / 2;
         
         /*Store the old value */
         old_adc_array[2] = adc_array[2];         
         /* Increment the adc count */
         adc_count = 3;
          /* Change the mux */
          change_adc(adc_count); 
          break;
      case 3:
         /*Keep a running average */
         adc_array[3] = (adc_array[3] + temp_adc) / 2;
         
         /*Store the old value */
         old_adc_array[3] = adc_array[3];         
         /* Increment the adc count */
         adc_count = 4;
          /* Change the mux */
          change_adc(adc_count);
          break; 
       case 4:
         /*Keep a running average */
         adc_array[4] = (adc_array[4] + temp_adc) / 2;
         
         /*Store the old value */
         old_adc_array[4] = adc_array[4];         
         /* Increment the adc count */
         adc_count = 0;
          /* Change the mux */
          change_adc(adc_count); 
          print_array = true;
          break;
                                             
    }
  delay(1);
  }

  
  if(print_array){
    Serial.print(adc_array[0]);
    Serial.print(", ");
    Serial.print(adc_array[1]);
    Serial.print(", ");
    Serial.print(adc_array[2]);
    Serial.print(", ");
    Serial.print(adc_array[3]);
    Serial.print(", ");
    Serial.print(adc_array[4]);
    Serial.println();
   

    print_array = false;
  }


}/*************************** End Loop *****************************/
