#include <LiquidCrystal_I2C.h>

PROGMEM const unsigned char sine256[] = {
  0, 6, 12, 19, 25, 31, 37, 43, 50, 56,
  62, 68, 74, 80, 86, 91, 97, 103, 109, 114,
  120, 125, 131, 136, 141, 146, 151, 156, 161, 166,
  171, 175, 180, 184, 188, 192, 196, 200, 204, 208,
  211, 215, 218, 221, 224, 227, 230, 232, 235, 237,
  239, 241, 243, 245, 246, 248, 249, 250, 251, 252,
  253, 253, 254, 254, 254, 254, 254, 253, 253, 252,
  251, 250, 249, 248, 246, 245, 243, 241, 239, 237,
  235, 232, 230, 227, 224, 221, 218, 215, 211, 208,
  204, 200, 196, 192, 188, 184, 180, 175, 171, 166,
  161, 156, 151, 146, 141, 136, 131, 125, 120, 114,
  109, 103, 97, 91, 86, 80, 74, 68, 62, 56,
  50, 43, 37, 31, 25, 19, 12, 6,

  0, 6, 12, 19, 25, 31, 37, 43, 50, 56,
  62, 68, 74, 80, 86, 91, 97, 103, 109, 114,
  120, 125, 131, 136, 141, 146, 151, 156, 161, 166,
  171, 175, 180, 184, 188, 192, 196, 200, 204, 208,
  211, 215, 218, 221, 224, 227, 230, 232, 235, 237,
  239, 241, 243, 245, 246, 248, 249, 250, 251, 252,
  253, 253, 254, 254, 254, 254, 254, 253, 253, 252,
  251, 250, 249, 248, 246, 245, 243, 241, 239, 237,
  235, 232, 230, 227, 224, 221, 218, 215, 211, 208,
  204, 200, 196, 192, 188, 184, 180, 175, 171, 166,
  161, 156, 151, 146, 141, 136, 131, 125, 120, 114,
  109, 103, 97, 91, 86, 80, 74, 68, 62, 56,
  50, 43, 37, 31, 25, 19, 12, 6

};
/*
PROGMEM const unsigned char sine256[] = {
  127, 130, 133, 136, 139, 143, 146, 149, 152, 155,
  158, 161, 164, 167, 170, 173, 176, 178, 181, 184,
  187, 190, 192, 195, 198, 200, 203, 205, 208, 210,
  212, 215, 217, 219, 221, 223, 225, 227, 229, 231,
  233, 234, 236, 238, 239, 240, 242, 243, 244, 245,
  247, 248, 249, 249, 250, 251, 252, 252, 253, 253,
  253, 254, 254, 254, 254, 254, 254, 254, 253, 253, 
  253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 
  244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 
  229, 227, 225, 223, 221, 219, 217, 215, 212, 210, 
  208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 
  181, 178, 176, 173, 170, 167, 164, 161, 158, 155, 
  152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 
  121, 118, 115, 111, 108, 105, 102,  99,  96,  93, 
   90,  87,  84,  81,  78,  76,  73,  70,  67,  64, 
   62,  59,  56,  54,  51,  49,  46,  44,  42,  39, 
   37,  35,  33,  31,  29,  27,  25,  23,  21,  20, 
   18,  16,  15,  14,  12,  11,  10,   9,   7,   6, 
    5,   5,   4,   3,   2,   2,   1,   1,   1,   0,
    0,   0,   0,   0,   0,   0,   1,   1,   1,   2, 
    2,   3,   4,   5,   5,   6,   7,   9,  10,  11, 
   12,  14,  15,  16,  18,  20,  21,  23,  25,  27, 
   29,  31,  33,  35,  37,  39,  42,  44,  46,  49, 
   51,  54,  56,  59,  62,  64,  67,  70,  73,  76, 
   78,  81,  84,  87,  90,  93,  96,  99, 102, 105,
  108, 111, 115, 118, 121, 124
};
*/
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// LiquidCrystal_I2C lcd(0x27, 16, 2);  //0X3F

#define start 2
#define stop 3
void Setup_timer1();
volatile float freq = 1;
const float refclk = 122.549;  //  16 MHz/510/256
volatile unsigned long sigma;  // phase accumulator
volatile unsigned long delta;  // phase increment
byte phase0;
unsigned int value1;

unsigned int VR_in, F_in, F_out = 0, valueVR;
unsigned long timeMillis = 0;
unsigned long timeDelay = 0, timeButton1 = 0, timeButton2 = 0;
char bitStart = 0;

void setup() {
  Serial.begin(9600);

  pinMode(9, OUTPUT);   // pin9= PWM  output / frequency output
  pinMode(10, OUTPUT);  // pin10= PWM  output / frequency output
  pinMode(11, OUTPUT);  // pin11= PWM  output / frequency output
  pinMode(6, OUTPUT);   // pin6= PWM  output / frequency output
  pinMode(5, OUTPUT);   // pin5= PWM  output / frequency output
  pinMode(start, INPUT_PULLUP);
  pinMode(stop, INPUT_PULLUP);

  Setup_timer2();
  Setup_timer1();
  Setup_timer0();
  cbi(TIMSK2, TOIE2);
  // lcd.init();
  // lcd.backlight();
  // lcd.setCursor(2, 0);
  // lcd.print("TAN SO CAI DAT");
  // lcd.setCursor(5, 1);
  // lcd.print("Hz");


  // the waveform index is the highest 8 bits of sigma
  // choose refclk as freq to increment the lsb of the 8 highest bits
  //    for every call to the ISR of timer2 overflow
  // the lsb of the 8 highest bits is 1<<24 (1LL<<24 for long integer literal)
  delta = (1LL << 24) * freq / refclk;
}

void loop() {

  //changeFreq(20);
  //delay(10000);
  //changeFreq(25);
  //delay(10000);



  if ((unsigned long)millis() - timeDelay > 200) {
    VR_in = analogRead(A0);
    F_in = map(VR_in, 0, 1023, 0, 100);
    valueVR = analogRead(A0);
    F_in = map(valueVR, 0, 1023, 0, 99);
    // lcd.setCursor(2, 0);
    // lcd.print("TAN SO CAI DAT");
    // lcd.setCursor(5, 1);
    // lcd.print("Hz");
    // lcd.setCursor(1, 1);
    // lcd.print(F_in);
  }
  if (bitStart == 1) {
    if (F_in != F_out) {
      F_out = F_in;
      changeFreq(F_out);
    }
  }

  if (bitStart == 0) {
    TIMSK2 = 0b00000000;
    OCR1A = 0;
    OCR1B = 0;
    OCR0A = 0;
    OCR0B = 0;
  }

  if (digitalRead(start) == 0) {
    timeButton1 = millis();
    while (digitalRead(start) == 0)
      ;
    if ((unsigned long)millis() - timeButton1 > 100) {
      bitStart = 1;
      F_out = F_in;
      changeFreq(F_out);
    }
  }

  if (digitalRead(stop) == 0) {
    timeButton2 = millis();
    while (digitalRead(stop) == 0)
      ;
    if ((unsigned long)millis() - timeButton2 > 100) {
      bitStart = 0;
    }
  }
}


void changeFreq(float _freq) {
  cbi(TIMSK2, TOIE2);  // disable timer2 overflow detect   - tắt ngắt timer2
  freq = _freq;
  delta = (1LL << 24) * freq / refclk;  // update phase increment
  sbi(TIMSK2, TOIE2);                   // enable timer2 overflow detect     - bật ngắt timer 2
}



//******************************************************************

/*void DungKhan()
{
  cbi (TIMSK2,TOIE2);
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR2B = 0;
  }
  */

//******************************************************************
// timer2 setup
// set prscaler to 1,  fast PWM
void Setup_timer2() {

  // Timer2 Clock Prescaler to : 8
  sbi(TCCR2B, CS20);  // set
  cbi(TCCR2B, CS21);  // clear
  cbi(TCCR2B, CS22);

  // Timer2 PWM Mode
  cbi(TCCR2A, COM2A0);  // clear OC2A on Compare Match, PWM pin 11
  sbi(TCCR2A, COM2A1);
  sbi(TCCR2A, COM2B0);
  sbi(TCCR2A, COM2B1);

  // set to fast PWM
  sbi(TCCR2A, WGM20);  // Mode 1, phase correct PWM
  cbi(TCCR2A, WGM21);
  cbi(TCCR2B, WGM22);

  sbi(TIMSK2, TOIE2);  // enable overflow detect
}
// timer1 setup  (sets pins 9 and 10)
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer1() {

  sbi(TCCR1B, CS10);
  cbi(TCCR1B, CS11);
  cbi(TCCR1B, CS12);
  // Timer1 PWM Mode set to Phase Correct PWM
  cbi(TCCR1A, COM1A0);  // clear OC1A on Compare Match, PWM pin 9
  sbi(TCCR1A, COM1A1);
  cbi(TCCR1A, COM1B0);  // clear OC1B on Compare Match, PWM pin 10
  sbi(TCCR1A, COM1B1);

  sbi(TCCR1A, WGM10);  // Mode 1  / phase correct PWM
  cbi(TCCR1A, WGM11);
  cbi(TCCR1B, WGM12);
  cbi(TCCR1B, WGM13);
}


//SET TIMER 0

void Setup_timer0() {

  // Timer0 Clock Prescaler to : 8
  sbi(TCCR0B, CS00);  // set
  cbi(TCCR0B, CS01);  // clear
  cbi(TCCR0B, CS02);

  // Timer0 PWM Mode
  cbi(TCCR0A, COM0A0);  // clear OC2A on Compare Match, PWM pin 11
  sbi(TCCR0A, COM0A1);
  cbi(TCCR0A, COM0B0);
  sbi(TCCR0A, COM0B1);


  // set to fast PWM
  sbi(TCCR0A, WGM00);  // Mode 1, phase correct PWM
  cbi(TCCR0A, WGM01);
  cbi(TCCR0B, WGM02);
}

//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// runtime : 8 microseconds ( inclusive push and pop)
// OC2A - pin 11
// OC2B - pin 3
// OC0A - pin 6
// OC0B - pin 5
// OC1B - pin 10
// OC1A - pin 9
// https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
ISR(TIMER2_OVF_vect) {

  //sbi(PORTD,testPin);

  sigma = sigma + delta;  // soft DDS, phase accu with 32 bits
  phase0 = sigma >> 24;   // use upper 8 bits for phase accu as frequency information
                          // read value fron ROM sine table and send to PWM DAC
                          //  phase1 = phase0 + 85;
                          // phase2 = phase0 + 170;
                          /*
  value1 = pgm_read_byte_near(sine256 + phase0);
  if (value1 > 195) value1 = 195;
  if (value1 < 25) value1 = 25;

  value2 = pgm_read_byte_near(sine256 + phase1);
  if (value2 > 195) value2 = 195;
  if (value2 < 25) value2 = 25;

  value3 = pgm_read_byte_near(sine256 + phase2);
  if (value3 > 195) value3 = 195;
  if (value3 < 25) value3 = 25;
*/

  ///--------------------------------------//
  if (phase0 < 128) {
    value1 = pgm_read_byte_near(sine256 + phase0);
    // if (value1 > 250) value1 = 250;
    // if (value1 < 4) value1 = 4;
    OCR1A = 0;
    OCR1B = 0;
    OCR0A = value1;
    OCR0B = value1;
  } else if (phase0 > 127) {
    value1 = pgm_read_byte_near(sine256 + phase0);
    // if (value1 > 250) value1 = 250;
    // if (value1 < 4) value1 = 4;
    OCR0A = 0;
    OCR0B = 0;
    OCR1A = value1;
    OCR1B = value1;
  }

  /*
    if(phase0<128)
  {
  value1 = pgm_read_byte_near(sine256 + phase0);
  if (value1 > 255) value1 = 255;
  OCR1A = 0;
  OCR1B = 0;
  OCR0A = value1;
  OCR0B = value1;
  }
  else if (phase0>127)
  {
  value1 = pgm_read_byte_near(sine256 + phase0);
  if (value1 > 255) value1 = 255;
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = value1;
  OCR1B = value1;
  }*/
}
