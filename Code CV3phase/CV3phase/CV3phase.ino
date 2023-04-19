#include <LiquidCrystal_I2C.h>



// 3 phase PWM sine
// (c) 2016 C. Masenas
// Modified from original DDS from: 
// KHM 2009 /  Martin Nawrath

// table of 256 sine values / one sine period / stored in flash memory
PROGMEM const unsigned char sine256[]  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

LiquidCrystal_I2C lcd(0x27, 20, 4);     //0X3F  

#define start 8
#define stop  7
#define enter 13

unsigned int timeSet=10000;
unsigned long timeTangGiam;
unsigned int value1, value2, value3;

volatile  float freq=1;
const float refclk=122.549  ;     //  16 MHz/510/256

// variables used inside interrupt service declared as voilatile
volatile unsigned long sigma;   // phase accumulator
volatile unsigned long delta;  // phase increment
byte phase0, phase1, phase2 ;
unsigned int VR_in, F_in, F_out =0;
unsigned long timeMillis = 0;

unsigned char bitTang = 0, bitGiam = 0, bitChange = 0, enableVr = 0;

void setup()
{
  Serial.begin(9600);        // connect to the serial port
  Serial.println("DDS Test");

  //pinMode(enablePin, OUTPUT);      // sets the digital pin as output
  //pinMode(testPin, OUTPUT);      // sets the digital pin as output
  pinMode(9, OUTPUT);     // pin9= PWM  output / frequency output
  pinMode(10, OUTPUT);     // pin10= PWM  output / frequency output
  pinMode(11, OUTPUT);     // pin11= PWM  output / frequency output
  pinMode(start,INPUT_PULLUP);
  pinMode(stop,INPUT_PULLUP);
  pinMode(enter,INPUT_PULLUP);

  pinMode(3, OUTPUT);     // pin3= PWM  output / frequency output

  pinMode(6, OUTPUT);     // pin6= PWM  output / frequency output
  pinMode(5, OUTPUT);     // pin5= PWM  output / frequency output
  pinMode(2, INPUT_PULLUP);

  Setup_timer2();
  Setup_timer1();
  Setup_timer0 ();
  OCR2A = 0;
  OCR1A = 0;
  OCR1B = 0;
  cbi (TIMSK2,TOIE2);

  attachInterrupt(0, DungKhan, FALLING ); 

  //digitalWrite(enablePin, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("TAN SO CAI DAT");
  lcd.setCursor(5, 1);
  lcd.print("Hz");


// the waveform index is the highest 8 bits of sigma
// choose refclk as freq to increment the lsb of the 8 highest bits
//    for every call to the ISR of timer2 overflow
// the lsb of the 8 highest bits is 1<<24 (1LL<<24 for long integer literal)
  delta = (1LL<<24)*freq/refclk ;  
}
void loop(){
  
  //changeFreq(20);
  //delay(10000);
  //changeFreq(25);
  //delay(10000);
  VR_in = analogRead(A0);
  F_in = map(VR_in,0,1023,0,100);

/*        if (digitalRead(enter)==0)
        {
          if (enableVr == 1)
              {enableVr = 0;
              lcd.setCursor(5, 2);
              lcd.print("disable VR");
              }
          else 
             {
                enableVr = 1;
                lcd.setCursor(5, 2);
                lcd.print("enable VR");
              
             }
        }
        if((F_in != F_out) && (enableVr ==1))
          {
          changeFreq(F_in);
          F_out=F_in;
          }


        if (digitalRead(start)==0)
          {
            timeTangGiam = timeSet / F_in;
            bitTang =1;
            bitChange = 1;
            sbi (TIMSK2,TOIE2);
            timeMillis = millis();
            Serial.println("Start");
          }


        if (digitalRead(stop)==0)
          {
            timeTangGiam = timeSet / F_out;
            bitGiam=1;
            bitChange = 1;
            timeMillis = millis();
            Serial.println("Stop");
            //OCR2A = 0;
            //OCR1A = 0;
            //OCR1B = 0;
            //cbi (TIMSK2,TOIE2);
          }

          if(((unsigned long)millis() - timeMillis > timeTangGiam) && (bitChange== 1))
          {
                if(bitTang)
                {                
                  changeFreq(F_out);
                  F_out++;
                  if(F_out> F_in)
                  {
                    F_out = F_in;
                    bitTang=0;
                    bitChange=0;
                    Serial.println("Start Finish");
                  }
                }

                if(bitGiam)
                {                
                  changeFreq(F_out);
                  F_out--;
                  if(F_out<=0)
                  {
                    F_out = 0;
                    bitGiam=0;
                    bitChange=0;
                    OCR2A = 0;
                    OCR1A = 0;
                    OCR1B = 0;
                    OCR2B = 0;
                    OCR0A = 0;
                    OCR0B = 0;
                    cbi (TIMSK2,TOIE2);
                    Serial.println("Stop finish");
                  }
                }
               timeMillis = millis();  
          }
*/
  if(F_in != F_out)
          {
          changeFreq(F_in);
          F_out=F_in;
          }
  lcd.setCursor(1, 1);
  lcd.print(F_out);
  delay(200);
}


void changeFreq(float _freq){
  cbi (TIMSK2,TOIE2);              // disable timer2 overflow detect   - tắt ngắt timer2 
  freq = _freq;
  delta=(1LL<<24)*freq/refclk;  // update phase increment
  sbi (TIMSK2,TOIE2);              // enable timer2 overflow detect     - bật ngắt timer 2
} 



//******************************************************************

void DungKhan()
{
  cbi (TIMSK2,TOIE2);
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR2B = 0;
  }

//******************************************************************
// timer2 setup
// set prscaler to 1,  fast PWM
void Setup_timer2() {

// Timer2 Clock Prescaler to : 1
  sbi (TCCR2B, CS20);  // set
  cbi (TCCR2B, CS21);  // clear
  cbi (TCCR2B, CS22);

  // Timer2 PWM Mode 
  cbi (TCCR2A, COM2A0);  // clear OC2A on Compare Match, PWM pin 11
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, COM2B0);
  sbi (TCCR2A, COM2B1);

  // set to fast PWM
  sbi (TCCR2A, WGM20);  // Mode 1, phase correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);

  sbi (TIMSK2,TOIE2);              // enable overflow detect
  
}
// timer1 setup  (sets pins 9 and 10)
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer1() {

// Timer1 Clock Prescaler to : 1
  sbi (TCCR1B, CS10);
  cbi (TCCR1B, CS11);
  cbi (TCCR1B, CS12);

  // Timer1 PWM Mode set to Phase Correct PWM
  cbi (TCCR1A, COM1A0);  // clear OC1A on Compare Match, PWM pin 9
  sbi (TCCR1A, COM1A1);
  sbi (TCCR1A, COM1B0);  // clear OC1B on Compare Match, PWM pin 10
  sbi (TCCR1A, COM1B1);

  sbi (TCCR1A, WGM10);  // Mode 1  / phase correct PWM
  cbi (TCCR1A, WGM11);
  cbi (TCCR1B, WGM12);
  cbi (TCCR1B, WGM13);
}


//SET TIMER 0

void Setup_timer0() {

// Timer0 Clock Prescaler to : 1
  sbi (TCCR0B, CS00);  // set
  cbi (TCCR0B, CS01);  // clear
  cbi (TCCR0B, CS02);

  // Timer0 PWM Mode 
  cbi (TCCR0A, COM0A0);  // clear OC2A on Compare Match, PWM pin 11
  sbi (TCCR0A, COM0A1);

  sbi (TCCR0A, COM0B0);
  sbi (TCCR0A, COM0B1);


  // set to fast PWM
  sbi (TCCR0A, WGM00);  // Mode 1, phase correct PWM
  cbi (TCCR0A, WGM01);
  cbi (TCCR0B, WGM02);

}

//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// runtime : 8 microseconds ( inclusive push and pop)
// OC2A - pin 11
// OC1B - pin 10
// OC1A - pin 9
// https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
ISR(TIMER2_OVF_vect) {

  //sbi(PORTD,testPin);          

  sigma=sigma+delta; // soft DDS, phase accu with 32 bits
  phase0=sigma >> 24;     // use upper 8 bits for phase accu as frequency information
                         // read value fron ROM sine table and send to PWM DAC
  phase1 = phase0 +85 ;
  phase2 = phase0 +170 ;

  value1 = pgm_read_byte_near(sine256 + phase0);
  if(value1 > 240)value1 = 240;
  if(value1 < 10)value1 = 10;

  value2 = pgm_read_byte_near(sine256 + phase1);
  if(value2 > 240)value2 = 240;
  if(value2 < 10)value2 = 10;

  value3 = pgm_read_byte_near(sine256 + phase2);
  if(value3 > 240)value3 = 240;
  if(value3 < 10)value3 = 10;

  //OCR2A=pgm_read_byte_near(sine256 + phase0)  ;  // pwm pin 11
  //OCR2B=pgm_read_byte_near(sine256 + phase0) + 20;  // pwm pin 3
  OCR2A = value1;
  OCR2B = value1 + 10;

  //OCR1B=pgm_read_byte_near(sine256 + phase1) ;  // pwm pin 10
  //OCR1A=pgm_read_byte_near(sine256 + phase1) ;  // pwm pin 9
  OCR1A = value2;
  OCR1B = value2 + 10;

  //OCR0A=pgm_read_byte_near(sine256 + phase2) ;  // pwm pin 6
  //OCR0B=pgm_read_byte_near(sine256 + phase2) ;  // pwm pin 5
  OCR0A = value3;
  OCR0B = value3 + 10;


  //cbi(PORTD,testPin);            
  
}
