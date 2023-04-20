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

LiquidCrystal_I2C lcd(0x27, 20, 4);     //0X27 or 0X3F  

#define start 4
#define stop  7
#define enter 13
#define up    8
#define down  7
#define enter 13

unsigned int timeSet=10000;
unsigned int value1, value2, value3;    //gia tri cai dat phu

volatile  float freq=1;
const float refclk=122.549  ;     //  16 MHz/510/256

// variables used inside interrupt service declared as voilatile
volatile unsigned long sigma;   // phase accumulator
volatile unsigned long delta;  // phase increment
byte phase0, phase1, phase2 ;
unsigned int VR_in, F_in, F_out =0;
unsigned long timeMillis = 0;            // thoi gian delay khong dung
unsigned char bitThuanNghich =0 ;        // 0 quay thuan,1 quay nghich.
unsigned char bitChayDung = 0;           // 1 chay 0 dung
unsigned char mode = 0 ;                 //mode =1 quay ve cac tuy chon, 2 control motor, 3 che do VR, 4 Mode TimeChange, 5 Mode Driver ,6 Mode SetTanSo
unsigned float timeTGT= 5;               //thoi gian tang giam toc
unsigned float f_set= 50;
unsigned char vitri;

void setup()
{
  Serial.begin(9600);        // connect to the serial port
  Serial.println("DDS Test");

  //pinMode(enablePin, OUTPUT);      // sets the digital pin as output
  //pinMode(testPin, OUTPUT);      // sets the digital pin as output
  pinMode(9, OUTPUT);      // pin9 =  PWM  output / frequency output  UH  OCR1A
  pinMode(10, OUTPUT);     // pin10=  PWM  output / frequency output  UL  OCR1B
  pinMode(11, OUTPUT);     // pin11=  PWM  output / frequency output  WH  0CR2A
  pinMode(3, OUTPUT);      // pin3 =  PWM  output / frequency output  WL  0CR2B
  pinMode(6, OUTPUT);      // pin6 =  PWM  output / frequency output  VH  0CR0A
  pinMode(5, OUTPUT);      // pin5 =  PWM  output / frequency output  VL  0CR0B

  pinMode(start,INPUT_PULLUP);
  pinMode(stop ,INPUT_PULLUP);
  pinMode(enter,INPUT_PULLUP);
  pinMode(up   ,INPUT_PULLUP);
  pinMode(down ,INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP); //ngắt ngoài tại chân số 2 có trở treo dương ngắt tín hiệu khi ngắn mạch

  Setup_timer2();
  Setup_timer1();
  Setup_timer0 ();
  OCR0A = 0;
  OCR0B = 0;
  OCR2A = 0;
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;
  cbi (TIMSK2,TOIE2);               // TẮT NGẮT TIMER 2

  attachInterrupt(0, DungKhan, FALLING );   // tắt phát xung SPWM khi có lỗi xảy ra.

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Module Mach Bien Tan");
  lcd.setCursor(5, 1);
  lcd.print("thanhhung.edu.vn");


// the waveform index is the highest 8 bits of sigma
// choose refclk as freq to increment the lsb of the 8 highest bits
//    for every call to the ISR of timer2 overflow
// the lsb of the 8 highest bits is 1<<24 (1LL<<24 for long integer literal)
  delta = (1LL<<24)*freq/refclk ;  
}
void loop(){
  switch (mode)
  {
  case 0:
    displayMain();
    break;
  case 1:
    displayControl();
    break;
  case 2:
    ChangeVR();
    break;
  case 3:
    TGTangGiamToc();
    break;
  case 4:
    ThuanNghich();
    break;
  case 5:
    SetTanSo();
    break;
  default:
    break;
  }
  
}
void displayMain(){
  if (vitri > 0 && vitri <5 )
  {
    lcd.setCursor(5,1);
    lcd.print("Slect Mode");
    lcd.setCursor(1,2);
    lcd.print("Mode 1: Display Control");
    lcd.setCursor(1,3);
    lcd.print("Mode 2: Control VR");
    lcd.setCursor(1,4);
    lcd.print("Mode 3: Time Change");
  }
  if (vitri>4)
  {
    lcd.setCursor(5,1);
    lcd.print("Slect Mode");
    lcd.setCursor(1,2);
    lcd.print("Mode 3: Motor Driver");
    lcd.setCursor(1,3);
    lcd.print("Mode 4: Set Freq");
  }

  if (digitalRead(up) == 0 )
  {
    delay(20);
    if (digitalRead(up) == 0 )
    {
      vitri ++;
      if (vitri < 1)
      {
        vitri = 1;
      } 
    }
  }
  if (digitalRead(down) == 0 )
  {
    delay(20);
    if (digitalRead(down) == 0 )
    {
      vitri --;
      if (vitri > 5)
      {
        vitri =5;
      }  
    }
  }
  if (digitalRead(enter)==0)
  {
    delay(20);
    if (digitalRead(enter)==0)
    {
      mode = vitri
    }    
  }  
  switch (vitri){
      case 1:
      {

        lcd.setCursor(2,1);
        lcd.print("*");
        break;
      }
      case 2:
      {
        lcd.setCursor(3,1);
        lcd.print("*");
        break;
      }
      case 3:
      {
        lcd.setCursor(4,1);
        lcd.print("*");
        break;
      }
      case 4:
      {
        lcd.setCursor(2,1);
        lcd.print("*");
        break;
      }
      case 5:
      {
        lcd.setCursor(3,1);
        lcd.print("*");
        break;
      }
  }
}

void displayControl(){
  if ((millis() - timeMillis) > 100)
  {
    lcd.setCursor(5,1);
    lcd.print("Control Motor");
    lcd.setCursor(1,2);
    lcd.print("Frequency:");
    lcd.setCursor(13,2);
    lcd.print(F_out);
    lcd.setCursor(1,3);
    lcd.print("Time Change");
    lcd.setCursor(13,3);
    lcd.print(timeTGT);
    lcd.setCursor(1,3);
    lcd.print("Chieu quay:");
    lcd.setCursor(1,3);
    if (bitThuanNghich == 0)
    {
      lcd.print("Thuan");
    }
    if (bitThuanNghich == 1)
    {
      lcd.print("Nghich");
    }
    
    lcd.print("Chieu quay:");
    timeMillis = millis(); 
    Back();
  }

}
//Mode 1: su dung bien tro
void ChangeVR()        
{
  if ((millis() - timeMillis) > 100)
  {
    lcd.setCursor(1,1);
    lcd.print("MODE 1: DUNG VR ");
    lcd.setCursor(1,3);
    lcd.print("Frequency:");
    lcd.setCursor(11,3);
    lcd.print(_freq);
    timeMillis = millis(); 
  }
  VR_in = analogRead(A0);
  F_in = map(VR_in,0,1023,0,100);
  if(F_in != F_out)
    {
      changeFreq(F_in);
      F_out=F_in;
    }
  if(digitalRead(start) == 0 )
    {
      chay();
    }
  else if (digitalRead(stop) == 0)
    {
      dung();
    }
    Back();
}
//Mode 2: set thoi gian tang giam toc
void TGTangGiamToc(){
  if ((unsigned long millis() - timeMillis())> 100)
  {
    lcd.setCursor(1.1);
    lcd.print("Mode 2: Time Change");
    lcd.setCursor(2.1);
    lcd.print("Time:");
    lcd.setCursor(8,1);
    lcd.print(timeTGT);
    timeMillis = millis();
  }
  if (digitalRead(up) == 0)
  {
    delay(20);
    if (digitalRead(up) == 0)
    {
      timeTGT +=0.1; 
      if (timeTGT>20)
      {
        timeTGT =20;
      }
    } 
  }
  if (digitalRead(down) == 0)
  {
    delay(20);
    if (digitalRead(down) == 0)
    {
      timeTGT -=0.1;
      if (timeTGT < 1)
      {
        timeTGT =1;
      }  
    }  
  }
  Back();    
}
//Mode 3: set chieu quay thuan nghich cua dong co
void ThuanNghich(){
  if ((unsigned long millis() - timeMillis())> 100)
  {
    lcd.setCursor(1.1);
    lcd.print("Mode 3: Thuan Nghich");
    lcd.setCursor(2.1);
    lcd.print("Chieu quay:");
    lcd.setCursor(14,1);
    if (bitThuanNghich == 0 )
    {
      lcd.print("Thuan");
    }
    else if (bitThuanNghich ==1)
    {
      lcd.print("Nghich");
    }
    lcd.print(timeTangGiam);
    timeMillis = millis();
  }

  if ((digitalRead(up) == 0) && (bitChayDung == 0 ))     //nut up duoc nhan va dong co dung  => set quay thuan
  {
    delay(20);
    if (digitalRead(up)==0)
    {
      bitThuanNghich = 0; 
    }              
  }
  if ((digital(down)==0) && (bitChayDung == 0))         // nut down duoc nhan va dong co dung => set quay nghich
  {
    delay(20);
    if (digitalRead(down)==0)
    {
      bitThuanNghich = 1; 
    }
  }
  Back();  
}
//Mode 4: set tan so bang nut bam
void SetTanSo(){
  if ((unsigned long millis() - timeMillis())> 100)
  {
    lcd.setCursor(1.1);
    lcd.print("Mode 4: Set Tan So");
    lcd.setCursor(2.1);
    lcd.print("Set F:");
    lcd.setCursor(8,1);
    lcd.print(f_set);
    timeMillis = millis();
  }
  if (digitalRead(up) == 0)
  {
    delay(20);
    if (digitalRead(up) == 0)
    {
      f_set +=0.1; 
      if (f_set>100)
      {
        f_set = 100;
      } 
    }   
  }
  if (digitalRead(down) == 0)
  {
    delay(20);
    if (digitalRead(up) == 0)
    {
      f_set -=0.1;
      if (f_set < 30)
      {
        f_set =30;
      }  
    }
  }
  Back();   
}
//Kiem tra thoat chuong trinh
void Back(){
  if (digitalRead(enter)==0)
  {
    delay(20);
    if (digitalRead(enter)==0)
    {
      mode=1;
    }
  }
}
// chay dong co
void chay(){
  OCR0A = 0;
  OCR0B = 0;
  OCR2A = 0;
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;
  sbi (TIMSK2,TOIE2);
}
// stop dong co
void dung(){
  cbi (TIMSK2,TOIE2);
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR2B = 0;
}
// thay doi tan so
void changeFreq(float _freq){
  cbi (TIMSK2,TOIE2);              // disable timer2 overflow detect   - tắt ngắt timer2 
  freq = _freq;
  delta=(1LL<<24)*freq/refclk;     // update phase increment
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
                          // read value fron ROM sine table and send to PWM DAC
  phase0 = sigma >> 24;     // use upper 8 bits for phase accu as frequency information                     
  phase1 = phase0 +85 ;
  phase2 = phase0 +170 ;

  value1 = pgm_read_byte_near(sine256 + phase0);
  if(value1 > 240)value1 = 240;
  if(value1 < 10) value1 = 10;

  value2 = pgm_read_byte_near(sine256 + phase1);
  if(value2 > 240)value2 = 240;
  if(value2 < 10) value2 = 10;

  value3 = pgm_read_byte_near(sine256 + phase2);
  if(value3 > 240)value3 = 240;
  if(value3 < 10) value3 = 10;

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
