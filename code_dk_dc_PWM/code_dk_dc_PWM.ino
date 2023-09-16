#include <avr/interrupt.h>
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

unsigned int value, pwm;
unsigned long time = 0;
char mode = 0;
int Left = 4;
int Right = 5;


void setup() {
  // put your setup code here, to run once:
  pinMode(Left, INPUT_PULLUP);
  pinMode(Right, INPUT_PULLUP);
  pinMode(11,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);


  Setup_timer2();
  Setup_timer1();
  OCR2A = 0;
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - time > 100)
    {
      time = millis();
      value = analogRead(A0);
      pwm = map(value, 0, 1020, 0, 255);  
    }
  if(digitalRead(Left) == 0 )
  {
    if(mode != 1)
    {
      sbi(TIMSK2, TOIE2);
      mode = 1;
    }  
  }
  if(digitalRead(Right) == 0 )
  {
    if(mode != 2)
    {
      sbi(TIMSK2, TOIE2);
      mode = 2;
    } 
  }
  else
  {
    if((digitalRead(Left) == 1) && (digitalRead(Right) == 1))
    {
      cbi(TIMSK2, TOIE2);
      OCR1A = 0;
      OCR1B = 0;
      OCR2A = 0;
      OCR2B = 0;
      mode = 0;
    }
  }
}


ISR(TIMER2_OVF_vect)
{
  //dem++;
  if(mode == 1)
  {
    OCR1A = pwm;
    OCR1B = pwm;
    OCR2A = 0;
    OCR2B = 0;
  }
  if(mode == 2)
  {
    OCR1A = 0;
    OCR1B = 0;
    OCR2A = pwm;
    OCR2B = pwm; 
  }
}
void Setup_timer1() {

  // Timer1 Clock Prescaler to : 1
  sbi(TCCR1B, CS10);
  cbi(TCCR1B, CS11);
  cbi(TCCR1B, CS12);

  // Timer1 PWM Mode set to Phase Correct PWM
  cbi(TCCR1A, COM1A0);  // clear OC1A on Compare Match, PWM pin 9
  sbi(TCCR1A, COM1A1);
  cbi(TCCR1A, COM1B0);  // clear OC1B on Compare Match, PWM pin 10
  sbi(TCCR1A, COM1B1);

  sbi(TCCR1A, WGM10);  // Mode 1  / phase correct PWM TOPVALUE = 0xFF
  cbi(TCCR1A, WGM11);
  cbi(TCCR1B, WGM12);
  cbi(TCCR1B, WGM13);
}
void Setup_timer2() {

  // Timer2 Clock Prescaler to : 1
  sbi(TCCR2B, CS20);  // set
  cbi(TCCR2B, CS21);  // clear
  cbi(TCCR2B, CS22);

  // Timer2 PWM Mode
  cbi(TCCR2A, COM2A0);  // clear OC2A on Compare Match, PWM pin 11
  sbi(TCCR2A, COM2A1);
  cbi(TCCR2A, COM2B0);
  sbi(TCCR2A, COM2B1);

  // set to fast PWM
  sbi(TCCR2A, WGM20);  // Mode 1, phase correct PWM
  cbi(TCCR2A, WGM21);
  cbi(TCCR2B, WGM22);

  //sbi(TIMSK2, TOIE2);  // enable overflow detect
}
