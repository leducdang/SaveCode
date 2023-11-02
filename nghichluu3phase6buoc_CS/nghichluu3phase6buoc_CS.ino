#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

#define start 7
#define stop 4

static int microMHz = 16;
unsigned char chay = 0, phase = 0 , change_freq=0, status = 0;
unsigned long period, time_delay=0;
unsigned int freq_start = 50, freq_run = 40 , freq_set = 50, value_freq;
LiquidCrystal_I2C lcd(0x27, 16, 2);       // hoặc 0x3f


void setup() {
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  pinMode(start, INPUT_PULLUP);
  pinMode(stop , INPUT_PULLUP);
  pinMode(A0, INPUT);

  digitalWrite(3, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);

  /// tính chu kỳ tan so mặc định - bộ chia 64 - 6 buoc
  period = microMHz * 1e6 / freq_start / 6 / 64;

  ///set up ngắt trên timer 1.
  TCCR1A =  0b00000010; //CAI DAT KIEU COUNTER 14 DISCONNECT DAU RA
  TCCR1B = 0;
  TIMSK1 = 0;

  TCCR1B =  0b00011011; // CHON MODE 12 NGAT TRAN WGM13=1 WGM12=1 WGM11=0 WGM10=0
  //TIMSK1 = (1 << TOIE1);
  TIMSK1 = 0;
  ICR1 = period;

  sei();

  // SETUP lcd
 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
//  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("F_SET:");
  lcd.setCursor(14, 0);
  lcd.print("Hz");
  lcd.setCursor(1, 1);
  lcd.print("F_RUN:");
  lcd.setCursor(14, 1);
  lcd.print("Hz");


  // SETUP NGAT NGOAI NUT AN CHAY - DUNG
  //attachInterrupt(1, CHAY, FALLING);
  attachInterrupt(0, dungKhan, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(start) == 0)
  {
    time_delay = millis();
    while(digitalRead(start) == 0);
    if((unsigned long)millis() - time_delay > 100)
      {
        chay = 1;
        TIMSK1 = ( 1 << TOIE1);
        status =1;
      }   
  }

  if (digitalRead(stop) == 0)
  {
    time_delay = millis();
    while(digitalRead(stop) == 0);
    if((unsigned long)millis() - time_delay > 100)
      {
        chay = 0;
//        TIMSK1 = 0;
//        freq_run = 35;
//        tatHet();       
      } 
  }
  if ((unsigned long )millis() - time_delay > 200)
      {   
        value_freq = analogRead(A0);
        freq_set = map(value_freq, 0 , 1023, 40, 70);
        if( freq_set <40 ) freq_set=40;     
        if(chay == 1)
        {           
            if (freq_set > freq_run)
          {
            freq_run++;
            period = microMHz * 1e6 / freq_run / 6 / 64;
            change_freq =1;
          }
          else if (freq_set < freq_run)
          {
            freq_run--;
            period = microMHz * 1e6 / freq_run / 6 / 64;
            change_freq =1;
          }
          else
          {
            change_freq =0;
          }
        }
        if ( (chay == 0) && (status ==1) )
        {
          freq_run--;
          period = microMHz * 1e6 / freq_run / 6 / 64;
          change_freq =1;
          if(freq_run <40)
          {
            TIMSK1 = 0;
            tatHet();
            status = 0;
          }
        }
      

      lcd.setCursor(11, 0);
      lcd.print(freq_set);
      lcd.setCursor(11, 1);
      lcd.print(freq_run);

      time_delay = millis();
  }
}

ISR (TIMER1_OVF_vect)
{
  tatHet();
  if ((phase == 0) && (change_freq ==1) )
  {
    ICR1 = period;
    change_freq = 0;
  }

  switch (phase)
  {
    //  phase 0
    case 0:
      { digitalWrite(5, LOW);
        digitalWrite(9, LOW);
        digitalWrite(3, LOW);
        digitalWrite(6, HIGH);
        digitalWrite(10, HIGH);
        digitalWrite(11, HIGH);
        break;
      }
    //phase 1
    case 1:
      {
        digitalWrite(11, LOW);
        digitalWrite(5, LOW);
        digitalWrite(9, LOW);
        digitalWrite(3, HIGH);
        digitalWrite(6, HIGH);
        digitalWrite(10, HIGH);
        break;
      }
    // phase 2
    case 2:
      {
        digitalWrite(10, LOW);
        digitalWrite(11, LOW);
        digitalWrite(5, LOW);
        digitalWrite(9, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(6, HIGH);
        break;
      }
    //phase 3
    case 3:
      {
        digitalWrite(6, LOW);
        digitalWrite(10, LOW);
        digitalWrite(11, LOW);
        digitalWrite(5, HIGH);
        digitalWrite(9, HIGH);
        digitalWrite(3, HIGH);
        break;
      }
    //phase 4
    case 4:
      {
        digitalWrite(3, LOW);
        digitalWrite(6, LOW);
        digitalWrite(10, LOW);
        digitalWrite(11, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(9, HIGH);
        break;
      }
    //phase 5
    case 5:
      {
        digitalWrite(9, LOW);
        digitalWrite(3, LOW);
        digitalWrite(6, LOW);
        digitalWrite(10, HIGH);
        digitalWrite(11, HIGH);
        digitalWrite(5, HIGH);
      }
  }
  phase++;
  if (phase > 5) phase = 0;
}


void tatHet()
{
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(3, LOW);
  delayMicroseconds(50);

}
void dungKhan()
{
  TIMSK1 = 0;
  chay = 0;
  status = 0;
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(3, LOW);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Toang");
}
/*
  void CHAY()
  {
  chay = 1;
  TIMSK1 = (1<<TOIE1);
  }

  void DUNG()
  {
  chay = 0;
  TIMSK1 = 0;
  tatHet();
  }*/