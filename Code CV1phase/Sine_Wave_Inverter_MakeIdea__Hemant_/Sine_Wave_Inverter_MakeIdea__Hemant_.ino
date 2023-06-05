 // MakeIdea [Hemant]
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

#define start 2
#define stop 3


#define SinDivisions 256

static int microMHz = 16;
int freq = 50;
int flus; 
int F_in, F_out , valueVR;     
long int period;  
double temp;  
static unsigned int lookUp[SinDivisions];
static unsigned int lookUpdate[SinDivisions];
static char theTCCR1A = 0b10000010; 
unsigned long timeDelay = 0, timeButton1 = 0,timeButton2 = 0;
char bitStart = 0;       //0 dung, 1 chay


LiquidCrystal_I2C lcd(0x27,16,2); 

void setup()
{
  Serial.begin(9600);

  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("TAN SO CAI DAT");
  lcd.setCursor(5,1);
  lcd.print("Hz");

  pinMode(2, INPUT_PULLUP); // sử dụng điện trở kéo lên cho chân số 2, ngắt 0
  pinMode(3, INPUT_PULLUP);

 // period = microMHz*1e6/freq/SinDivisions;
  period = 254;
  for(int i = 0; i < SinDivisions/2; i++)
  { 
    temp = sin(i*2*M_PI/SinDivisions)*period;
    lookUp[i] = (int)(temp+0.5);   
    Serial.print(lookUp[i]);
    Serial.print(",");        
  }
  
  TCCR1A = theTCCR1A;       
  TCCR1B = 0b00011001;     
  TIMSK1 = 0b00000000;         
  ICR1   = period;
  OCR1A = OCR1B = 0 ;
  sei();               
  DDRB = 0b00000110; 

  
  pinMode(13, OUTPUT);
 // attachInterrupt(1, DungKhan, FALLING); // gọi hàm tatled liên tục khi còn nhấn nút
}

void loop()
    {
    if((unsigned long)millis() - timeDelay > 200)
    {
      valueVR = analogRead(A0); 
      F_in = map( valueVR, 0, 1023, 0, 99);  
      lcd.setCursor(2,0);
      lcd.print("TAN SO CAI DAT");
      lcd.setCursor(5,1);
      lcd.print("Hz");
      lcd.setCursor(1,1); 
      lcd.print(F_in); 
    }

    if(bitStart == 1)
    {
      if( F_in != F_out)
      {
        F_out = F_in;
        SET_F();
      }
    }

    if(bitStart == 0)
    {
      TIMSK1 = 0b00000000;
      OCR1A = 0;
      OCR1B = 0;
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);    
      }

      if(digitalRead(start) == 0 )
    {
      timeButton1 = millis();
      while (digitalRead(start) == 0);
      if ((unsigned long) millis() - timeButton1 > 100)
      {
        bitStart = 1;
        F_out = F_in;
        SET_F();
      }   
    }

      if(digitalRead(stop) == 0 )
    {
      timeButton2 = millis();
      while (digitalRead(stop) == 0);
      if ((unsigned long) millis() - timeButton2 > 100)
      {
        bitStart = 0;
      }   
    }
    

}
void DungKhan()
{
  bitStart = 0;
}

ISR(TIMER1_OVF_vect)
{
    static int num;
    static int delay1;
    static char trig;
    
    if(delay1 == 1)
    {
      theTCCR1A ^= 0b10100000;
      TCCR1A = theTCCR1A;
      delay1 = 0;             
    } 
    else if(num >= SinDivisions/2)
    {
      num = 0;                
      delay1++;
      trig ^=0b00000001;
      digitalWrite(13,trig);
    }
   
    OCR1A = OCR1B = lookUp[num];
    num++;
}
void SET_F() 
 {
    TIMSK1 = 0b00000000;  
    
  period = microMHz*1e6/F_out/SinDivisions;
  
  for(int i = 0; i < SinDivisions/2; i++)
  { 
    temp = sin(i*2*M_PI/SinDivisions)*period;
    lookUp[i] = (int)(temp+0.5);           
  } 

    ICR1   = period;          

    TIMSK1 = 0b00000001;    //cho phép ngắt khi tràn timer
 }
