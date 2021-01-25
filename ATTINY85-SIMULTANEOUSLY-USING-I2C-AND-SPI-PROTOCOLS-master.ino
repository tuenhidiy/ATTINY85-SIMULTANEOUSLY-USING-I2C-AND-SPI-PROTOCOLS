/*
DigiSpark ATtiny85 project descriptions:
  1. Read data from MPU6050 via I2C.
  2. Control an 13x15-LED MATRIX via Software SPI. 
  3. Optional: Led matrix can be ajdusted brightness via Bit Angle Modulation Mode, or constant brightness mode.
This project use 2x74HC595 + Transistors for anodes scanning and 2xTPIC6B595 for cathodes scanning
Tested with Arduino 1.8.13, the ATTinyCore and TinyWireM library:

https://github.com/SpenceKonde/ATTinyCore
https://github.com/adafruit/TinyWireM

Software SPI Connections:
  DigiSpark ATtiny85 P1 (PB1) - DATA PIN OF THE 1st 74HC595
  DigiSpark ATtiny85 P3 (PB3) - CLOCK PIN OF 74HC595 & TPIC6B595.
  DigiSpark ATtiny85 P4 (PB4) - LATCH PIN OF 74HC595 & TPIC6B595.

I2C Connections:
  DigiSpark ATtiny85 P0 (PB0) - SDA
  DigiSpark ATtiny85 P2 (PB2) - SCL   

By TUENHIDIY - 2021. Jan. 25
*/

#include <TinyWireM.h>
#include "font3x5.h"
#include "font5x7.h"
#include "font8x8.h"
#include "font8x16.h"

#define BAM_USE       0           // Set to 1 to use BAM method (adjust led brightness), set to 0 for constant brightness

const int MPU_ADDR = 0x68;  // I2C address of the MPU-6050
int16_t AccX, AccY, AccZ, Temp, GygroX, GygroY, GygroZ;

//*****************************************************PINS SETUP*******************************************************//
const int
    DATA_PIN(1),                // Serial Data Pin
    CLOCK_PIN(3),               // Shift Register Clock Pin
    LATCH_PIN(4);               // Storage Register Clock
    //BLANK_PIN(5);               // Output Enable Pin - For this version, it is connected to GND

//*****************************************************B.A.M SETUP*******************************************************//

#define BAM_RESOLUTION  4
byte matrixBuffer[BAM_RESOLUTION][26];

// Anode low and high byte for shifting out.
byte anode[16][2]= {{B11111111, B11111110}, {B11111111, B11111101}, {B11111111, B11111011}, {B11111111, B11110111}, {B11111111, B11101111}, {B11111111, B11011111}, {B11111111, B10111111}, {B11111111, B01111111}, 
                    {B01111110, B11111111}, {B11111101, B11111111}, {B11111011, B11111111}, {B11110111, B11111111}, {B11101111, B11111111}, {B11011111, B11111111}, {B10111111, B11111111}, {B01111111, B11111111}};

byte row, level;
int BAM_Bit, BAM_Counter=0;   // Bit Angle Modulation variables to keep track of things
uint8_t R;
                  
//*****************************************************FONT SETUP*******************************************************//

#define FONT3x5         0
#define FONT5x7         1
#define FONT8x8         2
#define FONT8x16        3

//*****************************************************TEXTS*******************************************************//
//char scrolltext_1[]="     ATTINY85 AND LED MATRIX 13x15...     ";
//char scrolltext_2[]="     WELCOME TO TUENHIDIY YOUTUBE CHANNEL...     ";
//char scrolltext_3[]="     Xin chào...     ";

//*****************************************************SET UP*******************************************************//
void setup()
{
  row = 0;
  level = 0;
  noInterrupts();
  // Set up the pins for software SPI
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  // Clear registers
  TCNT1 = 0;
  TCCR1 = 0;
  // Reset to $00 in the CPU clock cycle after a compare match with OCR1C register value
  // 50 x 3.636 = 181.8us
  OCR1C = 50;
  // A compare match does only occur if Timer/Counter1 counts to the OCR1A value
  OCR1A = OCR1C;    
  // Clear Timer/Counter on Compare Match A
  TCCR1 |= (1 << CTC1);
  // Prescaler 64 - 16.5MHz/64 = 275Kz or 3,636us
  TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK |= (1 << OCIE1A);
  interrupts();
  TinyWireM.begin();
  MPU_INIT();
  clearscreen();
}

//*****************************************************MAIN PROGRAM*******************************************************//

void loop()
{
   clearscreen();
   hScroll_MPU(15, 0, "  DigiSpark ATTiny85 I2C and SPI  Smart Matrix 13x15 with MPU6050  Instructables  Subscribe TUENHIDIY YouTube Channel       ", FONT8x16, 20, 1); // hScroll_MPU(byte For_color, byte Bk_color, char *mystring, uint8_t font, uint8_t delaytime, uint8_t dir)
   clearscreen();
}

void LED(uint8_t X, uint8_t Y, uint8_t BB)
{    
  uint8_t whichbyte = ((Y*2)+X/8);
  uint8_t whichbit = 7-(X % 8); 
  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(matrixBuffer[BAM][whichbyte], whichbit, bitRead(BB, BAM));  
  } 
}

void fillTable(uint8_t BLUE)
{
    for (byte x=0; x<15; x++)
    {
      for (byte y=0; y<13; y++)
      {
        LED(x, y, BLUE);
      }
    }
}

void colorMorph(int timex)
{
  int keepColorTime = timex * 30;
  for(int BLUE = 0; BLUE <= 15; BLUE++)
  {
    fillTable(BLUE);
    delay(timex);
  }
  delay(keepColorTime);
  for(int BLUE = 15; BLUE >= 0; BLUE--)
  {
    fillTable(BLUE);
    delay(timex);
  }
  delay(keepColorTime);
}

void clearscreen()
{
  memset(matrixBuffer, 0, sizeof(matrixBuffer[0][0]) * BAM_RESOLUTION * 26);
}


void DIY_SPI(uint8_t DATA)
{    
    for (uint8_t i = 0; i<8; i++)  
    {
      digitalWrite(DATA_PIN, !!(DATA & (1 << (7-i))));
      PORTB |= 1<<CLOCK_PIN;
      PORTB &= ~(1<<CLOCK_PIN);    
    }    
}

ISR(TIMER1_COMPA_vect)
{     
  // If we use BAM 4 bit method
  #if BAM_USE == 1  
  if(BAM_Counter==8)    // Bit weight 2^0 of BAM_Bit, lasting time = 8 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==24)   // Bit weight 2^1 of BAM_Bit, lasting time = 24 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==56)   // Bit weight 2^3 of BAM_Bit, lasting time = 56 ticks x interrupt interval time
  BAM_Bit++;
  BAM_Counter++;
  // Cathodes scanning
  switch (BAM_Bit)
    {
    case 0:
      //  
        DIY_SPI(matrixBuffer[0][level + 0]);
        DIY_SPI(matrixBuffer[0][level + 1]);
      break;
    case 1:      
      //
        DIY_SPI(matrixBuffer[1][level + 0]);
        DIY_SPI(matrixBuffer[1][level + 1]);
      break;
    case 2:     
      //
        DIY_SPI(matrixBuffer[2][level + 0]);
        DIY_SPI(matrixBuffer[2][level + 1]);
      break;
    case 3:
      //
        DIY_SPI(matrixBuffer[3][level + 0]);
        DIY_SPI(matrixBuffer[3][level + 1]);      
    if(BAM_Counter==120)    //Bit weight 2^3 of BAM_Bit, lasting time = 120 ticks x interrupt interval time
    {
    BAM_Counter=0;
    BAM_Bit=0;
    }
    break;
  }
  
  // If we don't use BAM method
  #else
  // Cathodes scanning
  // Uncomment to adjust your constant brightness level
  // 8 ticks x interrupt interval time - or 181.8us x 8 = 1.4544 ms (6.875Kz)
  // DIY_SPI(matrixBuffer[0][level + 0]);
  // DIY_SPI(matrixBuffer[0][level + 1]);
  
  // 24 ticks x interrupt interval time - or 181.8us x 24 = 4.3632 ms (2.29Kz)
   DIY_SPI(matrixBuffer[1][level + 0]);
   DIY_SPI(matrixBuffer[1][level + 1]);

  // 56 ticks x interrupt interval time - or 181.8us x 56 = 10.1808 ms (0.98Kz)
  // DIY_SPI(matrixBuffer[2][level + 0]);
  // DIY_SPI(matrixBuffer[2][level + 1]);
  
  // 120 ticks x interrupt interval time - or 181.8us x 120 = 21.816 ms (0.457Kz)  
  // DIY_SPI(matrixBuffer[3][level + 0]);
  // DIY_SPI(matrixBuffer[3][level + 1]);
  #endif
  
  // Anode scanning
  DIY_SPI(anode[row][0]);   // Send out the anode level low byte
  DIY_SPI(anode[row][1]);  // Send out the anode level high byte
  
  PORTB |= 1<<LATCH_PIN;
  delayMicroseconds(2);
  PORTB &= ~(1<<LATCH_PIN);
  delayMicroseconds(2); 
  
  row++;
  level = row * 2; 
  if (row == 13) row=0;
  if (level == 26) level=0;  
}

byte getPixelChar(uint8_t x, uint8_t y, char ch, uint8_t font)
{
  if (font==FONT3x5)
  {
    if (x > 2) return 0;
    return bitRead(pgm_read_byte(&font3x5[ch-32][4-y]), 2-x);
  }
  
  else if (font==FONT5x7)
  {
    if (x > 4) return 0;
    return bitRead(pgm_read_byte(&font5x7[ch-32][6-y]), 4-x);
  }
  
  else if (font==FONT8x8)
  {
    if (x > 7) return 0;
    return bitRead(pgm_read_byte(&font8x8[ch-32][7-y]), 7-x); 
  }
  else if (font==FONT8x16)
  {
    if (x > 7) return 0;
    return bitRead(pgm_read_byte(&font8x16[ch-32][15-y]), 7-x);
  }  
}

byte getPixelHString(uint16_t x, uint16_t y, char *p, uint8_t font)

{
  if (font==FONT3x5)
  {
    p=p+x/4;
    return getPixelChar(x%4, y, *p, FONT3x5);
  }
  
  else if (font==FONT5x7)
  {
    p=p+x/6;
    return getPixelChar(x%6, y, *p, FONT5x7);
  }

  else if (font==FONT8x8)
  {
    p=p+x/8;
    return getPixelChar(x%8, y, *p, FONT8x8);  
  }

  else if (font==FONT8x16)
  {
    p=p+x/9;
    return getPixelChar(x%9, y, *p, FONT8x16); 
  }
}

unsigned int lenString(char *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}


void printChar(uint8_t x, uint8_t y, uint8_t For_color, uint8_t Bk_color, char ch, uint8_t font)
{
  uint8_t xx,yy;
  bool checkfontvn;
  xx=0;
  yy=0;
  //******************************************************FONT 3x5******************************************************//
  if (font == FONT3x5)
  {    
  for (yy=0; yy < 5; yy++)
    {
    for (xx=0; xx < 3; xx++)
      {
      if (bitRead(pgm_read_byte(&font3x5[ch-32][4-yy]),2-xx))
      
        {
            LED(x+xx, y+yy, For_color);
        }
      else
        {
            LED(x+xx, y+yy, Bk_color);      
        }
      }
    }
  }
  //******************************************************FONT 5x7******************************************************//
  else if (font == FONT5x7)
  {
    for (byte xx=0; xx<5; xx++)
      {
        for (byte yy=0; yy<7; yy++)
          {            
          if (bitRead(pgm_read_byte(&font5x7[ch-32][6-yy]), 4-xx))
            {
              LED(x+xx, y+yy, For_color);               
            }
            else 
            {
              LED(x+xx, y+yy, Bk_color);
            }
          }
        }
      }
//******************************************************FONT 8x8******************************************************//
  else if (font==FONT8x8)
  {
    for (byte xx=0; xx<7; xx++)
      {
        for (byte yy=0; yy<7; yy++)
          {            
          if (bitRead(pgm_read_byte(&font8x8[ch-32][7-yy]), 7-xx))
            {
              LED(x+xx, y+yy, For_color);               
            }
            else 
            {
              LED(x+xx, y+yy, Bk_color);
            }
          }
        }
      }
//*******************************************************FONT 8x16*****************************************************//
  else if (font == FONT8x16)
  {
    for (byte xx=0; xx<15; xx++)
      {
        for (byte yy=0; yy<7; yy++)
          {            
          if (bitRead(pgm_read_byte(&font8x16[ch-32][15-yy]), 7-xx))
            {
              LED(x+xx, y+yy, For_color);               
            }
            else 
            {
              LED(x+xx, y+yy, Bk_color);
            }
          }
        }
      }                  
}


void hScroll(uint8_t y, byte For_color, byte Bk_color, char *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  int offset;
  byte color;
  //******************************************************FONT 3x5******************************************************//
  if (font == FONT3x5)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-4)*4-1) ; (dir) ? offset <((lenString(mystring)-4)*4-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<5; yy++)
            {            
              if (getPixelHString(xx+offset, yy, mystring, FONT3x5))
              {
                color = For_color;                
              }
              else 
              {
                color = Bk_color;
              }
                LED(xx, (yy+y), color);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  }

//******************************************************FONT 5x7******************************************************//

  else if (font == FONT5x7)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-5)*6-1) ; (dir) ? offset <((lenString(mystring)-5)*6-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<7; yy++)
            {            
              if (getPixelHString(xx+offset, yy, mystring, FONT5x7))
              {
                color = For_color;                
              }
              else 
              {
                color = Bk_color;
              }
                LED(xx, (yy+y), color);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  }
//********************************************************FONT 8x8****************************************************//  

  else if (font == FONT8x8)
    {
    while (times)
      {
      for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*8-1); (dir) ? offset <((lenString(mystring)-6)*8-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<15; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT8x8)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx, (yy+y), color);
              }          
            }
      delay(delaytime);  
        }
      times--;
      }
    }
//*******************************************************FONT 8x16*****************************************************//

   else if (font == FONT8x16)
    {
    while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*9-1); (dir) ? offset <((lenString(mystring)-6)*9-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {     
            for (byte yy=0; yy<13; yy++)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT8x16)) 
                  {
                  color = For_color;                
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx, (yy+y), color);
              }   
          }
          delay(delaytime);  
        }
        times--;
      } 
    }
 }

 void MPU_INIT() 
 {
  TinyWireM.beginTransmission(MPU_ADDR);
  TinyWireM.send(0x6B);                             // PWR_MGMT_1 register
  TinyWireM.send(0);                                // Set to zero (wakes up the MPU-6050)
  TinyWireM.endTransmission();
}

void READ_MPU6050() 
{
  TinyWireM.beginTransmission(MPU_ADDR);
  TinyWireM.send(0x3B);                               // Starting with register 0x3B (ACCEL_XOUT_H)
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(MPU_ADDR, 14);                // Request a total of 14 registers
  AccX=TinyWireM.receive()<<8|TinyWireM.receive();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AccY=TinyWireM.receive()<<8|TinyWireM.receive();    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ=TinyWireM.receive()<<8|TinyWireM.receive();    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp=TinyWireM.receive()<<8|TinyWireM.receive();    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GygroX=TinyWireM.receive()<<8|TinyWireM.receive();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GygroY=TinyWireM.receive()<<8|TinyWireM.receive();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GygroZ=TinyWireM.receive()<<8|TinyWireM.receive();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


void hScroll_MPU(byte For_color, byte Bk_color, char *mystring, uint8_t font, uint8_t delaytime, uint8_t dir)
{
int offset =0;
byte setcolor;
uint8_t pos;
//*******************************************************FONT 3x5*****************************************************//
if (font == FONT3x5)
{  
  for ((dir) ? offset=0 : offset=((lenString(mystring)-4)*4-1) ; (dir) ? offset <((lenString(mystring)-4)*4-1) : offset >0; (dir) ? offset++ : offset--)
  {
    READ_MPU6050();
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<5; yy++)
            { 
              pos = (AccX >0 ? 1:0);
              if (pos)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT3x5)) 
                setcolor = For_color; 
                else setcolor = Bk_color;                 
                LED(14-xx,4-yy,setcolor);
              }

              else
              {
                if (flipbyte(getPixelHString((xx+offset),yy, mystring, FONT3x5)))             
                setcolor = For_color; 
                else setcolor=Bk_color;
                LED(xx, yy, setcolor);
              }

            }
        }
        delay(delaytime); 
      }
  }
  
//*******************************************************FONT 5x7*****************************************************//
else if (font == FONT5x7)
{  
  for ((dir) ? offset=0 : offset=((lenString(mystring)-5)*6-1) ; (dir) ? offset <((lenString(mystring)-5)*6-1) : offset >0; (dir) ? offset++ : offset--)
  {
    READ_MPU6050();
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<7; yy++)
            { 
              pos = (AccX >0 ? 1:0);
              if (pos)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT5x7)) 
                setcolor = For_color; 
                else setcolor = Bk_color;                 
                LED(14-xx,6-yy,setcolor);
              }

              else
              {
                if (flipbyte(getPixelHString((xx+offset),yy, mystring, FONT5x7)))             
                setcolor = For_color; 
                else setcolor=Bk_color;
                LED(xx, yy, setcolor);
              }

            }
        }
        delay(delaytime); 
      }
  }
//*******************************************************FONT 8x8*****************************************************//
else if (font == FONT8x8)
{  
  for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*8-1); (dir) ? offset <((lenString(mystring)-6)*8-1): offset >0; (dir) ? offset++ : offset--)
  {
    READ_MPU6050();
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<8; yy++)
            { 
              pos = (AccX >0 ? 1:0);
              if (pos)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT8x8)) 
                setcolor = For_color; 
                else setcolor = Bk_color;                 
                LED(14-xx,7-yy,setcolor);
              }

              else
              {
                if (flipbyte(getPixelHString((xx+offset),yy, mystring, FONT8x8)))             
                setcolor = For_color; 
                else setcolor=Bk_color;
                LED(xx, yy, setcolor);
              }

            }
        }
        delay(delaytime); 
      }
  } 
//*******************************************************FONT 8x16*****************************************************//
else if (font == FONT8x16)
{  
  for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*9-1); (dir) ? offset <((lenString(mystring)-6)*9-1) : offset >0; (dir) ? offset++ : offset--)
  {
    READ_MPU6050();
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<13; yy++)
            { 
              pos = (AccX >0 ? 1:0);
              if (pos)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT8x16)) 
                setcolor = For_color; 
                else setcolor = Bk_color;                 
                LED(14-xx,12-yy,setcolor);
              }

              else
              {
                if (flipbyte(getPixelHString((xx+offset),yy, mystring, FONT8x16)))             
                setcolor = For_color; 
                else setcolor=Bk_color;
                LED(xx, yy, setcolor);
              }

            }
        }
        delay(delaytime); 
      }
  } 
}

char flipbyte (char byte)
{
  char flop = 0x00;

  flop = (flop & 0b11111110) | (0b00000001 & (byte >> 7));
  flop = (flop & 0b11111101) | (0b00000010 & (byte >> 5));
  flop = (flop & 0b11111011) | (0b00000100 & (byte >> 3));
  flop = (flop & 0b11110111) | (0b00001000 & (byte >> 1));
  flop = (flop & 0b11101111) | (0b00010000 & (byte << 1));
  flop = (flop & 0b11011111) | (0b00100000 & (byte << 3));
  flop = (flop & 0b10111111) | (0b01000000 & (byte << 5));
  flop = (flop & 0b01111111) | (0b10000000 & (byte << 7));
  return flop;
}
