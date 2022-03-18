//
//  Control TM1637-driven 7-segment displays
//
//  Original author:Frankie.Chu
//  Date:9 April,2012
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//  Modified record:
//  Modified by Janne Korkkula <jk@iki.fi> 2018, TubeTab extended beyond digits
//
/*******************************************************************************/
#include "TM1637.h"
#include <Arduino.h>

// Extended ASCII, impossible chars (KMVWXkmvwx) render as 0x08 '_'
static int8_t TubeTab[] = {
  0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,
  0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71,  // 0~9,A,b,C,d,E,F
  0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x00,  // 0x10 each segment bit in seq
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x18 (control chars)
  0x00,0x2a,0x22,0x54,0x6d,0x2d,0x46,0x20,  // 0x20   ! " # $ % & '
  0x39,0x0f,0x5c,0x70,0x10,0x40,0x04,0x52,  // 0x28 ( ) * + , - . /
  0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,  // 0x30 0 1 2 3 4 5 6 7 
  0x7f,0x6f,0x09,0x0d,0x21,0x48,0x03,0x4b,  // 0x38 8 9 : ; < = > ?
  0x5f,0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,  // 0x40 @ A B C D E F G
  0x76,0x30,0x1e,0x08,0x38,0x08,0x37,0x3f,  // 0x48 H I J K L M N O
  0x73,0x67,0x50,0x6d,0x78,0x3e,0x08,0x08,  // 0x50 P Q R S T U V W
  0x08,0x6e,0x5b,0x39,0x64,0x5c,0x23,0x08,  // 0x58 X Y Z [ \ ] ^ _
  0x02,0x77,0x7c,0x58,0x5e,0x7b,0x71,0x6f,  // 0x60 ` a b c d e f g
  0x74,0x10,0x1e,0x08,0x18,0x08,0x54,0x5c,  // 0x68 h i j k l m n o
  0x73,0x67,0x50,0x6d,0x78,0x1c,0x08,0x63,  // 0x70 p q r s t u v w
  0x08,0x6e,0x5b,0x39,0x30,0x0f,0x08,0x00,  // 0x78 x y z { | } ~ DEL
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x80 (ext control)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x88 (ext control)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x90 (ext control)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x98 (ext control)
  0x00,0x00,0x00,0x00,0x63,0x00,0x00,0x00,  // 0xa0 . . . . ¤ . . .
  0x00,0x00,0x00,0x00,0x44,0x00,0x00,0x01,  // 0xa8 . . . . ¬ . . ¯
  0x44,0x00,0x00,0x00,0x20,0x1c,0x00,0x00,  // 0xb0 ° . . . ´ µ . .
  0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xb8 ¹ . . . . . . .
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xc0 . . . . . . . .
  0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x00,  // 0xc8 0xcc Ì 206 = 'II'
  0x00,0x00,0x00,0x00,0x00,0x14,0x00,0x00,  // 0xd0 . . . . . . . .
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xd8 0xdd Ý 221 = 'ii'
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xe0 . . . . . . . .
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xe8 . . . . . . . .
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xf0 . . . . . . . .
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00   // 0xf8 . . . . . . . .
};

TM1637::TM1637(uint8_t Clk, uint8_t Data) {
  Clkpin = Clk;
  Datapin = Data;
  pinMode(Clkpin,  OUTPUT);
  pinMode(Datapin, OUTPUT);
}

void TM1637::init(void) {
  clearDisplay();
}

void TM1637::writeByte(int8_t wr_data) {
  uint8_t i; 
  uint8_t count1 =  0;   
  for(i = 0; i < 8; i++) { // sent 8bit data
    digitalWrite(Clkpin, LOW);      
    if (wr_data & 0x01) digitalWrite(Datapin, HIGH); // LSB first
    else digitalWrite(Datapin, LOW);
    wr_data >>= 1;      
    digitalWrite(Clkpin, HIGH);
  }  
  digitalWrite(Clkpin,  LOW); // wait for the ACK
  digitalWrite(Datapin, HIGH);
  digitalWrite(Clkpin,  HIGH);     
  pinMode(Datapin, INPUT);
  while (digitalRead(Datapin)) { 
    count1++;
    if (count1 == 200) {
      pinMode(Datapin, OUTPUT);
      digitalWrite(Datapin, LOW);
      count1 = 0;
    }
    pinMode(Datapin, INPUT);
  }
  pinMode(Datapin, OUTPUT);
}

// send start signal to TM1637
void TM1637::start(void) {
  digitalWrite(Clkpin,  HIGH); // send start signal to TM1637
  digitalWrite(Datapin, HIGH); 
  digitalWrite(Datapin, LOW); 
  digitalWrite(Clkpin,  LOW); 
} 

// End of transmission
void TM1637::stop(void) {
  digitalWrite(Clkpin,  LOW);
  digitalWrite(Datapin, LOW);
  digitalWrite(Clkpin,  HIGH);
  digitalWrite(Datapin, HIGH); 
}

// display function. Write to full-screen.
void TM1637::display(int8_t DispData[]) {
  int8_t  SegData[4];
  uint8_t i;
  for(i = 0; i < 4; i ++) {
    SegData[i] = DispData[i];
  }
  coding(SegData);
  start();          // start signal sent to TM1637 from MCU
  writeByte(ADDR_AUTO);
  stop();
  start();
  writeByte(Cmd_SetAddr);
  for(i = 0; i < 4; i++) {
    writeByte(SegData[i]);
  }
  stop();
  start();
  writeByte(Cmd_DispCtrl);
  stop();
}

//******************************************
void TM1637::display(uint8_t BitAddr, int8_t DispData) {
  int8_t SegData;

  SegData = coding(DispData, BitAddr);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_FIXED);

  stop();
  start();
  writeByte(BitAddr|0xc0);
  
  writeByte(SegData);
  stop();
  start();
  
  writeByte(Cmd_DispCtrl);
  stop();
}

void TM1637::clearDisplay(void) {
  this->clearDisplay(true);
}
void TM1637::clearDisplay(boolean clear_all_points) {
  if (clear_all_points) _PointFlag = POINT_OFF;
  display(0, 0x7f);
  display(1, 0x7f);
  display(2, 0x7f);
  display(3, 0x7f);  
}

void TM1637::set(int8_t brightness, uint8_t SetData, uint8_t SetAddr) {
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = 0x88 + brightness; //Set brightness, takes effect at next display
}

//Whether to light the clock point ":".
//To take effect the next time it displays.
void TM1637::point(uint8_t PointFlag) {
  if(PointFlag == 1)
    _PointFlag = POINT_ALL;
  else
    _PointFlag = PointFlag;
}

void TM1637::coding(int8_t DispData[]) {
  for(uint8_t i = 0; i < 4; i++) {
    if (DispData[i] == 0x7f) 
      DispData[i] = 0;
    else
      DispData[i] = TubeTab[DispData[i]];
    if( _PointFlag & (1 << (i + 1)) ) 
      DispData[i] += 0x80;
  }
}

int8_t TM1637::coding(int8_t DispData) {
  return this->coding(DispData, 3); // legacy support, last digit -> clock ':' point
}
int8_t TM1637::coding(int8_t DispData, uint8_t BitAddr) {
  if (DispData == 0x7f) 
    DispData = 0;
  else
    DispData = TubeTab[DispData];
  if( _PointFlag & (1 << (BitAddr + 1)) ) 
    DispData += 0x80;

  return DispData;
}

// EOF
