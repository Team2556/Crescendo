//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  //delay(500);
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    
    //Serial.print("Detected ");
    //Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      // Serial.print("  block ");
      // Serial.print(i);
      // Serial.print(": ");
      //pixy.ccc.blocks[i].print();
      if (pixy.ccc.blocks[i].m_x < 317){
        delay(100);
      Serial.println(pixy.ccc.blocks[i].m_x);}
      else {
        Serial.println(0);
      }
      // Serial.print(pixy.ccc.blocks[i].m_y);
    }
  } 

        if(Serial.available()) {
    //Read a byte from the input buffer
    byte value = Serial.read();
    

    //If the byte is 0x12 (i.e. 18 in decimal)
    if(value == 0x12) {
      //delay(500);
      //Write a string to the output buffer as a response to send to the RoboRIO
      //Serial.println("Hello! I received code 0x12");
        if (pixy.ccc.blocks[i].m_x < 317){
          delay(100);
      Serial.println(pixy.ccc.blocks[i].m_x);}
      else Serial.println(0);

      //Serial.println(pixy.ccc.blocks[i].m_y);
    }
  }

}

