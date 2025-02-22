#include "Bitcraze_PMW3901.h"
#include <Wire.h>

#define UNO_ADDR 9    //I2C connected to Pico
#define RESP_SIZE 8   //message size 

Bitcraze_PMW3901 flow(10);  //use digital pin 10 for chip select
String msg;   //defome msg as String type

void setup() {
  Serial.begin(9600);   //begin serial tranmission for monitoring purposes
  Wire.begin(UNO_ADDR);   //begin I2C

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");   //check if flow has started as expected
    while(1) { }    //wait until initialization suceeded
  }
}

int16_t deltaX,deltaY;    //define deltaX and deltaY as int16_t datatype

void loop() {
  flow.readMotionCount(&deltaX, &deltaY);   // extract change of motion since last call
  //Serial.print("X: ");
  //Serial.print(int(deltaX));
  //Serial.print(", Y: ");
  //Serial.print(int(deltaY));
  //Serial.print("\n");
  //String deltaX_str = to_string(deltaX);
  //string str = "Hello " + std::string("world");

  msg = String(int(deltaX)) + ";"+ String(int(deltaY))+";";   //package message with delimiter
  Serial.print(msg);    //print message to terminal for monitoring purposes
  Serial.print("\n");   
  Wire.onRequest(DataRequest);    //send message to Pico

  //delay(50);
}

void DataRequest()
{
  byte resp[RESP_SIZE];
  for (byte i=0; i<RESP_SIZE; ++i) {    //package message into binary data 
    resp[i] = (byte)msg.charAt(i);
  }
  Wire.write(resp, sizeof(resp));   //write message to I2C bus
  Serial.println(msg);    //output formatted message in terminal for monitoring
}