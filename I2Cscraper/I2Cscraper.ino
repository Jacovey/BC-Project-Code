#include <Wire.h>
// Description: Boilerplate script that listens for a message from I2C communication then pings back a message
// Usage: test the receiving part of an I2C setup

uint8_t shake[1];

void setup() {
  Serial.begin(9600);
  Serial.println("ScraperScript running...");
  Wire.begin(1); // begin transmitting/receiving
  Wire.onReceive(pingBack); // setup response trigger
}

void pingBack(){ // 
  Serial.print("Hello: "); // print that a message was recived
  shake[1] = Wire.read(); // read the message
  uint8_t len = sizeof(shake); // get the size of the message
  Serial.print(shake[1], DEC); // print out the message
  Wire.beginTransmission(2);// begin transmission
  Wire.write(shake,len); // send the message right back
  Wire.endTransmission(2); // end transmission
  Serial.println();
}
void loop(){} // no need for loop, its all interrupts!
