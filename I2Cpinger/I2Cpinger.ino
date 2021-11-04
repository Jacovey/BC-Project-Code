#include <Wire.h>
// Description: Boilerplate script that sends messages until one is received and responded to
// Usage: test the transmitting part of an I2C setup

uint8_t shake[1];
bool received;

void setup() {
  Serial.begin(9600);
  Serial.println("PingerScript running...");
  Wire.begin(2);// begin transmitting back to the receiver
  Wire.onReceive(pingBack); // setup response triggering
  
  shake[1]=100;// message to send
  received = 0; // storage for message to be received
  
}

void loop(){
  if(!received){// if you havent received anything
    //sending initial message
    Serial.println("Hello");
    uint8_t len = sizeof(shake); // gets length of the packet to send
    Wire.beginTransmission(1); // begin transmission
    Wire.write(shake,len); // send the message
    Wire.endTransmission(1); // end transmission
  }
  delay(100);// wait .1 s
}

void pingBack(int howmany){ // called when a message is received (higher priority than loop)
  Serial.println("Hello to you too"); // print out that this device received a message
  uint8_t len = sizeof(shake);
  Wire.beginTransmission(1); // send back a messsage as above
  Wire.write(shake,len);
  Wire.endTransmission(1);
  Serial.println();
  received=1;// set response
}
