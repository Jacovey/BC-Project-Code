// LoRa receiver/server, i2c master

#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define TX_ADDRESS    7
#define RX_ADDRESS    8

#define LED           9
#define LED_GREEN_1   6
#define LED_GREEN_2   7
#define LED_RED_1     8
#define LED_RED_2     9

RH_RF95 driver;
RHReliableDatagram radioManager(driver, RX_ADDRESS);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t twoBytePacket[2];
uint8_t twelveBytes[12];

void setup() {
  Serial.begin(9600);  
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);      
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDdisplay);
  while (!Serial); 
  driver.setFrequency(433.0);    
  if (!radioManager.init()) {
    Serial.println("RX radio initialization failed");
    digitalWrite(LED_RED_1, HIGH);
  }
  
  else {
    Serial.println("RX radio initialized");
    Serial.print("Receiving transmissions at RX_ADDRESS: "); Serial.println(RX_ADDRESS);
    Serial.print("From TX_ADDRESS: "); Serial.println(TX_ADDRESS); Serial.println();
    digitalWrite(LED_GREEN_2, HIGH); 
  }
//  delay(1000);
  blinker(LED_RED_2);

}

void loop()
{
  if (radioManager.available()){
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (radioManager.recvfromAck(buf, &len, &from))
    {

      Wire.beginTransmission(1);
      Wire.write(buf, len);
      Wire.endTransmission();

      blinker(LED_GREEN_1);

      if (!radioManager.sendtoWait(twoBytePacket, 2, from)) {
        blinker(LED_RED_1);
        Serial.println("RX sendtoWait failed");
      }
    }
  }
}

void LEDdisplay(){                      // CALLED WHEN I2C MESSAGE ARRIVES from BC
//  Serial.print("LED valve indicator: ");
  for (byte i = 0; i < 2; i++) {
    twoBytePacket[i] = Wire.read();
//    Serial.print(twoBytePacket[0], DEC); Serial.print(" "); Serial.println(twoBytePacket[1]);
  }
//  if(!radioManager.sendtoWait(twoBytePacket, 2, TX_ADDRESS)) Serial.println("FUCKYOU");
//  radioTransmitter();
//  Serial.println();
}

//void radioTransmitter(){
//  Serial.print("Send "); Serial.print(twoBytePacket[0]); Serial.print(" ");
//  Serial.println(twoBytePacket[1]);
//}

void blinker(byte pin){
  digitalWrite(pin, HIGH);
  delayMicroseconds(10000);
  digitalWrite(pin, LOW);
}
