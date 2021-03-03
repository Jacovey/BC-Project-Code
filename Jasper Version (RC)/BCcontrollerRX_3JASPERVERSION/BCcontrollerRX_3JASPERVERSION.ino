#include <RadioHead.h>  

// LoRa receiver/server, i2c master

#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
                       //BC#: 4  3
#define TX_ADDRESS      7 //: 7  9
#define RX_ADDRESS      8 //: 8 10

#define RFM95_RST     2  // "A"

#define LED           9
#define LED_GREEN_1   6
#define LED_GREEN_2   7
#define LED_RED_1     8
#define LED_RED_2     9

RH_RF95 driver;
RHReliableDatagram radioManager(driver, RX_ADDRESS);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t LEDdata[RH_RF95_MAX_MESSAGE_LEN];

void setup() {
  Serial.begin(9600);   
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);      
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDread);
  driver.setFrequency(915.0); 
  
  if (!radioManager.init()) {
    Serial.println("RX radio initialization failed");
    digitalWrite(LED_RED_1, HIGH);
  }
  
  else {
    Serial.println("RX radio initialized");
    Serial.print("Receiving transmissions at RX_ADDRESS: "); Serial.println(RX_ADDRESS);
    Serial.print("From TX_ADDRESS: "); Serial.println(TX_ADDRESS); Serial.println();
    digitalWrite(LED_GREEN_2, HIGH); 
    radioManager.setTimeout(30);
    radioManager.setRetries(0);
    driver.setTxPower(19,true);
  }
  blinker(LED_RED_2);
}

void loop(){
  //check if packet is available
  if (radioManager.available()){
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    // get that packet, store in buf
    if (radioManager.recvfromAck(buf, &len, &from)){
      delay(20);
      //Attempt to respond with current LED data
      if (!radioManager.sendtoWait(LEDdata, sizeof(LEDdata), TX_ADDRESS)) {
         blinker(LED_RED_1);
      }
      
      //print out the received info in buffer
      for (int i=0; i<len;i++){
        Serial.print(buf[i]); Serial.print(" ");
      }
      Serial.println();

      //Write buf to the pneuma shield MEGA
      Wire.beginTransmission(1);
      Wire.write(buf, len);
      Wire.endTransmission();

      // Aknoweledgment blink
      blinker(LED_GREEN_1);
    }
  }
  delay(10);
}

void LEDread(int howmany){  // CALLED WHEN I2C MESSAGE ARRIVES from BC
  for (int i = 0; i < howmany; i++) {
    LEDdata[i] = Wire.read();
  }
}

void blinker(byte pin){
  digitalWrite(pin, HIGH);
  delay(10);
  digitalWrite(pin, LOW);
}
