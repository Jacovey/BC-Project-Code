// LoRa receiver/server, i2c master

#include <RadioHead.h> 
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

// LoRa Adressing
#define TX_ADDRESS      5
#define RX_ADDRESS      6

// Define the interupt pin for the LoRa
#define RFM95_RST     2

// Def LED pins
#define LED           9
#define LED_GREEN_1   6
#define LED_GREEN_2   7
#define LED_RED_1     8
#define LED_RED_2     9

// Initiliaze the LoRa driver and manager
RH_RF95 driver;
RHReliableDatagram radioManager(driver, RX_ADDRESS);

// Declare the input data buffer and the output LEDdata buffer
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t LEDdata[RH_RF95_MAX_MESSAGE_LEN];

void setup() {
  //Serial startup
  Serial.begin(9600);

  //Setup the LED pins
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);

  //Setup serial communication with the pneuma mega
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDread);

  //Set the config of the LoRa driver and manager
  driver.setFrequency(915.0);
  driver.setTxPower(23,false);
  radioManager.setTimeout(100);

  //Check if LoRa failed to initialize
  if (!radioManager.init()) {
    Serial.println("RX radio initialization failed");
    digitalWrite(LED_RED_1, HIGH);
  }
  //Communicate LoRa readiness
  else {
    Serial.println("RX radio initialized");
    Serial.print("Receiving transmissions at RX_ADDRESS: "); Serial.println(RX_ADDRESS);
    Serial.print("From TX_ADDRESS: "); Serial.println(TX_ADDRESS); Serial.println();
    digitalWrite(LED_GREEN_2, HIGH); 
  }
  delay(200);
}

void loop(){
  //check if packet is available
  if (radioManager.available()){
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    // get that packet, store in buf
    if (radioManager.recvfromAck(buf, &len, &from)){
      //print out the received info in buffer
      for (int i=0; i<len;i++){
        Serial.print(buf[i]); Serial.print(" ");
      }
      Serial.println();

      //Attempt to respond with current LED data
      if (!radioManager.sendtoWait(LEDdata, sizeof(LEDdata), TX_ADDRESS)) {
         blinker(LED_RED_1);
      }
      
      //pass on buf to the pneuma mega
      Wire.beginTransmission(1);
      Wire.write(buf, len);
      Wire.endTransmission();

      // Aknoweledgment blink
      blinker(LED_GREEN_1);
    }
  }
  delay(10); // give LoRa a bit of a breather
}

void LEDread(int howmany){  // CALLED WHEN I2C MESSAGE ARRIVES from BC
  for (int i = 0; i < howmany; i++) { // Reads wire into LEDdata piecemeal
    LEDdata[i] = Wire.read();
  }
}

void blinker(byte pin){ // blink an led
  digitalWrite(pin, HIGH);
  delay(10);
  digitalWrite(pin, LOW);
}
