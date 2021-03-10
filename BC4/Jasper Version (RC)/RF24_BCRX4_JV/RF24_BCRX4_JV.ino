// Radio receiver/server, i2c master

//BC4 Receiver Code

#include <RadioHead.h> 
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

// Radio Adressing
#define TX_ADDRESS      7
#define RX_ADDRESS      8

// Radio Software Setup
#define CHANNEL 4
#define TX_POWER RH_NRF24::TransmitPowerm18dBm
#define DATARATE RH_NRF24::DataRate1Mbps

// Initiliaze radio driver and manager
RH_NRF24 driver;
RHReliableDatagram radioManager(driver, RX_ADDRESS);

// Declare the input data buffer and the output LEDdata buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t LEDdata[RH_NRF24_MAX_MESSAGE_LEN];

void setup() {
  //Serial startup
  Serial.begin(9600);
  Serial.println("startin up");

  //Setup serial communication with the pneuma mega
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDread);

  //Check if radio failed to initialize
  if (!radioManager.init()) {
    Serial.println("RX radio initialization failed");
  }
  //Communicate radio readiness
  else {
    Serial.println("RX radio initialized");
    Serial.print("Receiving transmissions at RX_ADDRESS: "); Serial.println(RX_ADDRESS);
    Serial.print("From TX_ADDRESS: "); Serial.println(TX_ADDRESS); Serial.println();
    Serial.print("On Channel: "); Serial.println(CHANNEL); Serial.println();
    
    //Set the config of the radio driver and manager
    driver.setChannel(CHANNEL);
    driver.setRF(DATARATE, TX_POWER);
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
      //Attempt to respond with current LED data
      if (!radioManager.sendtoWait(LEDdata, sizeof(LEDdata), TX_ADDRESS)) {
         Serial.println("Failed Response");
      }
      
      //print out the received info in buffer
      for (int i=0; i<len;i++){
        Serial.print(buf[i]); Serial.print(" ");
      }
      Serial.println();

      //pass on buf to the pneuma mega
      Wire.beginTransmission(1);
      Wire.write(buf, len);
      Wire.endTransmission();
    }
  }
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
