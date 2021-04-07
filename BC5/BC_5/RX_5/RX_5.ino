// BC5 Receiver Code

#include <RadioHead.h> 
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

// Radio Addressing
#define TX_ADDRESS 9 //make sure it matches the Tx
#define RX_ADDRESS 10 //make sure it matches the Tx

// Radio Software Setup
#define CHANNEL      100 //(from 0-125) Note: be careful to space these out and match between Tx and Rx,
                                                       // otherwise they can interfere with eachother
#define TX_POWER RH_NRF24::TransmitPowerm18dBm // dont need to change this, but you could
#define DATARATE RH_NRF24::DataRate1Mbps // dont need to change this but you could

// LEDs
#define LED           9
#define LED_GREEN_1   6
#define LED_GREEN_2   7
#define LED_RED_1     8
#define LED_RED_2     9

// Initiliaze radio driver and manager
RH_NRF24 driver(2);
RHReliableDatagram radioManager(driver, RX_ADDRESS);

// Declare the input data buffer and the output LEDdata buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t LEDdata[RH_NRF24_MAX_MESSAGE_LEN];

void setup() {
  //Serial startup
  Serial.begin(9600);

  //LED pin init
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);
  
  //Setup serial communication with the pneuma mega
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDDataRead); // setup receive function

  //Check if radio failed to initialize
  if (!radioManager.init()) {
    Serial.println("RX radio initialization failed");
    digitalWrite(LED_RED_2, HIGH); // red failed init
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

    //Signal succesful init
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
      //Attempt to respond with current LED data
      if (!radioManager.sendtoWait(LEDdata, sizeof(LEDdata), TX_ADDRESS)) {
         Serial.println("Failed Response");
         blinker(LED_RED_1); // Signal failed response
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

      //Awknowledge the received signal
      blinker(LED_GREEN_1);
    }
  }
}

void LEDDataRead(int howmany){  // CALLED WHEN I2C MESSAGE ARRIVES from BC
  for (int i = 0; i < howmany; i++) { // Reads wire into LEDdata piecemeal
    LEDdata[i] = Wire.read();
  }
}

void blinker(byte pin){ // blink an led
  digitalWrite(pin, HIGH);
  delay(10);
  digitalWrite(pin, LOW);
}
