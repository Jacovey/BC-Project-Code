// BC2-Annie Receiver Code

#include <RadioHead.h> 
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

// Radio Software Setup
#define CHANNEL      50 //(from 0-125) Note: be careful to space these out and match between Tx and Rx,
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
RH_NRF24 driver;

// Declare the input data buffer and the output LEDdata buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t LEDdata[RH_NRF24_MAX_MESSAGE_LEN];

// delare blinker bool
bool blinkBool = false;

//Debug bool
bool debug = false;

void setup() {
  //Serial startup
  if (debug) Serial.begin(9600);

  //LED pin init
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);
  
  
  //Setup serial communication with the pneuma mega
  Wire.begin(2); // join i2c bus (address optional for master)
  Wire.onReceive(LEDDataRead); // setup receive function

  //Check if radio failed to initialize
  if (!driver.init()) {
    if (debug) Serial.println("RX radio initialization failed");
    digitalWrite(LED_RED_2, HIGH); // red failed init
  }
  //Communicate radio readiness
  else {
    if (debug) Serial.println("RX radio initialized");
    if (debug) Serial.print("On Channel: "); Serial.println(CHANNEL); Serial.println();
    
    //Set the config of the radio driver and manager
    driver.setChannel(CHANNEL);
    driver.setRF(DATARATE, TX_POWER);

    //Signal succesful init
    digitalWrite(LED_GREEN_2, HIGH); 
  }
  delay(200);
}

void loop(){
  //Wait for incoming packet
  if (driver.waitAvailableTimeout(1000)){
    //init the storage
    uint8_t len = sizeof(buf);

    // get the packet
    driver.recv (buf, &len);
    //awknoledge the receipt
    blinker(LED_GREEN_1);
    
    // pass on buf to the pneuma mega
    Wire.beginTransmission(1);
    Wire.write(buf, len);
    Wire.endTransmission();

    // send LED data back
    driver.send(LEDdata, sizeof(LEDdata));
    driver.waitPacketSent(); // wait till its done sending
    //print out the received info in buffer
    if (debug){
      for (int i=0; i<len;i++){ // print out received data
        Serial.print(buf[i]); Serial.print(" ");
      }
      Serial.println();
    }
  }
}

void LEDDataRead(int howmany){  // CALLED WHEN I2C MESSAGE ARRIVES from BC
  for (int i = 0; i < howmany; i++) { // Reads wire into LEDdata piecemeal
    LEDdata[i] = Wire.read();
  }
}

void blinker(byte pin){ // blink an led
  blinkBool = !blinkBool;
  if (blinkBool) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
}
