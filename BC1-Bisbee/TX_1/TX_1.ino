#include <RadioHead.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

// BC1 Transmitter code

/*
 RC-STYLE MULTICHANNEL COMMUNICATION IMPLEMENTATION

 CONTROL MAPPING FOR BC1: (This is all of the options, includin joystick control)
 Channel  :  1  2  3  4  5  6  7  8  9 10 11 12 13 14 17 18
 Order    :                                           PU MA
 PSaddy   :  4  6  8 11 13 15 17 19 21 23 24 27 28 30 34 32
 VSaddy   :  5  7  9 10 12 14 16 18 20 22 25 26 29 31 35 33
 POTaddy  : 54 55 56 57 58 59 60 61 62 63 64 65 66 67
 LED      : 48 49 47 46 45 44 43 42 41 40 39 38

 NOTES FOR BC1: 
 08/05/2021: INTIAL SETUP
*/

// Channel informations
#define numChan      15 // INCLUDING BLOWER, MASTER, AND ANY JOYSTICK
#define numValveChan 13 // number of exclusivly valve channels

// Radio Software Setup
#define CHANNEL      25 //(from 0-125) Note: be careful to space these out and match between Tx and Rx,
                                                       // otherwise they can interfere with eachother
#define TX_POWER RH_NRF24::TransmitPowerm18dBm // dont need to change this, but you could
#define DATARATE RH_NRF24::DataRate1Mbps // dont need to change this but you could

// Radio hardware setup
#define NRF24_CS       53 // Chip/Slave Select pin DONT CHANGE
#define NRF24_EN       2  // Enable pin DONT CHANGE

// Control tuning values
#define POT_DEADBAND   3 // change if you want? adjusts the size of the pot deadzone before proportional drive engages

//Radio driver and manager setup
RH_NRF24 driver(NRF24_EN, NRF24_CS);

//Initialize the input buffer and output data
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t data[numChan];

//Init Debug
bool debug = true;

// ******************* CONFIGURATION *********************************************************************************
const uint8_t potPINS[numValveChan] =   { 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66};
const uint8_t vSwitchPins[ numChan] =   { 12, 10,  9,  7,  5, 14, 16, 18, 20, 22, 25, 26, 29, 35, 33};
const uint8_t pSwitchPins[ numChan] =   { 13, 11,  8,  6,  4, 15, 17, 19, 21, 23, 24, 27, 28, 34, 32};
const uint8_t LEDpins[numValveChan] =   { 48, 49, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38};
// *******************************************************************************************************************

//Initialize the readings
uint16_t potReadings[numValveChan];
bool pSwitchReadings[numChan];
bool vSwitchReadings[numChan];

//Initialize the blinker bool
bool blinker;

void setup() {
  //LED pin blinking sequence
  for (int i = 0; i < numValveChan; i++) {
    // Init the pin
    pinMode (LEDpins[i],      OUTPUT);

    // flicker between red and green then off
    digitalWrite (LEDpins[i], HIGH);
    delay (100);
    digitalWrite (LEDpins[i], LOW);
    delay (100);
    pinMode (LEDpins[i],      INPUT);
  }

  // Init the switch pins
  for (int i = 0; i < numChan; i++) {
    pinMode (pSwitchPins[i],  INPUT_PULLUP);
    pinMode (vSwitchPins[i],  INPUT_PULLUP);
  }

  // Init the Serial
  if (debug) Serial.begin(9600);

  // See if radio init correctly
  if (!driver.init()) {
    if (debug) Serial.println("TX init failed");
    while(1);
  }
  else {
    if (debug) Serial.println("TX radio initialized");
    if (debug) Serial.print("On Channel: "); Serial.println(CHANNEL); Serial.println();
    driver.setChannel(CHANNEL);
    driver.setRF(DATARATE, TX_POWER);
  }
  // communicate its ready and then chill for a bit
  if (debug) Serial.println("BCcontrollerTX Ready.");
  if (debug) Serial.println();
  delay(500);
}

void loop() {
  static unsigned long last_display_time = 0; // for timekeeping
    
  // READING CONTROLS
  // Note: the last two channels are blower and master pressure, but they are handled by the reciever
  for (int i = 0; i < numChan; i++) {
    if (i < numChan-2) { // Valves and Utility channels only
      // Read pots
      potReadings[i] = 100 - analogRead(potPINS[i]) * (100.0 / 1023.0); // interprets pot signal from 0 to 100

      // Read switches
      pSwitchReadings[i] = !digitalRead(pSwitchPins[i]);
      vSwitchReadings[i] = !digitalRead(vSwitchPins[i]);

      // Interpret control to yield data entry
      // check for manual control
      if (pSwitchReadings[i] == 1 || vSwitchReadings[i] == 1) { // only overwrite the pots if switches are active or if its under the deadband
        data[i] = 100 + pSwitchReadings[i] + 2 * vSwitchReadings[i]; // means 104 is idle, 101 is vacuum, 102 is pressure, 103 is error
      }
      // otherwise if the pot is outside of the deadband, use that signal
      else if (potReadings[i] > POT_DEADBAND) data[i] = potReadings[i];
      // otherwise if in the deadband, idle
      else data[i] = 104;
    }
    else { //Blower and P/V Master
      pSwitchReadings[i] = !digitalRead(pSwitchPins[i]);
      vSwitchReadings[i] = !digitalRead(vSwitchPins[i]);
      data[i] = 100 + pSwitchReadings[i] + 2 * vSwitchReadings[i]; // same encoding as above
    }
  }

  //UPDATE DISPLAY
  if ((millis() - last_display_time) > 100) { // update every tenth of second
    last_display_time = millis(); // reset clock
    // actually update the display
    LEDdisplay();
  }

  // transmit and receive radio data and display data
  //Send data to RX
  driver.send(data, sizeof(data));
  driver.waitPacketSent(); // wait until it's sent fully
  if (driver.waitAvailableTimeout(100)) {// wait for an available message
    
    //initialize some variables to point to
    uint8_t len = sizeof(buf);
    uint8_t from;

    // Now attempt to pickup a reply from the receiver
    driver.recv(buf, &len);
    //if received and debug is on, print out the LED command
    if (debug){
      for (int i = 0; i < numValveChan; i++) {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  //else{Serial.println("FAILED RESPONSE");}//get ready for a lot of failed responses, its RC and signals fail often

  // if debug is on, print out what the TX just tried to send
  if (debug){
    Serial.print(" sent      ");
    for (int i = 0; i < sizeof(data); i++) {
      Serial.print(data[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void LEDdisplay() {
  //HIGH-->Red, LOW--> Green, INPUT-->Off
  blinker = !blinker;// self-invert the blinker so it uh functions as a blinker
  for (int i = 0; i < numValveChan; i++) {
    int state = buf[i];// getting the commanded LED state from the buffer
    if      (state==0) {pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i], HIGH);} //0 is Solid Red
    else if (state==1) {pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i],  LOW);} //1 is Solid Green
    else if (state==2) {pinMode(LEDpins[i],  INPUT);                                } //2 is Off 
    else{// Blinking lights
      if (state==3){
        if (blinker) { pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i], HIGH);} // 3 is blink red
        else         { pinMode(LEDpins[i],  INPUT);                                }
      }
      else if (state==4){
        if (blinker) { pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i],  LOW);} // 4 is blink green
        else         { pinMode(LEDpins[i],  INPUT);                                }
      }
    }
  }
}
