#include <RadioHead.h>

// LoRa transmitter/client, i2c slave

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

/* MEGA External Interrupt PINS:
   G0:
   2 (interrupt 0),
   3 (interrupt 1),
   18 (interrupt 5),
   19 (interrupt 4),
   20 (interrupt 3),
   21 (interrupt 2)
   u
   /////////////// SABERTOOTH H-BRIDGE CONTROL: /////////////////

  Control each motor with servo commands
  Use 3-argument init:

  servo.attach(pin, 1000, 2000);

  Compatible with R/C versions of Sabertooth
  Servo library comes with Arduino software

  0°  -  90° (1000-1500 μs) reverses
  90° - 180° (1500-2000 μs) goes forward
  90° (1500 μs) stops

  ///////////////////////////////////////////////////////////////

   MEGA:  FUNC:     UNO:
   2      G0        2     PURPLE
   52     SCK       13    YELLOW
   50     MISO      12    GREEN
   51     MOSI      11    BLUE
   53     CS        10    RED
   remove 2k2 resistors on pressure controller board for highest port, replace with 4resistor array

  Sabertooth 2x32 dimensionengineering.com is motor controller for 2 wheel drive

  BORDER CROSSER MIDI IMPLEMENTATION:

  Pressure      Fader Pres.   Vac.
  servoChannel  CC    NN      NN
  --------------------------------
  0           1     1       2
  1           2     3       4
  2           3     5       6
  3           4     7       8
  4           5     9       10
  5           6     11      12
  6           7     13      14
  7           8     15      16
  8           9     17      18
  9           10    19      20
  10          11    21      22
  11          12    23      24
  12          13    25      26
  13          14    27      28
  14          15    29      30
  15          16    31      32
  ----------------------------------
*/

#define numChan      11 // INCLUDING BLOWER AND MASTER
                       //BC#: 4  3
#define TX_ADDRESS      7 //: 7  9
#define RX_ADDRESS      8 //: 8 10

#define RFM95_CS        53 // Chip/Slave Select pin
#define RFM95_INT       2  // PinChangeInterrupt pin
#define POT_DEADBAND   3

RH_RF95 driver(RFM95_CS, RFM95_INT);
RHReliableDatagram radioManager(driver, TX_ADDRESS);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// ******************* CONFIGURATION *********************************************************************************
const uint8_t potPINS[numChan - 2] =   { 54,  55, 56, 57, 58, 59, 60, 61, 62};
//32 and up are pump/master valve
const uint8_t vSwitchPins[numChan] =   { 12, 10, 9, 7, 5, 14, 16, 20, 22, 35, 33};
const uint8_t pSwitchPins[numChan] =   { 13, 11, 8, 6, 4, 15, 17, 21, 23, 34, 32};

const uint8_t LEDpins[numChan - 2] =   { 48,  49, 47, 46, 45, 44, 43, 41, 40};
// *******************************************************************************************************************

uint16_t potReadings[numChan - 2] ;
bool pSwitchReadings[numChan] ;
bool vSwitchReadings[numChan] ;
bool blinker;
uint8_t data[numChan];

void setup() {
  //LED init
  for (int i = 0; i < numChan - 2; i++) {
    pinMode (LEDpins[i],      OUTPUT);
    digitalWrite (LEDpins[i], HIGH);
    delay (100);
    digitalWrite (LEDpins[i], LOW);
    delay (100);
    pinMode (LEDpins[i],      INPUT);
  }

  //  PUMP AND MASTER SWITCH PINS
  for (int i = 0; i < numChan; i++) {
    pinMode (pSwitchPins[i],  INPUT_PULLUP);
    pinMode (vSwitchPins[i],  INPUT_PULLUP);
  }

  // Initializing readout and radio
  Serial.begin(9600);
  Serial.println("BCcontrollerTX_4 Ready.");
  Serial.println();
  driver.setFrequency(915.0);
  if (!radioManager.init()) Serial.println("TX init failed");
  else {
    Serial.println("TX radio initialized");
    Serial.print("Transmitting from address: "); Serial.println(TX_ADDRESS);
    Serial.print("to RX_ADDRESS: "); Serial.println(RX_ADDRESS);  Serial.println();
    radioManager.setTimeout(30);
    radioManager.setRetries(0);
    driver.setTxPower(19,true);
  }
  delay(500);
}

void loop() {
    static unsigned long last_display_time = 0;
  // NEW FORMAT:::: data = L1, L2, L3, L4, L5, L6, L7, PIG, L8, BLWER, PVMAST

  // READING VALVE SWITCHES
  // Note: the last two channels are blower and master pressure, but they are handled by the reciever
  for (int i = 0; i < numChan; i++) {
    if (i < numChan - 2) { // Valves
      potReadings[i] = 100 - analogRead(potPINS[i]) * (100.0 / 1023.0);
      pSwitchReadings[i] = !digitalRead(pSwitchPins[i]);
      vSwitchReadings[i] = !digitalRead(vSwitchPins[i]);
      if (pSwitchReadings[i] == 1 || vSwitchReadings[i] == 1) { // only overwrite the pots if switches are active or if its under the deadband
        data[i] = 100 + pSwitchReadings[i] + 2 * vSwitchReadings[i]; // means 100 is idle, 101 is vacuum, 102 is pressure
      }
      else if (potReadings[i] > POT_DEADBAND) // else it should be up to the pots
        data[i] = potReadings[i];
      else // else it should be idle
        data[i] = 104;
    }
    else { //Blower and P/V Master
      pSwitchReadings[i] = !digitalRead(pSwitchPins[i]);
      vSwitchReadings[i] = !digitalRead(vSwitchPins[i]);
      data[i] = 100 + pSwitchReadings[i] + 2 * vSwitchReadings[i];
    }
  }

  //UPDATE DISPLAY
  if ((millis() - last_display_time) > 1000) { // update every tenth of second
    last_display_time = millis(); // reset clock
    LEDdisplay();
  }
  radioTransmitter();
  delay(200);
}

void radioTransmitter() {
  //Send data to RX
  if (radioManager.sendtoWait(data, sizeof(data), RX_ADDRESS)) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    // Now attempt to pickup a reply from the receiver
    if (radioManager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      for (int i = 0; i < numChan-2; i++) {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    else{Serial.println("FAILED RESPONSE");}
    
    Serial.print("TX_"); Serial.print(TX_ADDRESS);
    Serial.print(" sent      ");
    for (int i = 0; i < sizeof(data); i++) {
      Serial.print(data[i]);
      Serial.print(" ");
    }
    Serial.print(" to RX_"); Serial.println(RX_ADDRESS);
    Serial.println();
  }
}

void LEDdisplay() {
  //HIGH-->Red, LOW--> Green, INPUT-->Off
  blinker = !blinker;
  for (int i = 0; i < numChan-2; i++) {
    int state = buf[i];
    if      (state==0) {pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i], HIGH);} //0 is Solid Red
    else if (state==1) {pinMode(LEDpins[i], OUTPUT); digitalWrite(LEDpins[i],  LOW);} //1 is Solid Green
    else if (state==2) {pinMode(LEDpins[i],  INPUT);                                } //2 is Off 
    else{// Blinking lights
      if (state=3){
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
