// LoRa transmitter/client, i2c slave

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

/* MEGA External Interrupt PINS: 
 * G0:
 * 2 (interrupt 0),
 * 3 (interrupt 1),
 * 18 (interrupt 5),
 * 19 (interrupt 4),
 * 20 (interrupt 3),
 * 21 (interrupt 2) 
 * 
 * /////////////// SABERTOOTH H-BRIDGE CONTROL: /////////////////

  Control each motor with servo commands
  Use 3-argument init: 
  
  servo.attach(pin, 1000, 2000);

  Compatible with R/C versions of Sabertooth
  Servo library comes with Arduino software
  
  0°  -  90° (1000-1500 μs) reverses
  90° - 180° (1500-2000 μs) goes forward
  90° (1500 μs) stops 
  
  ///////////////////////////////////////////////////////////////

 * MEGA:  FUNC:     UNO:
 * 2      G0        2     PURPLE
 * 52     SCK       13    YELLOW  
 * 50     MISO      12    GREEN 
 * 51     MOSI      11    BLUE
 * 53     CS        10    RED
 * remove 2k2 resistors on pressure controller board for highest port, replace with 4resistor array
 * 
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

#define numOfChans      10

#define TX_ADDRESS      1
#define RX_ADDRESS      2

#define RFM95_CS        53 // Chip/Slave Select pin
#define RFM95_INT       2  // PinChangeInterrupt pin
#define CHANGE_THRESH   1

RH_RF95 driver(RFM95_CS, RFM95_INT);
RHReliableDatagram radioManager(driver, TX_ADDRESS);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t LEDvalveStates[numOfChans];

// ******************* STOCK CONFIGURATION *********************************************************************************
                                                                                            //JOYSTICK  
const uint8_t potCCs[numOfChans+2] =        {0,   1,  2,  3,  4,  5,  6,    7,  8,  9,      14, 15};   //, 10, 11,  12,  13,  14,  15};
const uint8_t potPINS[numOfChans+2] =       {54,  55, 56, 57, 58, 59, 60,   61, 62, 63,     68, 69};  //, 64, 65  66,  67,  68,  69};

const uint8_t VswitchPINS[numOfChans+6] =   {5,   7,  9,  10, 12, 15, 16,   18, 20, 22,     25, 26,  29,  31,     35,  33};
const uint8_t PswitchPINS[numOfChans+6] =   {4,   6,  8,  11, 13, 14, 17,   19, 21, 23,     24, 27,  28,  30,     34,  32};

const uint8_t PswitchNNs[numOfChans+6] =    {9 ,   7,  5,  3,  1, 11, 13,   15, 17, 19,     21, 23,  25,  27,     29,  31};
const uint8_t VswitchNNs[numOfChans+6] =    {10,   8,  6,  4,  2, 12, 14,   16, 18, 20,     22, 24,  26,  28,     30,  32};

const uint8_t LEDpins[12] =         {48,  49, 47, 46, 45, 44, 43,  42, 41, 40,  39,   38};  //, 37, 36};  // only 12 LEDs

// *************************************************************************************************************************
//
//// NJ SCIENCE MUSEUM CONFIGURATION SERVO      2   3   4   5   6   7   8   9   10  11      0   1         12    13    
//const uint8_t potPINS[numOfChans] =         { 56, 57, 58, 59, 60, 61, 62, 63, 64, 65 };              //A12,  A13  };
//const uint8_t PswitchPINS[numOfChans+2] =   { 9,  11, 13, 15, 17, 19, 21, 23, 25, 27,     5,  7  };   //29,   31   };
//const uint8_t VswitchPINS[numOfChans+2] =   { 8,  10, 12, 14, 16, 18, 20, 22, 24, 26,     4,  6  };   //28,   30   };
////  MIDI CC AND NN ASSIGNMENTS                2   3   4   5   6   7   8   9   10  11
//const uint8_t potCCs[numOfChans] =          { 2,  3,  4,  5,  6,  7,  8,  9,  10, 11  };   //12,   13   };
//const uint8_t PswitchNNs[numOfChans+2] =    { 6,  8,  10, 12, 14, 16, 18, 20, 22, 24,     2,  4  };   //26,   28   };
//const uint8_t VswitchNNs[numOfChans+2] =    { 5,  7,  9,  11, 13, 15, 17, 19, 21, 23,     1,  3  };   //25,   27   };
////  LEDs FOR DISPLAY OF FEEDBACK
//const uint8_t LEDpins[numOfChans] =         { 45, 44, 43, 42, 41, 40, 39, 38, 37, 36};// only 12 LEDs

uint16_t potReadings[numOfChans+2] ;
bool PswitchReadings[numOfChans+6] ;
bool VswitchReadings[numOfChans+6] ;
uint16_t lastPotReadings[numOfChans+2] ;
bool lastPswitchReadings[numOfChans+6] ;
bool lastVswitchReadings[numOfChans+6] ;
uint8_t data[3];
bool blinkBuffer[12];     // add a 1 as index to make it's LED blink
bool blinkState = 0;
bool offColorToggle;
void setup() {

  for (int i = 0; i < numOfChans; i++){       // SERVO CHANNELS
    pinMode (potPINS[i],      INPUT);
    pinMode (PswitchPINS[i],  INPUT_PULLUP);
    pinMode (VswitchPINS[i],  INPUT_PULLUP);
    pinMode (LEDpins[i],      OUTPUT);          // HIGH is RED, LOW IS GREEN, INPUT is off
    digitalWrite (LEDpins[i], HIGH);
    delay (100);
    digitalWrite (LEDpins[i], LOW);
    delay (100);
    pinMode (LEDpins[i],      INPUT);
  }


  for (int i = 0; i < 6; i ++){
    pinMode(PswitchPINS[numOfChans+i], INPUT_PULLUP);
    pinMode(VswitchPINS[numOfChans+i], INPUT_PULLUP);
    
  }
//  pinMode (24,               INPUT_PULLUP);
//  pinMode (25,               INPUT_PULLUP);
//  pinMode (26,               INPUT_PULLUP);
//  pinMode (27,               INPUT_PULLUP);
//  pinMode (28,               INPUT_PULLUP);
//  pinMode (29,               INPUT_PULLUP);
//  pinMode (30,               INPUT_PULLUP);
//  pinMode (31,               INPUT_PULLUP);
//  pinMode (32,               INPUT_PULLUP);
//  pinMode (33,               INPUT_PULLUP);
//  pinMode (34,               INPUT_PULLUP);  
//  pinMode (35,               INPUT_PULLUP);  


  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  Serial.println("BCcontrollerTX_5 Ready.");
  Serial.println();   Serial.println();   Serial.println();
  driver.setFrequency(915.0);    
  if (!radioManager.init()) Serial.println("TX init failed");
  else {
    Serial.println("TX radio initialized");
    Serial.print("Transmitting from address: "); Serial.println(TX_ADDRESS);
    Serial.print("to RX_ADDRESS: "); Serial.println(RX_ADDRESS);  Serial.println();
    }

  delay(500);
}

void loop() {
  readPots();
  readPswitches();
  readVswitches();
  LEDdisplay();
  radioTransmitter();
  delay(10);
//  blinker();  
}

//void readPots(){
//  for (int i = 0; i < numOfChans; i++){
//    potReadings[i] = analogRead(potPINS[i])>>4;   // may need to map 10 bit value for resolution
//    if (potReadings[i] != lastPotReadings[i]){
//      preparePotMessage(i);
//    }
//    lastPotReadings[i] = potReadings[i];
//  }
//}

void readPots(){

//    channels 0 - 9 (valves)
  
  for (int i = 0; i < numOfChans; i++){
    potReadings[i] = analogRead(potPINS[i])>>3;   //  May need to map 10 bit value for resolution
//    Serial.print(potReadings[i]); Serial.print(" ");
    if (i%2) potReadings[i] = 127-potReadings[i];       //    *** inverts range of odd numbered pots because wired backward in prototype ****
    if (abs(potReadings[i] - lastPotReadings[i]) > CHANGE_THRESH){
      preparePotMessage(i);
      blinkBuffer[i] = false;      
    }
    lastPotReadings[i] = potReadings[i];
  }
//      Serial.println();

//    channels 10 and 11 (joystick)

  for (int j = 10; j<12; j++){
    potReadings[j] = analogRead(potPINS[j])>>2;   // send 8 bit value, 0-255 
//    Serial.print(potReadings[j]); Serial.print(" ");
    preparePotMessage(j);
  }
//  Serial.println();
}

void readPswitches(){
  for (int i = 0; i < numOfChans+6; i++){
    PswitchReadings[i] = !digitalRead(PswitchPINS[i]);
    if (PswitchReadings[i] != lastPswitchReadings[i]){
      preparePswitchMessage(i);
      blinkBuffer[i] = true;
    }
    lastPswitchReadings[i] = PswitchReadings[i];
  }
}

void readVswitches(){
  for (int i = 0; i < numOfChans+6; i++){
    VswitchReadings[i] = !digitalRead(VswitchPINS[i]);
    if (VswitchReadings[i] != lastVswitchReadings[i]){
      prepareVswitchMessage(i);
      blinkBuffer[i] = true;

    }
    lastVswitchReadings[i] = VswitchReadings[i];
  }
}

void preparePotMessage(uint8_t val){
  data[0] = 176;
  data[1] = potCCs[val];
  data[2] = potReadings[val];
//  Serial.print ("potCC "); Serial.print(data[1]); Serial.print(" "); Serial.println(data[2]);
  radioTransmitter();
}

void preparePswitchMessage(uint8_t val){
  data[0] = 144;
  data[1] = PswitchNNs[val];
  data[2] = PswitchReadings[val];
//  Serial.print ("pSwitchNN ");Serial.print(data[1]); Serial.print(" "); Serial.println(data[2]);  
  radioTransmitter();
}

void prepareVswitchMessage(uint8_t val){
  data[0] = 144;
  data[1] = VswitchNNs[val];
  data[2] = VswitchReadings[val];
//  Serial.print ("vSwitchNN ");Serial.print(data[1]); Serial.print(" "); Serial.println(data[2]);  
  radioTransmitter();
}

void radioTransmitter(){
//  Serial.println("message to be sent");
  if (radioManager.sendtoWait(data, sizeof(data), RX_ADDRESS)){
//    Serial.print("TX_"); Serial.print(TX_ADDRESS);
//    Serial.print(" sent      ");
//    Serial.print(buf[0]);
//    Serial.print(" ");
//    Serial.print(buf[1]);
//    Serial.print(" ");
//    Serial.print(buf[2]);  
//    Serial.print(" to RX_"); Serial.println(RX_ADDRESS);
//    Serial.println();
    // Now wait for a reply from the receiver
    uint8_t len = sizeof(buf);
    uint8_t from;   

// INDEX BUF[] WITH LEDpins[] TO DRIVE RED/GREEN/OFF
//                                HI/LO/IN
  // const uint8_t LEDpins[numOfChans] {47,  46, 45, 44, 43, 42, 41, 40, 39, 38};  //, 37, 36};  // only 12 LEDs
  
    if (radioManager.recvfromAckTimeout(buf, &len, 2000, &from)){
      LEDvalveStates[buf[0]] = buf[1];
//        for(byte g = 0; g < numOfChans; g ++){
//          Serial.print(LEDvalveStates[g]); Serial.print(" ");
//        }
//      Serial.println();
//      Serial.print("RX_");
//      Serial.print(from, DEC);
//      Serial.print(" received  ");
//      Serial.print(buf[0]);
//      Serial.print(" ");
//      Serial.print(buf[1]);
//      Serial.print(" from TX_"); Serial.println(TX_ADDRESS);
    }
    else{
      Serial.println("No reply, is rf95_receiver running?");
    }
  }
  else
    Serial.println("TX sendtoWait fail");  
}

void LEDdisplay(){
  blinkState = !blinkState;
  if (blinkState) offColorToggle = !offColorToggle;
  for (byte i = 0; i < 12; i++){
    byte color;
    if (LEDvalveStates[i] && blinkBuffer[i]) {      // if this one is to blink
      color = LEDvalveStates[i] * blinkState;
    }
    else if (!LEDvalveStates[i] && blinkBuffer[i]) color = (offColorToggle+1) * blinkState;                    // if it's off but still in manual mode
    else color = LEDvalveStates[i];    
//    byte color = LEDvalveStates[i];
    switch (color){
      case 0:
        pinMode(LEDpins[i], INPUT);
        break;
      case 1:
        pinMode(LEDpins[i], OUTPUT);
        digitalWrite(LEDpins[i], HIGH);
        break;
      case 2:
        pinMode(LEDpins[i], OUTPUT);
        digitalWrite(LEDpins[i], LOW);
        break;
//      default:
//        pinMode(LEDpins[i], INPUT);
      
    }
  }
}

//void blinker(){
//  blinkState = !blinkState;
//  for (byte j=0; j < numOfChans; j++){
//    if (LEDvalveStates[j] && blinkBuffer[j]) {     // if this one is to blink
//      color =  LEDvalveStates[j] * blinkstate;
//    }
//    else color = LEDvalveStates[j];
//  }
//}
  /*
   * LEDpins[numOfChannels] holds 12 values which are 0 (off), 1 (vac), 2 (prs)
   * will && with LEDvalveStates[numOfChannels] to determine whethere it should blink
   *  
   * if a switch is hit, it's channel goes into blinker()
   * when a ch's pot value is changed, its led is taken out of blinkBuffer[]
   * 
   * preparePswitchMessage(uint8_t val)
   * prepareVswitchMessage(uint8_t val)
   * 
   * if it is to blink, it is set = to not-itself for 100ms
   * it then returns to its LEDvalveStates[]
   */
//  digitalWrite(pin, HIGH);
//  delayMicroseconds(10000);
//  digitalWrite(pin, LOW);
