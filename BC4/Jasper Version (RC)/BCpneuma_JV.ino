//#include <MIDI.h>
#include <Wire.h>
#include <Servo.h>

// i2c slave (address 1) receiving input from nano/LoRa radio (address 2)

/*
 
 NEW RC-STYLE MULTICHANNEL IMPLEMENTATION

 CONTROL MAPPING:
 ORDER    : L1 L2 L3 L4  S  A  C  P  B  BL  MA
 Channel  :  1  2  3  4  5  6  7  8  9  10  11
 Paddress : 22 24 26 28 30 32 34 36 38  50  42/45
 Vaddress : 23 25 27 29 31 33 35 37 39 N/A  44/43
 Psensor  : A0 A1 A3    A4 A5 A6    A8

  NOTES FOR BC3: 
  A0 and A2 pressure sensors are dead, swapped L3 to A3 and L1 to A7. 
  No Feedback on pig or L4 as of 3/1/2020.

*/

#define numChan 11
#define blowerRelay           50
#define PRESSURE_TOLERANCE 0.5//psi
#define MAXPRESSURE 14.5
                                        //SEE NOTES FOR A2-A3 SWAP
static int numValveChan = numChan-2;
static int pSensPins[numChan-2]           = {A0, A1, A2, A3, A4, A5, A6, A7, A8};  // A9, A10, A11,  A12,  A13,  A14,  A15};
static unsigned int pValvePins[numChan-2] = {22, 24, 26, 28, 30, 32, 34, 36, 38};  // 40, 42,  44,   46,   48,   50,   52};
static unsigned int vValvePins[numChan-2] = {23, 25, 27, 29, 31, 33, 35, 37, 39};  // 41, 43,  45,   47,   49,   51,   53};
static unsigned int masterPressurePins[2] = {42, 45};
static unsigned int masterVacuumPins[2]   = {44, 43};

int currentPressures[numChan-2] =           {0,  0,  0,  0,  0,  0,  0,  0,  0};
int targetPressures[numChan-2]  =           {1,  1,  1,  1,  1,  1,  1,  1,  1};
bool servoOverride[numChan-2]   =           {0,  0,  0,  0,  0,  0,  0,  0,  0};
int pressureErrors[numChan-2]   =           {0,  0,  0,  0,  0,  0,  0,  0,  0};
uint8_t data[numChan]           =                       {0,0,0,0,0,0,0,0,0,0,0};
uint8_t LEDdata[numChan-2]      =                           {0,0,0,0,0,0,0,0,0};

//DEBUGGING
static bool debug = true;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numValveChan; i++) {
    pinMode(pValvePins[i], OUTPUT);
    pinMode(vValvePins[i], OUTPUT);
    digitalWrite(vValvePins[i], LOW);
    digitalWrite(pValvePins[i], LOW);    
  }
  for (int i = 0; i < 2; i++) {
    pinMode(masterVacuumPins[i], OUTPUT);
    pinMode(masterPressurePins[i], OUTPUT);
    digitalWrite(masterVacuumPins[i], LOW);
    digitalWrite(masterPressurePins[i], LOW);
  }

  //Listen to wire 1 for Serial communications
  Wire.begin(1);
  Wire.onReceive(receiveCommand);

  Serial.println("Border Crosser 5 Ready.");
  Serial.println();

}

void receiveCommand(int howmany) {
  // Read the command into data
  for (int i = 0; i < howmany; i++) {
    data[i] = Wire.read();
  }
  // Handle each channel
  for (int i = 0; i<numChan; i++){ 
    handleValve(i, data[i]); 
  }
}
 
void loop() {
  static unsigned long last_display_time = 0;
  for (int i = 0; i<numValveChan; i++) {
    //Pressure Sensor Interpreting
    currentPressures[i] = analogRead(pSensPins[i]);
    currentPressures[i] = constrain(currentPressures[i], 41, 962); // keeps in range of 0-14.5 PSI (one bar)
    currentPressures[i] = ((((currentPressures[i] / 1023. * 5.0) - 0.2 ) / 4.5 * 1023.) / (MAXPRESSURE * 70.551727)) * 127; //0.2 Volts = 0psi 4.7 Volts = 14.5psi 4.5 Volts = Span of 0-14.5psi.  turned to 7 bit value
    
    //Handling Proportional Drive
    if (servoOverride[i] == 0) {  // if it's NOT in manual direct drive mode
      pressureErrors[i] = (targetPressures[i]) - (currentPressures[i]);
      if (abs(pressureErrors[i]) < PRESSURE_TOLERANCE) { // IDLE
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], LOW);
        LEDdata[i] = 1;
      }
      else if (pressureErrors[i] > 0) { // PRESSURIZE
        digitalWrite(pValvePins[i], HIGH);
        digitalWrite(vValvePins[i], LOW);
        LEDdata[i] = 0;
      }
      else if (pressureErrors[i] < 0) { // VENT
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], HIGH);
        LEDdata[i] = 0;
      }
    } // updating the LEDs for manual drive
    else{
      //get commanded state
      int state = data[i];

      // Update LED's based on condition
      if (state == 104) LEDdata[i]=2; //Manual idle--> light fully off
      else if (state == 102 && currentPressures[i]<MAXPRESSURE-2*PRESSURE_TOLERANCE) LEDdata[i]=0;
      else if (state == 102)                                                               LEDdata[i]=4;
      else if (state == 101)                                                               LEDdata[i]=3;
    }
  }
  
  //DISPLAY UPDATER
  if ((millis() - last_display_time) > 100) { // update every tenth of a second
    last_display_time = millis(); // reset clock
    
    //Send LED info
    Wire.beginTransmission(2);
    Wire.write(LEDdata, sizeof(LEDdata));
    Wire.endTransmission();     
  }
}

void handleValve(byte channel, byte state) {
  // Blower Control
  if (channel == 9){
    if  (state == 102) {digitalWrite(blowerRelay, LOW);}  //Serial.println("BLOWER OFF");}
    if  (state == 101) {digitalWrite(blowerRelay, HIGH);} //Serial.println("BLOWER ON");}
  }
  // Master Pressure Control
  else if (channel == 10){
    if  (state == 101){
      //Serial.println("MASTER VACUUM ON");
      for (int i = 0; i < 2; i ++){
        digitalWrite(masterPressurePins[i], LOW);
        digitalWrite(masterVacuumPins[i],   HIGH);
      }
    }
    else if (state == 102) {
      //Serial.println("MASTER PRESSURE ON");
      for (int i = 0; i < 2; i ++){
        digitalWrite(masterPressurePins[i], HIGH);
        digitalWrite(masterVacuumPins[i],   LOW);
      }      
    }
    else{
      //if (state == 104) Serial.println("MASTER IDLE");
      if (state == 103) Serial.println("ERROR:103, IDLING MASTER");
      for (int i = 0; i < 2; i ++){
        digitalWrite(masterVacuumPins[i],   LOW);          
        digitalWrite(masterPressurePins[i], LOW);
      }   
    }
  }
  // Valve Control
  else {
    if (state == 102) { // MANUAL PRESSURIZE
      digitalWrite(pValvePins[channel], HIGH);
      digitalWrite(vValvePins[channel], LOW);
      servoOverride[channel] = 1;
    }
    else if (state == 101) { // MANUAL VENT
      digitalWrite(pValvePins[channel], LOW);
      digitalWrite(vValvePins[channel], HIGH);
      servoOverride[channel] = 1;
    }
    else if (state == 103 || state == 104) { // MANUAL IDLE (or switch error)
      if (state == 103){ Serial.print("ERROR:103, IDLING CH"); Serial.print(channel); Serial.println();}
      digitalWrite(pValvePins[channel], LOW);
      digitalWrite(vValvePins[channel], LOW);
      servoOverride[channel] = 1;
    }
    else{
      targetPressures[channel] = (state/100.0)*MAXPRESSURE;
      servoOverride[channel] = 0;
    }
  }
}
