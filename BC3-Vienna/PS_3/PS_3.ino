#include <Wire.h>
#include <Servo.h>
#include <stdio.h>

// BC3 Pneumatic Shield Controller

/*
 RC-STYLE MULTICHANNEL IMPLEMENTATION

 CONTROL MAPPING FOR BC3:
 ORDER    : L1 L2 L3 L4  S  A  C  P BA AB  BR  BL   MA
 Channel  :  1  2  3  4  5  6  7  8  9 10  11  12   13
 Paddress : 22 24 26 38 30 32 34 36 28 40  46  50   44
 Vaddress : 23 25 27 39 31 33 35 37 29 41  47       43
 Psensor  : A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 A10      A15

 NOTES FOR BC3: 
 3/1/2021: No Feedback on pig or L4, A2 pressure sensors is dead
 4/26/2021: Replaced shield with less broken one, noted updated
 5/13/2021: Convert to full manual mode (slightly different than template)
*/

#define numChan 13                // Includes all channels (blower and master are final two channels)
#define numValveChan 11           // # of links
#define blowerRelay 10            // pin for blower relay
#define PRESSURE_TOLERANCE .5     // helps slow relay bouncing (generally .1-.5 psi)
#define MAXLINKPRESSURE 8         // max pressure in each link (generally 5)
#define MAXPRESSURE 14          // max pressure of the pump (generally 14.5-15)

#define pressureBusSensePin A15  // pin for the master pressure sensor

#define debug false                // boolean to report useful serial debug messages
                                        
//******************************************************CONFIGURATION******************************************************
static int pSensPins[numValveChan]           = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};// Pressure sense pins (A0-A14)
static unsigned int pValvePins[numValveChan] = {22, 24, 26, 38, 30, 32, 34, 36, 28, 40,  46};// Pressure valve pins (generally even numbered pins from 22 up)
static unsigned int vValvePins[numValveChan] = {23, 25, 27, 39, 31, 33, 35, 37, 29, 41,  47};//   Vacuum valve pins (generally odd  numbered pins from 23 up)
static unsigned int masterVPins[2]           = {43,42};//   Master valve pins (arranged {atmo inlet, atmo outlet})
//*************************************************************************************************************************

// Initialize the state arrays
float currentPressures[  numValveChan]; // stores the current pressure of each link
float targetPressures[   numValveChan]; // stores the assigned target pressure of each link for proportional control
float pressureErrors[    numValveChan]; // stores the pressure errors for each link for proportional control
bool  pbuffbool[         numValveChan]; // stores the pressure buffer bools

// Initialize communication arrays
uint8_t data[numChan];      // stores most recent data received from the transmitter
uint8_t LEDdata[numValveChan]; // stores what state the LEDs on the controller should be updated to

//Init the Master pressure control
int   mastState = 0; // State of the master pressure
float  mastPres = 0; // Master pressure

//Init the runtime variables (cuts down on new variable allocation)
bool ventFlag = false; // keep track of whether anything is venting, or whether we need to pull from atmo
bool pressFlag = false; // keep track of whether anything is pressurizing, or whether we can just vent the motor
int state = 103; //scanner variable that is used a lot during runtime

void setup() {
  //Start Serial
  Serial.begin(9600);

  //Initiliaze
  for (int i = 0; i < numChan; i++) {
    //Initialize the atmo valves at low
    if(i<2){
      pinMode(masterVPins[i], OUTPUT);
      digitalWrite(masterVPins[i], LOW);
    }
    if (i<numValveChan){
     //Initialize each valve pin at low
      pinMode(pValvePins[i], OUTPUT);
      pinMode(vValvePins[i], OUTPUT);
      digitalWrite(vValvePins[i], LOW);
      digitalWrite(pValvePins[i], LOW);
    }
    data[i]=104; // init the array so that it doesnt vent everything at startup
  }
  pinMode(pressureBusSensePin, INPUT); // init the master pressure sensor
  pinMode(blowerRelay, OUTPUT); //init blower relay (not always neccesary, but for standard setup it is)
  
  // Listen to wire 1 for Serial communications
  Wire.begin(1); // init the I2C connection on bus 1
  Wire.onReceive(handleRXCommand); // declares the function to be called when an I2C message is received

  // Init Message
  Serial.println("Border Crosser Ready.");
  Serial.println();
}

void handleRXCommand(int howmany) {
  // Read the command into data and handle each channel consecutively (i=channel#)
  state = 104; // temporary storage for each received command value
  for (int i = 0; i < howmany; i++) { // scan through each received command
    if (i < numChan){ // dont read more data than the number of channels?
      state = Wire.read(); // read each command from the I2C message
      data[i] = state; // store each command in data, used during runtime operations

      // ---------SPECIAL HANDLING---------
      // KEY: 101 is "on"/"pressurize", 102 is "off"/"depressurize", 103 is a switch error, 104 is "idle"
      //      any other value is read as a value from 0-100 inclusive (for analog systems)
      
      // Blower Control
      if (i == numChan-2){// Blower is always second to last channel
        if  (state == 102) {digitalWrite(blowerRelay, LOW);}//BLOWER OFF
        if  (state == 101) {digitalWrite(blowerRelay, HIGH);}//BLOWER ON
      }
      // Master Pressure Control
      else if (i == numChan-1){// Master is always last channel
        mastState = state;
      }
    }
    else if (i>=numChan) Serial.println("ERR: too many messages received");
  }
}
 
void loop() {
  static unsigned long last_display_time = 0; // to keep track of time since last display update

  // Global States resetting
  ventFlag = false;
  pressFlag = false;

  // Pressure Readings
  for (int i = 0; i<numValveChan; i++) { // scan through each valve channel
    if (data[i]<=100){ // checks if this link is under proportional control (all manual control signals are > 100)
      targetPressures[i] = (data[i]/100.0)*MAXLINKPRESSURE; //map proportional control to a percent of the max
      currentPressures[i] = readPress(pSensPins[i]); //Pressure Sensor reading
      pressureErrors[i] = (targetPressures[i]) - (currentPressures[i]); // get the errors
    }
  }
  
  // Valve Handling
  for (int i = 0; i<numValveChan; i++) { // scan through each valve channel
    state = data[i];

    //Handling Manual Drive
    if (state == 102) { // MANUAL PRESSURIZE
      digitalWrite(pValvePins[i], HIGH);//open inlet
      digitalWrite(vValvePins[i], LOW);//close outlet
      if (currentPressures[i]<MAXLINKPRESSURE){ LEDdata[i]=0; pressFlag = true;}// if below max, solid red
      else                                    { LEDdata[i]=3; pressFlag = true;}// if passed max pressure, blink red as a warning
    }
    else if (state == 101) { // MANUAL VENT
      digitalWrite(pValvePins[i], LOW);
      digitalWrite(vValvePins[i], HIGH);
      LEDdata[i]=4;// manual vent blinks green
    }
    else if (state == 103) { // SWITCH ERROR
      Serial.print("ERROR:103, IDLING CH"); Serial.print(i); Serial.println();
      digitalWrite(pValvePins[i], LOW);
      digitalWrite(vValvePins[i], LOW);
      LEDdata[i]=2; // if manual idle, turn off light
    }
    else if ( state == 104 ){ // MANUAL IDLE
      digitalWrite(pValvePins[i], LOW);
      digitalWrite(vValvePins[i], LOW);
      LEDdata[i]=2; // if manual idle, turn off light
    }
    // Handling Proportional Drive
    else if (pressureErrors[i] > 0 || pbuffbool[i]) { // PRESSURIZE up to the desired pressure
      if (mastPres>currentPressures[i]){// only actually pressurize if the master pressure would be able to help
        if(pressureErrors[i] < 0 ){// idle if passed the desired pressure
          pbuffbool[i]=false;// stop pressurizing past desired pressure!
          digitalWrite(pValvePins[i], LOW);
          digitalWrite(vValvePins[i], LOW);
          LEDdata[i] = 1;
        }
        else if (pbuffbool[i] || pressureErrors[i] > PRESSURE_TOLERANCE){// activate pressure if out of tolerance
          digitalWrite(pValvePins[i], HIGH);
          digitalWrite(vValvePins[i], LOW);
          pressFlag = true;
          if (pressureErrors[i] > PRESSURE_TOLERANCE){
            pbuffbool[i]=true; // start pressurizing to the desired pressure!
            LEDdata[i] = 3; // if the pressure is outside of tolerance, make the LED blink red as a warning
          }
          else{
            LEDdata[i] = 0; // while in tolerance but not at value yet, make LED red
          }
        }
      }
      else{ // if master cant help, idle
        if(pressureErrors[i] > 0 ) LEDdata[i]=0; //communicate link WANTS to pressurize if it could
        else if (pbuffbool[i] || pressureErrors[i] > PRESSURE_TOLERANCE) LEDdata[i] = 3;//communicate link REALLY WANTS to pressurize if it could
        else {LEDdata[i]=1;} // green if its satisfied TEMP TEMP
        digitalWrite(pValvePins[i], LOW);// idle
        digitalWrite(vValvePins[i], LOW);
      }
    }
    else if (pressureErrors[i] < PRESSURE_TOLERANCE*-1) { // VENT
      // stop flow in, start flow out
      digitalWrite(pValvePins[i], LOW);
      digitalWrite(vValvePins[i], HIGH);
      // if the pressure is outside of tolerance, make the LED red
      LEDdata[i] = 0; // LED is red when not at target in proportional control schema
      pbuffbool[i]=false;
    }
    else { // IDLE
      // stop flow in and out of link
      digitalWrite(pValvePins[i], LOW);
      digitalWrite(vValvePins[i], LOW);
      // if the pressure is within tolerance, make the LED green
      LEDdata[i] = 1; // LED is green when at target in proportional control schema
    }
  }

  //TEMPORARY CODE FOR INCOMPLETE VACCUUM BUS IMPLEMENTATION: VACUUM BUS IS ONLY ON SPIRAL, ARCH, BANANA, CONE
  if (data[4]==101 || data[5]==101 || data[6]==101 || data[8]==101){
    ventFlag=true;
  }

  //Master Pressure Control
  mastPres = readPress(pressureBusSensePin); //get current master pressure
  if  (mastState == 101){
    //Serial.println("MASTER VACUUM ON");
    digitalWrite(masterVPins[1], HIGH); // open  atmo outlet
    if(ventFlag) digitalWrite(masterVPins[0], LOW); // pull from links if anything is venting
    else digitalWrite(masterVPins[0], HIGH); // otherwise pull from atmo
  }
  else if (mastState == 102) {
    //Serial.println("MASTER PRESSURE ON");
    if(ventFlag) digitalWrite(masterVPins[0], LOW); // pull from links if anything is venting
    else digitalWrite(masterVPins[0], HIGH); // otherwise pull from atmo
    if(mastPres > MAXPRESSURE) digitalWrite(masterVPins[1], HIGH); // open pressure to atmo if its too high
    else digitalWrite(masterVPins[1], LOW); // otherwise keep that closed and build pressure
  }
  else{
    //if (mastState == 104) Serial.println("MASTER IDLE");
    if (mastState == 103) Serial.println("ERROR:103, IDLING MASTER");
    //CLOSE EVERYTHING
    digitalWrite(masterVPins[0],   LOW);
    digitalWrite(masterVPins[1],   LOW);
  }
  
  //DISPLAY UPDATER
  if ((millis() - last_display_time) > 500) { // update every tenth of a second
    last_display_time = millis(); // reset clock
    //Send LED info
    Wire.beginTransmission(2);
    Wire.write(LEDdata, sizeof(LEDdata));
    Wire.endTransmission();
    if (debug) DEBUGLOG();
  }
}

float readPress(byte pin){
  return 0.01653*analogRead(pin)-.85018;
}

void DEBUGLOG(){
  // Print out a useful informational display about the current state of the BC
  Serial.println("|  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 |"); Serial.print("|");
  for (int i=0;i<16;i++){ // Current behavior
    if(i<numValveChan){
      if      (data[i]==102 )                                                        Serial.print(" M+ "); // manual pressure
      else if (data[i]==101)                                                         Serial.print(" M- "); // manual vent
      else if (abs(pressureErrors[i]) > 0){ // proportional pressure
        float pres = pressureErrors[i];
        if (pres<0) Serial.print(pres,1);
        else Serial.print(pres,2);
      }
      else                                                                           Serial.print("    "); // hold
      Serial.print("|");
    }
    else if (i<numChan-2){
      if (data[i]<10)                Serial.print("  ");
      if (data[i]>10 && data[i]<100) Serial.print( " ");
      Serial.print(data[i]);
      Serial.print("|");
    }
    else if (i==numChan-2){
      if (data[i]==101)   Serial.print(" ON ");
      else                Serial.print(" OFF");
      Serial.print("|");
    }
    else if (i==numChan-1){
      if (data[i]==102)      Serial.print(" PRE");
      else if (data[i]==101) Serial.print(" VAC");
      else                   Serial.print(" IDL");
      Serial.print("|");
    }
  }
  Serial.println(); Serial.print("|");
  
  for (int i=0;i<16;i++){ // Current pressure
    float pres = readPress(pSensPins[i]);
    pres = round(pres * 10);
    pres = pres/10;
    if (pres > 0)Serial.print(pres,2); // read pressure
    else Serial.print("0.00");
    Serial.print("|");
  }
  Serial.println();
  Serial.println();
  Serial.println();
}
