#include <Wire.h>
#include <Servo.h>

// **TEMPLATE** Pneumatic Shield Controller

/*
 RC-STYLE MULTICHANNEL COMMUNICATION IMPLEMENTATION

 CONTROL MAPPING FOR **TEMPLATE**: (Please try and keep this legible, these are temp values)
 ORDER    :
 Channel  :  1  2  3  4  5  6  7  8  9 10 11 12 13 14 JX JY PU MA
 PVaddres : 
 VVaddres : 
 Psensor  : 

 NOTES FOR **TEMPLATE**: 
 MM/DD/YYYY: NOTE SOMETHING
*/

#define numChan /*CHANGE THIS*/              // Includes all channels (blower and master are final two channels)
#define numValveChan /*CHANGE THIS*/         // # of links
#define blowerRelay /*CHANGE THIS*/          // pin for blower relay
#define PRESSURE_TOLERANCE /*CHANGE THIS*/   // helps slow relay bouncing (generally .1-.5 psi)
#define MAXLINKPRESSURE /*CHANGE THIS*/      // max pressure in each link (generally 5)
#define MAXPRESSURE /*CHANGE THIS*/          // max pressure of the pump (generally 14.5-15)

#define pressureBusSensePin /*CHANGE THIS*/  // pin for the master pressure sensor

#define debug /*CHANGE THIS*/                // boolean to report useful serial debug messages
                                        
//******************************************************CONFIGURATION******************************************************
static int          pSensPins[numValveChan]  = { /*CHANGE THIS*/ }; // Pressure sense pins (A0-A14)
static unsigned int pValvePins[numValveChan] = { /*CHANGE THIS*/ }; // Pressure valve pins (generally even numbered pins from 22 up)
static unsigned int vValvePins[numValveChan] = { /*CHANGE THIS*/ }; // Vacuum valve pins (generally odd  numbered pins from 23 up)
static unsigned int masterVPins[2]           = { /*CHANGE THIS*/ }; // Master valve pins (arranged {atmo inlet, atmo outlet})
//*************************************************************************************************************************

// Initialize the state arrays
float currentPressures[  numValveChan]; // stores the current pressure of each link
float targetPressures[   numValveChan]; // stores the assigned target pressure of each link for proportional control
bool  manualOverride[    numValveChan]; // indicates whether proportional control or manual control should be used for each link
float pressureErrors[    numValveChan]; // stores the pressure errors for each link for proportional control

// Initialize communication arrays
uint8_t data[numChan];      // stores most recent data received from the transmitter
uint8_t LEDdata[numChan-2]; // stores what state the LEDs on the controller should be updated to

//Init the Master pressure control
int   mastState = 0; // State of the master pressure
float  mastPres = 0; // Master pressure

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
      
      //init the array so that it doesnt vent everything at startup
      manualOverride[i]=true;
    }

    //init the array so that it doesnt vent everything at startup
    data[i]=104;
    

  }
  pinMode(pressureBusSensePin, INPUT); // init the master pressure sensor

  // Listen to wire 1 for Serial communications
  Wire.begin(1); // init the I2C connection on bus 1
  Wire.onReceive(handleRXCommand); // declares the function to be called when an I2C message is received

  // Init Message
  Serial.println("Border Crosser Ready.");
  Serial.println();
}

void handleRXCommand() {
  // Read the command into data and handle each valve consecutively (i=channel#)
  for (int i = 0; i < 18; i++) { // scan through each valve command    
    int state = Wire.read(); // read each command from the I2C message
    data[i] = state; // store each command in data

    // ---------VALVE HANDLING---------
    // KEY: 101 is "on"/"pressurize", 102 is "off"/"depressurize", 103 is a switch error, 104 is "idle"
    //      any other value is read as a value from 0-100 inclusive (for analog systems)

    //ADD ANY EXTRA SERVO CHANNELS (LIKE THE RACK ON BC5) LIKE THIS:
    /*
    if (i == numChan-3){
      if  (state == 102) {analogWrite(ANALOGPIN, 0);}
      if  (state == 101) {analogWrite(ANALOGPIN, 255);}
    }*/

    // Joystick Control 
    /*
    if (i == 14){// JOYSTICK X AXIS, BUT IT AINT SETUP YET, SEE OBSCELETE CODE
    }
    if (i == 15){// JOYSTICK Y AXIS, BUT IT AINT SETUP YET, SEE OBSCELETE CODE
    }
    */
    // Blower Control
    if (i == 16){// Blower is always second to last channel
      if  (state == 102) {digitalWrite(blowerRelay, LOW);}//BLOWER OFF
      if  (state == 101) {digitalWrite(blowerRelay, HIGH);}//BLOWER ON
    }
    // Master Pressure Control
    else if (i == 17){// Master is always last channel
      mastState = state;
    }
    // Valve Control
    else {
      if (state == 102) { // MANUAL PRESSURIZE
        digitalWrite(pValvePins[i], HIGH);
        digitalWrite(vValvePins[i], LOW);
        manualOverride[i] = 1; // indicate valve is under manual control
      }
      else if (state == 101) { // MANUAL VENT
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], HIGH);
        manualOverride[i] = 1; // indicate valve is under manual control
      }
      else if (state == 103 || state == 104) { // MANUAL IDLE (or switch error)
        if (state == 103){ Serial.print("ERROR:103, IDLING CH"); Serial.print(i); Serial.println();}
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], LOW);
        manualOverride[i] = 1; // indicate valve is under manual control
      }
      else{
        targetPressures[i] = (state/100.0)*MAXPRESSURE; //map proportional control to a percent of the max
        manualOverride[i] = 0; // indicate valve is under proportional control
      }
    }
  }
}
 
void loop() {
  static unsigned long last_display_time = 0; // to keep track of time since last display update

  //Valve and LED handling
  bool ventFlag = false; // keep track of whether anything is venting, or whether we need to pull from atmo
  bool pressFlag = false; // keep track of whether anything is pressurizing, or whether we can just vent the motor
  for (int i = 0; i<14; i++) { // scan through each valve channel
    //Handling Proportional Drive
    if (manualOverride[i] == 0) {  // if it's NOT in manual direct drive mode
      //Pressure Sensor reading
      currentPressures[i] = readPress(pSensPins[i]);
      
      //Get the errors
      pressureErrors[i] = (targetPressures[i]) - (currentPressures[i]);
      if (abs(pressureErrors[i]) < PRESSURE_TOLERANCE) { // IDLE (if the pressure error is small)
        // stop flow in and out of link
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], LOW);
        // if the pressure is within tolerance, make the LED green
        LEDdata[i] = 1; // LED is green when at target in proportional control schema
      }
      else if (pressureErrors[i] > 0 && mastPres>currentPressures[i]) { // PRESSURIZE (if the master pressure is higher than the link pressure)
        // stop flow out, start flow in
        digitalWrite(pValvePins[i], HIGH);
        digitalWrite(vValvePins[i], LOW);
        // if the pressure is outside of tolerance, make the LED red
        LEDdata[i] = 0; // LED is red when not at target in proportional control schema
        pressFlag = true;
      }
      else if( pressureErrors[i] > 0){// indicate when a link needs to catch up
        LEDdata[i] = 0; // LED is red when not at target in proportional control schema
      }
      else if (pressureErrors[i] < 0) { // VENT
        // stop flow in, start flow out
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], HIGH);
        // if the pressure is outside of tolerance, make the LED red
        LEDdata[i] = 0; // LED is red when not at target in proportional control schema
        ventFlag = true;
      }
    } 
    // Updating the LED states for manual drive for channels that have working LEDs (updating the actual valves is done on-receive, also )
    else if (i < 12){
      //get commanded state
      int state = data[i];
      // Update LED's based on condition
      // if manual idle, turn off light
      if (state == 104) LEDdata[i]=2; 
      // if the link is in manual pressurize, make the LED red until you get to max pressure, then blink green
      else if (state == 102 && currentPressures[i]<MAXLINKPRESSURE){ LEDdata[i]=0; pressFlag = true;}
      else if (state == 102){                                        LEDdata[i]=4; pressFlag = true;}
      // if the link is in manual vent, blink red
      else if (state == 101){                                        LEDdata[i]=3;  ventFlag = true;}
    }
  }

  //Master Pressure Control
  mastPres = readPress(pressureBusSensePin); //get current master pressure
  if  (mastState == 101){
    //Serial.println("MASTER VACUUM ON");
    digitalWrite(masterVPins[1],   HIGH); // open  atmo outlet
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
  if ((millis() - last_display_time) > 100) { // update every tenth of a second
    last_display_time = millis(); // reset clock
    //Send LED info
    Wire.beginTransmission(2);
    Wire.write(LEDdata, sizeof(LEDdata));
    Wire.endTransmission();
  }
}

float readPress(byte pin){
  return 0.01653*analogRead(pin)-.85018;
}

void DEBUGLOG(){
  // Print out a useful informational display about the current state of the BC
  Serial.println("| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15|"); Serial.print("|");
  for (int i=0;i<16;i++){ // Current behavior
    if(i<numValveChan){
      if      (data[i]==102 )                                                        Serial.print("M+ "); // manual pressure
      else if (pressureErrors[i] > 0 && abs(pressureErrors[i]) > PRESSURE_TOLERANCE) Serial.print(" + "); // proportional pressure
      else if (data[i]==101)                                                         Serial.print("M- "); // manual vent
      else if (pressureErrors[i] < 0 && abs(pressureErrors[i]) > PRESSURE_TOLERANCE) Serial.print(" - "); // proportional vent
      else                                                                           Serial.print("   "); // hold
    }
    else if (i<numChan-2){
      if (data[i]<10)               {Serial.print("  ");Serial.print(data[i]);}
      if (data[i]>10 && data[i]<100){Serial.print(" "); Serial.print(data[i]);}
      else                          {                   Serial.print(data[i]);}
    }
    else if (i==numChan-2){
      if (data[i]==101)   Serial.print("ON ");
      else                Serial.print("OFF");
    }
    else if (i==numChan-1){
      if (data[i]==101)      Serial.print("PRE");
      else if (data[i]==104) Serial.print("IDL");
      else                   Serial.print("VAC");
    }
    Serial.print("|");
  }
  Serial.println(); Serial.print("|");
  
  for (int i=0;i<16;i++){ // Current pressure
    Serial.print(readPress(pSensPins[i]); // read pressure
    Serial.print("|");
  }
}
