#include <Wire.h>
#include <Servo.h>

// BC4 Pneumatic Shield Controller

/*
 RC-STYLE MULTICHANNEL IMPLEMENTATION

 CONTROL MAPPING FOR 4:
 ORDER    : L1 L2 L3 L4  S  A  C  P  B  T   L BL  MA
 Channel  :  1  2  3  4  5  6  7  8  9 11  12 12  13
 Paddress : 22 24 26 28 30 32 34 36 38 40  48 50  42/43
 Vaddress : 23 25 27 29 31 33 35 37 39 41  51     44
 Psensor  : A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 A10

 NOTES FOR BC4: 
 3/1/2020: three "extra" channels for Heels, Top and Lasso, none of which have feedback properly connected. the setup for Lasso is not sequenction, but the other nearby options were having problems
 4/8/2020: channel 45 died, also this specific BC uses three valves for the pressure and vacuum bus due to the on-board configuration.
*/

#define numChan 13              // Includes all channels (blower and master are final two channels)
#define numValveChan 11         // # of links
#define blowerRelay 50          // pin for blower relay
#define PRESSURE_TOLERANCE .1   // helps slow relay bouncing (generally .1-.5 psi)
#define MAXLINKPRESSURE 5      // max pressure in each link (generally 5)
#define MAXPRESSURE 14.5          // max pressure of the pump (generally 14.5-15)

#define debug true                // boolean to report useful serial debug messages
                                        
//******************************************************CONFIGURATION******************************************************
static int pSensPins[numValveChan]           = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};// Pressure sense pins (A0-A14)
static unsigned int pValvePins[numValveChan] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40,  48};// Pressure valve pins (generally even numbered pins from 22 up)
static unsigned int vValvePins[numValveChan] = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41,  51};//   Vacuum valve pins (generally odd  numbered pins from 23 up)
static unsigned int masterVPins[4]           = {42, 43, 44};//   Master valve pins (arranged {pbus, atmo outlet, atmo inlet})
//*************************************************************************************************************************

// Initialize the state arrays
float currentPressures[  numValveChan]; // stores the current pressure of each link
float targetPressures[   numValveChan]; // stores the assigned target pressure of each link for proportional control
bool  servoOverride[     numValveChan]; // indicates whether proportional control or manual control should be used for each link
float pressureErrors[    numValveChan]; // stores the pressure errors for each link for proportional control

// Initialize communication arrays
uint8_t data[numChan];         // stores most recent data received from the transmitter
uint8_t LEDdata[numValveChan]; // stores what state the LEDs on the controller should be updated to

//Init the Master pressure control
int   mastState = 0; // State of the master pressure
float  mastPres = 0; // Master pressure

void setup() {
  //Start Serial
  Serial.begin(9600);

  //Initiliaze output pins
  for (int i = 0; i < numValveChan; i++) {
    //Initialize each valve pin at low
    pinMode(pValvePins[i], OUTPUT);
    pinMode(vValvePins[i], OUTPUT);
    digitalWrite(vValvePins[i], LOW);
    digitalWrite(pValvePins[i], LOW);

    //Initialize the atmo valves at low
    if(i<2){
      pinMode(masterVPins[i], OUTPUT);
      digitalWrite(masterVPins[i], LOW);
    }
  }
  pinMode(A14, INPUT);

  // Listen to wire 1 for Serial communications
  Wire.begin(1); // init the I2C connection on bus 1
  Wire.onReceive(handleRXCommand); // declares the function to be called when an I2C message is received

  // Init Message
  Serial.println("Border Crosser 5 Ready.");
  Serial.println();
}

void handleRXCommand(int howmany) {
  // Read the command into data and handle each valve consecutively (i=channel#)
  int state = 104; // temporary storage for each received command value
  for (int i = 0; i < howmany; i++) { // scan through each received command
    if (i < numChan){ // dont read more data than the number of channels?
      int state = Wire.read(); // read each command from the I2C message
      data[i] = state; // store each command in data

      // ---------VALVE HANDLING---------
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
      // Valve Control
      else {
        if (state == 102) { // MANUAL PRESSURIZE
          digitalWrite(pValvePins[i], HIGH);
          digitalWrite(vValvePins[i], LOW);
          servoOverride[i] = 1; // indicate valve is under manual control
        }
        else if (state == 101) { // MANUAL VENT
          digitalWrite(pValvePins[i], LOW);
          digitalWrite(vValvePins[i], HIGH);
          servoOverride[i] = 1; // indicate valve is under manual control
        }
        else if (state == 103 || state == 104) { // MANUAL IDLE (or switch error)
          if (state == 103){ Serial.print("ERROR:103, IDLING CH"); Serial.print(i); Serial.println();}
          digitalWrite(pValvePins[i], LOW);
          digitalWrite(vValvePins[i], LOW);
          servoOverride[i] = 1; // indicate valve is under manual control
        }
        else{
          targetPressures[i] = (state/100.0)*MAXLINKPRESSURE; //map proportional control to a percent of the max
          servoOverride[i] = 0; // indicate valve is under proportional control
        }
      }
    }
    else if (i>=numChan) Serial.println("ERR: too many messages received");
  }
}
 
void loop() {
  static unsigned long last_display_time = 0; // to keep track of time since last display update

  //Valve and LED handling
  bool ventFlag = false; // keep track of whether anything is venting, or whether we need to pull from atmo
  bool pressFlag = false; // keep track of whether anything is pressurizing, or whether we can just vent the motor
  for (int i = 0; i<numValveChan; i++) { // scan through each valve channel
    //Handling Proportional Drive
    if (servoOverride[i] == 0) {  // if it's NOT in manual direct drive mode
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
      else if (pressureErrors[i] < 0) { // VENT
        // stop flow in, start flow out
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], HIGH);
        // if the pressure is outside of tolerance, make the LED red
        LEDdata[i] = 0; // LED is red when not at target in proportional control schema
        //ventFlag = true; TEMP FOR INCOMPLETE VAC BUS
      }
    } 
    // Updating the LED states for manual drive (updating the actual valves is done on-receive)
    else{
      //get commanded state
      int state = data[i];
      // Update LED's based on condition
      // if manual idle, turn off light
      if (state == 104) LEDdata[i]=2; 
      // if the link is in manual pressurize, make the LED red until you get to max pressure, then blink green
      else if (state == 102 && currentPressures[i]<MAXLINKPRESSURE){ LEDdata[i]=0; pressFlag = true;}
      else if (state == 102){                                        LEDdata[i]=4; pressFlag = true;}
      // if the link is in manual vent, blink red
      else if (state == 101){                                        LEDdata[i]=3;}//  ventFlag = true;} TEMP FOR INCOMPLETE VAC BUS
    }

    //TEMPORARY CODE FOR INCOMPLETE VACCUUM BUS IMPLEMENTATION: VACUUM BUS IS ONLY ON SPIRAL, ARCH, BANANA, CONE
    if (data[4]==101 || data[5]==101 || data[6]==101 || data[7]==101){
      ventFlag=true;
    }
  }

  //Master Pressure Control
  //   Master valve pins (arranged {pbus-0, atmo outlet-1, atmo inlet-2})
  mastPres = readPress(A14); //get current master pressure
  if  (mastState == 101){
    //Serial.println("MASTER VACUUM ON");
    digitalWrite(masterVPins[0],   LOW);  // close pressure bus
    digitalWrite(masterVPins[1],   HIGH); // open atmo outlet
    if(ventFlag) digitalWrite(masterVPins[2], LOW); // pull from links if anything is venting
    else digitalWrite(masterVPins[2], HIGH); // otherwise pull from atmo
  }
  else if (mastState == 102) {
    //Serial.println("MASTER PRESSURE ON");
    digitalWrite(masterVPins[0], HIGH);  // open pressure bus
    if(ventFlag) digitalWrite(masterVPins[2], LOW); // pull from links if anything is venting
    else digitalWrite(masterVPins[2], HIGH); // otherwise pull from atmo
    if(mastPres > MAXPRESSURE) digitalWrite(masterVPins[1], HIGH); // open pressure to atmo if its too high
    else digitalWrite(masterVPins[1], LOW); // otherwise keep that closed and build pressure
  }
  else{
    //if (mastState == 104) Serial.println("MASTER IDLE");
    if (mastState == 103) Serial.println("ERROR:103, IDLING MASTER");
    //CLOSE EVERYTHING
    digitalWrite(masterVPins[0],   LOW);
    digitalWrite(masterVPins[1],   LOW);
    digitalWrite(masterVPins[2],   LOW);
  }
  
  //DISPLAY UPDATER
  if ((millis() - last_display_time) > 500) { // update every tenth of a second
    last_display_time = millis(); // reset clock
    //Send LED info
    Wire.beginTransmission(2);
    Wire.write(LEDdata, sizeof(LEDdata));
    Wire.endTransmission();
    DEBUGLOG();
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
      else if (abs(pressureErrors[i]) > PRESSURE_TOLERANCE){ // proportional pressure
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
    if (pres > 0)Serial.print(pres); // read pressure
    else Serial.print("0.00");
    Serial.print("|");
  }
  Serial.println();
  Serial.println();
  Serial.println();
}
