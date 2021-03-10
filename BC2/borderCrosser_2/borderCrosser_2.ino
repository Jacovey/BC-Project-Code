//#include <MIDI.h>
#include <Wire.h>
//#include <Servo.h>

// i2c slave (address 1) receiving input from nano/LoRa radio (address 2)

/*
  ARDUINO MEGA SHIELD HAS PRESSURE SENSORS 0-15 AND PRESSURE/VACUUM PAIRS 0P, 0V - 15P, 15V, COMPRISING 16 SERVO CHANNELS
  EACH SENSOR IS READ AND ITS APPROPRIATE VALVES OPENED IN ORDER TO ACHIEVE A BLADDER PRESSURE VALUE SENT VIA MIDI CONTINUOUS CONTROLLER MESSAGES.
  THE READ PRESSURE IS REPORTED BACK TO THE MIDI HOST VIA THE SAME CONTINUOUS CONTROLLER NUMBERS AND IS SCALED TO A 7-BIT VALUE WHERE THE MAXIMUM
  SYSTEM PRESSURE SETTING IS THE MIDI VALUE 127.

  NOTES 1-32 CORRESPOND TO P/V VALVES ON THE 16 CHANNELS FOR DIRECT DRIVE MODE (OVERRIDING DEFAULT SERVO MODE DESCRIBED ABOVE)
  WHEN A MIDI NOTE 1-32 IS RECEIVED, SERVO MODE IS TURNED OFF UNTIL ANOTHER MIDI CC MESSAGE IS RECEIVED.  NOTE NUMBERS: 1 = 0P, 2 = 0V ... 31 = 15P, 32 = 15V

  MIDI IMPLEMENTATION CHART.  NN = NOTE NUMBER, CC = CONTINUOUS CONTROLLER

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

  BILL BOWEN 2020
*/

#define numberOfServoChannels 13 // 0-15 for for pressure servo channels, 16 is for winch
#define JOYSTICK_DEADZONE     2
#define JOYSTICK_BUTTON       2

#define LmotorPIN             4
#define RmotorPIN             5
#define linearMotor           9
#define blowerRelay           50  // 14P

//#define DDRIVE_MIN -128 //The minimum value x or y can be.
//#define DDRIVE_MAX 127  //The maximum value x or y can be.
//#define MOTOR_MIN_PWM 0 //The minimum value the motor output can be.
//#define MOTOR_MAX_PWM 180 //The maximum value the motor output can be.

int LeftMotorOutput;  //will hold the calculated output for the left motor
int RightMotorOutput; //will hold the calculated output for the right motor

Servo motorSpeed;  // create servo object to control a servo
Servo motorDirection;  // create servo object to control a servo

//    REDIRECTING VALVE SWITCH NOTES 48 AND 49 TO RUN LINEAR MOTOR IN (0) OR OUT (255) AND STOP (127)

//static unsigned int chanRemap[numberOfServoChannels] =   {11, 8,  4,  1,  0,  12,   7,  2,  6,  10,   3,    5,    9};//      13,   14,   15};
//    MUNICH PUREDATA ORDER                               11  8   4   1   0   12    7   2   6   10    3     5     9       13    14    15
//static int pSensPins[numberOfServoChannels] =           {A11, A8, A4, A1, A0, A12,  A7, A2, A6, A10,  A3,   A5,   A9};//  A13,  A14,  A15};
//static unsigned int pValvePins[numberOfServoChannels] = {44,  38, 30, 24, 22, 46,   36, 26, 34, 42,   28,   32,   40};//   48,   50,   52};
//static unsigned int vValvePins[numberOfServoChannels] = {45,  39, 31, 25, 23, 47,   37, 27, 35, 43,   29,   33,   41};//   49,   51,   53};

//                                                        1   2   3   4   5   6   7   8   9   10   11    12    13         14    15    16
static int pSensPins[numberOfServoChannels] =           {A0, A1, A2, A3, A4, A5, A6, A7, A8,  A9,  A10,  A11,  A12};// ,  A13,  A14,  A15};
static unsigned int pValvePins[numberOfServoChannels] = {22, 24, 26, 28, 30, 32, 34, 36, 38,  40,  42,   44,   46};//,    48,   50,   52};
static unsigned int vValvePins[numberOfServoChannels] = {23, 25, 27, 29, 31, 33, 35, 37, 39,  41,  43,   45,   47};//,    49,   51,   53};

//static unsigned int masterPressurePins[2] =             {48, 50};   // channels 14 and 15 p/v
//static unsigned int masterVacuumPins  [2] =             {51, 49};

static unsigned int winchControlPins[2] =               {6,   7};
                                   // 0   1   2   3   4   5   6   7   8   9    10   11    12    13    14    15
static unsigned int pNoteNums[16] =  {1,  3,  5,  7,  9,  11, 13, 15, 17, 19,  21,  23,   25,   27,   29,   31}; // midi note numbers
                                   // 0   1   2   3   4   5   6   7   8   9    10   11    12    13    14    15
static unsigned int vNoteNums[16] =  {2,  4,  6,  8,  10, 12, 14, 16, 18, 20,  22,  24,   26,   28,   30,   32};  // midi note numbers
                                   // 0   1   2   3   4   5   6   7   8   9    10   11    12    13    14    15
//                                                                                              WNCH  BLWR  MVLV

int currentPressures[numberOfServoChannels] =           {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0};
int lastCurrentPressures[numberOfServoChannels] =       {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0};
int targetPressures[numberOfServoChannels] =            {1,  1,  1,  1,  1,  1,  1,  1,  1,  1};   //1, 1, 1, 1, 1, 1};
byte pValveStates[numberOfServoChannels] =              {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0, 0};
bool lastPvalveStates[numberOfServoChannels] =          {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0, 0};
byte vValveStates[numberOfServoChannels] =              {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0, 0};
bool lastVvalveStates[numberOfServoChannels] =          {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0, 0};
bool servoOverride[numberOfServoChannels] =             {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0, 0};
int pressureErrors[numberOfServoChannels] =             {0,  0,  0,  0,  0,  0,  0,  0,  0,  0};   //0, 0, 0, 0, 0, 0};
float systemMaxPressure = 14.5; // **********************************maximum PSI**************
int threeBytePacket[3];
int motorSteer;
int motorThrottle;


void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numberOfServoChannels; i++) {
    pinMode(pSensPins[i],  INPUT);
    pinMode(pValvePins[i], OUTPUT);
    pinMode(vValvePins[i], OUTPUT);
    digitalWrite(vValvePins[i], LOW);
    digitalWrite(pValvePins[i], LOW);    
  }
  for (int i = 0; i < 2; i++) {
//    pinMode(masterVacuumPins[i], OUTPUT);
//    pinMode(masterPressurePins[i], OUTPUT);
//    digitalWrite(masterVacuumPins[i], LOW);
//    digitalWrite(masterPressurePins[i], LOW);
    
  }
  pinMode(linearMotor, OUTPUT); analogWrite(linearMotor, 127);
  Wire.begin(1);                      // Launch MIDI and listen to channel 1
  Wire.onReceive(receiveCommand);
//  Wire.onRequest(sendLEDfeedback);
  //  Wire.onRequest(requestEvent); //  MIDI.setHandleNoteOn(handleNoteOn);
  //  MIDI.setHandleNoteOff(handleNoteOff);
  //  MIDI.set(handleControlChange);
 
  pinMode(JOYSTICK_BUTTON,INPUT);
  motorSpeed.attach(LmotorPIN, 1000, 2000); // speed
  motorDirection.attach(RmotorPIN, 1000, 2000); // direction

  Serial.println("Border Crosser 1 Ready.");
  Serial.println(); Serial.println();
}

//void receiveCommand(){
//  Serial.println("wire received");
//}

//  threeBytePacket [ type, number, value ]


void receiveCommand(int howmany) {
//  Serial.print("BC received: ");
  for (int i = 0; i < howmany; i++) {
    threeBytePacket[i] = Wire.read();
//    Serial.print(threeBytePacket[i], DEC); Serial.print(" ");
  }
  if (threeBytePacket[0] == 176){
    // differential drive motor speed servo control:
    if(threeBytePacket[1] == 14 || threeBytePacket[1] == 15){   // if 14 or 15, handle as joystick X and Y
      differentialDrive(threeBytePacket[1], threeBytePacket[2]);  // send channel, 8bit value to dD();
//    Serial.print(threeBytePacket[1], DEC); Serial.print(" "); Serial.print(threeBytePacket[2], DEC);
    }
    else handleServoMessage(threeBytePacket[1], threeBytePacket[2]);  // valve control
  }
  else if (threeBytePacket[0] == 144) {
    if (threeBytePacket[2] == 0) handleValveOff(threeBytePacket[1], threeBytePacket[2]);
    else handleValveOn(threeBytePacket[1], threeBytePacket[2]);
    Serial.print(threeBytePacket[1]); Serial.print(" "); Serial.println(threeBytePacket[2]);
  }
//  Serial.println(); Serial.println();
}
 
void loop() {
  //  Serial.println("looping");

  static unsigned long last_display_time = 0;
  //    delay(1000);
  //--manage SERVO channels (should be made a lookup table to save calc time):
  for (int i = 0; i < numberOfServoChannels; i++) {
    currentPressures[i] = analogRead(pSensPins[i]);
    currentPressures[i] = constrain(currentPressures[i], 41, 962); // keeps in range of 0-14.5 PSI (one bar)
    currentPressures[i] = ((((currentPressures[i] / 1023. * 5.0) - 0.2 ) / 4.5 * 1023.) / (systemMaxPressure * 70.551727)) * 127; //0.2 Volts = 0psi 4.7 Volts = 14.5psi 4.5 Volts = Span of 0-14.5psi.  turned to 7 bit value
    if (servoOverride[i] == 0) {  // if it's NOT in manual direct drive mode
      if (targetPressures[i] == 0) {
        digitalWrite(pValvePins[i], LOW);
        digitalWrite(vValvePins[i], HIGH);
        pValveStates[i] = 0;
        vValveStates[i] = 1;
      }
      if (targetPressures[i] > 0) {
        pressureErrors[i] = (targetPressures[i] >> 4) - (currentPressures[i] >> 4); // shifts to a 3-bit 0-7 range for resolution on sixteenths of full scale
        if (pressureErrors[i] == 0) {
          digitalWrite(pValvePins[i], LOW);
          digitalWrite(vValvePins[i], LOW);
//                    Serial.print(i); Serial.println("OFF");
          pValveStates[i] = 0;
          vValveStates[i] = 0;
        }
        else if (pressureErrors[i] > 0) {
          digitalWrite(pValvePins[i], HIGH);
          digitalWrite(vValvePins[i], LOW);
//                    Serial.print(i); Serial.println("PRESSURE");
          pValveStates[i] = 1;
          vValveStates[i] = 0;
        }
        else if (pressureErrors[i] < 0) {
          digitalWrite(pValvePins[i], LOW);
          digitalWrite(vValvePins[i], HIGH);
//                    Serial.print(i); Serial.println("VACUUM");
          pValveStates[i] = 0;
          vValveStates[i] = 1;
        }
      }
    }
    motorSteer = abs(motorSteer) < JOYSTICK_DEADZONE ? 0 : motorSteer;
    motorThrottle = abs(motorThrottle) < JOYSTICK_DEADZONE ? 0 : motorThrottle;
    int b = digitalRead(JOYSTICK_BUTTON); 
  }
  if ((millis() - last_display_time) > 100) {
    last_display_time = millis();
    for (int i = 0; i < numberOfServoChannels; i++) {
      //--send 7bit pressure readings back to computer as base 1 MIDI CC's for display where values 0-127 scale to 1-14psi
      //        if (lastCurrentPressures[i] != currentPressures[i]) {Serial.print(i+1); Serial.print(" "); Serial.println(currentPressures[i]);} //  i+1 because CCs are base 1 -- (i+1, currentPressures[i], 1)
      //        lastCurrentPressures[i] = currentPressures[i];
      if ((pValveStates[i]) != (lastPvalveStates[i])) {
        byte a = 0;
        if (pValveStates[i]) a = 2;   // 2 becomes "pressure", 0 is off and 1 is "vacuum"
        byte twoBytes[2] = {i, a};
        Wire.beginTransmission(2);
        Wire.write(twoBytes, 2);
        Wire.endTransmission();
        
//        Serial.print("Send Pgrn LED ");
//        Serial.print(i);
//        Serial.print(" ");
//        Serial.println(a);
      }
      lastPvalveStates[i] = pValveStates[i];
      if ((vValveStates[i]) != (lastVvalveStates[i])) {
        byte twoBytes[2] = {i, vValveStates[i]};
        Wire.beginTransmission(2);
        Wire.write(twoBytes, 2);
        Wire.endTransmission();   
             
//        Serial.print("Send Vred LED ");
//        Serial.print(i);
//        Serial.print(" ");
//        Serial.println(vValveStates[i]);
      }
      lastVvalveStates[i] = vValveStates[i];
    }
    //            Serial.print("Pressure Error "); Serial.println(pressureErrors[0]);
  }
}

                       // axis
void differentialDrive(int XorYaxis, int joystickPosition){
  if (XorYaxis == 15) motorThrottle = map(joystickPosition, 0, 255, 96, 160);
  else motorSteer = map(joystickPosition, 0, 255, 96, 160);
    motorSpeed.write(motorThrottle);                  // sets the servo position according to the scaled value
    motorDirection.write(motorSteer);                  // sets the servo position according to the scaled value 
  

//  Serial.print("motorSteer: "); Serial.print(motorSteer);
//  Serial.print(" motorThrottle "); Serial.println(motorThrottle);
}

void handleValveOn(byte note, byte velocity) {
  //  note = note-1;            // 1-32 becomes 0-31
    if ((note == vNoteNums[13]) || (note == pNoteNums[13])){
      if (note == pNoteNums[13]) {digitalWrite(winchControlPins[1], LOW); digitalWrite(winchControlPins[0], HIGH); Serial.println("WINCH OUT");}
      if (note == vNoteNums[13]) {digitalWrite(winchControlPins[0], LOW); digitalWrite(winchControlPins[1], HIGH); Serial.println("WINCH IN");}
    }
    
    else if ((note == pNoteNums[14]) || (note == vNoteNums[14])){
      if  (note == vNoteNums[14])                    {digitalWrite(blowerRelay, LOW);  Serial.println("BLOWER OFF");}
      if ((note == pNoteNums[14]) && (velocity > 0)) {digitalWrite(blowerRelay, HIGH); Serial.println("BLOWER ON");}
    }
    
//    else if ((note == pNoteNums[15]) || (note == vNoteNums[15])){
//      if  (note == vNoteNums[15]){
//        Serial.println("MASTER VACUUM   ON");
//        for (int i = 0; i < 2; i ++){
//          digitalWrite(masterPressurePins[i], LOW);
//          digitalWrite(masterVacuumPins[i],   HIGH);
//        }
//      }
//      if ((note == pNoteNums[15]) && (velocity > 0)) {
//        Serial.println("MASTER PRESSURE ON");
//        for (int i = 0; i < 2; i ++){
//          digitalWrite(masterVacuumPins[i],   LOW);          
//          digitalWrite(masterPressurePins[i], HIGH);
//        }      
//      }
//    } 
        
    else if (note <= (numberOfServoChannels * 2)) {                    // note is in range of 1-32
      int servoChannelNumber = ((int)(note / 2) + (note % 2) - 1); // determine which channel note belongs to and put it in base0 by subtracting 1
      servoOverride[servoChannelNumber] = 1;                    // put that channel into direct drive (override the servo)
      if (note % 2 == 0) { // if it's odd, it's a pressure
        if (velocity) {
          digitalWrite(pValvePins[servoChannelNumber], HIGH);
          digitalWrite(vValvePins[servoChannelNumber], LOW);
          pValveStates[servoChannelNumber] = 1;
          vValveStates[servoChannelNumber] = 0;
        }
        else {//if (velocity == 0){
          digitalWrite(pValvePins[servoChannelNumber], LOW);
          digitalWrite(vValvePins[servoChannelNumber], LOW);
          pValveStates[servoChannelNumber] = 0;
          vValveStates[servoChannelNumber] = 0;
        }
      }
      else if (note % 2 == 1) { // it's even and therefore a vacuum
        if (velocity) {
          digitalWrite(pValvePins[servoChannelNumber], LOW);
          digitalWrite(vValvePins[servoChannelNumber], HIGH);
          pValveStates[servoChannelNumber] = 0;
          vValveStates[servoChannelNumber] = 1;
        }
        else {//if (velocity == 0){
          digitalWrite(pValvePins[servoChannelNumber], LOW);
          digitalWrite(vValvePins[servoChannelNumber], LOW);
          pValveStates[servoChannelNumber] = 0;
          vValveStates[servoChannelNumber] = 0;
        }
      }
    }
}

void handleValveOff(byte note, byte velocity) {
    if ((note == vNoteNums[13]) || (note == pNoteNums[13])){    // WINCH STOP
      digitalWrite(winchControlPins[1], LOW);
      digitalWrite(winchControlPins[0], LOW);
    }
//    if ((note == pNoteNums[15]) || (note == vNoteNums[15])){
//      if  (note == vNoteNums[15])                    {
//        Serial.println("MASTER VACUUM   OFF");
//        for (int i = 0; i < 2; i ++){
//          digitalWrite(masterPressurePins[i], LOW);
//          digitalWrite(masterVacuumPins[i],   LOW);
//        }
//      }
//      if (note == pNoteNums[15]) {
//        Serial.println("MASTER PRESSURE OFF");
//        for (int i = 0; i < 2; i ++){
//          digitalWrite(masterVacuumPins[i],   LOW);          
//          digitalWrite(masterPressurePins[i], LOW);
//        }      
//      }
//    }       
    else if (note <= (numberOfServoChannels * 2)) {
      int servoChannelNumber = ((int)(note / 2) + (note % 2) - 1); // put channel into direct drive (override the servo)
      servoOverride[servoChannelNumber] = 1;
      if (note % 2 == 1) { // if it's odd, it's a pressure
        digitalWrite(pValvePins[servoChannelNumber], LOW);
        digitalWrite(vValvePins[servoChannelNumber], LOW);
        pValveStates[servoChannelNumber] = 0;
        vValveStates[servoChannelNumber] = 0;
      }
      else if (note % 2 == 0) { // it's even and therefore a vacuum
        digitalWrite(pValvePins[servoChannelNumber], LOW);
        digitalWrite(vValvePins[servoChannelNumber], LOW);
        pValveStates[servoChannelNumber] = 0;
        vValveStates[servoChannelNumber] = 0;
      }
    }
}

void handleServoMessage(byte number, byte value) {
  //  number = number-1;
  servoOverride[number] = 0;
  if (number < numberOfServoChannels) targetPressures[number] = value;
}
