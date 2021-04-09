/* CONFIG TOOL FOR BC ARDUINOS*/
#include <SPI.h>

#define blowerRelay           50

void setup() {
  // UNCOMMENT ONE OF THESE
  //int mode = 0;// TX
  int mode = 1; //BC

  //Setup communication with computer
  Serial.begin(9600);
  Serial.println("Startup....");
  //Setup pins based on mode
  if (mode == 0){
    Serial.println("TX Mode: send any serial to pass to next step");Serial.println();

    //All possible pins and switches
    uint8_t potPins[16]             =       {54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69};
    uint8_t vSwitchPins[16]         =       { 5,  7,  9, 10, 12, 14, 16, 18, 20, 22, 25, 26, 29, 31, 35, 33};
    uint8_t pSwitchPins[16]         =       { 4,  6,  8, 11, 13, 15, 17, 19, 21, 23, 24, 27, 28, 30, 34, 32};
    uint8_t LEDpins[14]             =       {48, 49, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36};
    
    //initialize the pinmodes
    for (int i=0;i<16;i++){
      if (i<12) {pinMode(LEDpins[i], OUTPUT);digitalWrite(LEDpins[i], HIGH);}
      pinMode(potPins[i], INPUT);
      pinMode(vSwitchPins[i], INPUT_PULLUP);
      pinMode(pSwitchPins[i], INPUT_PULLUP);
    }

    Serial.println("Pressure Switches Display");
    DispArr(pSwitchPins,16,true);
    Serial.println("Vent Switches Display");
    DispArr(vSwitchPins,16,true);
    Serial.println("Pots Display");
    DispArr(potPins,16,false);

    //scan through LEDs
    Serial.println("LED Display");
    TestChans(LEDpins,14,200);
    
  }
  else if (mode == 1){
    uint8_t vValvePins[16] = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43,   45,  47,  49,  51,  53};
    uint8_t pValvePins[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42,   44,  46,  48,  50,  52};
    uint8_t pSensPins[16] =           {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

    //initialize the pinmodes
    for (int i=0;i<16;i++){
      pinMode(pSensPins[i], INPUT);
      pinMode(vValvePins[i], OUTPUT);
      pinMode(pValvePins[i], OUTPUT);
    }
    
    Serial.println("Feedback sensors");
    DispArr(pSensPins,16,false);
    Serial.println("Moving on to pressure valves");
    TestChans(pValvePins,16,1000);
    Serial.println("Moving on to vent valves");
    TestChans(vValvePins,16,1000);
  }
  Serial.println("All done!");
}

void TestChans(uint8_t dat[], int len, int delaytime){
  for (int i=0;i<len;i++){
    digitalWrite(dat[i], HIGH);
    Serial.print("Now flashing pin ");
    Serial.print(dat[i]);
    while (Serial.available()<1){// Wait for input to move on
      digitalWrite(dat[i], HIGH);
      delay(delaytime);
      digitalWrite(dat[i], LOW);
      delay(delaytime);
    }
    i+=Serial.available()-2;
    delay(30);
    drainSerial();
    digitalWrite(dat[i], LOW);
    Serial.println();
  }
}

void drainSerial(){
  while(Serial.available()>0) Serial.read();
}

void DispArr(uint8_t dat[], int len, bool dig){
    while(Serial.available()<1){// Wait for any message to skip
      //Print out each pin
      for (int i=0; i<len; i++){
        if (dat[i]>=10)Serial.print("| ");
        else Serial.print("|  ");
        Serial.print(dat[i]);
      }
      Serial.print("|");
      Serial.println();
      
      //Print out their current states
      for (int i=0; i<len; i++){
        if (dig){
          if (digitalRead(dat[i])) Serial.print("|ACT");
          else Serial.print("|   ");
        }
        else{
          int val = analogRead(dat[i]);
          val = ((float) val)*0.09775171065;// gets it to be 0-100
          val = (int) val;
          Serial.print("|");
          if (val<100 && val>=10)Serial.print(" ");//Two digit--> one space
          else if (val<10)Serial.print("  ");//one digit--> two spaces
          Serial.print(val);
        }
      }
      Serial.print("|");
      Serial.println();
      
      //Delay for a bit
      delay(500);
    }
    delay(30);
    drainSerial();
}
void loop(){}
