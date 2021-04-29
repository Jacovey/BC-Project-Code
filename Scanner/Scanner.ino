#include <RadioHead.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define TX_POWER RH_NRF24::TransmitPowerm18dBm // dont need to change this, but you could
#define DATARATE RH_NRF24::DataRate1Mbps // dont need to change this but you could

// Radio hardware setup
#define NRF24_CS       53 // Chip/Slave Select pin DONT CHANGE
#define NRF24_EN       2  // Enable pin DONT CHANGE

//Radio driver and manager setup
RH_NRF24 driver(NRF24_EN, NRF24_CS);

//Initialize the input buffer and output data
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
int chan = 0;

void setup() {
  // Init the Serial
  Serial.begin(9600);

  // See if radio init correctly
  if (!driver.init()) {
    Serial.println("Radio Init Failed");
    while(1);
  }
  else {
    Serial.println("Radio Scanner Initialized");
    driver.setChannel(chan);
    driver.setRF(DATARATE, TX_POWER);
  }
}

void loop() {
  while (chan < 125){
    static unsigned long last_display_time = 0; // to keep track of time
    
    if (driver.available()){
      Serial.println("got one!");
      uint8_t len = sizeof(buf);
      driver.recv(buf, &len);
      for (int i=0; i<len; i++){
        Serial.print(buf[i]);
      }
    }
    if ((millis() - last_display_time) > 200) { // update every tenth of a second
      last_display_time = millis(); // reset clock
      chan++;
      Serial.println(chan);
      driver.setChannel(chan);
    }
  }
}
