#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "config.h"


RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
dataTx rData; 

void initiallizeRadio() {
    while(!radio.begin()) {
      Serial.print("Radio did not start: \n");
      delay(100);
    }
    Serial.print("Radio init Sucessfull \n");
    delay(100);
    radio.setChannel(100);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.setAutoAck(false);
    radio.openReadingPipe(1, pipe);
    
}
void printData() {
    Serial.print(rData.imuTx.x,4);
    Serial.print(",");
    Serial.print(rData.imuTx.y,4);
    Serial.print(",");
    Serial.print(rData.imuTx.z,4);
    Serial.print(",");
    Serial.print(rData.altTx,2);
    if (rData.valid) {
    Serial.print(",");
    Serial.print(rData.lat,6);
    Serial.print(",");
    Serial.print(rData.lon,6);
    Serial.print(",");
    Serial.print(rData.velocity,2);
    Serial.print(",");
    Serial.print(rData.sat);
    }
    Serial.println("");
}

void recieveData(){
      // Start to listen
    radio.startListening();
    delay(10);
    while (radio.available()) {        
    radio.read(&rData, sizeof(dataTx));
    printData();
  }
  radio.stopListening();
}

void setup() {
    Serial.begin(115200);
    initiallizeRadio();
}
void loop() {
    recieveData();
}