/*
 * AJ Alves (aj.alves@zerokol.com)
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Instantiate RF24 class with CE and CSN values


//arduino shield value (9,10)
//pico controller (20,17)
RF24 radio(9, 10);
// Address to devices comunicate each other (same in both)
const uint64_t pipe = 0xE8E8F0F0E1LL;
// A variable to hold some info
boolean info = false;

void setup() {
  // Setup serial output
  Serial.begin(115200);
  // Start RF
   while(!radio.begin()) {
      Serial.print("Radio did not start: \n");
    }
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  
  // Setup the channel to work within, number 100
  radio.setChannel(100);
  // Open wite pipe
  radio.openWritingPipe(pipe);
}

void loop() {
  // it changes every interval
  info = !info;

  if (info) {
    Serial.print("Sending positive... ");
  } else {
    Serial.print("Sending negative... ");
  }

  // Send info over RF
  bool success = radio.write(&info, sizeof(boolean));

  if (success) {
    Serial.println("sent!");
  } else {
    Serial.println("fail!");
  }

}
