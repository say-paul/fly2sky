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
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the sameßßßßßß
  radio.setPALevel(RF24_PA_MIN);
  // Setup the channel to work within, number 100
  radio.setChannel(100);
  // Open recept pipe
  radio.openReadingPipe(1, pipe);
  // Start to listen
  radio.startListening();
}

void loop() {
  // Wait until some data
  if (radio.available()) {
    // Read payload, and check if it finished
    radio.read(&info, sizeof(info));
    // Manage info
    if (info) {
      Serial.println("We received positive!");
    } else {
      Serial.println("We received negative!");
    }
  }
  // Wait a bit
}
