#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "config.h"


RF24 radio(9, 10);
const uint64_t pipe = [0xE8E8F0F0E1LL,0xC3C3C3C3C3LL];
dataTx rData; 
driveData dData,iData;


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

void readJoyStick(driveData *data){
  data->x = analogRead(JOYSTICK_1_X);
  data->y = analogRead(JOYSTICK_1_Y);
}

void mapServo(driveData *data) {
  data->x = map(data->x,0,660,SERVO_MIN,SERVO_MAX);
  data->y = map(data->y,0,660,SERVO_MIN,SERVO_MAX);
}

bool withinDeadBand(driveData *data) {
  bool status = true;
  if (abs(data->x - iData.x) <= JOYSTICK_DEADBAND) {
    data->x = 0;
  } else {
    status = false;
  }
  if (abs(data->y - iData.y) <= JOYSTICK_DEADBAND) {
    data->y = 0;
  } else {
    status = false;
  }
  return status;
}

void sendData() {
   readJoyStick(&dData);
   mapServo(&dData);
   if (!withinDeadBand(&dData)) {
   radio.openWritingPipe(pipe[1]);
   radio.write(&dData,sizeof(driveData));
   Serial.print(dData.x);
   Serial.print(",");
   Serial.print(dData.y);
   Serial.println(""); 
   } else {
     Serial.print("within Dead Band");
     Serial.println(""); 
   }
   
}

void setup() {
    Serial.begin(115200);
    initiallizeRadio();
    pinMode(JOYSTICK_1_X,INPUT);
    pinMode(JOYSTICK_1_Y,INPUT);
    readJoyStick(&iData);
    mapServo(&iData);
}
void loop() {
    // recieveData();
    sendData();
}