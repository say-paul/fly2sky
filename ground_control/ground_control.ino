#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "config.h"


RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
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
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.writeAckPayload(1,&dData,sizeof(driveData));
    radio.startListening();
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
    if (radio.available()) {        
    radio.read(&rData, sizeof(dataTx));
    radio.writeAckPayload(1,&dData,sizeof(driveData));
    printData();
  }
}

void readJoyStick(driveData *data){
  data->x = analogRead(JOYSTICK_1_X);
  data->y = analogRead(JOYSTICK_1_Y);
}

void mapServo(driveData *data) {
  data->x = map(data->x,0,660,SERVO_MIN,SERVO_MAX);
  data->y = map(data->y,0,660,SERVO_MIN,SERVO_MAX);
}

void DeadBand(driveData *data) {
  if (abs(data->x - iData.x) <= JOYSTICK_DEADBAND) {
    data->x = 0;
  } 
  if (abs(data->y - iData.y) <= JOYSTICK_DEADBAND) {
    data->y = 0;
  } 
}

void controlData() {
   readJoyStick(&dData);
   mapServo(&dData);
   DeadBand(&dData); 
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
    controlData();
    recieveData();
}