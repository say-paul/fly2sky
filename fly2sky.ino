#include "config.h"
#include "simple_mpu9250.h"
#include <Adafruit_BMP085.h>
#include <TinyGPS.h>
#include "drive_control.h"
#include <SPI.h>
#include "RF24.h"

// Address to devices comunicate each other (same in both)
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long lastTransmission;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int SERVO_DEFAULT_POS =  90;

short int state;
SimpleMPU9250 mpu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPS gps;
RF24 radio(NRF240_CE_PIN, NRF240_CSN_PIN);
imuData imuD;
dataTx transmitData;
driveData dData;
int steppers[2] = {STEPPER1,STEPPER2};
int servos[2] = {BLDC1,BLDC2};
MotorControl harpy(steppers,servos,SERVO_DEFAULT_POS);


void intiallizeImuBmp() {
bool success = false;
while (!success) {
  if (mpu.begin()< 0) {
    Serial.printf("Error initializing communication with IMU\n");
    } else { 
      Serial.printf("IMU init Success\n");
      success = true; 
    }
  if (bmp.begin()) {
    Serial.printf("BMP180 init Success\n");
    } else {
      Serial.printf("BMP180 init Failed\n");
      success = false;
    }
  }
}

void initiallizeRadio() {
    while(!radio.begin()) {
      Serial.printf("Radio did not start: \n");
      delay(100);
    }
    Serial.printf("Radio init Sucessfull \n");
    delay(100);
    radio.setChannel(100);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.openReadingPipe(1,pipe);
    radio.startListening();
    radio.setRetries(15,15);
}

void readImu() {
  if (mpu.readSensor()) {
    imuD.accel.x = mpu.getAccelX_mss();
    imuD.accel.y = mpu.getAccelY_mss();
    imuD.accel.z = mpu.getAccelZ_mss();
    imuD.gyro.x = mpu.getGyroX_rads();
    imuD.gyro.y = mpu.getGyroY_rads();
    imuD.gyro.z = mpu.getGyroZ_rads();
    imuD.mag.x = mpu.getMagX_uT();
    imuD.mag.y = mpu.getMagY_uT();
    imuD.mag.z = mpu.getMagZ_uT();
  }
}

void readBaro() {
  // transmitData.altTx  = 99.99;
}

bool readGPS() {
  unsigned long age;
  bool newData = false;
  while (Serial2.available())
    {
      char c = Serial2.read();
      //  Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  if (newData) {
  gps.f_get_position(&transmitData.lat, &transmitData.lon, &age);
  // Serial.printf("%.6f,%.6f,%.2f\n",transmitData.gpsTx.lat,transmitData.gpsTx.lon,transmitData.gpsTx.velocity);
    if (transmitData.lat != TinyGPS::GPS_INVALID_F_ANGLE) {
      transmitData.valid = newData; 
      transmitData.altTx = gps.f_altitude();
      transmitData.velocity = gps.f_speed_kmph();
      transmitData.sat = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    } else {
      transmitData.valid = false;
    }
  }
  return transmitData.valid;
}

bool recieveData(){
  radio.writeAckPayload(1,&transmitData,sizeof(dataTx)); 
  if (radio.available()) {
    lastTransmission = micros();    
    radio.read(&dData,sizeof(driveData));
    // printRxData();
    return true;
  }
  if (micros()-lastTransmission >= RADIO_TIMEOUT ) {
    blinkLed(50);
  }
  return false;
}

void blinkLed(unsigned long time) {
     // set the brightness
  digitalWrite(led, !digitalRead(led));
  delay(time);
  digitalWrite(led, !digitalRead(led));
  delay(time);
}

void FilterData() {
  if (!readGPS()){
    transmitData.velocity = 0.0;
    transmitData.sat = 0;
    transmitData.lat = 0.0;
    transmitData.lon = 0.0;
  }
  transmitData.imuTx.x = imuD.accel.x;
  transmitData.imuTx.y = imuD.accel.y;
  transmitData.imuTx.z = imuD.accel.z;

}

void printData() {
    Serial.print(transmitData.imuTx.x,4);
    Serial.print(",");
    Serial.print(transmitData.imuTx.y,4);
    Serial.print(",");
    Serial.print(transmitData.imuTx.z,4);
    Serial.print(",");
    Serial.print(transmitData.altTx,2);
    if (transmitData.valid) {
    Serial.print(",");
    Serial.print(transmitData.lat,6);
    Serial.print(",");
    Serial.print(transmitData.lon,6);
    Serial.print(",");
    Serial.print(transmitData.velocity,2);
    Serial.print(",");
    Serial.print(transmitData.sat);
    }
    Serial.println("");
}

void printRxData(){
  Serial.print(dData.x);
  Serial.print(",");
  Serial.print(dData.y);
  Serial.print(",");
  Serial.print(dData.z);
  Serial.print(",");
  Serial.print(dData.yr);
  Serial.print(",");
  Serial.print(dData.auxA);
  Serial.print(",");
  Serial.print(dData.auxB);
  Serial.print(",");
  Serial.print(dData.auxC);
  Serial.print(",");
  Serial.print(dData.auxD);
  Serial.print(",");
  Serial.print(dData.auxE);
  Serial.print(",");
  Serial.print(dData.auxF);
  Serial.println("");

}

void setup() {
    
    Serial.begin(115200);
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.setClock(400000);
    Serial2.setRX(GPS_RX);
    Serial2.setTX(GPS_TX);
    Serial2.begin(9600);
    intiallizeImuBmp();
    initiallizeRadio();
    pinMode(led, OUTPUT);
    digitalWrite(led,HIGH);
}


void loop() {
    readImu();
    readBaro();
    FilterData();
    // if (recieveData()){
      
    // } 

    // printData();
}