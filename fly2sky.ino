#include "config.h"
#include "mpu9250.h"
#include <Adafruit_BMP085.h>
// #include "filterKalman.h"
#include <TinyGPS.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
// #include "ekf.h"

short int state;
bfs::Mpu9250 mpu(&Wire, MPU_ADDRESS);
Adafruit_BMP085 bmp;
TinyGPS gps;
RF24 radio(NRF240_CE_PIN, NRF240_CSN_PIN);
imuData imuD;
dataTx transmitData;
driveData dData;
driveData pState;
Servo servoL;
Servo servoR;

// Address to devices comunicate each other (same in both)
const uint64_t pipe = 0xE8E8F0F0E1LL;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by


void intiallizeImuBmp() {
bool success = false;
while (!success) {
        if (!mpu.Begin()) {
          Serial.printf("Error initializing communication with IMU\n");
        } else { success = true; }
        /* Set the sample rate divider */
        if (!mpu.ConfigSrd(19)) {
          Serial.printf("Error configured SRD\n");
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
    radio.setDataRate(RF24_2MBPS);
}

void readImu() {
  if (mpu.Read()) {
    imuD.gyro.x = mpu.gyro_x_radps();
    imuD.gyro.y = mpu.gyro_y_radps();
    imuD.gyro.z = mpu.gyro_z_radps();
    imuD.accel.x = mpu.accel_x_mps2();
    imuD.accel.y = mpu.accel_y_mps2();
    imuD.accel.z = mpu.accel_z_mps2();
    imuD.mag.x = mpu.mag_x_ut();
    imuD.mag.y = mpu.mag_y_ut();
    imuD.mag.z = mpu.mag_z_ut();
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

void sendData() {
    radio.openWritingPipe(pipe);
    radio.write(&transmitData,sizeof(dataTx));
}

void ledControl() {
     // set the brightness
  analogWrite(led, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  delay(30);
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

void resetServo(){
  servoL.write(int(SERVO_DEFAULT_POS));
  servoR.write(int(SERVO_DEFAULT_POS));
}

bool recvData() {
  bool status = false;
  radio.openReadingPipe(0, pipe);
  radio.startListening();

    delay(10);
    while (radio.available()) {        
        radio.read(&dData, sizeof(driveData));
        status = true;
        Serial.printf("transmission recieved . \n");
    }
    radio.stopListening();
  return status;
}

void driveMotor(driveData gc) {
  Serial.printf("trigerring servo.. \n");
  servoL.write(int(SERVO_DEFAULT_POS)-gc.x);
  servoR.write(int(SERVO_DEFAULT_POS)+gc.x);
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

void holdStable() {
  resetServo();
}

void setup() {
    
    Serial.begin(115200);
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.setClock(400000);
    Serial2.setRX(GPS_RX);
    Serial2.setTX(GPS_TX);
    Serial2.begin(9600);
    servoL.attach(SERVO1);
    servoR.attach(SERVO2);
    intiallizeImuBmp();
    initiallizeRadio();
    resetServo();
    pinMode(led, OUTPUT);
    digitalWrite(led,HIGH);
}


void loop() {
    readImu();
    readBaro();
    FilterData();
    if (recvData()){
      driveMotor(dData);
    } else {
      holdStable();
    }
    sendData();
    printData();
}