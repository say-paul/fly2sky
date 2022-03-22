#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// PIN CONFIGS
#define GPS_TX 4
#define GPS_RX 5
#define NRF240_CE_PIN 20
#define NRF240_CSN_PIN 17
int I2C_SDA  = 8;
int I2C_SCL = 9;
#define STEPPER1 6
#define STEPPER2 7
#define BLDC1 0
#define BLDC2 1


#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2400 // Maximum pulse length in µs

// address mpu
#define MPU_ADDRESS 0x68
#define RADIO_TIMEOUT 250000 //in microsec converts to 250 millisec

// driveData units will be velocity(m/s) if data recived from onboard flight computer.
// driveData units will be accleration(m/s^2) if data recieved from ground control.
struct driveData {
    short int x;
    short int y;
    short int z;
    short int yr; //for head (r)otation / yaw
    bool auxA;
    bool auxB;
    bool auxC;
    bool auxD;
    bool auxE;
    bool auxF;

};

struct tuning {
    short int joystickSensitivity;
    short int obstructionWarning;
    short int maxCruiseVelocity;
    short int maxAcceleration;
};

struct threeAxis {
  float x;
  float y;
  float z;
};

struct imuData {
  threeAxis gyro;
  threeAxis accel;
  threeAxis mag;
};

struct dataTx {
  threeAxis imuTx;
  float altTx;
  float lat;
  float lon;
  unsigned short sat;
  float velocity;
  bool valid;
};
#endif