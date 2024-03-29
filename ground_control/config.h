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
#define SERVO1 6
#define SERVO2 7
#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3

#define JOYSTICK_1_Y A0
#define JOYSTICK_1_X A1
#define JOYSTICK_2_Y A3
#define JOYSTICK_2_X A2


#define JOYSTICK_DEADBAND 2

#define ESC_MIN 1000
#define ESC_MAX 2000


#define SERVO_MIN -30
#define SERVO_MAX 30


#define A 2
#define B 3
#define C 4
#define D 5
#define E 6
#define F 7

// address mpu
#define MPU_ADDRESS 0x68


struct gpsData {
    bool valid;
    
};

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