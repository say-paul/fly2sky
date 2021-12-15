// PIN connection with peripherals.


// PIN CONFIGS
#define NRF240_CE_PIN 20
#define NRF240_CSN_PIN 17
#define IMU_ADDRESS 0x68


struct transmissionData {
    short int pitch;
    short int roll;
    short int heading;
    unsigned short int altitude;
    unsigned short int velocity;
    float temperature;
};

// driveData units will be velocity(m/s) if data recived from onboard flight computer.
// driveData units will be accleration(m/s^2) if data recieved from ground control.
struct driveData {
    short int x;
    short int y;
    short int z;
};

struct tuning {
    short int joystickSensitivity;
    short int obstructionWarning;
    short int maxCruiseVelocity;
};