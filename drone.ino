#define Nsta 3
#define Mobs 6

#include <MPU9250.h>
#include <TinyEKF.h>

//Sensors
#define mpu9250Add 0x68



struct IMU {
  float pitch;
  float yaw;
  float roll;
};

struct Heading {
  double x;
  double y;
  double z;
};

struct Position {
  double x;
  double y;
  double z;
};

struct INS {
  struct IMU imu;
  struct Heading heading;
  struct Postion;
};

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);
            this->setQ(2, 2, .0001);

            // Same for measurement noise
            this->setR(0, 0, .0001);
            this->setR(1, 1, .0001);
            this->setR(2, 2, .0001);
            this->setR(3, 3, .0001);
            this->setR(4, 4, .0001);
            this->setR(5, 5, .0001);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0];
            fx[1] = this->x[1];
            fx[2] = this->x[2];

            // So process model Jacobian is identity matrix
            F[0][0] = 1;
            F[1][1] = 1;
            F[2][2] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            hx[0] = this->x[0]; // Barometric pressure from previous state
            hx[1] = this->x[1]; // Baro temperature from previous state
            hx[2] = this->x[2]; // LM35 temperature from previous state
            hx[3] = this->x[0];
            hx[4] = this->x[1];
            hx[5] = this->x[2];

            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1;       // Baro temperature from previous state
            H[2][2] = 1;       // LM35 temperature from previous state
            H[3][0] = 1;
            H[4][1] = 1;
            H[5][2] = 1;
            
        }
};

//objects
MPU9250 mpu; // You can also use MPU9250 as is
IMU imuData;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
    mpu.setup(mpu9250Add);  // change to your own addressßßßß
}

void ReadImuData () {
  imuData.pitch = mpu.getPitch();
  imuData.roll = mpu.getRoll();
  imuData.yaw = mpu.getYaw();
}


void loop() {
    if (mpu.update()) {
        ReadImuData();
    }
Serial.print("IMU-PITCH: ");     Serial.print(imuData.pitch);      Serial.print("   ");
Serial.print("IMU-ROLL:  ");     Serial.print(imuData.roll);      Serial.print("    ");
Serial.print("IMU-YAW:  ");     Serial.print(imuData.yaw);      Serial.print("\n");

}
