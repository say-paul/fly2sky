#define Nsta 3  
#define Mobs 6

#include <MPU9250.h>
#include <TinyEKF.h>             /* from https://github.com/simondlevy/TinyEKF */
#include <TinyGPS++.h>           /* from https://github.com/mikalhart/TinyGPSPlus */
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>          /*from https://github.com/LowPowerLab/SFE_BMP180 */
#include <Wire.h>

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
  float x;
  float y;
  float z;
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

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

//objects
MPU9250 mpu; // You can also use MPU9250 as is
IMU gyro;
IMU accel;
Heading mag;
Position gpsPos;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin,TXPin);
SFE_BMP180 pressure;
double baseline,T,baroAlt;

void setup() {
    baseline = 0.0;
    Serial.begin(115200);
    Wire.begin();
    ss.begin(GPSBaud);
    delay(2000);
   
   if (!mpu.setup(mpu9250Add)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
//    CalibrateIMU();
}

void ReadImu () {
  gyro.pitch = mpu.getGyroX();
  gyro.roll = mpu.getGyroY();
  gyro.yaw = mpu.getGyroZ();
  accel.pitch = mpu.getAccX();
  accel.roll = mpu.getAccY();
  accel.yaw = mpu.getAccZ();
  mag.x = mpu.getMagX();
  mag.y = mpu.getMagY();
  mag.z = mpu.getMagZ();
  
}

void ReadGPS () {
  gpsPos.z = (double)(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  gpsPos.y = (float)(gps.location.lng(), gps.location.isValid(), 12, 6);
  gpsPos.x = (float)(gps.location.lat(), gps.location.isValid(), 11, 6);
}

void CalibrateIMU () {
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
    Serial.println("Done.");
}

double ReadBaro (double *P) {
  char status;
    status = pressure.startTemperature();
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      { 
        status = pressure.getPressure(*P,T);
        if (status != 0)
        {
          return((double)(pressure.altitude(*P,baseline),2));
        }
      }
    }
    return *P;
}
void startInit () {
  baseline = ReadBaro(&baseline);
}

void loop() {
        ReadImu();
        ReadGPS();
        baroAlt = ReadBaro(&baroAlt) - baseline;
    
Serial.print("Gyro-P");     Serial.print(gyro.pitch);      Serial.print("   ");
Serial.print("Gyro-R:");     Serial.print(gyro.roll);      Serial.print("    ");
Serial.print("Gyro-Y:");     Serial.print(gyro.yaw);      Serial.print("   ");
Serial.print("Accel-P:");     Serial.print(gyro.pitch);      Serial.print("   ");
Serial.print("Accel-R:");     Serial.print(gyro.roll);      Serial.print("    ");
Serial.print("Accel-Y:");     Serial.print(gyro.yaw);      Serial.print("   ");
Serial.print("Mag-X:");     Serial.print(mag.x);      Serial.print("   ");
Serial.print("Mag-Y:");     Serial.print(mag.y);      Serial.print("    ");
Serial.print("Mag-Z:");     Serial.print(mag.z);      Serial.print("   ");
Serial.print("Gps-X:");     Serial.print(gpsPos.x);      Serial.print("   ");
Serial.print("Gps-Y:");     Serial.print(gpsPos.y);      Serial.print("    ");
Serial.print("Gps-Z:");     Serial.print(gpsPos.z);      Serial.print("   ");
Serial.print("Baro-Z:");     Serial.print(baroAlt);      Serial.print("\n");


}
