#define Nsta 9 /* pitch,roll,yaw,heading,lat,long,alt */
#define Mobs 13 /* gyro X 3,accel X 3,mag X 3,gps X 3,baro */
 
#include <MPU9250.h>
#include <TinyEKF.h>             /* from https://github.com/simondlevy/TinyEKF */
#include <TinyGPS++.h>           /* from https://github.com/mikalhart/TinyGPSPlus */
#include <Wire.h>
#include <Adafruit_BMP085.h>

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
  int z;
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
            F[3][3] = 1;
            F[4][4] = 1;
            F[5][5] = 1;
            F[6][6] = 1;
            F[7][7] = 1;
            F[8][8] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            hx[0] = this->x[0];
            hx[1] = this->x[1];
            hx[2] = this->x[2];
            hx[3] = this->x[0];
            hx[4] = this->x[1];
            hx[5] = this->x[2];
            hx[6] = this->x[3];
            hx[7] = this->x[4];
            hx[8] = this->x[5];
            hx[9] = this->x[6];
            hx[10] = this->x[7];
            hx[11] = this->x[8];
            hx[12] = this->x[6];
            

            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1;       // Baro temperature from previous state
            H[2][2] = 1;       // LM35 temperature from previous state
            H[3][0] = 1;
            H[4][1] = 1;
            H[5][2] = 1;
            
        }
};

static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

//objects
MPU9250 mpu;
IMU attitude;
Position gpsPos;
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
double baseline,T,baroAlt;
bool success = false;
bool enableGPS = true;
float timer = millis();
String rtc = "";


double ReadBaro() {
  T = bmp.readTemperature();
  return bmp.readAltitude(bmp.readPressure());
}

void setup() {
    Serial.begin(9600);

    Serial2.setRX(5);
    Serial2.setTX(4);
    Serial2.begin(9600);
    Wire.setSCL(1);//gp1 
    Wire.setSDA(0);//gp0
    Wire.begin();
    delay(2000);
   
   while (!success) { 
    if (mpu.setup(0x68)) {
              Serial.println("MPU init Sucess");
              success = true;
      } else {
               success = false;
      }
    if (bmp.begin()) {
      Serial.println("BMP180 init Success");
      delay(2000);
      baseline = ReadBaro();
      success = true;
        } else {
          Serial.println("BMP180 init Failed");
          success = false;
        }
     delay(2000);
    }
}

void ReadImu() {
  if (mpu.update()) {
  attitude.pitch = mpu.getPitch();
  attitude.roll = mpu.getRoll();
  attitude.yaw = mpu.getYaw();
  }
  
}

void ReadGPS() {
 while (Serial2.available() > 0)
     Serial.print("Serial avialable \n");
      if (gps.encode(Serial2.read())) {
         Serial.print("Serial reading \n");
        displayGpsDetails();
          if (gps.location.isValid()) {
            gpsPos.z = gps.altitude.feet();
            gpsPos.y = gps.location.lng();
            gpsPos.x = gps.location.lat();
            
          } else {
            Serial.print("Invalid \n");
          }
      
      if (millis()-timer > 5000 && gps.charsProcessed() < 10)
          {
            Serial.printf("No GPS detected: check wiring. \n");
            timer = millis();
          }
         
      }
  
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

void PrintData () {
Serial.print("MPU-PITCH:");     Serial.print(attitude.pitch,2);      Serial.print("   ");
Serial.print("MPU-ROLL:");     Serial.print(attitude.roll,2);      Serial.print("    ");
Serial.print("MPU-YAW:");     Serial.print(attitude.yaw,2);      Serial.print("   ");
if (enableGPS) {
Serial.print("gps-lat:");     Serial.print(gpsPos.x,6);      Serial.print("   ");
Serial.print("gps-lng:");     Serial.print(gpsPos.y,6);      Serial.print("    ");
Serial.print("gps-alt:");     Serial.print(gpsPos.z);      Serial.print("   ");
}
Serial.print("Baro-Z: ");     Serial.print(baroAlt);      Serial.print(" ft  ");
Serial.print("Time: ");     Serial.print(rtc);      Serial.print("  ");
Serial.print("Temp:");     Serial.print(T);      Serial.print("\n");
}

void displayGpsDetails() {
        Serial.print(gps.location.lat(), 6);
        Serial.print("\t");
        Serial.print(gps.location.lng(), 6);
        Serial.print("\t");
        Serial.print(gps.altitude.feet());
        Serial.print("\t");
        Serial.print(gps.speed.kmph());
        Serial.print("\t");
        Serial.print(gps.course.deg());
        Serial.print("\t");
        Serial.print(gps.hdop.value());
        Serial.print("\t");  
        Serial.println(gps.satellites.value());
        Serial.print("gps-lat:");     Serial.print(gps.location.lat());      Serial.print("    ");
        Serial.print("gps-lng:");     Serial.print(gps.location.lng());      Serial.print("   ");
        Serial.print("GPS-ALT: ");     Serial.print(gps.altitude.feet());      Serial.print(" ft ");

  }



void loop() {
        ReadImu();
        if (enableGPS) {
          while (Serial2.available() > 0)
              if (gps.encode(Serial2.read()))
                displayInfo();
          
            if (millis() > 5000 && gps.charsProcessed() < 10)
            {
              Serial.println(F("No GPS detected: check wiring."));
            }
        }
        baroAlt = ReadBaro() - baseline;
        
        PrintData();
}



void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    gpsPos.x = gps.location.lat();
    gpsPos.y = gps.location.lng();
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {                 
     rtc = gps.time.value();
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
