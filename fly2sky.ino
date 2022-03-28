#include "config.h"
#include "simple_mpu9250.h"
#include <Adafruit_BMP085.h>
#include <TinyGPS.h>
#include "drive_control.h"
#include <SPI.h>
#include "RF24.h"
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"

// Address to devices comunicate each other (same in both)
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long lastTransmission;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int SERVO_DEFAULT_POS =  90;

/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerEKF;
uint64_t u64compuTime;
char bufferTxSer[100];

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
MotorControl harpy(steppers,servos,SERVO_DEFAULT_POS,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);

#define IMU_ACC_Z0          (1)
/* Magnetic vector constant (align with local magnetic vector) */
float_prec IMU_MAG_B0_data[3] = {cos(0), sin(0), 0.000000};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
/* The hard-magnet bias */
float_prec HARD_IRON_BIAS_data[3] = {8.832973, 7.243323, 23.95714};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);



/* ============================================ EKF variables/function declaration ============================================ */
/* Just example; in konfig.h: 
 *  SS_X_LEN = 4
 *  SS_Z_LEN = 6
 *  SS_U_LEN = 3
 */
/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.0015/10.)
#define R_INIT_MAG  (0.0015/10.)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,      0,      0,
                                                0,      Q_INIT, 0,      0,
                                                0,      0,      Q_INIT, 0,
                                                0,      0,      0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT_ACC, 0,          0,          0,          0,          0,
                                                0,          R_INIT_ACC, 0,          0,          0,          0,
                                                0,          0,          R_INIT_ACC, 0,          0,          0,
                                                0,          0,          0,          R_INIT_MAG, 0,          0,
                                                0,          0,          0,          0,          R_INIT_MAG, 0,
                                                0,          0,          0,          0,          0,          R_INIT_MAG};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* EKF system declaration ------------------------------------------------------------------------------------------- */
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);


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

void printImuData(){
  Serial.print(imuD.accel.x);
  Serial.print(",");
  Serial.print(imuD.accel.y);
  Serial.print(",");
  Serial.print(imuD.accel.z);
  Serial.print(",");
  Serial.print(imuD.gyro.x);
  Serial.print(",");
  Serial.print(imuD.gyro.y);
  Serial.print(",");
  Serial.print(imuD.gyro.z);
  Serial.print(",");
  Serial.print(imuD.mag.x);
  Serial.print(",");
  Serial.print(imuD.mag.y);
  Serial.print(",");
  Serial.print(imuD.mag.z);
  // Serial.print(",");
  // Serial.print(dData.auxF);
  Serial.println("");
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

void printAHRS() {
  Serial.print(quaternionData[0][0]);
        Serial.print(",");
        Serial.print(quaternionData[1][0]);
        Serial.print(",");
         Serial.print(quaternionData[2][0]);
        Serial.print(",");
         Serial.print(quaternionData[3][0]);
        Serial.print(",");
         Serial.print((float)u64compuTime);
        Serial.print(",");
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
    // initiallizeRadio();
    pinMode(led, OUTPUT);
    digitalWrite(led,HIGH);
    quaternionData.vSetToZero();
    quaternionData[0][0] = 1.0;
    EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
    
    snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "EKF in Pi Pico (%s)\r\n",
                                                 (FPU_PRECISION == PRECISION_SINGLE)?"Float32":"Double64");
}

void loop() {
    // readBaro();
    // FilterData();
    // if (recieveData()){
      
    // } 

    // printImuData();
    filter();
    printAHRS();
}

void filter() {
   if (timerEKF >= SS_DT_MILIS) {
        timerEKF = 0;
        mpu.readSensor();
        /* Input 1:3 = gyroscope */
        U[0][0] = mpu.getGyroX_rads();  U[1][0] = mpu.getGyroY_rads();  U[2][0] = mpu.getGyroZ_rads();
        /* Output 1:3 = accelerometer */
        Y[0][0] = mpu.getAccelX_mss(); Y[1][0] = mpu.getAccelY_mss(); Y[2][0] = mpu.getAccelZ_mss();
        /* Output 4:6 = magnetometer */
        Y[3][0] = mpu.getMagX_uT(); Y[4][0] = mpu.getMagY_uT(); Y[5][0] = mpu.getMagZ_uT();
        
        /* Compensating Hard-Iron Bias for magnetometer */
        Y[3][0] = Y[3][0]-HARD_IRON_BIAS[0][0];
        Y[4][0] = Y[4][0]-HARD_IRON_BIAS[1][0];
        Y[5][0] = Y[5][0]-HARD_IRON_BIAS[2][0];
        
        /* Normalizing the output vector */
        float_prec _normG = sqrt(Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]);
        Y[0][0] = Y[0][0] / _normG;
        Y[1][0] = Y[1][0] / _normG;
        Y[2][0] = Y[2][0] / _normG;
        float_prec _normM = sqrt(Y[3][0] * Y[3][0]) + (Y[4][0] * Y[4][0]) + (Y[5][0] * Y[5][0]);
        Y[3][0] = Y[3][0] / _normM;
        Y[4][0] = Y[4][0] / _normM;
        Y[5][0] = Y[5][0] / _normM;
        /* ------------------ Read the sensor data / simulate the system here ------------------ */
        
        
        /* ============================= Update the Kalman Filter ============================== */
        u64compuTime = micros();
        if (!EKF_IMU.bUpdate(Y, U)) {
            quaternionData.vSetToZero();
            quaternionData[0][0] = 1.0;
            EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        }
        u64compuTime = (micros() - u64compuTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
    }
    
    
            /* =========================== Print to serial (for plotting) ========================== */
        quaternionData = EKF_IMU.GetX();
}

/* Function to interface with the Processing script in the PC */
void serialFloatPrint(float f) {
    byte * b = (byte *) &f;
    for (int i = 0; i < 4; i++) {
        byte b1 = (b[i] >> 4) & 0x0f;
        byte b2 = (b[i] & 0x0f);

        char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
        char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

        Serial.print(c1);
        Serial.print(c2);
    }
}

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;
    
    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    p = U[0][0];
    q = U[1][0];
    r = U[2][0];
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false so we can reset the UKF */
        return false;
    }
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;
    
    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0][0]
             +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1][0]
             +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2][0];

    Y[4][0] = (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0][0]
             +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1][0]
             +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2][0];

    Y[5][0] = (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0][0]
             +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1][0]
             +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2][0];
    
    return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    /* In Main_bUpdateNonlinearX():
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec p, q, r;

    p = U[0][0];
    q = U[1][0];
    r = U[2][0];

    F[0][0] =  1.000;
    F[1][0] =  0.5*p * SS_DT;
    F[2][0] =  0.5*q * SS_DT;
    F[3][0] =  0.5*r * SS_DT;

    F[0][1] = -0.5*p * SS_DT;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * SS_DT;
    F[3][1] =  0.5*q * SS_DT;

    F[0][2] = -0.5*q * SS_DT;
    F[1][2] =  0.5*r * SS_DT;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * SS_DT;

    F[0][3] = -0.5*r * SS_DT;
    F[1][3] = -0.5*q * SS_DT;
    F[2][3] =  0.5*p * SS_DT;
    F[3][3] =  1.000;
    
    return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /* In Main_bUpdateNonlinearY():
     * 
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 -g]            --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    H[0][0] = -2*q2 * IMU_ACC_Z0;
    H[1][0] = +2*q1 * IMU_ACC_Z0;
    H[2][0] = +2*q0 * IMU_ACC_Z0;
    H[3][0] =  2*q0*IMU_MAG_B0[0][0] + 2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];
    H[4][0] = -2*q3*IMU_MAG_B0[0][0] + 2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[5][0] =  2*q2*IMU_MAG_B0[0][0] - 2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];
    
    H[0][1] = +2*q3 * IMU_ACC_Z0;
    H[1][1] = +2*q0 * IMU_ACC_Z0;
    H[2][1] = -2*q1 * IMU_ACC_Z0;
    H[3][1] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[4][1] =  2*q2*IMU_MAG_B0[0][0]-2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];
    H[5][1] =  2*q3*IMU_MAG_B0[0][0]-2*q0*IMU_MAG_B0[1][0] - 2*q1*IMU_MAG_B0[2][0];
    
    H[0][2] = -2*q0 * IMU_ACC_Z0;
    H[1][2] = +2*q3 * IMU_ACC_Z0;
    H[2][2] = -2*q2 * IMU_ACC_Z0;
    H[3][2] = -2*q2*IMU_MAG_B0[0][0]+2*q1*IMU_MAG_B0[1][0] - 2*q0*IMU_MAG_B0[2][0];
    H[4][2] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[5][2] =  2*q0*IMU_MAG_B0[0][0]+2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];
    
    H[0][3] = +2*q1 * IMU_ACC_Z0;
    H[1][3] = +2*q2 * IMU_ACC_Z0;
    H[2][3] = +2*q3 * IMU_ACC_Z0;
    H[3][3] = -2*q3*IMU_MAG_B0[0][0]+2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[4][3] = -2*q0*IMU_MAG_B0[0][0]-2*q3*IMU_MAG_B0[1][0] + 2*q2*IMU_MAG_B0[2][0];
    H[5][3] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    
    return true;
}

void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}