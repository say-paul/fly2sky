#include "imu.hpp"
#include <MPU9250.h>
#ifndef config.hpp
#include "config.hpp"
#endif


MPU9250 mpu;

void setup() {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_333HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    if (!mpu.setup(MPU_ADDRESS, setting)) {  // change to your own address
        while (1) {
            printf("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

}


void readData() {

}