#include "MPU6050.h"
#include <string.h>
#include <dmem.h>
using namespace DI2C;

MPU6050::MPU6050()
{
    lineData = get_zmem<int16_t>(6);
}
int MPU6050::open() {
    handle = openI2CDevice("/dev/i2c-0", MPU6050_DEFAULT_ADDRESS);
    return handle;
}
int MPU6050::open(const char *name, DI2C::DI2CRegister address) {
    handle = openI2CDevice(name, address);
    return handle;
}
int MPU6050::init() {
    DI2CHandle h = handle;
    if(h < 0)
        return -1;

    /*
     MPU SENS SETTINGS:

     Set to MPU6050_RA_ACCEL_CONFIG:
 ACCELSEN_2G	0b00000000 rate: 16384
 ACCELSEN_4G	0b00001000 rate: 8192
 ACCELSEN_8G	0b00010000 rate: 4096
 ACCELSEN_16G	0b00011000 rate: 2048

    Set to MPU6050_RA_GYRO_CONFIG:
 GYROSEN_250	0b00000000 rate: 131
 GYROSEN_500	0b00001000 rate: 65.5
 GYROSEN_1000	0b00010000 rate: 32.8
 GYROSEN_2000	0b00011000 rate: 16.4
    */

    i2c_smbus_read_byte_data(h, MPU_WAI);
#define transmit DI2CWrite

    transmit(h, MPU6050_RA_SMPLRT_DIV, 0x07);
    transmit(h, MPU6050_RA_CONFIG, 0x00); //Disable FSync, 256Hz DLPF
    transmit(h,MPU6050_RA_GYRO_CONFIG, GYROSEN_2000); //Disable gyro self tests, scale of 250 degrees/s
    transmit(h,MPU6050_RA_ACCEL_CONFIG, ACCELSEN_16G); //Disable accel self tests, scale of +-2g, no DHPF
    transmit(h,MPU6050_RA_FF_THR, 0x00); //Freefall threshold of |0mg|
    transmit(h,MPU6050_RA_FF_DUR, 0x00); //Freefall duration limit of 0
    transmit(h,MPU6050_RA_MOT_THR, 0x00); //Motion threshold of 0mg
    transmit(h,MPU6050_RA_MOT_DUR, 0x00); //Motion duration of 0s
    transmit(h,MPU6050_RA_ZRMOT_THR, 0x00); //Zero motion threshold
    transmit(h,MPU6050_RA_ZRMOT_DUR, 0x00); //Zero motion duration threshold
    transmit(h,MPU6050_RA_FIFO_EN, 0x00); //Disable sensor output to FIFO buffer

   //AUX I2C setup
   //Sets AUX I2C to single master control, plus other config
    transmit(h, MPU6050_RA_I2C_MST_CTRL, 0x00);
   //Setup AUX I2C slaves
    transmit(h,MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV0_REG, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV1_REG, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV2_REG, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV3_REG, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV4_REG, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV4_DO, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV4_DI, 0x00);

   //MPU6050_RA_I2C_MST_STATUS //Read-only

   transmit(h,MPU6050_RA_INT_PIN_CFG, 0x00); //Setup INT pin and AUX I2C pass through
   transmit(h,MPU6050_RA_INT_ENABLE, 0x00);//Enable data ready interrupt

   //Slave out, dont care
    transmit(h,MPU6050_RA_I2C_SLV0_DO, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV1_DO, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV2_DO, 0x00);
    transmit(h,MPU6050_RA_I2C_SLV3_DO, 0x00);

    transmit(h,MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);  //More slave config
    transmit(h,MPU6050_RA_SIGNAL_PATH_RESET, 0x00); //Reset sensor signal paths
    transmit(h,MPU6050_RA_MOT_DETECT_CTRL, 0x00); //Motion detection control
    transmit(h,MPU6050_RA_USER_CTRL, 0x00); //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    transmit(h,MPU6050_RA_PWR_MGMT_1, 0b00000010); //Sets clock source to gyro reference w/ PLL

   //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    transmit(h,MPU6050_RA_PWR_MGMT_2, 0x00);
    transmit(h,MPU6050_RA_FIFO_R_W, 0x00);

    return 0;
}
void MPU6050::readAll() {
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_H, data.gx.high);
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_L, data.gx.low);

    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_H, data.gy.high);
    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_L, data.gy.low);

    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_H, data.gz.high);
    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_L, data.gz.low);


    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_H, data.ax.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_L, data.ax.low);

    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_H, data.ay.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_L, data.ay.low);

    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_H, data.az.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_L, data.az.low);
}
void MPU6050::readGyroX() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_H, data.gx.high);
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_L, data.gx.low);
}
void MPU6050::readGyroY() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_H, data.gy.high);
    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_L, data.gy.low);
}
void MPU6050::readGyroZ() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_H, data.gz.high);
    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_L, data.gz.low);
}
void MPU6050::readGyro() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_H, data.gx.high);
    DI2CRead(handle, MPU6050_RA_GYRO_XOUT_L, data.gx.low);

    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_H, data.gy.high);
    DI2CRead(handle, MPU6050_RA_GYRO_YOUT_L, data.gy.low);

    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_H, data.gz.high);
    DI2CRead(handle, MPU6050_RA_GYRO_ZOUT_L, data.gz.low);
}
void MPU6050::readAccelX() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_H, data.ax.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_L, data.ax.low);
}
void MPU6050::readAccelY() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_H, data.ay.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_L, data.ay.low);
}
void MPU6050::readAccelZ() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_H, data.az.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_L, data.az.low);
}
void MPU6050::readAccel() {

    zeroData();
    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_H, data.ax.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_XOUT_L, data.ax.low);

    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_H, data.ay.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_YOUT_L, data.ay.low);

    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_H, data.az.high);
    DI2CRead(handle, MPU6050_RA_ACCEL_ZOUT_L, data.az.low);
}
int16_t *MPU6050::getLineData() {


//    copy_mem(lineData, reinterpret_cast<int16_t*>(&data), sizeof(MPU6050Data));
    lineData[0] = data.ax.value;
    lineData[1] = data.ay.value;
    lineData[2] = data.az.value;

    lineData[3] = data.gx.value;
    lineData[4] = data.gy.value;
    lineData[5] = data.gz.value;
    return lineData;
}
void MPU6050::zeroData() {
    memset(&data, 0, sizeof(data));
}
