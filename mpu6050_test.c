#include <unistd.h>				
#include <fcntl.h>				
#include <sys/ioctl.h>	
#include <iostream>
extern "C"
{
#include <linux/i2c-dev.h>	
//#include <linux/i2c.h>
}

#include <stdio.h>
#include <stdint.h>
#include "defines.h"
#include <stdlib.h>

#include "DTcp.h"

#define MPU_WAI 0x75


void receive(const int& i2c_handle, uint8_t registerAddr, uint8_t& data)
{
    data = i2c_smbus_read_byte_data(i2c_handle, registerAddr);
}
void transmit(int i2c_handle, int registerAddr, uint8_t data)
{
    i2c_smbus_write_byte_data(i2c_handle, registerAddr, data);
}
void default_transmits(const int& i2c_handle)
{
     int h = i2c_handle;

     transmit(h, MPU6050_RA_SMPLRT_DIV, 0x07);
     transmit(h, MPU6050_RA_CONFIG, 0x00); //Disable FSync, 256Hz DLPF
     transmit(h,MPU6050_RA_GYRO_CONFIG, GYROSEN_250); //Disable gyro self tests, scale of 250 degrees/s
     transmit(h,MPU6050_RA_ACCEL_CONFIG, ACCELSEN_2G); //Disable accel self tests, scale of +-2g, no DHPF
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
}
int connect_i2c_mpu6050(const char* device_name)
{
    int i2c_handle = 0;
    i2c_handle = open(device_name, O_RDWR | O_NOCTTY);
    ioctl(i2c_handle, I2C_SLAVE, MPU6050_DEFAULT_ADDRESS);
    i2c_smbus_read_byte_data(i2c_handle, MPU_WAI);
    default_transmits(i2c_handle);
    return i2c_handle;
}
typedef union
{
    int16_t value;
    struct
    {
        uint8_t low;
        uint8_t high;
    };
}mpu_data;

struct MPU6050
{
    mpu_data gx;
    mpu_data gy;
    mpu_data gz;

    mpu_data ax;
    mpu_data ay;
    mpu_data az;
};




int main ()
{

//    DArray<int> v;
//    v.push_back(1);
//    v.push_back(2);
//    v.push_back(3);
//    v.push_back(4);

//    DArray<int> c = v;
//    v.setMode(ShareWatcher);
//    c.setMode(ShareWatcher);

//    v.push_back(123);
//    std::cout << v.size() << ' ' << c.size() << std::endl;

    DTcp c;
    c.make_client(58049, "176.111.73.80");
//    c.make_client(58049, "192.168.1.67");
    c.try_connect();


    int i2c_handle = -1;
    const char device_name[] = "/dev/i2c-0";
    i2c_handle = connect_i2c_mpu6050(device_name);
    MPU6050 data;



    for(;true;)
    {
        receive(i2c_handle, MPU6050_RA_GYRO_XOUT_H, data.gx.high);
        receive(i2c_handle, MPU6050_RA_GYRO_XOUT_L, data.gx.low);

        receive(i2c_handle, MPU6050_RA_GYRO_YOUT_H, data.gy.high);
        receive(i2c_handle, MPU6050_RA_GYRO_YOUT_L, data.gy.low);

        receive(i2c_handle, MPU6050_RA_GYRO_ZOUT_H, data.gz.high);
        receive(i2c_handle, MPU6050_RA_GYRO_ZOUT_L, data.gz.low);


        receive(i2c_handle, MPU6050_RA_ACCEL_XOUT_H, data.ax.high);
        receive(i2c_handle, MPU6050_RA_ACCEL_XOUT_L, data.ax.low);

        receive(i2c_handle, MPU6050_RA_ACCEL_YOUT_H, data.ay.high);
        receive(i2c_handle, MPU6050_RA_ACCEL_YOUT_L, data.ay.low);

        receive(i2c_handle, MPU6050_RA_ACCEL_ZOUT_H, data.az.high);
        receive(i2c_handle, MPU6050_RA_ACCEL_ZOUT_L, data.az.low);


        c.send_it(&data, sizeof(MPU6050));

//        printf("GYRO X: %d\n",data.gx.value);
//        printf("GYRO Y: %d\n",data.gy.value);
//        printf("GYRO Z: %d\n",data.gz.value);


//        printf("ACCEL X: %d\n",data.ax.value);
//        printf("ACCEL Y: %d\n",data.ay.value);
//        printf("ACCEL Z: %d\n",data.az.value);

    }


    printf("OK!\n");
    return 0;
}
