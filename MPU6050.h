#include <DI2C.h>
#include "defines.h"

typedef union {
    int16_t value;
    struct {
        uint8_t low;
        uint8_t high;
    };
} MPUUnit;
struct MPU6050Data {
    MPUUnit gx;
    MPUUnit gy;
    MPUUnit gz;

    MPUUnit ax;
    MPUUnit ay;
    MPUUnit az;
};

class MPU6050 {

public:
    MPU6050();

    int open();
    int open(const char *name, DI2C::DI2CRegister address);
    int init();
    void readAll();

    void readGyroX();
    void readGyroY();
    void readGyroZ();
    void readGyro();

    void readAccelX();
    void readAccelY();
    void readAccelZ();
    void readAccel();


    const MPU6050Data& getData() const {return data;}
    int16_t* getLineData();

    int16_t GX() const {return data.gx.value;}
    int16_t GY() const {return data.gy.value;}
    int16_t GZ() const {return data.gz.value;}

    int16_t AX() const {return data.ax.value;}
    int16_t AY() const {return data.ay.value;}
    int16_t AZ() const {return data.az.value;}

private:
    void zeroData();
private:
    int16_t *lineData;
    MPU6050Data data;
    DI2C::DI2CHandle handle;
};
