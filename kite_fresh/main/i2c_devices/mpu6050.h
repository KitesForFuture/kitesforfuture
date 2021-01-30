#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include "interchip.h"

struct position_data {
	float accel[3];
  float gyro[3];
};

void initMPU6050(struct i2c_bus bus_arg, struct position_data callibration_data);

void readMPURawData(struct position_data *out);

#endif