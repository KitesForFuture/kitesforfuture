// HOW TO CALIBRATE:
// output acc_calibrationx,y,z preferably via wifi, set accel_offset_* in constants.c to the midpoints between highest and lowest reading.

#include "kitemath/helpers.h"
#include <string.h>

struct position_data{
	float accel_x;
	float accel_y;
	float accel_z;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
};

struct position_data mpu_pos;
struct position_data mpu_pos_callibration;



float gyro_precision_factor = 0;	//factor needed to get to deg/sec
float accel_precision_factor = 0;	//factor needed to get to m/s

//sens = 0 <-> +- 250 deg/sec
//sens = 1 <-> +- 500 deg/sec
//sens = 2 <-> +- 1000 deg/sec
//sens = 3 <-> +- 2000 deg/sec
void init_gyro_sensitivity(int sens){
	if(sens < 4 && sens >=0){
		i2c_send(0, 104, 27, 8*sens, 1);
		gyro_precision_factor = 250*smallpow(2,sens)/32768.0;
	}else{
		printf("setGyroSensitivity(int sens), sensitivity must be between 0 and 3");
	}
}

//sens = 0 <-> +- 2g
//sens = 1 <-> +- 4g
//sens = 2 <-> +- 8g
//sens = 3 <-> +- 16g
void init_accel_sensitivity(int sens){
	if(sens < 4 && sens >=0){
		i2c_send(0, 104, 28, 8*sens, 1);
		accel_precision_factor = 2*9.81*smallpow(2,sens)/32768.0;
	}else{
		printf("setAccelSensitivity(int sens), sensitivity must be between 0 and 3");
	}
}

//cut off low frequencies using a Digital Low Pass Filter
void enableDLPF(){
	i2c_send(0, 104, 26, 3, 1);
}

void readMPURawData(){
	uint8_t highByte;
	uint8_t lowByte;
	
	//read acc/gyro data at register 59..., 67...
	//GYRO X
	highByte = i2c_receive(0, 104, 67, 1);
	lowByte = i2c_receive(0, 104, 68, 1);
	mpu_pos.gyro_x = gyro_precision_factor*(int16_t)((highByte << 8) | lowByte);
	//GYRO Y
	highByte = i2c_receive(0, 104, 69, 1);
	lowByte = i2c_receive(0, 104, 70, 1);
	mpu_pos.gyro_y = gyro_precision_factor*(int16_t)((highByte << 8) | lowByte);
	//GYRO Z
	highByte = i2c_receive(0, 104, 71, 1);
	lowByte = i2c_receive(0, 104, 72, 1);
	mpu_pos.gyro_z = gyro_precision_factor*(int16_t)((highByte << 8) | lowByte);
	
	//ACCEL X
	highByte = i2c_receive(0, 104, 59, 1);
	lowByte = i2c_receive(0, 104, 60, 1);
	mpu_pos.accel_x = -accel_precision_factor*(int16_t)((highByte << 8) | lowByte);
	//ACCEL Y
	highByte = i2c_receive(0, 104, 61, 1);
	lowByte = i2c_receive(0, 104, 62, 1);
	mpu_pos.accel_y = accel_precision_factor*(int16_t)((highByte << 8) | lowByte);
	//ACCEL Z
	highByte = i2c_receive(0, 104, 63, 1);
	lowByte = i2c_receive(0, 104, 64, 1);
	mpu_pos.accel_z = accel_precision_factor*(int16_t)((highByte << 8) | lowByte);

}

// ToDo read calibration
calibration_pos.accel_x = readEEPROM(0);
calibration_pos.accel_y = readEEPROM(1);
calibration_pos.accel_z = readEEPROM(2);
calibration_pos.gyro_x = readEEPROM(3);
calibration_pos.gyro_y = readEEPROM(4);
calibration_pos.gyro_z = readEEPROM(5);
MINUS_DP_BY_DT = readEEPROM(6);

void processMPURawData(float time_difference, rot){
		
	// matrix based:
	// rotation-matrix:
	// angles in radians
	// 0.01745329 = pi/180
	float alpha = 0.01745329*(mpu_pos.gyro_x-mpu_pos_avg.gyro_x) * time_difference;
	float beta = 0.01745329*(mpu_pos.gyro_y-mpu_pos_avg.gyro_y) * time_difference;
	float gamma = 0.01745329*(mpu_pos.gyro_z-mpu_pos_avg.gyro_z) * time_difference;
	
	// infinitesimal rotation matrix:
	float diff[9];
	diff[0] = cos(beta)*cos(gamma); //maybe can replace by 1 here
	diff[1] = sin(gamma);
	diff[2] = -sin(beta);
	
	diff[3] = -sin(gamma);
	diff[4] = cos(alpha) * cos(gamma);
	diff[5] = -sin(alpha);
	
	diff[6] = sin(beta);
	diff[7] = sin(alpha);
	diff[8] = cos(alpha)*cos(beta);
	
	float new_rot[9];
	mat_mult(rot, diff, new_rot);
	
	// normalized current acceleration vector
	float acc_x = mpu_pos.accel_x-accel_offset_x;// zero offset to compensate for constant accelerometer inaccuracies.
	float acc_y = mpu_pos.accel_y-accel_offset_y;// to be found in constants.c
	float acc_z = mpu_pos.accel_z-accel_offset_z;
	
	float tmp_vec[3];

	// rotate acc_*_smooth
	mat_transp_mult_vec(diff, acc_x_smooth, acc_y_smooth, acc_z_smooth, tmp_vec);
	acc_x_smooth = tmp_vec[0];
	acc_y_smooth = tmp_vec[1];
	acc_z_smooth = tmp_vec[2];
	
	acc_x_smooth = 0.8* acc_x_smooth + 0.2*acc_x;
	acc_y_smooth = 0.8* acc_y_smooth + 0.2*acc_y;
	acc_z_smooth = 0.8* acc_z_smooth + 0.2*acc_z;
	accel_norm_smooth = sqrt((acc_x_smooth*acc_x_smooth)+(acc_y_smooth*acc_y_smooth)+(acc_z_smooth*acc_z_smooth));
	
	
	// rotate acc_*_very_smooth
	mat_transp_mult_vec(diff, acc_x_very_smooth, acc_y_very_smooth, acc_z_very_smooth, tmp_vec);
	acc_x_very_smooth = tmp_vec[0];
	acc_y_very_smooth = tmp_vec[1];
	acc_z_very_smooth = tmp_vec[2];
	
	acc_x_very_smooth = 0.999* acc_x_very_smooth + 0.001*acc_x;
	acc_y_very_smooth = 0.999* acc_y_very_smooth + 0.001*acc_y;
	acc_z_very_smooth = 0.999* acc_z_very_smooth + 0.001*acc_z;
	//accel_norm_very_smooth = sqrt((acc_x_very_smooth*acc_x_very_smooth)+(acc_y_very_smooth*acc_y_very_smooth)+(acc_z_very_smooth*acc_z_very_smooth));
	
	// ACCELERATION MINUS GRAVITY in KITE COORDINATES
	acc_without_g_x = acc_x - acc_x_very_smooth;
	acc_without_g_y = acc_y - acc_y_very_smooth;
	acc_without_g_z = acc_z - acc_z_very_smooth;
	// ACCELERATION MINUS GRAVITY from KITE to WORLD COORDINATES
	mat_mult_vec(rot, acc_without_g_x, acc_without_g_y, acc_without_g_z, tmp_vec);
	acc_height = tmp_vec[0];
	
	mpu_pos.accel_norm = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
	acc_x /= mpu_pos.accel_norm;
	acc_y /= mpu_pos.accel_norm;
	acc_z /= mpu_pos.accel_norm;
	
	// rotate rotation matrix slightly in the direction where the expected and the currently measured acceleration vectors align.
	rotate_towards_g(new_rot, mpu_pos_avg.accel_x, mpu_pos_avg.accel_y, mpu_pos_avg.accel_z, acc_x, acc_y, acc_z, rot); // result is written to tmp_mat
	
	// how rarely can we normalize without impacting the precision and drift?
	//timeForNormalization ++;
	//if(timeForNormalization > 10){
	normalize_matrix(rot);
	//	timeForNormalization = 0;
	//}
}

void initMPU6050(){
	//wake up MPU6050 from sleep mode
	i2c_send(0, 104, 107, 0, 1);
	
	init_gyro_sensitivity(1);
	init_accel_sensitivity(2);
	enableDLPF();


}
