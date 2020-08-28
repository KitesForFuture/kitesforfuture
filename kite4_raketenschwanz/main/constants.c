// To calibrate BMP280 set CALIBRATION_MODE_BMP280 to 1, heat the sensor with e.g. a cup of tea (for about 90 seconds) and output MINUS_DP_BY_DT_reading preferably via wifi
#define CALIBRATION_MODE_BMP280 0
// kite 1: (0.000022), kite2: (0.000016), kiteJochen und raketenSchwanz (0.000024)
#define MINUS_DP_BY_DT 0.000024
float MINUS_DP_BY_DT_reading = 0;

// To calibrate set CALIBRATION_MODE_MPU6050 to 1, output acc_calibrationx,y,z preferably via wifi, set accel_offset_* to the midpoints between highest and lowest reading.
#define CALIBRATION_MODE_MPU6050 0
float acc_calibrationx = 0;float acc_calibrationy = 0;float acc_calibrationz = 0;
//kite2: (-0.41, -0.04, 1.8)
//jochens Lieblingsflieger (-0.65, -0.1, 0.25)
//kite4_raketenschwanz: (-0.45, -0.2, -1.7)

float accel_offset_x = -0.45;
float accel_offset_y = -0.2;
float accel_offset_z = -1.7;
/*
float accel_offset_x = -0.45;
float accel_offset_y = -0.2;
float accel_offset_z = -1.7;
*/

#define SERVO_RUDDER_MAX_ANGLE 45
#define SERVO_RUDDER_MIN_ANGLE -45
#define SERVO_ELEVON_MAX_ANGLE 45
#define SERVO_ELEVON_MIN_ANGLE -45
#define MOTOR_MAX_SPEED 55 // maximum motor speed, usually used for ensuring safety in testing
#define MOTOR_SPEED_WHEN_HORIZONTAL 14

float y_axis_trim = 0;
float z_axis_trim = 0;
float aux_trim = 0;
float elev_trim = 0;

// PINS for I2C:
// (JLCPCB v1 board)
// mpu6050
static gpio_num_t i2c_gpio_sda_bus0 = 14;
static gpio_num_t i2c_gpio_scl_bus0 = 25;
// height sensor
static gpio_num_t i2c_gpio_sda_bus1 = 18;
static gpio_num_t i2c_gpio_scl_bus1 = 19;

// SENSOR NAMES:
#define BATTERY_VOLTAGE 3
#define WIND1 2
#define WIND1_TEMP 5
#define WIND2 0
#define WIND3 1
//#define WIND4 4

#define TWR 5
#define liftCoeffTimesArea 0.5 // lift coefficient * wing area * 2.78

// here we can define a(n orthogonal) basis transformation of the coordinate system, reminder T*A*T^{-1} (, T^{-1} = T^{\transp} )
#define rot0 rot[4]
#define rot1 -rot[3]
#define rot2 rot[5]
#define rot3 -rot[1]
#define rot4 rot[0]
#define rot5 -rot[2]
#define rot6 rot[7]
#define rot7 -rot[6]
#define rot8 rot[8]

#define gyrox -mpu_pos.gyro_y
#define gyroy mpu_pos.gyro_x
#define gyroz mpu_pos.gyro_z
#define avgGyrox -mpu_pos_avg.gyro_y
#define avgGyroy mpu_pos_avg.gyro_x
#define avgGyroz mpu_pos_avg.gyro_z

#define TOP_LEFT 2
#define TOP_RIGHT 1
#define BOTTOM_LEFT 0
#define BOTTOM_RIGHT 3
