// To calibrate BMP280 set CALIBRATION_MODE_BMP280 to 1, heat the sensor with e.g. a cup of tea (for about 90 seconds) and output MINUS_DP_BY_DT_reading preferably via wifi
#define CALIBRATION_MODE_BMP280 0
// kite 1: (0.000022), kite2: (0.000016)
#define MINUS_DP_BY_DT 0.000024
float MINUS_DP_BY_DT_reading = 0;

// To calibrate set CALIBRATION_MODE_MPU6050 to 1, output acc_calibrationx,y,z preferably via wifi, set accel_offset_* to the midpoints between highest and lowest reading.
#define CALIBRATION_MODE_MPU6050 0
float acc_calibrationx = 0;float acc_calibrationy = 0;float acc_calibrationz = 0;
//kite2: (-0.41, -0.04, 1.8)
float accel_offset_x = -0.41;
float accel_offset_y = -0.04;
float accel_offset_z = 1.8;


#define SERVO_P_MAX_ANGLE 45
#define SERVO_P_MIN_ANGLE -45

#define SERVO_N_MAX_ANGLE 45
#define SERVO_N_MIN_ANGLE -45

#define MOTOR_MAX_SPEED 60 // maximum motor speed, usually used for ensuring safety in testing

float y_axis_trim = 0;
float z_axis_trim = 0;
float aux_trim = 0;
float aux_trim2 = 0;

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
