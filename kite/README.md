# Autonomous Kite Airplane

Code runs for example on ESP32 Devkit C.

## DEFAULT PIN ASSIGNMENTS

- Motors on 12, 13, 26, 27
- Sensors on 32, 33, 34, 35, SP and SN (fixed by hardware?)
- MPU6050 on 14(sda), 25(scl)
- BMP280 on 18(sda), 19(scl)

See analog_sensors.c, motors.c, constants.c

## HARDWARE CONFIGURATION and SENSOR CALIBRATION

See constants.c
