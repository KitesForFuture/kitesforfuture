float lastHeight = 0;
float P_h; // for smoothing
float I_y = 0;

float goalHeight = -5;
float targetHeight = -1;
float rateOfClimb = 0;

float oldGoalHeight = -5;

void setGoalHeight(float goalheight){
	goalHeight = goalheight;
	//targetHeight = getHeight();
}

void setRateOfClimb(float rate){
	rateOfClimb = fabs(rate);
}

void setYAxisTrim(float trim){
	y_axis_trim = trim;
}

void setZAxisTrim(float trim){
	z_axis_trim = trim;
}

void setAUXTrim(float trim){
	aux_trim = trim;
}



void calculatePID(){
	
	// PID CONTROLLER FOR THE HEIGHT
	// letting targetHeight go towards goalHeight at rateOfClimb
	if(fabs(goalHeight-targetHeight) > 0.1){
		float adjustedClimbRate = rateOfClimb;
		if(targetHeight < 3 || goalHeight < 2) adjustedClimbRate = 0.1; // for descending slowly during the last 3 meters
		if(goalHeight-targetHeight > 0) adjustedClimbRate = 2.0;
		targetHeight += time_difference*adjustedClimbRate*sign(goalHeight-targetHeight);
	}
	
	// exponential average smoothing of P_h
	P_h = 0.3*P_h + 0.7*(3.5+(targetHeight - getHeight()));
	
	// in meters per second
	// D_h, deviation from desired rate of climb
	// negative: climbing faster than needed
	// positive: climbing slower than needed
	
	float tmp_variable = 0;
	if(time_difference != 0){
		tmp_variable = (lastHeight - getHeight())/time_difference;
	}
	float D_h = (tmp_variable+sign(goalHeight-getHeight())*rateOfClimb);
	lastHeight = getHeight();
	
	float C_h = 3*D_h + 10*P_h;
	if(C_h > MOTOR_MAX_SPEED - 10) C_h = MOTOR_MAX_SPEED - 10; // VERY IMPORTANT to keep C_z effective
	if(C_h < 0) C_h = 0;
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// PID CONTROLLER FOR KEEPING THE Y-AXIS LEVEL (Z-AXIS CONTROLLER)
	// rot is the rotation matrix of the drone in space. last column (z-axis) and z-values of the rotated x and y axes are accurate to 0.03 degrees, other values drift by 1 degree per minute. rot is an array of dimension 9, rot[6] is 3rd (2nd) row, 1st (0th) column.
	
	// calculate angle around z-axis
	// vector a is where the y-axis should be
	float a[3];
	crossProduct(rot[2], rot[5], rot[8], 1, 0, 0, a);
	
	// this factor ensures that when it is horizontal like a plane, i.e. when the neutral position is undefined, it doesn't control
	float factor = norm3(a);
	float factorSEND = factor;
	normalize(a, a, 3);
	float y[3];
	y[0] = rot[1];
	y[1] = rot[4];
	y[2] = rot[7];
	float angleDifference = acos(scalarProductOfMatrices(a, y, 3));
	// getting a sign on the angle:
	float b[3];
	crossProduct(rot[2], rot[5], rot[8], a[0], a[1], a[2], b);
	tmp_variable = scalarProductOfMatrices(b, y, 3);
	angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0));
	
	float P_z = factor*(200 * (angleDifference + z_axis_trim));
	// D_z is in degree per second
	// D_z STAYS ACTIVE EVEN WHEN KITE IS HORIZONTAL
	float D_z = mpu_pos.gyro_z - mpu_pos_avg.gyro_z;
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// PID CONTROLLER FOR KEEPING THE Z-AXIS LEVEL (Y-AXIS CONTROLLER)
	// a is the desired position of the z-axis (orthogonal to the up-vector and orthogonal to the y-axis)
	crossProduct(rot[1], rot[4], rot[7], -1, 0, 0, a);
	
	// TODO TODO TODO: when props point sideways y-axis needs to be controlled by wind direction!
	// this factor ensures that when the props point sideways, it doesn't control
	factor = norm3(a);
	
	normalize(a, a, 3);
	float z[3];
	z[0] = rot[2];
	z[1] = rot[5];
	z[2] = rot[8];
	angleDifference = acos(scalarProductOfMatrices(a, z, 3));
	// getting a sign on the angle:
	crossProduct(rot[1], rot[4], rot[7], a[0], a[1], a[2], b); // a cross y
	tmp_variable = scalarProductOfMatrices(b, z, 3);
	angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0));// <--- ???
	
	//ONLY ALLOW TILTING WHEN HEIGHT SUFFICIENT
	float save_y_axis_trim = y_axis_trim;
	if(getHeight() < 10){
		if(y_axis_trim < -0.5){
			save_y_axis_trim = -0.5;
		}else if(y_axis_trim > 0.5){
			save_y_axis_trim = 0.5;
		}
	}
	
	float P_y = factor*(200 * (angleDifference + save_y_axis_trim));
	float D_y = mpu_pos.gyro_y-mpu_pos_avg.gyro_y;
	float C_y = -0.03*D_y+0.3*P_y + I_y;
	
	// I_y
	#define MAX_I_Y 30
	if(P_y > 0){
		if(I_y < MAX_I_Y) I_y += 7.0*time_difference;
	}else{
		if(I_y > -MAX_I_Y) I_y -= 7.0*time_difference;
	}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// TRANSITION TO GLIDING FLIGHT
	
	// STOP CONTROLLING THE HEIGHT using the motors when angle to horizon is 0 or negative
	
	angleDifference *= -0.64; // 2/pi
	if(angleDifference > 1.0){
		angleDifference = 1.0; 
	}
	if(angleDifference < 0.0){
		angleDifference = 0.0; 
	}
	// re-using the angleDifference from the Y-AXIS controller.
	// angleDifference = 0 <=> hover position
	// angleDifference = 1 <=> gliding position
	
	// GRADUALLY INCREASE X-AXIS ROLL
	float P_x = -60.0*z_axis_trim - rot[1]*200;
	float D_x = 0.2*(mpu_pos.gyro_x - mpu_pos_avg.gyro_x);
	float C_x = angleDifference*(D_x - 0.3*P_x);
	
	// GRADUALLY INCREASE YAW CONTROL (STEERING, Z-AXIS) BY MOTORS
	// GRADUALLY DECREASE Z-AXIS-CONTROL
	float C_z = -0.10*D_z + (1.0-angleDifference)*0.20*P_z - 5*angleDifference*aux_trim;
	#define MOTOR_SPEED_WHEN_HORIZONTAL 18
	C_h = C_h*(1.0-angleDifference) + angleDifference*MOTOR_SPEED_WHEN_HORIZONTAL;
	if(angleDifference == 1.0){C_h = 0.0;}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// MOTORS
	float motor = C_h;
	//float motorYN = + C_z + C_h;
	
	// LIMIT MOTOR SPEED FOR SAFETY
	if(motor < 0)
		motor = 0;
	if(motor > MOTOR_MAX_SPEED)
		motor = MOTOR_MAX_SPEED;
	
	// TURN MOTOR OFF, IF SECOND RIGHT MOST POTI IS TURNED LEFT
	if((float)receivedSignal[4] > 750){
		motor = 0;
	}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// MIXING PID FOR Z-AXIS AND X-AXIS
	float servoRudder = -3*C_z;
	float servoElevon = 2*C_y;
	
	if(servoRudder < SERVO_P_MIN_ANGLE)
		servoRudder = SERVO_P_MIN_ANGLE;
	if(servoRudder > SERVO_P_MAX_ANGLE)
		servoRudder = SERVO_P_MAX_ANGLE;
	
	if(servoElevon < SERVO_N_MIN_ANGLE)
		servoElevon = SERVO_N_MIN_ANGLE;
	if(servoElevon > SERVO_N_MAX_ANGLE)
		servoElevon = SERVO_N_MAX_ANGLE;
	
	sendData(200*save_y_axis_trim, 200*z_axis_trim, 200*aux_trim, motor / MOTOR_MAX_SPEED, C_h / MOTOR_MAX_SPEED, height_accelerometer, height_bmp280_smoothed, angleDifference, goalHeight, getHeight());
	
	setAngle(1, servoElevon);  //y+
	setSpeed(0, motor);  //y+
	setSpeed(3, motor);  //y-
	setAngle(2, servoRudder);  //y-
}
