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


void setAUX2Trim(float trim){
	aux_trim2 = trim;
}



void calculatePID(){
	
	// MIXING PID FOR Z-AXIS AND X-AXIS
	float servoYP = 70*y_axis_trim;
	float servoYN = 70*z_axis_trim;
	/*
	if(servoYP < SERVO_P_MIN_ANGLE)
		servoYP = SERVO_P_MIN_ANGLE;
	if(servoYP > SERVO_P_MAX_ANGLE)
		servoYP = SERVO_P_MAX_ANGLE;
	
	if(servoYN < SERVO_N_MIN_ANGLE)
		servoYN = SERVO_N_MIN_ANGLE;
	if(servoYN > SERVO_N_MAX_ANGLE)
		servoYN = SERVO_N_MAX_ANGLE;
		
		*/
		
		
	float servoYP2 = 70*aux_trim + 70*aux_trim2;
	float servoYN2 = 70*aux_trim - 70*aux_trim2;
	/*
	if(servoYP2 < SERVO_P_MIN_ANGLE)
		servoYP2 = SERVO_P_MIN_ANGLE;
	if(servoYP2 > SERVO_P_MAX_ANGLE)
		servoYP2 = SERVO_P_MAX_ANGLE;
	
	if(servoYN2 < SERVO_N_MIN_ANGLE)
		servoYN2 = SERVO_N_MIN_ANGLE;
	if(servoYN2 > SERVO_N_MAX_ANGLE)
		servoYN2 = SERVO_N_MAX_ANGLE;
	*/
	
	sendData(70*y_axis_trim, 70*z_axis_trim, 70*aux_trim, 70*aux_trim2, servoYP, servoYN, servoYP2, servoYN2, 0, 0);
	
	setAngle(0, servoYP);  //y+
	setAngle(1, servoYP2);  //y+
	setAngle(2, servoYN2);  //y-
	setAngle(3, servoYN);  //y-
}
