#define HOVER_MODE 0
#define GLIDE_MODE 1
#define MOTORPLANE_MODE 2

int flightMode = GLIDE_MODE;

void gotoHoverMode(){
	flightMode = HOVER_MODE;
	targetHeight = getHeight();
}

void gotoGlideMode(){
	flightMode = GLIDE_MODE;
}

void calculatePID(){

	// MOTORS
	/*
	// VERY IMPORTANT to keep C_z effective (necessary only if z-axis is being controlled/stabilized by motor thrust):
	if(C_h > (MOTOR_MAX_SPEED - 10)*(1-angleDifference)) C_h = (MOTOR_MAX_SPEED - 10)*(1-angleDifference) + MOTOR_SPEED_WHEN_HORIZONTAL*angleDifference;
	if(C_h < 0) C_h = 0;
	*/
	
	float motorLeft = flightMode == HOVER_MODE ? hover_height_control() : 0;
	float motorRight = motorLeft;
	
	//TODO: power mode, haha :D - Use with care!!!!!
	/*
	if(fabs(signal2) > 0.3 && flightMode != GLIDE_MODE ){
		motorLeft =  90.;
		motorRight = motorLeft;
	}
	*/
	
	// MIXING PID FOR Z-AXIS AND X-AXIS
	float servoRudder = (flightMode == GLIDE_MODE || flightMode == MOTORPLANE_MODE) ? glide_rudder_control() : hover_rudder_control();
	float servoElevator = (flightMode == GLIDE_MODE || flightMode == MOTORPLANE_MODE) ? glide_elevator_control() : hover_elevator_control();
	
	
	setAngle(TOP_RIGHT, servoElevator);
	//if(getUptime() > 5){
		setSpeed(BOTTOM_LEFT, motorLeft);
		setSpeed(BOTTOM_RIGHT, motorRight);
	//}
	setAngle(TOP_LEFT, servoRudder);
	
	// SENDING DEBUGGING DATA TO GROUND
	sendData(angle_nose_horizon() * 0.64, 0, 0, 0, 0, 0, 0, 0, LinksRechtsOffset, HochRunterOffset);
}
