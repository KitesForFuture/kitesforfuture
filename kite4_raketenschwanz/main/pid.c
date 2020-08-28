#define HOVER_MODE 0
#define GLIDE_MODE 1
#define MOTORPLANE_MODE 2

int flightMode = GLIDE_MODE;


float lastHeight = 0;
float P_h; // for smoothing
float smooth_C_h = 0;
float I_y = 0;
float I_z = 0;

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

void setElevatorTrim(float trim){
	elev_trim = trim;
}

void setAUXTrim(float trim){
	aux_trim = trim;
}

float limitMotorForSafety(float motor){
	
	// TURN MOTOR OFF, IF SECOND RIGHT MOST POTI IS TURNED LEFT
	if(flightMode == GLIDE_MODE){
		return 0;
	}
	
	// LIMIT MOTOR SPEED FOR SAFETY
	if(motor < 0 ){
		return 0;
	}
	if(motor > MOTOR_MAX_SPEED){
		return MOTOR_MAX_SPEED;
	}
	return motor;
}

float limitRudderAngle(float rudder){
	
	if(rudder < SERVO_RUDDER_MIN_ANGLE)
		return SERVO_RUDDER_MIN_ANGLE;
	if(rudder > SERVO_RUDDER_MAX_ANGLE)
		return SERVO_RUDDER_MAX_ANGLE;
	return rudder;
}

float limitElevonAngle(float elevon){
	
	if(elevon < SERVO_ELEVON_MIN_ANGLE)
		return SERVO_ELEVON_MIN_ANGLE;
	if(elevon > SERVO_ELEVON_MAX_ANGLE)
		return SERVO_ELEVON_MAX_ANGLE;
	return elevon;
}


void calculatePID(){
	
	// PID CONTROLLER FOR THE HEIGHT
	// letting targetHeight go towards goalHeight at rateOfClimb
	if(fabs(goalHeight-targetHeight) > 0.1){
		float adjustedClimbRate = rateOfClimb;
		if(targetHeight < 2) adjustedClimbRate = 0.2; // for descending slowly during the last 3 meters
		if(goalHeight-targetHeight > 0) adjustedClimbRate = 2.0;
		targetHeight += time_difference*adjustedClimbRate*sign(goalHeight-targetHeight);
	}
	
	// exponential average smoothing of P_h
	//TODO: bound 3.5+(targetHeight - getHeight()) from below.
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
	
	float C_h = 0.7*(3*D_h + 10*P_h);
	
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// PID CONTROLLER FOR KEEPING THE Y-AXIS LEVEL (Z-AXIS CONTROLLER)
	// rot is the rotation matrix of the drone in space. last column (z-axis) and 
	// z-values of the rotated x and y axes are accurate to 0.03 degrees, other values drift by 1 degree per minute. rot is an array of dimension 9, rot[6] is 3rd (2nd) row, 1st (0th) column.
	
	// calculate angle around z-axis
	// vector a is where the y-axis should be
	float a[3];
	crossProduct(rot2, rot5, rot8, 1, 0, 0, a);
	
	// this factor ensures that when it is horizontal like a plane, i.e. when the neutral position is undefined, it doesn't control
	float factor = norm3(a);
	normalize(a, a, 3);
	float y[3];
	y[0] = rot1;
	y[1] = rot4;
	y[2] = rot7;
	float angleDifference = acos(scalarProductOfMatrices(a, y, 3));
	// getting a sign on the angle:
	float b[3];
	crossProduct(rot2, rot5, rot8, a[0], a[1], a[2], b);
	tmp_variable = scalarProductOfMatrices(b, y, 3);
	angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0));
	
	float P_z = factor*(200 * (angleDifference + z_axis_trim));
	// D_z is in degree per second
	// D_z STAYS ACTIVE EVEN WHEN KITE IS HORIZONTAL
	float D_z = gyroz - avgGyroz;

	// I_z
	#define MAX_I_Z 30
	if(flightMode == HOVER_MODE){
		if(P_z > 0){
			if(I_z < MAX_I_Z) I_z += 7.0*time_difference;
		}else{
			if(I_z > -MAX_I_Z) I_z -= 7.0*time_difference;
		}
	}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// PID CONTROLLER FOR KEEPING THE Z-AXIS LEVEL (Y-AXIS CONTROLLER)
	// a is the desired position of the z(perp to airfoil)-axis (orthogonal to both the up-vector and the y-axis)
	crossProduct(rot1, rot4, rot7, -1, 0, 0, a);
	
	// when props point sideways y-axis may be controlled by wind direction (... or better "controlled" through aerodynamic "longitudinal static stability" + remaining D_y term)
	// this factor ensures that when the props point sideways, it doesn't control
	factor = norm3(a);
	
	normalize(a, a, 3);
	float z[3];
	z[0] = rot2;
	z[1] = rot5;
	z[2] = rot8;
	angleDifference = acos(scalarProductOfMatrices(a, z, 3)); // is exactly the angle between the two vectors a and z.
	// getting a sign on the angle:
	crossProduct(rot1, rot4, rot7, a[0], a[1], a[2], b); // a cross y, a pitched 90 degrees upwards
	tmp_variable = scalarProductOfMatrices(b, z, 3); // tmp_variable positive when kite tilted backwards, negative, when tilted forward. (...modulo -1)
	angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0));// <--- sign(<b,z>)
	
	//ONLY ALLOW TILTING WHEN HEIGHT SUFFICIENT
	float save_y_axis_trim = y_axis_trim;
	if(getHeight() < 10){
		if(y_axis_trim < -0.5){
			save_y_axis_trim = -0.5;
		}else if(y_axis_trim > 0.5){
			save_y_axis_trim = 0.5;
		}
	}
	
	float P_y = factor*(37/*pcb glued in with slight forward pitch*/+200 * (angleDifference + y_axis_trim));
	float D_y = gyroy-avgGyroy;
	float C_y = -0.12*Dy*D_y + 0.6*Py*P_y + Iy*I_y;
	// transition to aerodynamically guided pitch
	if(flightMode == GLIDE_MODE){
		C_y = -0.5*Dy*D_y - 17 + 50*elev_trim;
	}
	
	// I_y
	#define MAX_I_Y 30
	if(flightMode == HOVER_MODE){
		if(P_y > 0){
			if(I_y < MAX_I_Y) I_y += 7.0*time_difference;
		}else{
			if(I_y > -MAX_I_Y) I_y -= 7.0*time_difference;
		}
	}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// TRANSITION TO GLIDING FLIGHT
	
	angleDifference *= -0.64; // 2/pi
	//2,0,-2 respond to pi, 0, -pi, so diff between 0 and 1 is 90 degrees. Also angleDiff is centered (i.e. = 0) at hover position.
	angleDifference -= 0.2; // this is also because pcb is glued in tilted !!!
	//TODO: squaring to let it take effect later
	angleDifference *= angleDifference;
	if(angleDifference > 1.0){
		angleDifference = 1.0;
	}
	if(angleDifference < 0.0){
		angleDifference = 0.0;
	}
	
	// re-using the angleDifference from the Y-AXIS controller.
	// angleDifference = 0 <=> hover position
	// angleDifference = 1 <=> gliding position
	
	// GRADUALLY INCREASE YAW CONTROL (STEERING, Z-AXIS) BY RUDDER
	// GRADUALLY DECREASE Z-AXIS-CONTROL
	float C_z = -0.3*Dz*D_z + (1.0-angleDifference)*0.6*Pz*P_z + Iz*I_z - 15*angleDifference*aux_trim;
	if(flightMode == GLIDE_MODE || flightMode == MOTORPLANE_MODE){ // third knob from the right turns on manual kite fly mode, which ignores orientation
		C_z = -0.1*Dz*D_z + 50*aux_trim; // TODO: find right stiffness for D_z by testing
	}
	
	// (GRADUALLY) ...
	C_h = C_h*(1.0-angleDifference) + angleDifference*MOTOR_SPEED_WHEN_HORIZONTAL;
	
	// ... STOP CONTROLLING THE HEIGHT using the motors when angle to horizon is 0 or negative
	if(angleDifference == 1.0){C_h = 0.0;}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// MOTORS
	
	if(C_h > (MOTOR_MAX_SPEED - 10)*(1-angleDifference)) C_h = (MOTOR_MAX_SPEED - 10)*(1-angleDifference) + MOTOR_SPEED_WHEN_HORIZONTAL*angleDifference; // VERY IMPORTANT to keep C_z effective
	if(C_h < 0) C_h = 0;
	smooth_C_h = smooth_C_h * 0.9 + C_h * 0.1;
	float motor = limitMotorForSafety(smooth_C_h);
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// MIXING PID FOR Z-AXIS AND X-AXIS
	float servoRudder = limitRudderAngle(C_z);
	float servoElevon = limitElevonAngle(C_y);
	
	setAngle(TOP_RIGHT, servoElevon);
	setSpeed(BOTTOM_LEFT, motor);
	setSpeed(BOTTOM_RIGHT, motor);
	setAngle(TOP_LEFT, servoRudder);
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	// SENDING DEBUGGING DATA TO GROUND
	
	sendData(P_y, D_y, I_y/30., P_z, D_z, I_z/30., flightMode, servoRudder/30., 0, 0);
	
}
