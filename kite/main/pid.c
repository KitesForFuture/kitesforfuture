#define HOVER_MODE 0
#define GLIDE_MODE 1
#define MOTORPLANE_MODE 2

int flightMode = GLIDE_MODE;

void setGoalHeight(float goalheight)
{
	goalHeight = goalheight;
	//targetHeight = getHeight();
}

void setRateOfClimb(float rate)
{
	rateOfClimb = fabs(rate);
}

void setYAxisTrim(float trim)
{
	y_axis_trim = trim;
}

void setZAxisTrim(float trim)
{
	z_axis_trim = trim;
}

void setElevatorTrim(float trim)
{
	elev_trim = trim;
}

void setAUXTrim(float trim)
{
	aux_trim = trim;
}

float limitMotorForSafety(float motor)
{

	// TURN MOTOR OFF, IF SECOND RIGHT MOST POTI IS TURNED LEFT
	if (flightMode == GLIDE_MODE)
	{
		return 0;
	}

	// LIMIT MOTOR SPEED FOR SAFETY
	if (motor < 0)
	{
		return 0;
	}
	if (motor > MOTOR_MAX_SPEED)
	{
		return MOTOR_MAX_SPEED;
	}
	return motor;
}

float limitRudderAngle(float rudder)
{

	if (rudder < SERVO_RUDDER_MIN_ANGLE)
		return SERVO_RUDDER_MIN_ANGLE;
	if (rudder > SERVO_RUDDER_MAX_ANGLE)
		return SERVO_RUDDER_MAX_ANGLE;
	return rudder;
}

float limitElevonAngle(float elevon)
{

	if (elevon < SERVO_ELEVON_MIN_ANGLE)
		return SERVO_ELEVON_MIN_ANGLE;
	if (elevon > SERVO_ELEVON_MAX_ANGLE)
		return SERVO_ELEVON_MAX_ANGLE;
	return elevon;
}

void calculatePID()
{
	// PID CONTROLLER FOR THE HEIGHT
	float C_h = 0;
	if (flightMode == HOVER_MODE)
        C_h = control_hover_height(0.7 * 10 * Ph, 0, 0.7 * 3 * Dh, 0);


	// TODO: MOVE THIS TO THE OTHER FILE

	// (GRADUALLY) ...
	C_h = C_h * (1.0 - angleDifference) + angleDifference * MOTOR_SPEED_WHEN_HORIZONTAL;

	// ... STOP CONTROLLING THE HEIGHT using the motors when angle to horizon is 0 or negative
	if (angleDifference == 1.0)
	{
		C_h = 0.0;
	}

	if (C_h > (MOTOR_MAX_SPEED - 10) * (1 - angleDifference))
		C_h = (MOTOR_MAX_SPEED - 10) * (1 - angleDifference) + MOTOR_SPEED_WHEN_HORIZONTAL * angleDifference; // VERY IMPORTANT to keep C_z effective
	if (C_h < 0)
		C_h = 0;
	smooth_C_h = smooth_C_h * 0.9 + C_h * 0.1;
	float motor = limitMotorForSafety(smooth_C_h);

	//---------------------------------------------------------------
	//---------------------------------------------------------------

	// PID CONTROLLER FOR KEEPING THE Y-AXIS LEVEL (Z-AXIS CONTROLLER)

	float C_z;
	if (flightMode == GLIDE_MODE || flightMode == MOTORPLANE_MODE){
		C_z = control_glide_rudder(0, 0, Dz, 50 * aux_trim);
	}
	else{
		// GRADUALLY INCREASE YAW CONTROL (STEERING, Z-AXIS) BY RUDDER
		// GRADUALLY DECREASE Z-AXIS-CONTROL
		C_z = control_hover_rudder(Pz, 0, Dz, - 15 * aux_trim);
	}

	//---------------------------------------------------------------
	//---------------------------------------------------------------

	// PID CONTROLLER FOR KEEPING THE Z-AXIS LEVEL (Y-AXIS CONTROLLER)
	
	// transition to aerodynamically guided pitch
	float C_y;
	if (flightMode == GLIDE_MODE)
		C_y = control_glide_elevator(0, 0, Dy);
	else
		C_y = control_hover_elevator(Py, 0, Dy);


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

	//sendData(P_y, D_y, I_y / 30., P_z, D_z, I_z / 30., flightMode, servoRudder / 30., 0, 0);
}
