
	// ID CONTROLLER FOR THE HEIGHT
	// height difference, can be large
	float heightDifference = goalHeight - getHeight();
	if(goalHeight == oldGoalHeight){
		float x = (lastHeight - getHeight())/time_difference;
		// D_h, deviation from desired rate of climb
		// negative: climbing faster than needed
		// positive: climbing slower than needed
		float D_h = (x+sign(heightDifference)*rateOfClimb);
	}
	oldGoalHeight = goalHeight;
	lastHeight = getHeight();
	
	if(D_h > 0){
		I_h += 0.02;
	}else{
		I_h -= 0.02;
	}
	
	//TODO: still oscillates at target height
	
	float C_h = I_h + 3*D_h;
	if(C_h > 40) C_h = MOTOR_MAX_SPEED - 10; // VERY IMPORTANT to keep C_z effective
	if(C_h < 0) C_h = 0;
	
	
	
	
	
	
	
	
	
	
	
	// PID CONTROLLER FOR THE HEIGHT
	// +3.5 to set average motor speed to 35 ESC-'degrees'
	float P_h = 0.3*P_h + 0.7*(3.5+(goalHeight - getHeight()));
	
	// in meters per second
	// ! do not calculate difference quotient if P_h has changed because of updated goalHeight
	if(goalHeight == oldGoalHeight){
		float x = (P_h - last_P_h)/time_difference;
		D_h = (x-sign(goalHeight-getHeight())*rateOfClimb);
	}
	oldGoalHeight = goalHeight;
	last_P_h = P_h;
	
	float C_h = 3*D_h + 10*P_h;
	
	if(C_h > 40) C_h = MOTOR_MAX_SPEED - 10; // VERY IMPORTANT to keep C_z effective
	if(C_h < 0) C_h = 0;
	
