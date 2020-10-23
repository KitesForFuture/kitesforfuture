float goalHeight = -5;
float targetHeight = -1;
float rateOfClimb = 0;
float lastHeight = 0;
float oldGoalHeight = -5;

void setGoalHeight(float goalheight)
{
	goalHeight = goalheight;
	//targetHeight = getHeight();
}

float control_hover_height(float P, float I, float D, float C)
{
    float current_height = getHeight();

	if (fabs(goalHeight - targetHeight) > 0.1)
	{
		float adjustedClimbRate = rateOfClimb;
		if (targetHeight < 2)
			adjustedClimbRate = 0.2; // for descending slowly during the last 3 meters
		if (goalHeight - targetHeight > 0)
			adjustedClimbRate = 2.0;
		targetHeight += time_difference * adjustedClimbRate * sign(goalHeight - targetHeight);
	}

	// exponential average smoothing of P_h
	//TODO: bound 3.5+(targetHeight - current_height) from below.
	P_h = 0.3 * P_h + 0.7 * (3.5 + (targetHeight - current_height));

	// in meters per second
	// D_h, deviation from desired rate of climb
	// negative: climbing faster than needed
	// positive: climbing slower than needed

	float tmp_variable = 0;
	if (time_difference != 0)
	{
		tmp_variable = (lastHeight - current_height) / time_difference;
	}
	float D_h = (tmp_variable + sign(goalHeight - current_height) * rateOfClimb);
	lastHeight = current_height;

	return D * D_h + P * P_h;
}

