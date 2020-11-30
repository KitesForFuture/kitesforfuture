int speeds[21] = {
13,	16,	17,	19,	21,	23,	25,	28,	32,	35,	40,	45,	50,	55,	60,	65,	70,	75,	80,	85,	90	};
float thresholds[21] = {
1.2,1.2,1.2,1.2,1.2,1.2,1.3,1.3,1.4,1.4,1.5,1.6,1.7,1.7,1.8,2.0,2.2,2.3,2.5,2.6,2.8	};
//1,1,	1,	1,	1.1,1.2,1.3,1.3,1.4,1.4,1.5,1.6,1.7,1.7,1.8,2.0,2.2,2.3,2.5,2.6,2.8	};

int speed_step_index = 0;


float lowRange = 0;
float highRange = 0;
float lowRangeInAmps = 0;
float highRangeInAmps = 0;
float getReelInCurrentInAmps(){
	lowRange = 0.9 * lowRange + 0.1 * 3.12*((float)getSensor(2)-142)/(3003.0); // 5A ampere meter, reading in volts
	highRange = 0.9 * highRange + 0.1 * 3.12*((float)getSensor(5)-142)/(3003.0); // 20A ampere meter, reading in volts
	lowRangeInAmps = 0.95 * lowRangeInAmps + 0.05 * (2.55-lowRange)*5.41; // 0.185 Volts/Ampere and neutral point at 2.53 Volts
	highRangeInAmps = 0.95 * highRangeInAmps + 0.05 * (2.55-highRange)*10.21; // 0.100 Volts/Ampere and neutral point at 2.53 Volts
	float returnValue = 0;
	if(lowRangeInAmps > 4.5){
		returnValue = highRangeInAmps;
	}else{
		returnValue = lowRangeInAmps;
	}
	if(returnValue < 0) return 0;
	else return returnValue;
}

Timer temp_timer = 0;

void initReelInAutomation(){
	temp_timer = startTimer();
}

int decideSpeed(){
	//only act every 0.3 seconds
	if(queryTimer(temp_timer) > 0.05){
		temp_timer = startTimer();
		if(getReelInCurrentInAmps() > thresholds[speed_step_index]){
			speed_step_index --;
			if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.2){
				speed_step_index --;
				if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.4){
					speed_step_index --;
				}
			}
			
		}else if(getReelInCurrentInAmps() < thresholds[speed_step_index]/* - 0.2*/){
			speed_step_index ++;
		}
		if(speed_step_index < 0) {speed_step_index = 0; return 1;}
		if(speed_step_index > 20) speed_step_index = 20;
	}
	return 0;
}

int getSpeed(){
	return speeds[speed_step_index];
}
