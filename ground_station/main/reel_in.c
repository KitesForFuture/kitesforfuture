int speeds[19] = {
13, 16,	18, 20,	24, 27,	30,	35,	40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90};
//13,	16,	17,	19,	21,	23,	25,	28,	32,	35,	40,	45,	50,	55,	60,	65,	70,	75,	80,	85,	90	}; // # = 21
float thresholds[19] = {
1.5,1.5,1.6,1.6,1.6,1.7,1.7,1.7,1.8,1.8,1.9,2.0,2.2,2.3,2.3,2.4,2.5,2.5,2.6};
//1.2,1.2,1.2,1.2,1.2,1.2,1.3,1.3,1.4,1.4,1.5,1.6,1.7,1.7,1.8,2.0,2.2,2.3,2.5,2.6,2.8	};
//1,1,	1,	1,	1.1,1.2,1.3,1.3,1.4,1.4,1.5,1.6,1.7,1.7,1.8,2.0,2.2,2.3,2.5,2.6,2.8	};

int speed_step_index = 0;


float lowRange = 0;
float highRange = 0;
float lowRangeInAmps = 0;
float highRangeInAmps = 0;

void updateCurrentSensing(){
	// reel in current
	lowRange = 0.8 * lowRange + 0.2 * 3.12*((float)getSensor(2)-142)/(3003.0); // 5A ampere meter, reading in volts
	lowRangeInAmps = 0.9 * lowRangeInAmps + 0.1 * (2.55-lowRange)*5.41; // 0.185 Volts/Ampere and neutral point at 2.53 Volts
	
	// power production current
	highRange = 0.8 * highRange + 0.2 * 3.12*((float)getSensor(5)-142)/(3003.0); // 20A ampere meter, reading in volts
	highRangeInAmps = 0.9 * highRangeInAmps + 0.1 * ((2.55-highRange)*10.21 - 1.0); // 0.100 Volts/Ampere and neutral point at 2.53 Volts
}

float getReelInCurrentInAmps(){
	//float returnValue = 0;
	//if(lowRangeInAmps > 4.5){
	//	returnValue = highRangeInAmps;
	//}else{
	//	returnValue = lowRangeInAmps;
	//}
	//if(returnValue < 0) return 0;
	//else return returnValue;
	
	return (lowRangeInAmps < 0) ? 0 : lowRangeInAmps;
}

float getProductionCurrentInAmps(){
	return (highRangeInAmps < 0) ? 0 : highRangeInAmps;
}

Timer temp_timer = 0;
Timer temp_timerUp = 0;

void initReelInAutomation(){
	temp_timer = startTimer();
	temp_timerUp = startTimer();
}

int decideSpeed(){
	//only act every ... seconds
	if(queryTimer(temp_timer) > 0.2){
		temp_timer = startTimer();
		if(getReelInCurrentInAmps() > thresholds[speed_step_index]){
			speed_step_index --;
			if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.2){
				speed_step_index --;
				if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.4){
					speed_step_index --;
					if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.6){
						speed_step_index --;
						if(getReelInCurrentInAmps() > thresholds[speed_step_index] + 0.8){
							speed_step_index --;
						}
					}
				}
			}
		}
		if(speed_step_index < 0) {speed_step_index = 0; return 1;}
		
	}
	if(queryTimer(temp_timerUp) > 0.2){
		temp_timerUp = startTimer();
		if(getReelInCurrentInAmps() < thresholds[speed_step_index]){
			speed_step_index ++;
		}
		if(speed_step_index > 20) speed_step_index = 20;
	}
	return 0;
}

int getSpeed(){
	return speeds[speed_step_index];
}
