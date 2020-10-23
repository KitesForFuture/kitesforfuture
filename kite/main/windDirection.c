
#define windAverageConstBig		0.9999
#define windAverageConstSmall	0.0001

float windY = 0;
float windZ = 0;

void updateWindDirection(){
	if(rot[2] > 0.9){ // kite flies horizontally like a plane
		float inverseNorm = 1.0/sqrt(rot[3]*rot[3] + rot[6]*rot[6]); // = sqrt(1-rot[2]*rot[2]); // horizontal components of x-axis
		windY = windAverageConstBig*windY - windAverageConstSmall*rot[3]*inverseNorm;
		windZ = windAverageConstBig*windZ - windAverageConstSmall*rot[6]*inverseNorm;
	/*
	}else if(rot[2] < -0.9){
		float inverseNorm = 1.0/sqrt(rot[3]*rot[3] + rot[6]*rot[6]); // = sqrt(1-rot[2]*rot[2]);
		windY = windAverageConstBig*windY + windAverageConstSmall*rot[3]*inverseNorm;
		windZ = windAverageConstBig*windZ + windAverageConstSmall*rot[6]*inverseNorm;
	*/
	}else{
		float inverseNorm = 1.0/sqrt(rot[5]*rot[5] + rot[8]*rot[8]); // = sqrt(1-rot[2]*rot[2]); // horizontal components of z-axis
		windY = windAverageConstBig*windY + windAverageConstSmall*rot[5]*inverseNorm;
		windZ = windAverageConstBig*windZ + windAverageConstSmall*rot[8]*inverseNorm;
	}
	//TODO normalize!
}
