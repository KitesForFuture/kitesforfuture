void setDACVoltage(int pin, float voltage){
	int int_voltage = (int)(255*(voltage/3.3));
	if(int_voltage > 255) int_voltage = 255;
	if(int_voltage < 0) int_voltage = 0;
	printf("%d",pin);
	if(pin == 25){
		printf("%d",int_voltage);
		dac_output_voltage(DAC_CHANNEL_1, int_voltage);
	}//else if (pin == 26){
	//	dac_output_voltage(DAC_CHANNEL_2, int_voltage);
	//}
}

void initDAC(){
	dac_output_enable(DAC_CHANNEL_1);
	//dac_output_enable(DAC_CHANNEL_2);
	setDACVoltage(25, 0.0);
	//setDACVoltage(26, 0.0);
}
