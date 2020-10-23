float control_glide_rudder(float P, float I, float D, float C){
    // D_z is in degree per second
    // D_z STAYS ACTIVE EVEN WHEN KITE IS HORIZONTAL
    float D_z = gyroz - avgGyroz;

    // third knob from the right turns on manual kite fly mode, which ignores orientation
	return -0.1 * D * D_z + C; // TODO: find right stiffness for D_z by testing
}