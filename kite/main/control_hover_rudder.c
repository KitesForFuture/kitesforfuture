// I_z
#define MAX_I_Z 30

float control_hover_rudder(float P, float I, float D, float C)
{
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
    
    float tmp_variable = scalarProductOfMatrices(b, y, 3);
    
    angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0));

    float P_z = factor * (200 * (angleDifference + z_axis_trim));
    // D_z is in degree per second
    // D_z STAYS ACTIVE EVEN WHEN KITE IS HORIZONTAL
    float D_z = gyroz - avgGyroz;

    angleDifference *= -0.64; // 2/pi
	//2,0,-2 respond to pi, 0, -pi, so diff between 0 and 1 is 90 degrees. Also angleDiff is centered (i.e. = 0) at hover position.
	angleDifference -= 0.2; // this is also because pcb is glued in tilted !!!
	//TODO: squaring to let it take effect later
	angleDifference *= angleDifference;
	if (angleDifference > 1.0)
	{
		angleDifference = 1.0;
	}
	if (angleDifference < 0.0)
	{
		angleDifference = 0.0;
	}

    return -0.3 * D * D_z + 0.6 * (1.0 - angleDifference) * P * P_z + angleDifference * C
}