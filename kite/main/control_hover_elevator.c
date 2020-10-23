float control_hover_elevator(float P, float I, float D){
    // a is the desired position of the z(perp to airfoil)-axis (orthogonal to both the up-vector and the y-axis)
    float a[3];
    crossProduct(rot1, rot4, rot7, -1, 0, 0, a);

    // when props point sideways y-axis may be controlled by wind direction (... or better "controlled" through aerodynamic "longitudinal static stability" + remaining D_y term)
    // this factor ensures that when the props point sideways, it doesn't control
    float factor = norm3(a);

	normalize(a, a, 3);
	float z[3];
	z[0] = rot2;
	z[1] = rot5;
	z[2] = rot8;

    float angleDifference = acos(scalarProductOfMatrices(a, z, 3)); // is exactly the angle between the two vectors a and z.
    // getting a sign on the angle:

    float b[3];
    crossProduct(rot1, rot4, rot7, a[0], a[1], a[2], b);          // a cross y, a pitched 90 degrees upwards
    
    float tmp_variable = scalarProductOfMatrices(b, z, 3);              // tmp_variable positive when kite tilted backwards, negative, when tilted forward. (...modulo -1)
    angleDifference *= ((tmp_variable > 0) - (tmp_variable < 0)); // <--- sign(<b,z>)

    //ONLY ALLOW TILTING WHEN HEIGHT SUFFICIENT
    float save_y_axis_trim = y_axis_trim;
    if (getHeight() < 10)
    {
        if (y_axis_trim < -0.5)
        {
            save_y_axis_trim = -0.5;
        }
        else if (y_axis_trim > 0.5)
        {
            save_y_axis_trim = 0.5;
        }
    }
    float P_y = factor * (37 /*pcb glued in with slight forward pitch*/ + 200 * (angleDifference + y_axis_trim));
	float D_y = gyroy - avgGyroy;

	return -0.12 * D * D_y + 0.6 * P * P_y;
}