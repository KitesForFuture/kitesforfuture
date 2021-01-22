
#ifndef KITE_MATH_HELPERS
#define KITE_MATH_HELPERS
 
float safe_acos(float number_more_or_less_between_one_and_minus_one);

int smallpow(int x, int p);

void crossProduct(float a, float b, float c, float x, float y, float z, float result[]);

float sign(float x);

void mat_mult(float a[], float b[], float out[]);

void mat_mult_mat_transp(float a[], float b[], float out[]);

void mat_transp_mult_vec(float mat[], float a, float b, float c, float out[]);

void mat_mult_vec(float mat[], float a, float b, float c, float out[]);

void normalize_matrix(float a[]);

void rotate_towards_g(float mat[], float a_init, float b_init, float c_init, float a, float b, float c, float tmp_vec[], float out[]);

float scalarProductOfMatrices(float A[], float B[], int length);

float normalize(float a[], int length);
 
#endif