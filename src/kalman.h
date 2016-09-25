#include <stdint.h>


#define _dt			0.01f					// 100 Hz

#define _Q00		0.1f
#define _Q11		10.0f
#define _Q22		0.001f

#define _R00		1000.0f
#define _R11		1000.0f


typedef struct {
	float x[3];				// [angle, angular velocity, angular drift velocity]
	float P[3][3];
} kalman_data;

void kalman_init(kalman_data * data);
void kalman_innovate(kalman_data * data, float z1, float z2);
