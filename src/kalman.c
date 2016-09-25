#include <stdint.h>
#include "kalman.h"


void kalman_init(kalman_data * kd) {
	
	kd->x[0] = 0.0f;
	kd->x[1] = 0.0f;
	kd->x[2] = 0.0f;
	
	kd->P[0][0] = 1000.0f;
	kd->P[0][1] = 0.0f;
	kd->P[0][2] = 0.0f;
	kd->P[1][0] = 0.0f;
	kd->P[1][1] = 1000.0f;
	kd->P[1][2] = 0.0f;
	kd->P[2][0] = 0.0f;
	kd->P[2][1] = 0.0f;
	kd->P[2][2] = 1000.0f;

}

void kalman_innovate(kalman_data * kd, float z1, float z2) {
	float pred_x[3];
	float pred_P[3][3];
	float AP[3];
	float K[3][2];
	float S[2][2];
	float detS;
	
	///--- Prediction Step ---///
	// x'_(k) = A * x_(k-1)
	pred_x[0] = kd->x[0] + (kd->x[1] - kd->x[2]) * _dt;
	pred_x[1] = kd->x[1];
	pred_x[2] = kd->x[2];
	
	// P'_(k) = A * P_(k-1) * A^T + Q
	AP[0] = (kd->P[0][0] + _dt * (kd->P[1][0] - kd->P[2][0]));
	AP[1] = (kd->P[0][1] + _dt * (kd->P[1][1] - kd->P[2][1]));
	AP[2] = (kd->P[0][2] + _dt * (kd->P[1][2] - kd->P[2][2]));
	pred_P[0][0] = AP[0] + _dt * (AP[1] - AP[2]) + _Q00;
	pred_P[0][1] = AP[1];
	pred_P[0][2] = AP[2];
	pred_P[1][0] = kd->P[1][0] + _dt * (kd->P[1][1] - kd->P[1][2]);
	pred_P[1][1] = kd->P[1][1] + _Q11;
	pred_P[1][2] = kd->P[1][2];
	pred_P[2][0] = kd->P[2][0] + _dt * (kd->P[2][1] - kd->P[2][2]);
	pred_P[2][1] = kd->P[2][1];
	pred_P[2][2] = kd->P[2][2] + _Q22;
	
	
	///--- Correction Step ---///
	// K_(k) = P * H^T * S^(-1), where S = H * P'_(k) * H^T + R
	S[0][0] = pred_P[0][0] + _R00;
	S[0][1] = pred_P[0][1];
	S[1][0] = pred_P[1][0];
	S[1][1] = pred_P[1][1] + _R11;
	detS = (S[0][0]*S[1][1] - S[0][1]*S[1][0]);
	K[0][0] = (pred_P[0][0] * S[1][1] - pred_P[0][1] * S[1][0]) / detS;
	K[0][1] = (pred_P[0][1] * S[0][0] - pred_P[0][0] * S[0][1]) / detS;
	K[1][0] = (pred_P[1][0] * S[1][1] - pred_P[1][1] * S[1][0]) / detS;
	K[1][1] = (pred_P[1][1] * S[0][0] - pred_P[1][0] * S[0][1]) / detS;
	K[2][0] = (pred_P[2][0] * S[1][1] - pred_P[2][1] * S[1][0]) / detS;
	K[2][1] = (pred_P[2][1] * S[0][0] - pred_P[2][0] * S[0][1]) / detS;
	
	// x_(k) = x'_(k) + K_(k) * (z_k - H * x'_(k))
	kd->x[0] = pred_x[0] + K[0][0] * (z1 - pred_x[0]) + K[0][1] * (z2 - pred_x[1]);
	kd->x[1] = pred_x[1] + K[1][0] * (z1 - pred_x[0]) + K[1][1] * (z2 - pred_x[1]);
	kd->x[2] = pred_x[2] + K[2][0] * (z1 - pred_x[0]) + K[2][1] * (z2 - pred_x[1]);
	
	// P_(k) = (I - K_(k) * H) * P'_(k)
	kd->P[0][0] = (1.0f - K[0][0]) * pred_P[0][0] - K[0][1] * pred_P[1][0];
	kd->P[0][1] = (1.0f - K[0][0]) * pred_P[0][1] - K[0][1] * pred_P[1][1];
	kd->P[0][2] = (1.0f - K[0][0]) * pred_P[0][2] - K[0][1] * pred_P[1][2];
	kd->P[1][0] = (1.0f - K[1][1]) * pred_P[1][0] - K[1][1] * pred_P[0][0];
	kd->P[1][1] = (1.0f - K[1][1]) * pred_P[1][1] - K[1][1] * pred_P[0][1];
	kd->P[1][2] = (1.0f - K[1][1]) * pred_P[1][2] - K[1][1] * pred_P[0][2];
	kd->P[2][0] = pred_P[2][0] - K[2][0] * pred_P[0][0] - K[2][1] * pred_P[1][0];
	kd->P[2][1] = pred_P[2][1] - K[2][0] * pred_P[0][1] - K[2][1] * pred_P[1][1];
	kd->P[2][2] = pred_P[2][2] - K[2][0] * pred_P[0][2] - K[2][1] * pred_P[1][2];
	
}
