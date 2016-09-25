#include <stdint.h>

#include "i2cu.h"
#include "itg3200.h"

void itg3200_Init(void) {
	i2c_WriteByte(I2C_ID_ITG3200, ITG3200_RA_DLPF_FS, ITG3200_DLPF_FS_FULL_SCALE);
}

/*
 * @brief: Read all 3 axes of ITG3200
 * @param[in]: ptr output to individual axes
 * @param[out]: none
 */
void itg3200_ReadXYZ(int16_t *x, int16_t *y, int16_t *z) {
uint8_t b[6];
	
	i2c_ReadBuf(I2C_ID_ITG3200, ITG3200_RA_GYRO_XOUT_H, 6, b);
	*x = ((int16_t)(((uint16_t)b[0]<<8) | (uint16_t)b[1]));
	*y = ((int16_t)(((uint16_t)b[2]<<8) | (uint16_t)b[3]));
	*z = ((int16_t)(((uint16_t)b[4]<<8) | (uint16_t)b[5]));
}
