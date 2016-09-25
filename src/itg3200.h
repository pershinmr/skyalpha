#include <stdint.h>

#define I2C_ID_ITG3200											0x68

#define ITG3200_RA_WHO_AM_I									0x00
#define ITG3200_RA_SMPLRT_DIV								0x15 // sample rate divider
#define ITG3200_RA_DLPF_FS									0x16
#define ITG3200_RA_INT_CFG									0x17
#define ITG3200_RA_INT_STATUS								0x1A
#define ITG3200_RA_TEMP_OUT_H								0x1B
#define ITG3200_RA_TEMP_OUT_L								0x1C
#define ITG3200_RA_GYRO_XOUT_H							0x1D
#define ITG3200_RA_GYRO_XOUT_L							0x1E
#define ITG3200_RA_GYRO_YOUT_H							0x1F
#define ITG3200_RA_GYRO_YOUT_L							0x20
#define ITG3200_RA_GYRO_ZOUT_H							0x21
#define ITG3200_RA_GYRO_ZOUT_L							0x22
#define ITG3200_RA_PWR_MGM									0x3E

#define ITG3200_DLPF_FS_FULL_SCALE					0x03<<3
#define ITG3200_DLPF_FS_FILTER_256HZ				0x00
#define ITG3200_DLPF_FS_FILTER_188HZ				0x01
#define ITG3200_DLPF_FS_FILTER_98HZ					0x02
#define ITG3200_DLPF_FS_FILTER_42HZ					0x03
#define ITG3200_DLPF_FS_FILTER_20HZ					0x04
#define ITG3200_DLPF_FS_FILTER_10HZ					0x05
#define ITG3200_DLPF_FS_FILTER_5HZ					0x06

#define ITG3200_INT_CFG_ACTL								0x01<<7
#define ITG3200_INT_CFG_OPEN								0x01<<6
#define ITG3200_INT_CFG_LATCH_INT_EN				0x01<<5
#define ITG3200_INT_CFG_INT_ANYRD_2CLEAR		0x01<<4
#define ITG3200_INT_CFG_ITG_RDY_EN					0x01<<2
#define ITG3200_INT_CFG_RAW_DRY_EN					0x01<<0

#define ITG3200_PWR_MGM_H_RESET							0x01<<7
#define ITG3200_PWR_MGM_SLEEP								0x01<<6
#define ITG3200_PWR_MGM_STBY_XG							0x01<<5
#define ITG3200_PWR_MGM_STBY_YG							0x01<<4
#define ITG3200_PWR_MGM_STBY_ZG							0x01<<3
#define ITG3200_PWR_MGM_CLC_INTERNAL				0
#define ITG3200_PWR_MGM_CLC_PLL_X						1
#define ITG3200_PWR_MGM_CLC_PLL_Y						2
#define ITG3200_PWR_MGM_CLC_PLL_Z						3
#define ITG3200_PWR_MGM_CLC_PLL_EXT_32KHZ		4
#define ITG3200_PWR_MGM_CLC_PLL_EXT_19MHZ		5



extern void itg3200_Init(void);
extern void itg3200_ReadXYZ(int16_t *x, int16_t *y, int16_t *z);
