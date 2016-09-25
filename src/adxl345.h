#include <stdint.h>

#define I2C_ID_ADXL345																0x53

#define ADXL345_ADDRESS_ALT_LOW												0x53 // alt address pin low (GND)
#define ADXL345_ADDRESS_ALT_HIGH											0x1D // alt address pin high (VCC)
#define ADXL345_DEFAULT_ADDRESS												ADXL345_ADDRESS_ALT_LOW

#define ADXL345_RA_DEVID															0x00
#define ADXL345_RA_RESERVED1													0x01
#define ADXL345_RA_THRESH_TAP													0x1D
#define ADXL345_RA_OFSX																0x1E
#define ADXL345_RA_OFSY																0x1F
#define ADXL345_RA_OFSZ																0x20
#define ADXL345_RA_DUR																0x21
#define ADXL345_RA_LATENT															0x22
#define ADXL345_RA_WINDOW															0x23
#define ADXL345_RA_THRESH_ACT													0x24
#define ADXL345_RA_THRESH_INACT												0x25
#define ADXL345_RA_TIME_INACT													0x26
#define ADXL345_RA_ACT_INACT_CTL											0x27
#define ADXL345_RA_THRESH_FF													0x28
#define ADXL345_RA_TIME_FF														0x29
#define ADXL345_RA_TAP_AXES														0x2A
#define ADXL345_RA_ACT_TAP_STATUS											0x2B
#define ADXL345_RA_BW_RATE														0x2C
#define ADXL345_RA_POWER_CTL													0x2D
#define ADXL345_RA_INT_ENABLE													0x2E
#define ADXL345_RA_INT_MAP														0x2F
#define ADXL345_RA_INT_SOURCE													0x30
#define ADXL345_RA_DATA_FORMAT												0x31
#define ADXL345_RA_DATAX0															0x32
#define ADXL345_RA_DATAX1															0x33
#define ADXL345_RA_DATAY0															0x34
#define ADXL345_RA_DATAY1															0x35
#define ADXL345_RA_DATAZ0															0x36
#define ADXL345_RA_DATAZ1															0x37
#define ADXL345_RA_FIFO_CTL														0x38
#define ADXL345_RA_FIFO_STATUS												0x39

#define ADXL345_ACT_ACDC_ENABLE												0x01<<7
#define ADXL345_ACTX_ENABLE														0x01<<6
#define ADXL345_ACTY_ENABLE														0x01<<5
#define ADXL345_ACTZ_ENABLE														0x01<<4
#define ADXL345_INACT_ACDC_ENABLE											0x01<<3
#define ADXL345_INACTX_ENABLE													0x01<<2
#define ADXL345_INACTY_ENABLE													0x01<<1
#define ADXL345_INACTZ_ENABLE													0x01<<0

#define ADXL345_ACT_ACDC_DISABLE											0x00<<7
#define ADXL345_ACTX_DISABLE													0x00<<6
#define ADXL345_ACTY_DISABLE													0x00<<5
#define ADXL345_ACTZ_DISABLE													0x00<<4
#define ADXL345_INACT_ACDC_DISABLE										0x00<<3
#define ADXL345_INACTX_DISABLE												0x00<<2
#define ADXL345_INACTY_DISABLE												0x00<<1
#define ADXL345_INACTZ_DISABLE												0x00<<0

#define ADXL345_TAPAXES_SUP_ENABLE										0x01<<3
#define ADXL345_TAPAXES_TAPX_ENABLE										0x01<<2
#define ADXL345_TAPAXES_TAPY_ENABLE										0x01<<1
#define ADXL345_TAPAXES_TAPZ_ENABLE										0x01<<0

#define ADXL345_TAPAXES_SUP_DISABLE										0x00<<3
#define ADXL345_TAPAXES_TAPX_DISABLE									0x00<<2
#define ADXL345_TAPAXES_TAPY_DISABLE									0x00<<1
#define ADXL345_TAPAXES_TAPZ_DISABLE									0x00<<0

#define ADXL345_LowPower															0x00<<4
#define ADXL345_NormalPower														0x01<<4

#define ADXL345_BW_1600																0x0F
#define ADXL345_BW_800																0x0E
#define ADXL345_BW_400																0x0D
#define ADXL345_BW_200																0x0C
#define ADXL345_BW_100																0x0B
#define ADXL345_BW_50																	0x0A
#define ADXL345_BW_25																	0x09
#define ADXL345_BW_12P5																0x08
#define ADXL345_BW_6P25																0x07
#define ADXL345_BW_3P13																0x06
#define ADXL345_BW_1P56																0x05
#define ADXL345_BW_0P78																0x04
#define ADXL345_BW_0P39																0x03
#define ADXL345_BW_0P20																0x02
#define ADXL345_BW_0P10																0x01
#define ADXL345_BW_0P05																0x00

#define ADXL345_LINK_ENABLE														0x01<<5
#define ADXL345_LINK_DISABLE													0x00<<5
#define ADXL345_ASLEEP_ENABLE													0x01<<4
#define ADXL345_ASLEEP_DISABLE												0x00<<4
#define ADXL345_MEASURE_ENABLE												0x01<<3
#define ADXL345_MEASURE_DISABLE												0x00<<3
#define ADXL345_SLEEP_ENABLE													0x01<<2
#define ADXL345_SLEEP_DISABLE													0x00<<2
#define ADXL345_WAKE_8HZ															0x00
#define ADXL345_WAKE_4HZ															0x01
#define ADXL345_WAKE_2HZ															0x02
#define ADXL345_WAKE_1HZ															0x03

#define ADXL345_INT_DATARDY_ENABLE										0x01<<7
#define ADXL345_INT_DATARDY_DISABLE										0x00<<7
#define ADXL345_INT_SINGLETAP_ENABLE									0x01<<6
#define ADXL345_INT_SINGLETAP_DISABLE									0x00<<6
#define ADXL345_INT_DOUBLETAP_ENABLE									0x01<<5
#define ADXL345_INT_DOUBLTAP_DISABLE									0x00<<5
#define ADXL345_INT_ACTIVITY_ENABLE										0x01<<4
#define ADXL345_INT_ACTIVITY_DISABLE									0x00<<4
#define ADXL345_INT_INACTIVITY_ENABLE									0x01<<3
#define ADXL345_INT_INACTIVITY_DISABLE								0x00<<3
#define ADXL345_INT_FREEFALL_ENABLE										0x01<<2
#define ADXL345_INT_FREEFALL_DISABLE									0x00<<2
#define ADXL345_INT_WATERMARK_ENABLE									0x01<<1
#define ADXL345_INT_WATERMARK_DISABLE									0x00<<1
#define ADXL345_INT_OVERRUN_ENABLE										0x01<<0
#define ADXL345_INT_OVERRUN_DISABLE										0x00<<0

#define ADXL345_DATA_SELFTEST_ENABLE									0x01<<7
#define ADXL345_DATA_SELFTEST_DISABLE									0x00<<7
#define ADXL345_DATA_SPI_3														0x01<<6
#define ADXL345_DATA_SPI_4														0x00<<6
#define ADXL345_DATA_INT_INVERT_ENABLE								0x01<<5
#define ADXL345_DATA_INT_INVERT_DISABLE								0x00<5
#define ADXL345_DATA_FULLRES_ENABLE										0x01<<3
#define ADXL345_DATA_FULLRES_DISABLE									0x00<<3
#define ADXL345_DATA_JUSTIFY_LEFT											0x01<<2
#define ADXL345_DATA_JUSTIFY_RIGHT										0x00<<2
#define ADXL345_DATA_RANGE_2G													0x00
#define ADXL345_DATA_RANGE_4G													0x01
#define ADXL345_DATA_RANGE_8G													0x02
#define ADXL345_DATA_RANGE_18G												0x03

#define ADXL345_FIFO_BYPASS														0x00<<6
#define ADXL345_FIFO_FIFO															0x01<<6
#define ADXL345_FIFO_STREAM														0x02<<6
#define ADXL345_FIFO_TRIGGER													0x03<<6
#define ADXL345_TRIG_INT1															0x00<<5
#define ADXL345_TRIG_INT2															0x01<<5
#define ADXL345_SAMPLE_WATERMARKENABLE								0x00
#define ADXL345_SAMPLE_WATERMARKDISABLE								0x01



extern void adxl345_Init(void);
extern void adxl345_selfTest(int16_t *xdata, int16_t *ydata, int16_t *zdata);
extern uint8_t adxl345_test(void);
extern void  adxl345_ReadTapThresh(uint8_t *tap);
extern void adxl345_WriteTapThresh(uint8_t tap);
extern void adxl345_ReadXOffSet(int8_t *xoff);
extern void adxl345_ReadYOffSet(int8_t *yoff);
extern void adxl345_ReadZOffSet(int8_t *zoff);
extern void adxl345_ReadXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff);
extern void adxl345_WriteXOffSet(int8_t xoff);
extern void adxl345_WriteYOffSet(int8_t yoff);
extern void adxl345_WriteZOffSet(int8_t zoff);
extern void adxl_WriteXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff);
extern void adxl345_ReadDur(uint8_t *dur);
extern void adxl345_WriteDur(uint8_t dur);
extern void adxl345_ReadLat(uint8_t *lat);
extern void adxl345_WriteLat(uint8_t lat);
extern void adxl345_ReadWindow(uint8_t *win);
extern void adxl345_WriteWindow(uint8_t win);
extern void adxl345_ReadAct(uint8_t *act);
extern void adxl345_WriteAct(uint8_t act);
extern void adxl345_ReadThreshInact(int8_t *inact);
extern void adxl345_WriteThreshInact(int8_t inact);
extern void adxl345_ReadTimeInact(uint8_t *timinact);
extern void adxl345_WriteTimeInact(uint8_t timinact);
extern void adxl345_WriteACDC(uint8_t ActEN, uint8_t ActX, uint8_t ActY, uint8_t ActZ, uint8_t InActEN, uint8_t InActX, uint8_t InActY, uint8_t InActZ);
extern void adxl345_ReadACDC(uint8_t *acdc);
extern void adxl345_ReadThreshFF(uint8_t *threshff);
extern void adxl345_WriteThreshFF(uint8_t threshff);
extern void adxl345_ReadTimeFF(uint8_t *timeff);
extern void adxl345_WriteTimeFF(uint8_t timeff);
extern void adxl345_ReadTapAxes(uint8_t *tap_axes);
extern void adxl345_WriteTapAxes(uint8_t sup, uint8_t tapx, uint8_t tapy, uint8_t tapz);
extern void adxl345_ReadActTapStatus(uint8_t *status);
extern void adxl345_ReadBWRate(uint8_t *bw);
extern void adxl345_WriteBWRate(uint8_t pwr, uint8_t bw);
extern void adxl345_ReadPWRCtl(uint8_t *pwrctl);
extern void adxl345_WritePWRCtl(uint8_t link, uint8_t autosleep, uint8_t measure, uint8_t sleep, uint8_t wake);
extern void adxl345_WriteINTEnable(uint8_t DataRDY, uint8_t singletap, uint8_t doubletap, uint8_t act, uint8_t inact, uint8_t ff, uint8_t watermrk, uint8_t overrun);
extern void adxl345_ReadINTEnable(uint8_t *inten);
extern void adxl345_ReadINTSource(uint8_t *intsource);
extern void adxl345_ReadDataFormat(uint8_t *data);
extern void adxl345_WriteDataFormat(uint8_t selftest, uint8_t spi, uint8_t intinv, uint8_t fullres, uint8_t justify, uint8_t range);
extern void adxl345_ReadXYZ(int16_t *xdata, int16_t *ydata, int16_t *zdata);
extern void adxl345_ReadFIFOCtl(uint8_t *fifo);
extern void adxl345_WriteFIFOCtl(uint8_t fifo, uint8_t trigger, uint8_t sample);
extern void adxl345_ReadFIFOStatus(uint8_t *fifost);
