#include <stdint.h>

extern uint8_t i2c_ReadByte(uint8_t devId, uint8_t addr);
extern int32_t i2c_WriteByte(uint8_t devId, uint8_t addr, uint8_t data);

extern int32_t i2c_ReadBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf );
extern int32_t i2c_WriteBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf);
