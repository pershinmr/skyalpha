#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "i2c.h"
#include "hw_i2c.h"
#include "pin_map.h"

#include "i2cu.h"
#include "config.h"
#include "defines.h"


//*****************************************************************************
//
//! Reads the I2C slave register.
//!
//! \param I2C_PORT is the base for the I2C module.
//! \param SlaveID is the 7-bit address of the slave to be addressed.
//! \param addr is the register to read from.
//!
//! This function initiates a read from the slave device.
//! The I2C_PORT parameter is the I2C modules master base address.
//! \e I2C_PORT parameter can be one of the following values:
//!
//! \return Register value in an unsigned long format. Note that 0 will be
//! returned if there is ever an error, 1 if there was not.
//
//*****************************************************************************
uint8_t i2c_ReadByte( uint8_t SlaveID, uint8_t addr) {
unsigned long ulRegValue = 0;
	
	while(I2CMasterBusy(I2C_PORT))
  {
  };
	
	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);
	I2CMasterDataPut(I2C_PORT, addr);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);
	
	while(I2CMasterBusy(I2C_PORT))
	{
	};

	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}

	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 1);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_RECEIVE);

	while(I2CMasterBusy(I2C_PORT))
	{
	};
	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}

	ulRegValue = I2CMasterDataGet(I2C_PORT);

	return ulRegValue;
}


//*****************************************************************************
//
//! Writes to the specified I2C slave register.
//!
//! \param I2C_PORT is the base for the I2C module.
//! \param SlaveID is the 7-bit address of the slave to be addressed.
//! \param addr is the register to write data to.
//! \param data is the 8-bit data to be written.
//!
//! This function initiates a read from the I2C slave device.
//! The I2C_PORT parameter is the I2C modules master base address.
//! \e I2C_PORT parameter can be one of the following values:
//!
//! \return Register value in an unsigned long format. Note that 0 will be
//! returned if there is ever an error, 1 if there was not.
//
//int32_t i2c_WriteByte(uint8_t SlaveID, uint8_t addr, uint8_t data);
//*****************************************************************************
int32_t i2c_WriteByte(uint8_t SlaveID, uint8_t addr, uint8_t data) {

	while(I2CMasterBusy(I2C_PORT))
	{
	};

	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);
	I2CMasterDataPut(I2C_PORT, addr);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);
	
	while(I2CMasterBusy(I2C_PORT))
	{
	};
	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}
	
	I2CMasterDataPut(I2C_PORT, data);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_CONT);

	while(I2CMasterBusy(I2C_PORT))
	{
	};
	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}

	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_FINISH);

	while(I2CMasterBusy(I2C_PORT))
	{
	};

	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}
	
	return 1;
}


//*****************************************************************************
//
//! Reads one/multiple bytes of data from an I2C slave device.
//!
//! \param I2C_PORT is the base for the I2C module.
//! \param SlaveID is the 7-bit address of the slave to be addressed.
//! \param addr is the register to start reading from.
//! \param pBuf is a pointer to the array to store the data.
//! \param nBytes is the number of bytes to read from the slave.
//!
//! This function reads one/multiple bytes of data from an I2C slave device.
//! The I2C_PORT parameter is the I2C modules master base address.
//! \e I2C_PORT parameter can be one of the following values:
//!
//!
//! \return 0 if there was an error or 1 if there was not.
//
//int32_t i2c_RcvBuf(uint8_t SlaveID, uint8_t addr, int32_t nBytes, uint8_t* pBuf);
//*****************************************************************************
int32_t i2c_ReadBuf(uint8_t SlaveID, uint8_t addr, int32_t nBytes , uint8_t* pBuf ) {

uint8_t nBytesCount; // local variable used for byte counting/state determination
uint16_t MasterOptionCommand; // used to assign the commands for I2CMasterControl() function

	while(I2CMasterBusy(I2C_PORT))
	{
	};

	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);
	I2CMasterDataPut(I2C_PORT, addr);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);

	while(I2CMasterBusy(I2C_PORT))
	{
	};
	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)
	{
		return 0;
	}

	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, true);
	MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_START;

	for(nBytesCount = 0; nBytesCount < nBytes; nBytesCount++) {
		
		if(nBytesCount == 1)					MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
		if(nBytesCount == nBytes - 1)	MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
		if(nBytes == 1)								MasterOptionCommand = I2C_MASTER_CMD_SINGLE_RECEIVE;

		I2CMasterControl(I2C_PORT, MasterOptionCommand);

		while(I2CMasterBusy(I2C_PORT))
		{
		};

		if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE) {
			return 0;
		}
		
		pBuf[nBytesCount] = I2CMasterDataGet(I2C_PORT);
	}
	
	return nBytesCount;
}


//*****************************************************************************
//
//! Writes one/multiple bytes of data to an I2C slave device.
//! Ensure to use auto-increment options on some devices
//! (Control Registers, refer to data sheet).
//! I.e. store related command in the first position of your data array.
//!
//! \param I2C_PORT is the base for the I2C module.
//! \param SlaveID is the 7-bit address of the slave to be addressed.
//! \param addr is the register to start writing to.
//! \param pBuf is a pointer to the array to be send.
//! \param nBytes is the number of bytes to send from array pBuf[].
//!
//! This function writes multiple bytes of data an I2C slave device.
//! The I2C_PORT parameter is the I2C modules master base address.
//! \e I2C_PORT parameter can be one of the following values:
//!
//!
//! \return 0 if there was an error or 1 if there was not.

//int32_t i2c_XmtBuf(uint8_t SlaveID, uint8_t addr, int32_t nBytes, uint8_t* pBuf );
//
//*****************************************************************************

int32_t i2c_WriteBuf( uint8_t SlaveID, uint8_t addr, int32_t nBytes , uint8_t* pBuf) {

uint8_t nBytesCount; // local variable used for byte counting/state determination
uint16_t MasterOptionCommand; // used to assign the commands for I2CMasterControl() function

	while(I2CMasterBusy(I2C_PORT))
	{
	};

	I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, false);
	I2CMasterDataPut(I2C_PORT, addr);
	I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);

	while(I2CMasterBusy(I2C_PORT))
	{
	};
	
	if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE) {
		return 0;
	}

	MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;

	for(nBytesCount = 0; nBytesCount < nBytes; nBytesCount++) {
		
		if(nBytesCount == 1)					MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;
		if(nBytesCount == nBytes - 1)	MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_FINISH;
		if(nBytes == 1)								MasterOptionCommand = I2C_MASTER_CMD_SINGLE_SEND;

		I2CMasterDataPut(I2C_PORT, pBuf[nBytesCount]);
		I2CMasterControl(I2C_PORT, MasterOptionCommand);

		while(I2CMasterBusy(I2C_PORT))
		{
		};
		
		if(I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE) {
			return 0;
		}
	}
	
	return 1;
}
