/**
 * @file DRV8350.c
 *
 * @brief Source file for controlling BLDC gate driver.
 *
 * @details This file contains implementations of functions used to control the
 * BLDC driver via SPI communication.
 *
 * @date Jul 11, 2023
 * @author Mina Habibi
 */

#include "DRV8350.h"

//---------------------------------------HAL Functions---------------------------------------
static uint16_t DRV8350_exchangeSpi(SPIDriver *spip, uint16_t frame)
{
	return (spi_lld_polled_exchange(spip, frame));
}

void DRV8350_spiStart(SPIDriver *spip, SPIConfig *config)
{
	spi_lld_start(spip, config);
}

void DRV8350_spiStop(SPIDriver *spip)
{
	spi_lld_stop(spip);
}

static void DRV8350_clearChipSelect(uint8_t port, uint8_t pin)
{
	pal_lld_clearpad(port, pin);
}

static void DRV8350_setChipSelect(uint8_t port, uint8_t pin)
{
	pal_lld_setpad(port, pin);
}

static void DRV8350_delay(uint32_t usec)
{
	osalThreadDelayMicroseconds(usec);
}
//-------------------------------------------------------------------------------------------
/**
 * @brief Constructs the control word for communication with the DRV8350 driver.
 *
 * This function constructs the control word for communication with the DRV8350 driver
 * based on the control mode, register address, and data.
 *
 * @param[in] ctrlMode Control mode for the communication.
 * @param[in] regAddr Register address to access.
 * @param[in] data Data to be written or read.
 *
 * @return Control word constructed for the DRV8350 driver.
 */
static inline DRV8350_Word_t DRV8350_buildCtrlWord(const DRV8350_CtrlMode ctrlMode, const DRV8350_Address regAddr, const uint16_t data)
{
	DRV8350_Word_t ctrlWord = (ctrlMode << 15) | (regAddr << 11) | (data & DRV8350_DATA_MASK);

	return (ctrlWord);
}

void DRV8350_initObj(DRV8350_Registers* pObj)
{
	// Initialize registers
	pObj->Fault_Status1.R = 0;
	pObj->VGS_Status2.R = 0;
	pObj->Driver_Control.R = 0;
	pObj->Gate_Drive_HS.R = 0;
	pObj->Gate_Drive_LS.R = 0;
	pObj->OCP_Control.R = 0;
}

void DRV8350_updateObject(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_Registers* pDrv8350)
{
	pDrv8350->Fault_Status1.R 	= DRV8350_read(spip, port, pin, Address_Fault_Status1);
	DRV8350_delay(1);
	pDrv8350->VGS_Status2.R 	= DRV8350_read(spip, port, pin, Address_VGS_Status2);
	DRV8350_delay(1);
	pDrv8350->Driver_Control.R 	= DRV8350_read(spip, port, pin, Address_Driver_Control);
	DRV8350_delay(1);
	pDrv8350->Gate_Drive_HS.R 	= DRV8350_read(spip, port, pin, Address_Gate_Drive_HS);
	DRV8350_delay(1);
	pDrv8350->Gate_Drive_LS.R 	= DRV8350_read(spip, port, pin, Address_Gate_Drive_LS);
	DRV8350_delay(1);
	pDrv8350->OCP_Control.R 	= DRV8350_read(spip, port, pin, Address_OCP_Control);
}

uint16_t DRV8350_read(SPIDriver* spip, uint8_t port, uint8_t pin, const DRV8350_Address regAddr)
{
	uint16_t readWord = 0;

	// build the control word
	DRV8350_Word_t ctrlWord = DRV8350_buildCtrlWord(CtrlMode_Read, regAddr, 0);

	// exchange data
	DRV8350_clearChipSelect(port, pin);
	readWord = DRV8350_exchangeSpi(spip, ctrlWord);
	DRV8350_setChipSelect(port, pin);

	return (readWord & DRV8350_DATA_MASK);
}

void DRV8350_write(SPIDriver* spip, uint8_t port, uint8_t pin, const DRV8350_Address regAddr, uint16_t data)
{
	if (data > 0x7FF)
	{
		return;
	}

	// build the control word
	DRV8350_Word_t ctrlWord = DRV8350_buildCtrlWord(CtrlMode_Write, regAddr, data);

	// exchange data
	DRV8350_clearChipSelect(port, pin);
	DRV8350_exchangeSpi(spip, ctrlWord);
	DRV8350_setChipSelect(port, pin);
}

void DRV8350_setPwmMode(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_DriverCntrl_PwmMode mode)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Driver_Control);
	data = (data & 0x79F) | (mode << 5);
	DRV8350_write(spip, port, pin, Address_Driver_Control, data);
}

void DRV8350_lock(SPIDriver* spip, uint8_t port, uint8_t pin)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_HS);
	data = (data & 0xFF) | (DRV8350_LOCK << 8);
	DRV8350_write(spip, port, pin, Address_Gate_Drive_HS, data);
}

void DRV8350_unlock(SPIDriver* spip, uint8_t port, uint8_t pin)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_HS);
	data = (data & 0xFF) | (DRV8350_UNLOCK << 8);
	DRV8350_write(spip, port, pin, Address_Gate_Drive_HS, data);
}

void DRV8350_setHighSideSourceCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSrcGateCurr curr)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_HS);
	data = (data & 0x70F) | (curr << 4);
	DRV8350_write(spip, port, pin, Address_Gate_Drive_HS, data);
}

void DRV8350_setHighSideSinkCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSinkGateCurr curr)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_HS);
	data = (data & 0x7F0) | (curr) ;
	DRV8350_write(spip, port, pin, Address_Gate_Drive_HS, data);
}

void DRV8350_setLowSideSourceCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSrcGateCurr curr)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_LS);
	data = (data & 0x70F) | (curr << 4);
	DRV8350_write(spip, port, pin, Address_Gate_Drive_LS, data);
}

void DRV8350_setLowSideSinkCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSinkGateCurr curr)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_LS);
	data = (data & 0x7F0) | (curr) ;
	DRV8350_write(spip, port, pin, Address_Gate_Drive_LS, data);
}

void DRV8350_setPeakDriveTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakTime time)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_Gate_Drive_LS);
	data = (data & 0x4FF) | (time << 8);
	DRV8350_write(spip, port, pin, Address_Gate_Drive_LS, data);
}

void DRV8350_setDeadTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_DeadTime time)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_OCP_Control);
	data = (data & 0x4FF) | (time << 8);
	DRV8350_write(spip, port, pin, Address_OCP_Control, data);
}

void DRV8350_setOcpMode(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_OcpMode mode)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_OCP_Control);
	data = (data & 0x73F) | (mode << 6);
	DRV8350_write(spip, port, pin, Address_OCP_Control, data);
}

void DRV8350_setDeglitchTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_OcpDeg time)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_OCP_Control);
	data = (data & 0x7CF) | (time << 4);
	DRV8350_write(spip, port, pin, Address_OCP_Control, data);
}

void DRV8350_setVdsThreshold(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_VDSLVL vdslvl)
{
	uint16_t data = DRV8350_read(spip, port, pin, Address_OCP_Control);
	data = (data & 0x7F0) | (vdslvl);
	DRV8350_write(spip, port, pin, Address_OCP_Control, data);
}

// end of file
