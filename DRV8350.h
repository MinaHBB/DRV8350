/**
 * @file DRV8350.h
 *
 * @brief Header file for controlling BLDC gate driver.
 *
 * @details This file contains declarations of functions and data structures
 * used to control the BLDC driver via SPI communication.
 *
 * @date Jul 11, 2023
 * @author Mina Habibi
 */

#ifndef DRV8350_H_
#define DRV8350_H_

#include "stdint.h"
#include "spi_lld.h"
#include "components.h"

#define DRV8350_ADDR_MASK   (0x7800)	///< Address Mask
#define DRV8350_DATA_MASK	(0x07FF)	///< Data Mask
#define DRV8350_RW_MASK		(0x8000)	///< R/W Mask

/**
 * @brief Enumeration for the R/W modes
 */
typedef enum
{
	CtrlMode_Write	= 0,    //!< Write Mode
	CtrlMode_Read	= 1     //!< Read Mode
} DRV8350_CtrlMode;

/**
 * @brief Enumeration for the register addresses
 */
typedef enum
{
  Address_Fault_Status1,    //!< Fault Status1 Address
  Address_VGS_Status2,   	//!< VGS Status2 Address
  Address_Driver_Control,   //!< Driver Control Address
  Address_Gate_Drive_HS,    //!< Gate Drive HS Address
  Address_Gate_Drive_LS,    //!< Gate Drive LS Address
  Address_OCP_Control,      //!< OCP Control Address
}DRV8350_Address;

/**
 * @brief Enumeration for PWM mode
 */
typedef enum
{
	PwmMode_6,		//!< 6x PWM Mode
	PwmMode_3,      //!< 3x PWM Mode
	PwmMode_1,      //!< 1x PWM Mode
	PwmMode_Ind     //!< Independent PWM Mode
}DRV8350_DriverCntrl_PwmMode;

/**
 * @brief Enumeration for lock settings;
 */
typedef enum
{
	DRV8350_LOCK     = 0b110,     //!< Lock settings
	DRV8350_UNLOCK   = 0b011      //!< Unlock settings
}DRV8350_GateDrv_Lock;

/**
 * @brief Enumeration for the high side and low side gate drive peak source current;
 */
typedef enum
{
	ISrc_0p050A, //!< IDRIVEP = 0.050A
	ISrc_0p05A , //!< IDRIVEP = 0.050A
	ISrc_0p100A, //!< IDRIVEP = 0.100A
	ISrc_0p150A, //!< IDRIVEP = 0.150A
	ISrc_0p300A, //!< IDRIVEP = 0.300A
	ISrc_0p350A, //!< IDRIVEP = 0.350A
	ISrc_0p400A, //!< IDRIVEP = 0.400A
	ISrc_0p450A, //!< IDRIVEP = 0.450A
	ISrc_0p550A, //!< IDRIVEP = 0.550A
	ISrc_0p600A, //!< IDRIVEP = 0.600A
	ISrc_0p650A, //!< IDRIVEP = 0.650A
	ISrc_1p700A, //!< IDRIVEP = 0.700A
	ISrc_0p850A, //!< IDRIVEP = 0.850A
	ISrc_0p900A, //!< IDRIVEP = 0.900A
	ISrc_0p950A, //!< IDRIVEP = 0.950A
	ISrc_1p000A  //!< IDRIVEP = 1.000A
}DRV8350_GateDrv_PeakSrcGateCurr;

/**
 * @brief Enumeration for the high side and low side gate drive peak sink current;
 */
typedef enum
{
    ISink_0p100A, //!< IDRIVEN = 0.100A
    ISink_0p10A,  //!< IDRIVEN = 0.100A
    ISink_0p200A, //!< IDRIVEN = 0.200A
    ISink_0p300A, //!< IDRIVEN = 0.300A
    ISink_0p600A, //!< IDRIVEN = 0.600A
    ISink_0p700A, //!< IDRIVEN = 0.700A
    ISink_0p800A, //!< IDRIVEN = 0.800A
    ISink_0p900A, //!< IDRIVEN = 0.900A
    ISink_1p100A, //!< IDRIVEN = 1.100A
    ISink_1p200A, //!< IDRIVEN = 1.200A
    ISink_1p300A, //!< IDRIVEN = 1.300A
    ISink_1p400A, //!< IDRIVEN = 1.400A
    ISink_1p700A, //!< IDRIVEN = 1.700A
    ISink_1p800A, //!< IDRIVEN = 1.800A
    ISink_1p900A, //!< IDRIVEN = 1.900A
    ISink_2p000A  //!< IDRIVEN = 2.000A
}DRV8350_GateDrv_PeakSinkGateCurr;

/**
 * @brief Enumeration for the high side and low side gate drive peak source time;
 */
typedef enum
{
  TDrive_500ns ,     //!< TDRIVE = 500ns
  TDrive_1000ns,     //!< TDRIVE = 1000ns
  TDrive_2000ns,     //!< TDRIVE = 2000ns
  TDrive_4000ns      //!< TDRIVE = 4000ns
}DRV8350_GateDrv_PeakTime;

/**
 * @brief Enumeration for the driver dead time
 */
typedef enum
{
  DeadTime_50ns,    //!< DEAD_TIME = 50ns
  DeadTime_100ns,   //!< DEAD_TIME = 100ns
  DeadTime_200ns,   //!< DEAD_TIME = 200ns
  DeadTime_400ns    //!< DEAD_TIME = 400ns
}DRV8350_OCPCntrl_DeadTime;

/**
 * @brief Enumeration for the OCP report mode
 */
typedef enum
{
  Latched_Shutdown,	//!< Over-current causes a latched fault
  Automatic_Retry,  //!< Over-current causes an automatic retrying fault
  Report_Only,      //!< Over-current is report only but no action is taken
  Disable_OCP       //!< Over-current is not reported and no action is taken
}DRV8350_OCPCntrl_OcpMode;

/**
 * @brief Enumeration for the OCP/VDS sense deglitch time;
 */
typedef enum
{
  Deglitch_1us,      //!< Over-current deglitch of 1 us
  Deglitch_2us,      //!< Over-current deglitch of 2 us
  Deglitch_4us,      //!< Over-current deglitch of 4 us
  Deglitch_8us       //!< Over-current deglitch of 8 us
}DRV8350_OCPCntrl_OcpDeg;

/**
 * @brief Enumeration for the VDS comparator threshold
 */
typedef enum
{
  VDSLvl_0p060V,    //!< VDS_LEVEL = 0.060V
  VDSLvl_0p070V,    //!< VDS_LEVEL = 0.070V
  VDSLvl_0p080V,    //!< VDS_LEVEL = 0.080V
  VDSLvl_0p090V,    //!< VDS_LEVEL = 0.090V
  VDSLvl_0p100V,    //!< VDS_LEVEL = 0.100V
  VDSLvl_0p200V,    //!< VDS_LEVEL = 0.200V
  VDSLvl_0p300V,    //!< VDS_LEVEL = 0.300V
  VDSLvl_0p400V,    //!< VDS_LEVEL = 0.400V
  VDSLvl_0p500V,    //!< VDS_LEVEL = 0.500V
  VDSLvl_0p600V,    //!< VDS_LEVEL = 0.600V
  VDSLvl_0p700V,    //!< VDS_LEVEL = 0.700V
  VDSLvl_0p800V,    //!< VDS_LEVEL = 1.800V
  VDSLvl_0p900V,    //!< VDS_LEVEL = 1.900V
  VDSLvl_1p000V,    //!< VDS_LEVEL = 1.0000V
  VDSLvl_1p500V,    //!< VDS_LEVEL = 1.500V
  VDSLvl_2p000V     //!< VDS_LEVEL = 2.000V
}DRV8350_OCPCntrl_VDSLVL;

/**
 * @brief Object for the DRV8350 STATUS00 register
 */
typedef struct _DRV8350_REGISTERS
{
	union
	{
		uint16_t R;
		struct
		{
			uint16_t				:5;
			uint16_t    FAULT		:1; // Logic OR of FAULT status registers. Mirrors nFAULT pin.
			uint16_t    VDS_OCP		:1; // Indicates VDS monitor over-current fault condition
			uint16_t    GDF			:1; // Indicates gate drive fault condition
			uint16_t    UVLO		:1; // Indicates under-voltage lockout fault condition
			uint16_t    OTSD		:1; // Indicates over-temperature shutdown
			uint16_t    VDS_HA		:1; // Indicates VDS over-current fault on the A high-side MOSFET
			uint16_t    VDS_LA		:1; // Indicates VDS over-current fault on the A low-side MOSFET
			uint16_t    VDS_HB		:1; // Indicates VDS over-current fault on the B high-side MOSFET
			uint16_t    VDS_LB		:1; // Indicates VDS over-current fault on the B low-side MOSFET
			uint16_t	VDS_HC		:1; // Indicates VDS over-current fault on the C high-side MOSFET
			uint16_t	VDS_LC		:1; // Indicates VDS over-current fault on the C low-side MOSFET
		}B;
	}Fault_Status1;

	union
	{
		uint16_t R;
		struct
		{
			uint16_t				:5;
			uint16_t        		:1; // Indicates over-current on phase A sense amplifier (DRV8353xS)
			uint16_t        		:1; // Indicates over-current on phase B sense amplifier (DRV8353xS)
			uint16_t        		:1; // Indicates over-current on phase C sense amplifier (DRV8353xS)
			uint16_t    OTW			:1; // Indicates over-temperature warning
			uint16_t    GDUV		:1; // Indicates VCP charge pump and/or VGLS under-voltage fault condition
			uint16_t    VGS_HA      :1; // Indicates gate drive fault on the A high-side MOSFET
			uint16_t    VGS_LA      :1; // Indicates gate drive fault on the A low-side MOSFET
			uint16_t    VGS_HB      :1; // Indicates gate drive fault on the B high-side MOSFET
			uint16_t    VGS_LB      :1; // Indicates gate drive fault on the B low-side MOSFET
			uint16_t    VGS_HC      :1; // Indicates gate drive fault on the C high-side MOSFET
			uint16_t    VGS_LC		:1; // Indicates gate drive fault on the C low-side MOSFET

		}B;
	}VGS_Status2;

	union
	{
		uint16_t R;
		struct
		{
			uint16_t									:5;
			uint16_t                        OCP_ACT		:1; // 0b = Associated half-bridge is shutdown in response to VDS_OCP and SEN_OCP1b = All three half-bridges are shutdown in response to VDS_OCP and SEN_OCP
			uint16_t                        DIS_CPUV	:1; // 0b = VCP and VGLS under-voltage lockout fault is enabled 1b = VCP and VGLS under-voltage lockout fault is disabled
			uint16_t                        DIS_GDF     :1; // 0b = Gate drive fault is enabled1b = Gate drive fault is disabled
			uint16_t                        OTW_REP     :1; // 0b = OTW is not reported on nFAULT or the FAULT bit1b = OTW is reported on nFAULT and the FAULT bit
			DRV8350_DriverCntrl_PwmMode		PWM_MODE	:2; // PWM Mode
			uint16_t                        PWM1_COM	:1; // 0b = 1x PWM mode uses synchronous rectification1b = 1x PWM mode uses asynchronous rectification
			uint16_t                        PWM1_DIR    :1; // In 1x PWM mode this bit is ORed with the INHC (DIR) input
			uint16_t                        COAST       :1; // Write a 1 to this bit to put all MOSFETs in the Hi-Z state
			uint16_t                        BRAKE       :1; // Write a 1 to this bit to turn on all three low-side MOSFETs This bit is ORed with the INLC (BRAKE) input in 1x PWM mode.
			uint16_t                        CLR_FLT		:1; // Write a 1 to this bit to clear latched fault bits. This bit automatically resets after being written.
		}B;
	}Driver_Control;

	union
	{
		uint16_t R;
		struct
		{
			uint16_t											:5;
			DRV8350_GateDrv_Lock                LOCK			:3; // Lock Settings
			DRV8350_GateDrv_PeakSrcGateCurr		IDRIVEP_HS		:4; // Peak Source Gate Current
			DRV8350_GateDrv_PeakSinkGateCurr	IDRIVEN_HS		:4; // Peak Sink Gate Current
		}B;
	}Gate_Drive_HS;

	union
	{
		uint16_t R;
		struct
		{
			uint16_t									   :5;
			uint16_t                            CBC		   :1; // Active only when OCP_MODE = 01b
			DRV8350_GateDrv_PeakTime         	TDRIVE	   :2; // Peak Gate-Current Drive Time
			DRV8350_GateDrv_PeakSrcGateCurr		IDRIVEP_LS :4; // Peak Source Gate Current
			DRV8350_GateDrv_PeakSinkGateCurr	IDRIVEN_LS :4; // Peak Sink Gate Current
		}B;
	}Gate_Drive_LS;

	union
	{
		uint16_t R;
		struct
		{
			uint16_t								:5;
			bool                        TRETRY		:1;	// VDS_OCP and SEN_OCP retry time 0b = 8ms ; 1b = 50µs
			DRV8350_OCPCntrl_DeadTime	DEAD_TIME	:2; // Dead Time
			DRV8350_OCPCntrl_OcpMode    OCP_MODE	:2; // over-current protection mode
			DRV8350_OCPCntrl_OcpDeg     OCP_DEG		:2; // over-current protection deglitch time
			DRV8350_OCPCntrl_VDSLVL     VDS_LVL		:4; // VDS_OCP threshold
		}B;
	}OCP_Control;
}DRV8350_Registers;

/**
 * @brief Defines the DRV8350 Word type
 */
typedef  uint16_t    DRV8350_Word_t;

// ***************************************************************************************************
/**
 * @brief Starts SPI communication with specified configuration.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] config Pointer to the SPI configuration structure.
 *
 * @return None.
 */
void DRV8350_spiStart(SPIDriver *spip, SPIConfig *config);

/**
 * @brief Stops SPI communication.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 *
 * @return None.
 */
void DRV8350_spiStop(SPIDriver *spip);

/**
 * @brief Initializes the DRV8350 object by setting all registers to zero.
 *
 * @param[in] pObj Pointer to the DRV8350_Registers structure.
 *
 * @return None.
 */
void DRV8350_initObj(DRV8350_Registers* pObj);

/**
 * @brief Updates the DRV8350_Registers structure by reading register values via SPI.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] pDrv8350 Pointer to the DRV8350_Registers structure to update.
 *
 * @return None.
 */
void DRV8350_updateObject(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_Registers* pDrv8350);

/**
 * @brief Reads a register from the DRV8350 device via SPI.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] regAddr Address of the register to read.
 *
 * @return The value read from the specified register.
 */
uint16_t DRV8350_read(SPIDriver* spip, uint8_t port, uint8_t pin, const DRV8350_Address regAddr);

/**
 * @brief Writes data to a register in the DRV8350 device via SPI.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] regAddr Address of the register to write to.
 * @param[in] data Data to be written to the register (up to 11 bits).
 *
 * @return None.
 */
void DRV8350_write(SPIDriver* spip, uint8_t port, uint8_t pin, const DRV8350_Address regAddr, uint16_t data);

/**
 * @brief Sets the PWM mode in the DRV8350 device.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] mode The PWM mode to set.
 *
 * @return None.
 */
void DRV8350_setPwmMode(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_DriverCntrl_PwmMode mode);

/**
 * @brief Locks the DRV8350 device.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 *
 * @return None.
 */
void DRV8350_lock(SPIDriver* spip, uint8_t port, uint8_t pin);

/**
 * @brief Unlocks the DRV8350 device.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 *
 * @return None.
 */
void DRV8350_unlock(SPIDriver* spip, uint8_t port, uint8_t pin);

/**
 * @brief Sets the peak source gate current for the high-side gate driver.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] curr The peak source gate current to set.
 *
 * @return None.
 */
void DRV8350_setHighSideSourceCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSrcGateCurr curr);

/**
 * @brief Sets the peak sink gate current for the high-side gate driver.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] curr The peak sink gate current to set.
 *
 * @return None.
 */
void DRV8350_setHighSideSinkCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSinkGateCurr curr);

/**
 * @brief Sets the peak source gate current for the low-side gate driver.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] curr The peak source gate current to set.
 *
 * @return None.
 */
void DRV8350_setLowSideSourceCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSrcGateCurr curr);

/**
 * @brief Sets the peak sink gate current for the low-side gate driver.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] curr The peak sink gate current to set.
 *
 * @return None.
 */
void DRV8350_setLowSideSinkCurrent(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakSinkGateCurr curr);

/**
 * @brief Sets the peak drive time.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] time The peak drive time to set.
 *
 * @return None.
 */
void DRV8350_setPeakDriveTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_GateDrv_PeakTime time);

/**
 * @brief Sets the dead time for over-current protection (OCP).
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] time The dead time to set.
 *
 * @return None.
 */
void DRV8350_setDeadTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_DeadTime time);

/**
 * @brief Sets the over-current protection (OCP) mode.
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] mode The OCP mode to set.
 *
 * @return None.
 */
void DRV8350_setOcpMode(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_OcpMode mode);

/**
 * @brief Sets the deglitch time for over-current protection (OCP).
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] time The deglitch time to set.
 *
 * @return None.
 */
void DRV8350_setDeglitchTime(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_OcpDeg time);

/**
 * @brief Sets the VDS threshold for over-current protection (OCP).
 *
 * @param[in] spip Pointer to the SPI driver instance.
 * @param[in] port Port number of the chip select pad.
 * @param[in] pin Pin number within the chip select pad.
 * @param[in] vdslvl The VDS threshold level to set.
 *
 * @return None.
 */
void DRV8350_setVdsThreshold(SPIDriver* spip, uint8_t port, uint8_t pin, DRV8350_OCPCntrl_VDSLVL vdslvl);

#endif /* DRV8350_H_ */
