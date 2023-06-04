/* Includes ------------------------------------------------------------------*/
#include "rc522.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "spi.h"

/* Private defines -----------------------------------------------------------*/
#define RESET_ModWidthReg 0x26
#define RESET_TxModeReg   0x00
#define RESET_RxModeReg   0x00

#define RC522_FIFO_SIZE 64

// defines to TModeReg, TPrescalerReg, TReloadReg:
#define RC522_F_CLK 13560000 // Hz
#define RC522_F_TIMER 40000 // Hz
// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=13.56 MHz/(2*169+1)=40kHz, ie a timer period of 25μs.
#define T_PRESCALER (uint16_t)(((RC522_F_CLK / RC522_F_TIMER) - 1) / 2) // Section 9.3.3.10
#define T_RELOAD_VAL 1000 // Reload timer with 1000, ie 25ms before timeout.


// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef enum
{
  RC522_Idle =             0x00, // no action, cancels current command execution
  RC522_Mem =              0x01, // stores 25 bytes into the internal buffer
  RC522_GenerateRandomID = 0x02, // generates a 10-byte random ID number
  RC522_CalcCRC =          0x03, // activates the CRC coprocessor or performs a self-test
  RC522_Transmit =         0x04, // transmits data from the FIFO buffer
  RC522_NoCmdChange =      0x07, // no command change, can be used to modify the
                               // CommandReg register bits without affecting the command,
                               // for example, the PowerDown bit
  RC522_Receive =          0x08, // activates the receiver circuits
  RC522_Transceive =       0x0C, // transmits data from FIFO buffer to antenna and automatically
                               // activates the receiver after transmission
  RC522_MFAuthent =        0x0E, // performs the MIFARE standard authentication as a reader
  RC522_SoftReset =        0x0F	 // resets the MFRC522
} RC522_Command;

// MFRC522 registers. Described in chapter 9 of the datasheet.
// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
typedef enum
{
  // Page 0: Command and status
  //                  0x00  // reserved for future use
  CommandReg =        0x01, // starts and stops command execution
  ComIEnReg =         0x02, // enable and disable interrupt request control bits
  DivIEnReg =         0x03, // enable and disable interrupt request control bits
  ComIrqReg =         0x04, // interrupt request bits
  DivIrqReg =         0x05, // interrupt request bits
  ErrorReg =          0x06, // error bits showing the error status of the last command executed 
  Status1Reg =        0x07, // communication status bits
  Status2Reg =        0x08, // receiver and transmitter status bits
  FIFODataReg =       0x09, // input and output of 64 byte FIFO buffer
  FIFOLevelReg =      0x0A, // number of bytes stored in the FIFO buffer
  WaterLevelReg =     0x0B, // level for FIFO underflow and overflow warning
  ControlReg =        0x0C, // miscellaneous control registers
  BitFramingReg =     0x0D, // adjustments for bit-oriented frames
  CollReg =           0x0E, // bit position of the first bit-collision detected on the RF interface
  //                  0x0F  // reserved for future use
		
  // Page 1: Command
  //                  0x10  // reserved for future use
  ModeReg =           0x11, // defines general modes for transmitting and receiving 
  TxModeReg =         0x12, // defines transmission data rate and framing
  RxModeReg =         0x13, // defines reception data rate and framing
  TxControlReg =      0x14, // controls the logical behavior of the antenna driver pins TX1 and TX2
  TxASKReg =          0x15, // controls the setting of the transmission modulation
  TxSelReg =          0x16, // selects the internal sources for the antenna driver
  RxSelReg =          0x17, // selects internal receiver settings
  RxThresholdReg =    0x18, // selects thresholds for the bit decoder
  DemodReg =          0x19, // defines demodulator settings
  //                  0x1A  // reserved for future use
  //                  0x1B  // reserved for future use
  MfTxReg =           0x1C, // controls some MIFARE communication transmit parameters
  MfRxReg =           0x1D, // controls some MIFARE communication receive parameters
  //                  0x1E  // reserved for future use
  SerialSpeedReg =    0x1F, // selects the speed of the serial UART interface
		
  // Page 2: Configuration
  //                  0x20  // reserved for future use
  CRCResultRegH =     0x21, // shows the MSB and LSB values of the CRC calculation
  CRCResultRegL =     0x22,
  //                  0x23  // reserved for future use
  ModWidthReg =       0x24, // controls the ModWidth setting
  //                  0x25  // reserved for future use
  RFCfgReg =          0x26, // configures the receiver gain
  GsNReg =            0x27, // selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
  CWGsPReg =          0x28, // defines the conductance of the p-driver output during periods of no modulation
  ModGsPReg =         0x29, // defines the conductance of the p-driver output during periods of modulation
  TModeReg =          0x2A, // defines settings for the internal timer
  TPrescalerReg =     0x2B, // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
  TReloadRegH =       0x2C, // defines the 16-bit timer reload value
  TReloadRegL =       0x2D,
  TCounterValueRegH = 0x2E, // shows the 16-bit timer value
  TCounterValueRegL = 0x2F,
		
  // Page 3: Test Registers
  //                  0x30  // reserved for future use
  TestSel1Reg =       0x31, // general test signal configuration
  TestSel2Reg =       0x32, // general test signal configuration
  TestPinEnReg =      0x33, // enables pin output driver on pins D1 to D7
  TestPinValueReg =   0x34, // defines the values for D1 to D7 when it is used as an I/O bus
  TestBusReg =        0x35, // shows the status of the internal test bus
  AutoTestReg =       0x36, // controls the digital self-test
  VersionReg =        0x37, // shows the software version
  AnalogTestReg =     0x38, // controls the pins AUX1 and AUX2
  TestDAC1Reg =       0x39, // defines the test value for TestDAC1
  TestDAC2Reg =       0x3A, // defines the test value for TestDAC2
  TestADCReg =        0x3B  // shows the value of ADC I and Q channels
  //                  0x3C  // reserved for production tests
  //                  0x3D  // reserved for production tests
  //                  0x3E  // reserved for production tests
  //                  0x3F  // reserved for production tests
} RC522_Register;

typedef enum
{
  //               0 // reserved
  //               1 // reserved
  CRCIRq_Msk = (1 << 2),
  //               3 // reserved
  MfinActIRq_Msk = (1 << 4),
  //               5 // reserved
  //               6 // reserved
  Set2_Msk = (1 << 7)
} RC522_DivIrqReg;

typedef enum
{
  FIFOLevel_Msk = 0x7F, // bitmask for [6:0]
  FlushBuffer_Msk = (1 << 7)
} RC522_FIFOLevelReg;

typedef enum
{
  Command_Msk = 0x0F, // bitmask for [3:0]
  PowerDown_Msk = (1 << 4),
  RcvOff_Msk = (1 << 5)
  //               6 // reserved
  //               7 // reserved
} RC522_CommandReg;

typedef enum
{
  Tx1RFEn_Msk = (1 << 0),
  Tx2RFEn_Msk = (1 << 1),
  //               2 // reserved
  Tx2CW_Msk = (1 << 3),
  InvTx1RFOff_Msk = (1 << 4),
  InvTx2RFOff_Msk = (1 << 5),
  InvTx1RFOn_Msk = (1 << 6),
  InvTx2RFOn_Msk = (1 << 7)
} RC522_TxControlReg;

typedef enum
{
  CRCPreset0000h_Msk = 0x00,
  CRCPreset6363h_Msk = 0x01,
  CRCPresetA671h_Msk = 0x02,
  CRCPresetFFFFh_Msk = 0x03,
  //               2 // reserved
  PolMFin_Msk = (1 << 3),
  //               4 // reserved
  TxWaitRF_Msk = (1 << 5),
  //               6 // reserved
  MSBFirst_Msk = (1 << 7)
} RC522_ModeReg;

typedef enum
{
  TPrescaler_Hi_Msk = 0x0F, // bitmask for [3:0]
  TAutoRestart_Msk = (1 << 4),
  TGated_Msk = 0x60, // bitmask for [6:5]
  TAuto_Msk = (1 << 7)
} RC522_TModeReg;

typedef enum
{
  //          5 to 0 // reserved
  Force100ASK_Msk = (1 << 6)
  //               7 // reserved
} RC522_TxASKReg;

typedef enum
{
  TxLastBits_Msk = 0x07, // bitmask for [2:0]
  //               3 // reserved
  RxAlign_Msk = 0x70, // bitmask for [6:4]
  StartSend_Msk = (1 << 7)
} RC522_BitFramingReg;

typedef enum
{
  CollPos_Msk = 0x0F, // bitmask for [4:0]
  CollPosNotValid_Msk = (1 << 5),
  //               6 // reserved
  ValuesAfterColl_Msk = (1 << 7)
} RC522_CollReg;

typedef enum
{
  ModemState_Msk = 0x07, // bitmask for [2:0]
  MFCrypto1On_Msk = (1 << 3),
  //               4 // reserved
  //               5 // reserved
  I2CForceHS_Msk = (1 << 6),
  TempSensClear_Msk = (1 << 7)
} RC522_Status2Reg;

typedef enum
{
  TimerIRq_Msk = (1 << 0),
  ErrIRq_Msk = (1 << 1),
  LoAlertIRq_Msk = (1 << 2),
  HiAlertIRq_Msk = (1 << 3),
  IdleIRq_Msk = (1 << 4),
  RxIRq_Msk = (1 << 5),
  TxIRq_Msk = (1 << 6),
  Set1_Msk = (1 << 7)
} RC522_ComIrqReg;

typedef enum
{
  ProtocolErr_Msk = (1 << 0),
  ParityErr_Msk = (1 << 1),
  CRCErr_Msk = (1 << 2),
  CollErr_Msk = (1 << 3),
  BufferOvfl_Msk = (1 << 4),
  //               5 // reserved
  TempErr_Msk = (1 << 6),
  WrErr_Msk = (1 << 7)
} RC522_ErrorReg;

typedef enum
{
  RxLastBits_Msk = 0x07, // bitmask for [2:0]
  //               3 // reserved
  //               4 // reserved
  //               5 // reserved
  TStartNow_Msk = (1 << 6),
  TStopNow_Msk = (1 << 7)
} RC522_ControlReg;

typedef enum
{
  SelfTest_Disable = 0x00,
  SelfTest_Enable =  0x09
} RC522_SelfTest;


// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const uint8_t MFRC522_FirmwareReferenceV00[] = {
  0x00, 0x87, 0x98, 0x0F, 0x49, 0xFF, 0x07, 0x19,
  0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
  0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
  0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
  0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
  0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
  0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
  0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_FirmwareReferenceV10[] = {
  0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
  0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
  0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
  0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
  0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
  0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
  0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
  0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_FirmwareReferenceV20[] = {
  0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
  0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
  0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
  0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
  0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
  0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
  0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
  0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const uint8_t FM17522_FirmwareReference88[] = {
  0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
  0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
  0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
  0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
  0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
  0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
  0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
  0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};
// Another "FM17522" ic form Aliexpress (0xB2 or 0x92)
const uint8_t FM17522_FirmwareReferenceB2[] = {
  0x00, 0xEB, 0x44, 0x85, 0xFA, 0x9A, 0x78, 0x01,
  0x74, 0xE5, 0x1C, 0x7A, 0x0A, 0xA0, 0x71, 0xE1,
  0xF3, 0xFA, 0x96, 0x6D, 0x28, 0xA1, 0x34, 0x46,
  0x3A, 0x1C, 0x32, 0x96, 0xB9, 0xE6, 0x44, 0x87,
  0x0A, 0x45, 0x98, 0xA9, 0x36, 0x60, 0x89, 0x0F,
  0x06, 0x9B, 0x7B, 0x17, 0xB3, 0x0C, 0x1A, 0x6C,
  0x1A, 0xAE, 0x2C, 0xAC, 0x0E, 0x6F, 0x2E, 0x02,
  0x2B, 0xCB, 0x8A, 0xB2, 0x45, 0xDD, 0x7E, 0x3C
};
// Fudan Semiconductor FM17522E (0x89)
const uint8_t FM17522E_FirmwareReference[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x00, 0x04, 0xCC, 0xC8, 0x00,
  0x10, 0x04, 0x00, 0xC0, 0x00, 0x90, 0x00, 0x20,
  0x00, 0x00, 0x23, 0x00, 0x38, 0x06, 0x01, 0x33,
  0x98, 0xF3, 0x80, 0x06, 0xC0, 0xF9, 0x80, 0x08,
  0x27, 0x04, 0x23, 0x82, 0x21, 0x12, 0xF9, 0xC7
};

/* Variables -----------------------------------------------------------------*/
extern SPI_HandleTypeDef RC522_HSPI;
// A struct used for passing the UID of a PICC.
typedef struct
{
  size_t size; // Number of bytes in the UID. 4, 7 or 10.
  uint8_t uidByte[10];
  uint8_t sak; // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

static Uid uid_; // Used by RC522_PICC_ReadCardUid().


/*############################################################################*/
// Basic interface functions for communicating with the MFRC522
/*############################################################################*/

static void RC522_Select(void)
{
  HAL_GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
}

static void RC522_Deselect(void)
{
  HAL_GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
}

/**
  * @brief  Writes a byte to the specified register in the MFRC522 chip.
  * @note   The interface is described in the datasheet section 8.1.2.
  * @param  reg : The register to write to. One of the RC522_Register enums.
  * @param  value : The value to write.
  */
static void RC522_WriteRegister(const RC522_Register reg, const uint8_t value)
{
  RC522_Select();
  // Address format: 0XXXXXX0
  SPI_TransmitByte(&RC522_HSPI, (reg << 1) & 0x7E);
  SPI_TransmitByte(&RC522_HSPI, value);
  RC522_Deselect();
}

/**
  * @brief  Writes an array of bytes to the specified register in the MFRC522 chip.
  * @note   The interface is described in the datasheet section 8.1.2.
  * @param  reg : The register to write to. One of the RC522_Register enums.
  * @param  buff : The values to write. Byte array.
  * @param  size : The number of bytes to write to the register
  */
static void RC522_WriteBuffer(const RC522_Register reg, uint8_t *const buff, const size_t size)
{
  RC522_Select();
  // Address format: 0XXXXXX0
  SPI_TransmitByte(&RC522_HSPI, (reg << 1) & 0x7E);
  SPI_TransmitBuffer(&RC522_HSPI, buff, size);
  RC522_Deselect();
}

/**
  * @brief  Reads a byte from the specified register in the MFRC522 chip.
  * @note   The interface is described in the datasheet section 8.1.2.
  * @param  reg : The register to read from. One of the RC522_Register enums.
  * @retval The read value.
  */
static uint8_t RC522_ReadRegister(const RC522_Register reg)
{
  RC522_Select();
  // Address Format: 1XXXXXX0
  SPI_TransmitByte(&RC522_HSPI, ((reg << 1) & 0x7E) | 0x80);
  // Read the value back. Send 0 to stop reading:
  const uint8_t receivedValue = SPI_TransmitReceiveByte(&RC522_HSPI, 0x00); 
  RC522_Deselect();
  return receivedValue;	
}

/**
  * @brief  Reads a byte from the specified register in the MFRC522 chip.
  * @note   The interface is described in the datasheet section 8.1.2.
  * @param  reg : The register to read from. One of the RC522_Register enums.
  * @param  buff : Byte array to store the values in.
  * @param  size : The number of bytes to read.
  * @param  rxAlign : Only bit positions rxAlign..7 in values[0] are updated.
  */
static void RC522_ReadBuffer(const RC522_Register reg, uint8_t *const buff, const size_t size, const uint8_t rxAlign)
{
  if (size == 0) 
  {
    return;
  }
  size_t index = 0; // Index in buff array.
  // Address Format: 1XXXXXX0
  const uint8_t address = ((reg << 1) & 0x7E) | 0x80;
  RC522_Select();
  SPI_TransmitByte(&RC522_HSPI, address);
  if (rxAlign) 
  { // Only update bit positions rxAlign..7 in buff[0]
    const uint8_t mask  = (uint8_t)(0xFF << rxAlign);
    // Read value and tell that we want to read the same address again.
    const uint8_t value = SPI_TransmitReceiveByte(&RC522_HSPI, address); 
    // Apply mask to both current value of buff[0] and the new data in value.
    buff[0] = (buff[0] & ~mask) | (value & mask);
    index++;
  }
  for (; index < size - 1; index++)
  { // Read value and tell that we want to read the same address again.
    buff[index] = SPI_TransmitReceiveByte(&RC522_HSPI, address); 
  }
  // Read the final byte. Send 0 to stop reading:
  buff[index] = SPI_TransmitReceiveByte(&RC522_HSPI, 0x00);
  RC522_Deselect();
}

/**
  * @brief  Sets the bits given in mask in register reg.
  * @param  reg : The register to update. One of the RC522_Register enums.
  * @param  mask : The bits to set.
  */
static void RC522_SetRegisterBitMask(const RC522_Register reg, const uint8_t mask)
{
  const uint8_t regValue = RC522_ReadRegister(reg);
  if ((regValue & mask) != mask)
  {
    RC522_WriteRegister(reg, regValue | mask);
  }
}

/**
  * @brief  Clears the bits given in mask from register reg.
  * @param  reg : The register to update. One of the RC522_Register enums.
  * @param  mask : The bits to clear.
  */
static void RC522_ClearRegisterBitMask(const RC522_Register reg, const uint8_t mask)
{
  const uint8_t regValue = RC522_ReadRegister(reg);
  if ((regValue & mask) != 0)
  {
    RC522_WriteRegister(reg, regValue & (~mask));
  }
}

/**
  * @brief  Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
  * @param  data : Pointer to the data to transfer to the FIFO for CRC calculation.
  * @param  length : The number of bytes to transfer.
  * @param  result : Pointer to result buffer. Result is written to result[0..1], low byte first.
  * @retval STAT_OK on success, STAT_??? otherwise.
  */
static StatusCode RC522_CalculateCRC(uint8_t *const data, const size_t length, uint8_t *const result)
{
  RC522_WriteRegister(CommandReg, RC522_Idle); // Stop any active command.
  RC522_WriteRegister(DivIrqReg, CRCIRq_Msk); // Clear the CRCIRq interrupt request bit
  RC522_WriteRegister(FIFOLevelReg, FlushBuffer_Msk); // FIFO initialization
  RC522_WriteBuffer(FIFODataReg, data, length);
  RC522_WriteRegister(CommandReg, RC522_CalcCRC); // Start the calculation
  /* Wait for the CRC calculation to complete. Check for the register to
     indicate that the CRC calculation is complete in a loop. If the
     calculation is not indicated as complete in ~90ms, then time out
     the operation. */
  const uint32_t deadline = HAL_GetTick() + 89;
  do 
  {
    const uint8_t irqRegValue = RC522_ReadRegister(DivIrqReg);
    if (irqRegValue & CRCIRq_Msk) 
    {
      RC522_WriteRegister(CommandReg, RC522_Idle); // Stop calculating CRC for new content in the FIFO.
      // Transfer the result from the registers to the result buffer
      result[0] = RC522_ReadRegister(CRCResultRegL);
      result[1] = RC522_ReadRegister(CRCResultRegH);
      return STAT_OK;
    }
  } while (HAL_GetTick() < deadline);
  // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
  return STAT_TIMEOUT;
}

/*############################################################################*/
// Functions for manipulating the MFRC522
/*############################################################################*/

/**
  * @brief  Turns the antenna on by enabling pins TX1 and TX2.
  * @note   After a reset these pins are disabled.
  */
void RC522_AntennaOn(void)
{
  RC522_SetRegisterBitMask(TxControlReg, Tx1RFEn_Msk | Tx2RFEn_Msk);
}

/**
  * @brief  Turns the antenna off by disabling pins TX1 and TX2.
  */
void RC522_AntennaOff(void)
{
  RC522_ClearRegisterBitMask(TxControlReg, Tx1RFEn_Msk | Tx2RFEn_Msk);
}

/**
  * @brief  Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
  * @note   See 9.3.3.6 / table 98
  * @note   Return value scrubbed with 0x70=01110000b as RCFfgReg may use reserved bits.
  * @return Value of the RxGain, scrubbed to the 3 bits used.
  */
RC522_RxGain RC522_GetAntennaGain(void)
{
  return RC522_ReadRegister(RFCfgReg) & 0x70;
}

/**
  * @brief  Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
  * @note   See 9.3.3.6 / table 98
  * @note   Given mask is scrubbed with 0x70=01110000b as RCFfgReg may use reserved bits.
  */
void RC522_SetAntennaGain(const RC522_RxGain gain)
{
  if (RC522_GetAntennaGain() != gain) 
  { // only bother if there is a change
    RC522_ClearRegisterBitMask(RFCfgReg, 0x70); // clear needed to allow 000 pattern
    RC522_SetRegisterBitMask(RFCfgReg, (uint8_t)gain);
  }
}

/**
  * @brief  Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
  */
void RC522_Reset(void)
{
  RC522_WriteRegister(CommandReg, RC522_SoftReset);
  /* The datasheet does not mention how long the SoftRest command takes to complete.
     But the MFRC522 might have been in soft power-down mode (triggered by PowerDown bit of CommandReg). 
     Section 8.8.2 in the datasheet says the oscillator start-up time is the
     start up time of the crystal + 37,74μs. Let us be generous: 50ms. */
  uint8_t countTries = 0;
  do {
    // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
    // TODO: check what lower delay is effective. 50ms block is very long.
    HAL_Delay(50);
  } while ((RC522_ReadRegister(CommandReg) & PowerDown_Msk) && (++countTries) < 3);
}

/**
  * @brief  Determine firmware version. Firmware is equal to chip version.
  * @return (Known) firmware version of MFRC522.
  */
RC522_FirmwareVersion RC522_GetVersion(void)
{ // Determine firmware version (see section 9.3.4.8 in spec).
  RC522_FirmwareVersion version = RC522_ReadRegister(VersionReg);
  switch (version)
  {
  case Version_Counterfeit:
  case Version_FM17522:
  case Version_FM17522_1:
  case Version_FM17522E:
  case Version_0_0:
  case Version_1_0:
  case Version_2_0:
    break;
  default:
    version = Version_Unknown;
  }
  return version;
}

/**
 * @brief  Initializes the MFRC522 chip.
 */
bool RC522_Init(void)
{
  RC522_Reset();

  RC522_WriteRegister(TxModeReg, RESET_TxModeReg); // Section 9.3.2.3
  RC522_WriteRegister(RxModeReg, RESET_RxModeReg); // Section 9.3.2.4
  RC522_WriteRegister(ModWidthReg, RESET_ModWidthReg); // Section 9.3.3.4

  /* When communicating with a PICC we need a timeout if something goes wrong.
     f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
     TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
     Timer starts automatically at the end of the transmission in all communication modes at all speeds */
  RC522_WriteRegister(TModeReg, TAuto_Msk | ((T_PRESCALER >> 8) & TPrescaler_Hi_Msk)); // Section 9.3.3.10
  RC522_WriteRegister(TPrescalerReg, (uint8_t)T_PRESCALER);
  RC522_WriteRegister(TReloadRegH, (T_RELOAD_VAL >> 8));
  RC522_WriteRegister(TReloadRegL, (uint8_t)T_RELOAD_VAL);

  //RC522_SetAntennaGain(RxGain_38db); // Section 9.3.3.6
  RC522_SetRegisterBitMask(TxASKReg, Force100ASK_Msk); // Section 9.3.2.6
  // Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  RC522_WriteRegister(ModeReg, TxWaitRF_Msk | PolMFin_Msk | CRCPreset6363h_Msk); // Section 9.3.2.2
  RC522_AntennaOn(); // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)

  HAL_Delay(4); // Optional delay of 4ms. Some board do need more time after init to be ready.

  return RC522_GetVersion() != Version_Unknown;
}

/**
 * @brief  Performs a self-test of the MFRC522
 * @note   See 16.1.1
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
bool RC522_PerformSelfTest(void)
{
  // 1. Perform a soft reset.
  RC522_Reset();

  // 2. Clear the internal buffer by writing 25 bytes of 00h
  uint8_t zeroes[25] = { 0x00 };
  RC522_WriteRegister(FIFOLevelReg, FlushBuffer_Msk); // FIFO initialization. Section 9.3.1.11
  RC522_WriteBuffer(FIFODataReg, zeroes, sizeof(zeroes));
  RC522_WriteRegister(CommandReg, RC522_Mem); // transfer to internal buffer

  // 3. Enable self-test
  RC522_SetRegisterBitMask(AutoTestReg, SelfTest_Enable);

  // 4. Write 00h to FIFO buffer
  RC522_WriteRegister(FIFODataReg, 0x00);

  // 5. Start self-test by issuing the CalcCRC command
  RC522_WriteRegister(CommandReg, RC522_CalcCRC);

  // 6. Wait for self-test to complete
  for (uint8_t i = 0; i < UINT8_MAX; i++) 
  {
    /* The datasheet does not specify exact completion condition except
       that FIFO buffer should contain RC522_FIFO_SIZE bytes.
       While self-test is initiated by CalcCRC command
       it behaves differently from normal CRC computation,
       so one can't reliably use DivIrqReg to check for completion.
       It is reported that some devices does not trigger CRCIRq flag
       during self-test. */
    const uint8_t numberOfBytesInFIFO = RC522_ReadRegister(FIFOLevelReg);
    if (numberOfBytesInFIFO >= RC522_FIFO_SIZE) 
    {
      break;
    }
  }
  RC522_WriteRegister(CommandReg, RC522_Idle); // Stop calculating CRC for new content in the FIFO.

  // 7. Read out resulting RC522_FIFO_SIZE bytes from the FIFO buffer.
  uint8_t result[RC522_FIFO_SIZE];
  RC522_ReadBuffer(FIFODataReg, result, sizeof(result), 0);

  // Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
  RC522_SetRegisterBitMask(AutoTestReg, SelfTest_Disable);

  // Determine firmware version (see section 9.3.4.8 in spec)
  const RC522_FirmwareVersion version = RC522_GetVersion();

  bool verified = true;
  // Pick the appropriate reference values
  const uint8_t *reference = NULL;
  switch (version)
  { // Fudan Semiconductor clone:
  case Version_FM17522:
    reference = FM17522_FirmwareReference88;
    break;
  case Version_FM17522_1:
    reference = FM17522_FirmwareReferenceB2;
    break;
  case Version_FM17522E:
    reference = FM17522E_FirmwareReference;
    break;
  case Version_0_0:
    reference = MFRC522_FirmwareReferenceV00;
    break;
  case Version_1_0:
    reference = MFRC522_FirmwareReferenceV10;
    break;
  case Version_2_0:
    reference = (result[2] == FM17522_FirmwareReferenceB2[2])
                ? FM17522_FirmwareReferenceB2
                : MFRC522_FirmwareReferenceV20;
    break;
  default:	// Unknown version
    verified = false; // abort test
  }

  if (reference != NULL)
  { // Verify that the results match up to our expectations
    for (size_t i = 0; i < sizeof(result); i++) 
    {
      if (result[i] != reference[i]) 
      {
        verified = false;
      }
    }
  }

  // 8. Perform a re-init, because RC522 does not work after test.
  // Reset does not work as expected.
  // "Auto self-test done" does not work as expected.
  RC522_Init();
	
  // Test process done.
  return verified;
}

/*############################################################################*/
// Power control
/*############################################################################*/

/**
  * Only soft power down mode is available through software.
  * IMPORTANT NOTE!!!!
  * Calling any other function that uses CommandReg will disable soft power down mode.
  * For more details about power control, refer to the datasheet - page 33 (8.6).
  */
void RC522_SoftPowerDown(void)
{
  RC522_SetRegisterBitMask(CommandReg, PowerDown_Msk);
}

/**
  * @brief  Power on the RC522 again, after RC522_SoftPowerDown.
  */
void RC522_SoftPowerUp(void)
{
  RC522_ClearRegisterBitMask(CommandReg, PowerDown_Msk);
  // wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
  const uint32_t timeout = HAL_GetTick() + 500; // create timer for timeout (just in case) 
	
  while (HAL_GetTick() <= timeout) 
  {
    if (!(RC522_ReadRegister(CommandReg) & PowerDown_Msk)) 
    { // if PowerDown bit is 0 
      break; // wake up procedure is finished 
    }
  }
}

/*############################################################################*/
// Functions for communicating with PICCs
/*############################################################################*/

/**
  * @brief  Transfers data to the MFRC522 FIFO, executes a command, waits for
  *         completion and transfers data back from the FIFO.
  * @note   CRC validation can only be done if backData and backLen are specified.
  * 
  * @return STAT_OK on success, STAT_??? otherwise.
  */
static StatusCode RC522_TxRxDataToPICC(const RC522_Command command, ///< The command to execute.
  const uint8_t waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
  uint8_t *const sendData,  ///< Pointer to the data to transfer to the FIFO.
  const size_t sendLen,    ///< Number of bytes to transfer to the FIFO.
  uint8_t *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
  uint8_t *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
  uint8_t *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 7 valid bits. Default 0.
  const uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. 0 for 7 valid bits. Default 0.
  const bool checkCrc)      ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{
  // Prepare values for BitFramingReg
  const uint8_t txLastBits = (validBits != NULL) ? *validBits : 0;
  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
  const uint8_t bitFraming = (uint8_t)((rxAlign & 0x07) << 4) | (txLastBits & 0x07);

  RC522_WriteRegister(CommandReg, RC522_Idle); // Stop any active command.
  RC522_WriteRegister(ComIrqReg, (uint8_t)~Set1_Msk); // Clear all seven interrupt request bits
  RC522_WriteRegister(FIFOLevelReg, FlushBuffer_Msk); // FIFO initialization
  RC522_WriteBuffer(FIFODataReg, sendData, sendLen);
  RC522_WriteRegister(BitFramingReg, bitFraming); // Bit adjustments
  RC522_WriteRegister(CommandReg, (uint8_t)command);
  if (command == RC522_Transceive) 
  {
    RC522_SetRegisterBitMask(BitFramingReg, StartSend_Msk); // Transmission of data starts
  }

  /* In RC522_Init() we set the TAuto flag in TModeReg. This means the timer
     automatically starts when the RC522 stops transmitting.
	  
     Wait here for the command to complete. The bits specified in the
     `waitIRq` variable define what bits constitute a completed command.
     When they are set in the ComIrqReg register, then the command is
     considered complete. If the command is not indicated as complete in
     ~36ms, then consider the command as timed out. */
  const uint32_t deadline = HAL_GetTick() + 36;
  bool completed = false;
  do 
  {
    const uint8_t irqRegValue = RC522_ReadRegister(ComIrqReg);
    if (irqRegValue & waitIRq) 
    { // One of the interrupts that signal success has been set.
      completed = true;
      break;
    }
    if (irqRegValue & TimerIRq_Msk) 
    { // Timer interrupt - nothing received in 25ms
      return STAT_TIMEOUT;
    }
  } while (HAL_GetTick() < deadline);
  // 36ms and nothing happened. Communication with the MFRC522 might be down.
  if (completed == false) 
  {
    return STAT_TIMEOUT;
  }

  // Stop now if any errors except collisions were detected.
  const uint8_t errorRegValue = RC522_ReadRegister(ErrorReg);
  if (errorRegValue & (BufferOvfl_Msk | ParityErr_Msk | ProtocolErr_Msk)) 
  {
    return STAT_ERROR;
  }

  uint8_t validBitsTmp = 0;

  // If the caller wants data back, get it from the MFRC522.
  if ((backData != NULL) && (backLen != NULL)) 
  {
    const uint8_t numberOfBytesInFIFO = RC522_ReadRegister(FIFOLevelReg);
    if (numberOfBytesInFIFO > *backLen) 
    {
      return STAT_NO_ROOM;
    }
    *backLen = numberOfBytesInFIFO;
    RC522_ReadBuffer(FIFODataReg, backData, numberOfBytesInFIFO, rxAlign);

    // RxLastBits[2:0] indicates the number of valid bits in the last received byte.
    // If this value is 000b, the whole byte is valid.
    validBitsTmp = RC522_ReadRegister(ControlReg) & RxLastBits_Msk;
    if (validBits != NULL) 
    {
      *validBits = validBitsTmp;
    }
  }

  // Tell about collisions
  if (errorRegValue & CollErr_Msk) 
	{
    return STAT_COLLISION;
  }

  // Perform CRC_A validation if requested.
  if ((backData != NULL) && (backLen != NULL) && checkCrc) 
  {
    // In this case a MIFARE Classic NACK is not OK.
    if ((*backLen == 1) && (validBitsTmp == 4)) 
    {
      return STAT_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if ((*backLen < 2) || (validBitsTmp != 0)) 
    {
      return STAT_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    uint8_t controlBuffer[2];
    const StatusCode status = RC522_CalculateCRC(backData, *backLen - 2, controlBuffer);
    if (status != STAT_OK) 
    {
      return status;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) 
      || (backData[*backLen - 1] != controlBuffer[1])) 
    {
      return STAT_CRC_WRONG;
    }
  }

  return STAT_OK;
}

/**
  * @brief  Executes the Transceive command.
  * @note   CRC validation can only be done if backData and backLen8 are specified.
  * 
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_TransceiveData(uint8_t *const sendData,  ///< Pointer to the data to transfer to the FIFO.
  const size_t sendLen,    ///< Number of bytes to transfer to the FIFO.
  uint8_t *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
  uint8_t *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
  uint8_t *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 7 valid bits. Default 0.
  const uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. 0 for 7 valid bits. Default 0.
  const bool checkCRC)      ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{
  const uint8_t waitIRq = RxIRq_Msk | IdleIRq_Msk;
  return RC522_TxRxDataToPICC(RC522_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

/**
  * @brief  Transmits REQA or WUPA commands.
  * @note   Beware: When two PICCs are in the field at the same time I often get STAT_TIMEOUT
  *                 - probably due do bad antenna design.
  * @param  command : The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
  * @param  bufferATQA : The buffer to store the ATQA (Answer to request) in
  * @param  bufferSize : Buffer size, at least two bytes. Also number of bytes returned if STAT_OK.
  * @return STAT_OK on success, STAT_??? otherwise.
  */ 
StatusCode RC522_PICC_REQA_or_WUPA(PICC_Command command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
  if ((bufferATQA == NULL) || (*bufferSize < 2)) 
  { // The ATQA response is 2 bytes long.
    return STAT_NO_ROOM;
  }
  RC522_ClearRegisterBitMask(CollReg, ValuesAfterColl_Msk); // Bits received after collision are cleared.
  // For REQA and WUPA we need the short frame format - transmit only 7 bits
  // of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  uint8_t validBits = 7;
  const StatusCode status = RC522_TransceiveData((uint8_t *)&command, 1, 
    bufferATQA, bufferSize, &validBits, 0, false);
  if (status != STAT_OK) 
  {
    return status;
  }
  if ((*bufferSize != 2) || (validBits != 0)) 
  { // ATQA must be exactly 16 bits.
    return STAT_ERROR;
  }
  return STAT_OK;
}

/**
  * @brief  Transmits a REQuest command, Type A. Invites PICCs in state IDLE
  *         to go to READY and prepare for anti-collision or selection. 7 bit frame.
  * @note   Beware: When two PICCs are in the field at the same time I often get STAT_TIMEOUT
  *                 - probably due do bad antenna design.
  * @param  bufferATQA : The buffer to store the ATQA (Answer to request) in
  * @param  bufferSize : Buffer size, at least two bytes. Also number of bytes returned if STAT_OK.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
  return RC522_PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

/**
  * @brief  Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE
  *         and HALT to go to READY(*) and prepare for anti-collision or selection. 7 bit frame.
  * @note   Beware: When two PICCs are in the field at the same time I often get STAT_TIMEOUT
  *                 - probably due do bad antenna design.
  * @param  bufferATQA : The buffer to store the ATQA (Answer to request) in
  * @param  bufferSize : Buffer size, at least two bytes. Also number of bytes returned if STAT_OK.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
  return RC522_PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
}

/**
  * @brief  Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
  * @param  cascadeLevel : In
  * @param  buffer : Out
  * @param  uidIndex : Out
  * @param  useCascadeTag : Out
  * @param  validBits : In: The number of known UID bits supplied in uid_. Normally 0.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
static StatusCode SetTheCascadeLevel(const uint8_t cascadeLevel, uint8_t *buffer, uint8_t *uidIndex, bool *useCascadeTag, const size_t validBits)
{
  switch (cascadeLevel)
  {
  case 1:
    buffer[0] = PICC_CMD_SEL_CL1;
    *uidIndex = 0;
    *useCascadeTag = validBits && uid_.size > 4; // When we know that the UID has more than 4 bytes
    break;
      
  case 2:
    buffer[0] = PICC_CMD_SEL_CL2;
    *uidIndex = 3;
    *useCascadeTag = validBits && uid_.size > 7; // When we know that the UID has more than 7 bytes
    break;
      
  case 3:
    buffer[0] = PICC_CMD_SEL_CL3;
    *uidIndex = 6;
    *useCascadeTag = false; // Never used in CL3.
    break;
      
  default:
    return STAT_INTERNAL_ERROR;
  }
  return STAT_OK;
}

static int8_t GetNumberOfKnownUidBitsAtTheCurrentCascadeLevel(const size_t validBits, const uint8_t uidIndex)
{
  int8_t currentLevelKnownBits = (int8_t)(validBits - (8 * uidIndex));
  if (currentLevelKnownBits < 0) 
  {
    currentLevelKnownBits = 0;
  }
  return currentLevelKnownBits;
}

static void CopyTheKnownBytesFromUidByteArrToBuffer(int8_t *currentLevelKnownBits, const size_t uidIndex, const bool useCascadeTag, uint8_t *buffer)
{
  size_t index = 2; // destination index in buffer[]
  if (useCascadeTag) 
  {
    buffer[index++] = PICC_CMD_CT;
  }
  // The number of bytes needed to represent the known bits for this level.
  size_t bytesToCopy = *currentLevelKnownBits / 8 + ((*currentLevelKnownBits % 8) ? 1 : 0);
  if (bytesToCopy > 0) 
  {
    const size_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
    if (bytesToCopy > maxBytes) 
    {
      bytesToCopy = maxBytes;
    }
    for (size_t count = 0; count < bytesToCopy; count++) 
    {
      buffer[index++] = uid_.uidByte[uidIndex + count];
    }
  }

  // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
  if (useCascadeTag) 
  {
    *currentLevelKnownBits += 8;
  }
}

static void CopyTheFoundUidBytesFromBufferToUidByteArray(const size_t uidIndex, const uint8_t *buffer)
{
  // Copy the found UID bytes from buffer[] to uid_.uidByte[]
  size_t index             = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
  const size_t bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
  for (size_t count = 0; count < bytesToCopy; count++) 
  {
    uid_.uidByte[uidIndex + count] = buffer[index++];
  }
}

static StatusCode SolvePiccCollision(int8_t currentLevelKnownBits, uint8_t *buffer)
{
  // More than one PICC in the field => collision.
  // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
  const uint8_t valueOfCollReg = RC522_ReadRegister(CollReg);
  if (valueOfCollReg & CollPosNotValid_Msk) 
  {
    return STAT_COLLISION; // Without a valid collision position we cannot continue
  }
  uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
  if (collisionPos == 0) 
  {
    collisionPos = 32;
  }
  if (collisionPos <= currentLevelKnownBits) 
  { // No progress - should not happen
    return STAT_INTERNAL_ERROR;
  }
  // Choose the PICC with the bit set.
  currentLevelKnownBits = (int8_t)collisionPos;
  const uint8_t count = currentLevelKnownBits % 8; // The bit to modify
  const uint8_t checkBit = (currentLevelKnownBits - 1) % 8;
  const uint8_t index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
  buffer[index] |= (1 << checkBit);

  return STAT_OK;
}

/**
  * @brief  Transmits SELECT/ANTICOLLISION commands to select a single PICC.
  *
  * Before calling this function the PICCs must be placed in the READY(*) state
  * by calling PICC_RequestA() or PICC_WakeupA().
  * On success:
  *     - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned
  *       to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
  *     - The UID size and value of the chosen PICC is written to the global variable uid_ along with the SAK.
  * 
  * A PICC UID consists of 4, 7 or 10 bytes.
  * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
  *     UID size  Number of UID bytes   Cascade levels    Example of PICC
  *     ========  ===================   ==============    ===============
  *     single            4                   1           MIFARE Classic
  *     double            7                   2           MIFARE Ultralight
  *     triple            10                  3           Not currently in use?
  *
  * @param  validBits : The number of known UID bits supplied in uid_. Normally 0. If set you must also supply uid_.size.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
static StatusCode RC522_PICC_Select(const size_t validBits)
{
  /* Description of buffer structure:
      Byte 0: SEL       Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
      Byte 1: NVB       Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
      Byte 2: UID-data or CT  See explanation below. CT means Cascade Tag.
      Byte 3: UID-data
      Byte 4: UID-data
      Byte 5: UID-data
      Byte 6: BCC       Block Check Character - XOR of bytes 2-5
      Byte 7: CRC_A
      Byte 8: CRC_A
    The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
  
    Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
      UID size  Cascade level Byte2 Byte3 Byte4 Byte5
      ========  ============= ===== ===== ===== =====
      4 bytes         1       uid0  uid1  uid2  uid3
      7 bytes         1       CT    uid0  uid1  uid2
                      2       uid3  uid4  uid5  uid6
      10 bytes        1       CT    uid0  uid1  uid2
                      2       CT    uid3  uid4  uid5
                      3       uid6  uid7  uid8  uid9
  */

  // Sanity checks
  if (validBits > 80) 
  {
    return STAT_INVALID;
  }

  // Prepare MFRC522
  RC522_ClearRegisterBitMask(CollReg, ValuesAfterColl_Msk); // Bits received after collision are cleared.

  uint8_t uidIndex; // The first index in uid_.uidByte[] that is used in the current Cascade Level.
  bool useCascadeTag;
  uint8_t cascadeLevel = 1;
  // Repeat Cascade Level loop until we have a complete UID.
  bool uidComplete = false;
  while (!uidComplete) 
  {
    uint8_t buffer[9]; // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    if (SetTheCascadeLevel(cascadeLevel, buffer, &uidIndex, &useCascadeTag, validBits) != STAT_OK)
    {
      return STAT_INTERNAL_ERROR;
    }
    int8_t currentLevelKnownBits = GetNumberOfKnownUidBitsAtTheCurrentCascadeLevel(validBits, uidIndex);
    CopyTheKnownBytesFromUidByteArrToBuffer(&currentLevelKnownBits, uidIndex, useCascadeTag, buffer);

    uint8_t txLastBits; // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *responseBuffer;
    uint8_t responseLength;
    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    bool selectDone = false;
    do
    {
      uint8_t bufferUsed; // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.

      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32) 
      {
        // All UID bits in this Cascade Level are known. This is a SELECT.
        //println("SELECT: currentLevelKnownBits=%u", currentLevelKnownBits);
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
        // Calculate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        // Calculate CRC_A
        const StatusCode result = RC522_CalculateCRC(buffer, 7, &buffer[7]);
        if (result != STAT_OK) 
        {
          return result;
        }
        txLastBits = 0; // 0 => All 8 bits are valid.
        bufferUsed = 9;
        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer = &buffer[6];
        responseLength = 3;
      }
      else 
      {
        // This is an ANTICOLLISION.
        //println("ANTICOLLISION: currentLevelKnownBits=%u", currentLevelKnownBits);
        txLastBits = currentLevelKnownBits % 8;
        const uint8_t count = currentLevelKnownBits / 8; // Number of whole bytes in the UID part.
        const uint8_t index = 2 + count; // Number of whole bytes: SEL + NVB + UIDs
        buffer[1] = (uint8_t)(index << 4) | txLastBits; // NVB - Number of Valid Bits
        bufferUsed = index + (txLastBits ? 1 : 0);
        // Store response in the unused part of buffer
        responseBuffer = &buffer[index];
        responseLength = sizeof(buffer) - index;
      }
      
      // Set bit adjustments
      // Used in BitFramingReg. Defines the bit position for the first bit received.
      const uint8_t rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
      RC522_WriteRegister(BitFramingReg, (uint8_t)((rxAlign & 0x07) << 4) | (txLastBits & 0x07));
      
      // Transmit the buffer and receive the response.
      StatusCode result = RC522_TransceiveData(buffer, bufferUsed, responseBuffer,
                                      &responseLength, &txLastBits, rxAlign, false);
      if (result == STAT_COLLISION) 
      {
        result = SolvePiccCollision(currentLevelKnownBits, buffer);
        if (result != STAT_OK) 
        {
          return result;
        }
      }
      else if (result != STAT_OK) 
      {
        return result;
      }
      else // STAT_OK
      {
        if (currentLevelKnownBits >= 32) 
        {
          // This was a SELECT.
          selectDone = true; // No more anticollision 
          // We continue below outside the while.
        }
        else 
        {
          // This was an ANTICOLLISION.
          // We now have all 32 bits of the UID in this Cascade Level
          currentLevelKnownBits = 32;
          // Run loop again to do the SELECT.
        }
      }
    } while (!selectDone);
    
    // We do not check the CBB - it was constructed by us above.

    CopyTheFoundUidBytesFromBufferToUidByteArray(uidIndex, buffer);
    
    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0) 
    { // SAK must be exactly 24 bits (1 byte + CRC_A).
      return STAT_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    const StatusCode result = RC522_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if (result != STAT_OK) 
    {
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) 
    {
      return STAT_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04) 
    { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else 
    {
      uidComplete = true;
      uid_.sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)
  
  // Set correct uid_.size
  uid_.size = 3 * cascadeLevel + 1;
  
  return STAT_OK;
}

/**
  * @brief  Instructs a PICC in state ACTIVE(*) to go to state HALT.
  *
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_PICC_HaltA(void)
{
  uint8_t commandBuffer[] = {PICC_CMD_HLTA, 0, 0, 0};
  StatusCode result = RC522_CalculateCRC(commandBuffer, 2, &commandBuffer[2]);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  // Send the command.
  // The standard says:
  //    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
  //    HLTA command, this response shall be interpreted as 'not acknowledge'.
  // We interpret that this way: Only STAT_TIMEOUT is a success.
  result = RC522_TransceiveData(commandBuffer, sizeof(commandBuffer), NULL, NULL, NULL, 0, false);
  if (result == STAT_TIMEOUT) 
  {
    return STAT_OK;
  }
  if (result == STAT_OK) 
  { // That is ironically NOT ok in this case ;-)
    return STAT_ERROR;
  }
  return result;
}

/*############################################################################*/
// Functions for communicating with MIFARE PICCs
/*############################################################################*/

/**
  * @brief  Executes the MFRC522 MFAuthent command.
  * @note   This command manages MIFARE authentication to enable a secure
  *         communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
  *         The authentication is described in the MFRC522 datasheet section 10.3.1.9
  *         and https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf section 12.1.
  *         For use with MIFARE Classic PICCs.
  *         The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
  *         Remember to call RC522_StopCrypto1() after communicating with the
  *         authenticated PICC - otherwise no new communications can start.
  * 
  *         All keys are set to FFFFFFFFFFFFh at chip delivery.
  *
  * @param  command : PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
  * @param  blockAddr : The block number. See numbering in the comments in the .h file.
  * @param  key : Pointer to the Crypto1 key to use (6 bytes).
  * @return STAT_OK on success, STAT_??? otherwise. Probably STAT_TIMEOUT if you supply the wrong key.
  */
StatusCode RC522_Authenticate(const PICC_Command command, const uint8_t blockAddr, const MIFARE_Key *key)
{
  // Build command buffer
  uint8_t sendData[12];
  sendData[0] = (uint8_t)command;
  sendData[1] = blockAddr;
  for (size_t i = 0; i < MF_KEY_SIZE; i++) 
  {
    sendData[2 + i] = key->keyByte[i];
  }
  // Use the last uid bytes as specified in https://www.nxp.com/docs/en/application-note/AN10927.pdf
  // section 3.2.5 "MIFARE Classic Authentication".
  // The only missed case is the MF1Sxxxx shortcut activation,
  // but it requires cascade tag (CT) byte, that is not part of uid.
  for (size_t i = 0; i < 4; i++) 
  { // The last 4 bytes of the UID
    sendData[MF_KEY_SIZE + 2 + i] = uid_.uidByte[i + uid_.size - 4];
  }
  
  // Start the authentication.
  return RC522_TxRxDataToPICC(RC522_MFAuthent, IdleIRq_Msk, &sendData[0], 
    sizeof(sendData), NULL, NULL, NULL, 0, false);
}

/**
  * @brief  Used to exit the PCD from its authenticated state.
  * @note   Remember to call this function after communicating with an
  *         authenticated PICC - otherwise no new communications can start.
  */
void RC522_StopCrypto1(void)
{
  RC522_ClearRegisterBitMask(Status2Reg, MFCrypto1On_Msk);
}

/**
  * @brief  Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
  * 
  * @note   For MIFARE Classic the sector containing the block must be authenticated
  *         before calling this function.
  * 
  * @note   For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
  *         The MF0ICU1 returns a NAK for higher addresses.
  *         The MF0ICU1 responds to the READ command by sending 16 bytes starting
  *         from the page address defined by the command argument.
  *         For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
  *         A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages
  *         0Eh, 0Fh, 00h and 01h are returned.
  * 
  * @note   The buffer must be at least 18 bytes because a CRC_A is also returned.
  *         Checks the CRC_A before returning STAT_OK.
  *
  * @param  blockAddr : MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
  * @param  buffer : The buffer to store the data in.
  * @param  bufferSize : Buffer size, at least 18 bytes. Also number of bytes returned if STAT_OK.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Read(const uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize)
{
  // Sanity check
  if (buffer == NULL || *bufferSize < 18) 
  {
    return STAT_NO_ROOM;
  }
  
  // Build command buffer
  buffer[0] = PICC_CMD_MF_READ;
  buffer[1] = blockAddr;
  // Calculate CRC_A
  const StatusCode result = RC522_CalculateCRC(buffer, 2, &buffer[2]);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  // Transmit the buffer and receive the response, validate CRC_A.
  return RC522_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
}

/**
  * @brief  Writes 16 bytes to the active PICC.
  * 
  * @note   For MIFARE Classic the sector containing the block must be
  *         authenticated before calling this function.
  * 
  * @note   For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
  *         Even though 16 bytes are transferred to the Ultralight PICC, only
  *         the least significant 4 bytes (bytes 0 to 3) are written to the specified address.
  *         It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
  *
  * @param  blockAddr : MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
  * @param  buffer : The 16 bytes to write to the PICC.
  * @param  bufferSize : Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Write(const uint8_t blockAddr, const uint8_t *buffer, const size_t bufferSize)
{
  // Sanity check
  if (buffer == NULL || bufferSize < 16) 
  {
    return STAT_INVALID;
  }
  
  // Mifare Classic protocol requires two communications to perform a write.
  // Step 1: Tell the PICC we want to write to block blockAddr.
  const uint8_t cmdBuffer[] = { PICC_CMD_MF_WRITE, blockAddr };
  // Adds CRC_A and checks that the response is MF_ACK.
  StatusCode result = RC522_MIFARE_Transceive(cmdBuffer, sizeof(cmdBuffer), false);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  // Step 2: Transfer the data
  // Adds CRC_A and checks that the response is MF_ACK.
  result = RC522_MIFARE_Transceive(buffer, bufferSize, false);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  return STAT_OK;
}

/**
  * @brief  Writes a 4 byte page to the active MIFARE Ultralight PICC.
  * @param  page : The page (2-15) to write to.
  * @param  buffer : The 4 bytes to write to the PICC
  * @param  bufferSize : Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Ultralight_Write(const uint8_t page, const uint8_t *buffer, const size_t bufferSize)
{
  // Sanity check
  if (buffer == NULL || bufferSize < 4) 
  {
    return STAT_INVALID;
  }
  
  // Build command buffer
  uint8_t cmdBuffer[6];
  cmdBuffer[0] = PICC_CMD_UL_WRITE;
  cmdBuffer[1] = page;
  memcpy(&cmdBuffer[2], buffer, 4);
  
  // Perform the write
  // Adds CRC_A and checks that the response is MF_ACK.
  const StatusCode result = RC522_MIFARE_Transceive(cmdBuffer, sizeof(cmdBuffer), false); 
  if (result != STAT_OK) 
  {
    return result;
  }
  return STAT_OK;
}

/**
  * @brief  Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
  * @param  command : The command to use
  * @param  blockAddr : The block (0-0xff) number.
  * @param  data : The data to transfer in step 2
  * @return STAT_OK on success, STAT_??? otherwise.
  */
static StatusCode RC522_MIFARE_TwoStepHelper(const PICC_Command command, const uint8_t blockAddr, int32_t data)
{
  // Step 1: Tell the PICC the command and block address
  const uint8_t cmdBuffer[] = { (uint8_t)command, blockAddr }; // We only need room for 2 bytes.
  // Adds CRC_A and checks that the response is MF_ACK.
  StatusCode result = RC522_MIFARE_Transceive(cmdBuffer, sizeof(cmdBuffer), false);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  // Step 2: Transfer the data
  // Adds CRC_A and accept timeout as success.
  result = RC522_MIFARE_Transceive((uint8_t*)&data, sizeof(data), true);
  if (result != STAT_OK) 
  {
    return result;
  }
  
  return STAT_OK;
}

/**
  * @brief  MIFARE Increment adds the delta to the value of the addressed block,
  *         and stores the result in a volatile memory.
  * @note   For MIFARE Classic only. The sector containing the block must be
  *         authenticated before calling this function.
  * @note   Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  *         Use MIFARE_Transfer() to store the result in a block.
  * @param  blockAddr : The block (0-0xff) number.
  * @param  delta : This number is added to the value of block blockAddr.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Increment(const uint8_t blockAddr, const int32_t delta)
{
  return RC522_MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
}

/**
  * @brief  MIFARE Decrement subtracts the delta from the value of the addressed
  *         block, and stores the result in a volatile memory.
  * @note   For MIFARE Classic only. The sector containing the block must be
  *         authenticated before calling this function.
  * @note   Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  *         Use MIFARE_Transfer() to store the result in a block.
  * @param  blockAddr : The block (0-0xff) number.
  * @param  delta : This number is subtracted from the value of block blockAddr.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Decrement(const uint8_t blockAddr, const int32_t delta)
{
   return RC522_MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
}

/**
  * @brief  MIFARE Restore copies the value of the addressed block into a volatile memory.
  * @note   For MIFARE Classic only. The sector containing the block must be
  *         authenticated before calling this function.
  * @note   Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  *         Use MIFARE_Transfer() to store the result in a block.
  * @param  blockAddr : The block (0-0xff) number.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Restore(const uint8_t blockAddr)
{
  // The datasheet describes Restore as a two step operation, but does not explain
  // what data to transfer in step 2.
  // Doing only a single step does not work, so I chose to transfer 0L in step two.
  return RC522_MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
}

/**
  * @brief  MIFARE Transfer writes the value stored in the volatile memory into
  *         one MIFARE Classic block.
  * @note   For MIFARE Classic only. The sector containing the block must be
  *         authenticated before calling this function.
  * @note   Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  * @param  blockAddr : The block (0-0xff) number.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Transfer(const uint8_t blockAddr)
{
  // Tell the PICC we want to transfer the result into block blockAddr.
  // We only need room for 2 bytes.
  const uint8_t cmdBuffer[] = { PICC_CMD_MF_TRANSFER, blockAddr };
  // Adds CRC_A and checks that the response is MF_ACK.
  const StatusCode result = RC522_MIFARE_Transceive(cmdBuffer, sizeof(cmdBuffer), false);
  if (result != STAT_OK) 
  {
    return result;
  }
  return STAT_OK;
}

/**
  * @brief  Helper routine to read the current value from a Value Block.
  * 
  * @note   Only for MIFARE Classic and only for blocks in "value block" mode, that
  *         is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
  *         the block must be authenticated before calling this function. 
  * 
  * @param[in]   blockAddr : The block (0x00-0xff) number.
  * @param[out]  value : Current value of the Value Block.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_GetValue(const uint8_t blockAddr, int32_t *value)
{
  uint8_t buffer[18];
  uint8_t size = sizeof(buffer);
  
  // Read the block
  const StatusCode status = RC522_MIFARE_Read(blockAddr, buffer, &size);
  if (status == STAT_OK) 
  { // Extract the value
    *value = ((int32_t)(buffer[3]) << 24) | ((int32_t)(buffer[2]) << 16) | ((int32_t)(buffer[1]) << 8) | (int32_t)(buffer[0]);
  }
  return status;
}

/**
  * @brief  Helper routine to write a specific value into a Value Block.
  * 
  * @note   Only for MIFARE Classic and only for blocks in "value block" mode, that
  *         is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
  *         the block must be authenticated before calling this function. 
  * 
  * @param[in]  blockAddr : The block (0x00-0xff) number.
  * @param[in]  value : New value of the Value Block.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_SetValue(const uint8_t blockAddr, const int32_t value)
{
  uint8_t buffer[18];
  
  // Translate the int32_t into 4 bytes; repeated 2x in value block
  buffer[0]  = buffer[8]  = (value & 0xFF);
  buffer[1]  = buffer[9]  = (value & 0xFF00) >> 8;
  buffer[2]  = buffer[10] = (value & 0xFF0000) >> 16;
  buffer[3]  = buffer[11] = (value & 0xFF000000) >> 24;
  // Inverse 4 bytes also found in value block
  buffer[4]  = ~buffer[0];
  buffer[5]  = ~buffer[1];
  buffer[6]  = ~buffer[2];
  buffer[7]  = ~buffer[3];
  // Address 2x with inverse address 2x
  buffer[12] = buffer[14] = blockAddr;
  buffer[13] = buffer[15] = ~blockAddr;
  
  // Write the whole data block
  return RC522_MIFARE_Write(blockAddr, buffer, 16);
}

/**
  * @brief  Authenticate with a NTAG216.
  * 
  * @note   Only for NTAG216. Authenticate with 32bit password.
  *         First implemented by Gargantuanman.
  * 
  * @param[in]  password : password (32bit).
  * @param[out]  pACK : result success.
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_PCD_NTAG216_AUTH(const uint8_t password[4], uint8_t *pACK)
{
  uint8_t cmdBuffer[7]; // We need room for 5 bytes data and 2 bytes CRC_A.
  cmdBuffer[0] = 0x1B; // Authentication command.
  for (size_t i = 0; i < 4; i++) 
  {
    cmdBuffer[i + 1] = password[i];
  }
  const uint8_t dataLength = 5;

  StatusCode result = RC522_CalculateCRC(cmdBuffer, dataLength, &cmdBuffer[dataLength]);
  if (result != STAT_OK) 
  {
    return result;
  }

  // Transceive the data, store the reply in cmdBuffer[]
  const uint8_t waitIRq = RxIRq_Msk | IdleIRq_Msk;
  uint8_t validBits = 0;
  uint8_t cmdBufferSize = sizeof(cmdBuffer);
  result = RC522_TxRxDataToPICC(RC522_Transceive, waitIRq, cmdBuffer, cmdBufferSize, 
    cmdBuffer, &cmdBufferSize, &validBits, 0, false);
  
  pACK[0] = cmdBuffer[0];
  pACK[1] = cmdBuffer[1];
  
  if (result != STAT_OK) 
  {
    return result;
  }
  
  return STAT_OK;
}

/*############################################################################*/
// Support functions
/*############################################################################*/

/**
  * @brief  Wrapper for MIFARE protocol communication.
  * @note   Adds CRC_A, executes the Transceive command and checks that
  *         the response is MF_ACK or a timeout.
  *
  * @param  sendData : Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
  * @param  sendLen : Number of bytes in sendData.
  * @param  acceptTimeout : True => A timeout is also success
  * @return STAT_OK on success, STAT_??? otherwise.
  */
StatusCode RC522_MIFARE_Transceive(const uint8_t *sendData, size_t sendLen, const bool acceptTimeout)
{
  uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
  
  // Sanity check
  if (sendData == NULL || sendLen > 16) 
  {
    return STAT_INVALID;
  }
  
  // Copy sendData[] to cmdBuffer[] and add CRC_A
  memcpy(cmdBuffer, sendData, sendLen);
  StatusCode result = RC522_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
  if (result != STAT_OK) 
  {
    return result;
  }
  sendLen += 2;
  
  // Transceive the data, store the reply in cmdBuffer[]
  const uint8_t waitIRq = RxIRq_Msk | IdleIRq_Msk;
  uint8_t cmdBufferSize = sizeof(cmdBuffer);
  uint8_t validBits = 0;
  result = RC522_TxRxDataToPICC(RC522_Transceive, waitIRq, cmdBuffer, 
    sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false);
  if (acceptTimeout && result == STAT_TIMEOUT) 
  {
    return STAT_OK;
  }
  if (result != STAT_OK) 
  {
    return result;
  }
  // The PICC must reply with a 4 bit ACK
  if (cmdBufferSize != 1 || validBits != 4) 
  {
    return STAT_ERROR;
  }
  if (cmdBuffer[0] != MF_ACK) 
  {
    return STAT_MIFARE_NACK;
  }
  return STAT_OK;
}

/**
  * @brief  Translates the SAK (Select Acknowledge) to a PICC type.
  * @param  sak : The SAK byte returned from PICC_Select().
  * @return PICC_Type
  */
PICC_Type RC522_PICC_GetType(void)
{
  // https://www.nxp.com/docs/en/application-note/AN10833.pdf
  // 2.4 Coding of Select Acknowledge (SAK)
  // ignore 8-bit (iso14443 starts with LSBit = bit 1)
  // fixes wrong type for manufacturer Infineon (https://nfc-tools.github.io/resources/standards/iso14443A/)
  switch (uid_.sak)
  {
  case 0x04:
    return PICC_TYPE_NOT_COMPLETE;  // UID not complete
  case 0x09:
    return PICC_TYPE_MIFARE_MINI;
  case 0x08:
    return PICC_TYPE_MIFARE_1K;
  case 0x18:
    return PICC_TYPE_MIFARE_4K;
  case 0x00:
    return PICC_TYPE_MIFARE_ULTRALIGHT;
  case 0x10:
  case 0x11:
    return PICC_TYPE_MIFARE_PLUS;
  case 0x01:
    return PICC_TYPE_TNP3XXX;
  case 0x20:
    return PICC_TYPE_ISO_14443_4;
  case 0x40:
    return PICC_TYPE_ISO_18092;
  default:
    return PICC_TYPE_UNKNOWN;
  }
}

/**
  * @brief  Calculates the bit pattern needed for the specified access bits.
  *         In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
  * @param  accessBitBuffer : Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
  * @param  g0 : Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
  * @param  g1 : Access bits [C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
  * @param  g2 : Access bits [C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
  * @param  g3 : Access bits [C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
  */
void RC522_MIFARE_CalculateAccessBits(uint8_t accessBitBuffer[3],
  const uint8_t g0, const uint8_t g1, const uint8_t g2, const uint8_t g3)
{
  const uint8_t c1 = (uint8_t)((g3 & 4) << 1) | (uint8_t)((g2 & 4) << 0) | (uint8_t)((g1 & 4) >> 1) | (uint8_t)((g0 & 4) >> 2);
  const uint8_t c2 = (uint8_t)((g3 & 2) << 2) | (uint8_t)((g2 & 2) << 1) | (uint8_t)((g1 & 2) << 0) | (uint8_t)((g0 & 2) >> 1);
  const uint8_t c3 = (uint8_t)((g3 & 1) << 3) | (uint8_t)((g2 & 1) << 2) | (uint8_t)((g1 & 1) << 1) | (uint8_t)((g0 & 1) << 0);
  
  accessBitBuffer[0] = (uint8_t)((~c2 & 0x0F) << 4) | (~c1 & 0x0F);
  accessBitBuffer[1] = (uint8_t)(c1 << 4) | (~c3 & 0x0F);
  accessBitBuffer[2] = (uint8_t)(c3 << 4) | c2;
}

/*############################################################################*/
// Convenience functions - does not add extra functionality
/*############################################################################*/

/**
  * @brief  Returns true if a PICC responds to PICC_CMD_REQA.
  *         Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
  * 
  * @return bool
  */
bool RC522_PICC_IsNewCardPresent(void)
{
  // Reset baud rates
  RC522_WriteRegister(TxModeReg, RESET_TxModeReg); // Section 9.3.2.3
  RC522_WriteRegister(RxModeReg, RESET_RxModeReg); // Section 9.3.2.4
  // Reset ModWidthReg
  RC522_WriteRegister(ModWidthReg, RESET_ModWidthReg); // Section 9.3.3.4

  uint8_t bufferATQA[2];
  uint8_t bufferSize = sizeof(bufferATQA);
  const StatusCode result = RC522_PICC_RequestA(bufferATQA, &bufferSize);
  return ((result == STAT_OK) || (result == STAT_COLLISION));
}

/**
  * @brief  Simple wrapper around RC522_PICC_Select.
  * @note   Remember to call RC522_PICC_IsNewCardPresent(), RC522_PICC_RequestA() or RC522_PICC_WakeupA() first.
  * @note   The read UID is available in the global variable uid_.
  * 
  * @return true if a UID could be read.
  */
bool RC522_PICC_ReadCardUid(void)
{
  return RC522_PICC_Select(0) == STAT_OK;
}

/**
  * @brief  The function is used to select a PICC with the same UID if the authentication fails.
  * @note   Remember to call RC522_PICC_IsNewCardPresent(), RC522_PICC_RequestA() or RC522_PICC_WakeupA() first.
  * @note   The read UID is available in the global variable uid_.
  * 
  * @return true if a UID could be read.
  */
bool RC522_PICC_SelectCardWithLastUid(void)
{
  return RC522_PICC_Select(uid_.size * 8) == STAT_OK;
}

/*############################################################################*/
// Functions for dump to serial
/*############################################################################*/
#if RC522_DUMP_TO_SERIAL_USED

#include "usart.h"

/**
  * @param  piccType : One of the PICC_Type enums.
  * @return Pointer to the PICC type name.
  */
const char* RC522_PICC_GetTypeName(const PICC_Type piccType)
{
  switch (piccType)
  {
  case PICC_TYPE_ISO_14443_4:
    return "PICC compliant with ISO/IEC 14443-4";
  case PICC_TYPE_ISO_18092:
    return "PICC compliant with ISO/IEC 18092 (NFC)";
  case PICC_TYPE_MIFARE_MINI:
    return "MIFARE Mini, 320 bytes";
  case PICC_TYPE_MIFARE_1K:
    return "MIFARE 1KB";
  case PICC_TYPE_MIFARE_4K:
    return "MIFARE 4KB";
  case PICC_TYPE_MIFARE_ULTRALIGHT:
    return "MIFARE Ultralight or Ultralight C";
  case PICC_TYPE_MIFARE_PLUS:
    return "MIFARE Plus";
  case PICC_TYPE_MIFARE_DESFIRE:
    return "MIFARE DESFire";
  case PICC_TYPE_TNP3XXX:
    return "MIFARE TNP3XXX";
  case PICC_TYPE_NOT_COMPLETE:
    return "SAK indicates UID is not complete.";
  case PICC_TYPE_UNKNOWN:
  default:
    return "Unknown type";
  }
}

/**
  * @param  code : One of the StatusCode enums.
  * @return Pointer to a status code name.
  */ 
const char* RC522_GetStatusCodeName(const StatusCode code)
{
  switch (code)
  {
  case STAT_OK:             return "Success.";
  case STAT_ERROR:          return "Error in communication.";
  case STAT_COLLISION:      return "Collision detected.";
  case STAT_TIMEOUT:        return "Timeout in communication.";
  case STAT_NO_ROOM:        return "A buffer is not big enough.";
  case STAT_INTERNAL_ERROR: return "Internal error in the code. Should not happen.";
  case STAT_INVALID:        return "Invalid argument.";
  case STAT_CRC_WRONG:      return "The CRC_A does not match.";
  case STAT_MIFARE_NACK:    return "A MIFARE PICC responded with NAK.";
  default:                  return "Unknown error";
  }
}

/**
  * @brief  Dumps debug info about the connected PCD to Serial.
  *         Shows all known firmware versions
  */
void RC522_DumpVersionToSerial(void)
{
  const RC522_FirmwareVersion version = RC522_GetVersion();
  if (version != Version_Unknown) 
  {
    printf("Firmware Version: 0x%02X = ", version);
  }

  switch (version)
  {
  case Version_FM17522:
    println("FM17522");
    break;
  case Version_FM17522_1:
    println("FM17522_1");
    break;
  case Version_FM17522E:
    println("FM17522E");
    break;
  case Version_0_0:
    println("v0.0");
    break;
  case Version_1_0:
    println("v1.0");
    break;
  case Version_2_0:
    println("v2.0");
    break;
  case Version_Counterfeit:
    println("counterfeit chip");
    break;
  default:
    println("Unknown firmware");
  }
  if (version == Version_Unknown)
    println("WARNING: Communication failure, is the MFRC522 properly connected?");
}

void RC522_PrintUID(void)
{
  for (size_t i = 0; i < uid_.size; i++) 
  {
    printf("%02X ", uid_.uidByte[i]);
  }
}

/**
 * @brief  Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 */
void RC522_PICC_DumpDetailsToSerial(void)
{
  // UID
  printf("Card UID: ");
  RC522_PrintUID();
  println(NULL);
	
  // SAK
  println("Card SAK: %02X", uid_.sak);
	
  // (suggested) PICC type
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));
}

/**
  * @brief  Dumps memory contents of a sector of a MIFARE Classic PICC.
  *         Uses RC522_Authenticate(), RC522_MIFARE_Read() and RC522_StopCrypto1.
  *         Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read
  *         the sector trailer access bits.
  * @param  key : Key A for the sector.
  * @param  sector : The sector to dump, 0..39.
  */
void RC522_PICC_DumpMifareClassicSectorToSerial(const MIFARE_Key *key, const uint8_t sector)
{
  // Determine position and size of sector.
  uint8_t firstBlock; // Address of lowest address to dump actually last block dumped)
  uint8_t noOfBlocks; // Number of blocks in sector
  if (sector < 32) 
  {
    // Sectors 0..31 has 4 blocks each
    noOfBlocks = 4;
    firstBlock = sector * noOfBlocks;
  }
  else if (sector < 40) 
  {
    // Sectors 32-39 has 16 blocks each
    noOfBlocks = 16;
    firstBlock = 128 + (sector - 32) * noOfBlocks;
  }
  else 
  {
    // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
    return;
  }

  bool isSectorTrailer = true; // Set to true while handling the "last" (ie highest address) in the sector.
  bool invertedError = false; // True if one of the inverted nibbles did not match
  for (int8_t blockOffset = (int8_t)(noOfBlocks - 1); blockOffset >= 0; blockOffset--) 
  {
    uint8_t blockAddr = firstBlock + (uint8_t)blockOffset;
    // Sector number - only on first line
    if (isSectorTrailer) 
    {
      printf("  %2u   ", sector);
    }
    else 
    {
      printf("       ");
    }
    // Block number
    printf("%3u   ", blockAddr);
    // Establish encrypted communications before reading the first block
    if (isSectorTrailer) 
    {
      const StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key);
      if (status != STAT_OK) 
      {
        printf("RC522_Authenticate() failed: ");
        println(RC522_GetStatusCodeName(status));
        return;
      }
    }
    // Read block
    uint8_t buffer[18]; 
    uint8_t byteCount = sizeof(buffer);
    const StatusCode status = RC522_MIFARE_Read(blockAddr, buffer, &byteCount);
    if (status != STAT_OK) 
    {
      printf("RC522_MIFARE_Read() failed: ");
      println(RC522_GetStatusCodeName(status));
      continue;
    }
    // Dump data
    for (size_t index = 0; index < 16; index++) 
    {
      printf(" %02X", buffer[index]);
      if ((index % 4) == 3) 
      {
        printf(" ");
      }
    }
    // Parse sector trailer data
    uint8_t g[4]; // Access bits for each of the four groups.
    if (isSectorTrailer) 
    {
      // The access bits are stored in a peculiar fashion.
      // There are four groups:
      //    g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
      //    g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
      //    g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
      //    g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
      // Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
      // The four CX bits are stored together in a nibble cx and an inverted nibble cx_.
      // Nibbles
      const uint8_t c1 = buffer[7] >> 4;
      const uint8_t c2 = buffer[8] & 0x0F;
      const uint8_t c3 = buffer[8] >> 4;
      // Inverted nibbles
      const uint8_t c1_ = buffer[6] & 0x0F;
      const uint8_t c2_ = buffer[6] >> 4;
      const uint8_t c3_ = buffer[7] & 0x0F;
      invertedError = (c1 != (~c1_ & 0x0F)) || (c2 != (~c2_ & 0x0F)) || (c3 != (~c3_ & 0x0F));
      g[0] = (uint8_t)((c1 & 1) << 2) | (uint8_t)((c2 & 1) << 1) | (uint8_t)((c3 & 1) << 0);
      g[1] = (uint8_t)((c1 & 2) << 1) | (uint8_t)((c2 & 2) << 0) | (uint8_t)((c3 & 2) >> 1);
      g[2] = (uint8_t)((c1 & 4) << 0) | (uint8_t)((c2 & 4) >> 1) | (uint8_t)((c3 & 4) >> 2);
      g[3] = (uint8_t)((c1 & 8) >> 1) | (uint8_t)((c2 & 8) >> 2) | (uint8_t)((c3 & 8) >> 3);
      isSectorTrailer = false;
    }
    
    // Which access group is this block in?
    uint8_t group; // 0-3 - active group for access bits
    bool firstInGroup; // True for the first block dumped in the group
    if (noOfBlocks == 4) 
    {
      group = blockOffset;
      firstInGroup = true;
    }
    else 
    {
      group = blockOffset / 5;
      firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
    }
    
    if (firstInGroup) 
    {
      // Print access bits
      printf(" [ %u %u %u ] ", (g[group] >> 2) & 1, (g[group] >> 1) & 1, (g[group] >> 0) & 1);
      if (invertedError) 
      {
        printf(" Inverted access bits did not match! ");
      }
    }
    
    if (group != 3 && (g[group] == 1 || g[group] == 6)) 
    {
      // Not a sector trailer, a value block
      printf(" Value=0x%02X 0x%02X 0x%02X 0x%02X", buffer[3], buffer[2], buffer[1], buffer[0]);
      printf(" Adr=0x%02X", buffer[12]);
    }
    println(NULL);
  }
}

/**
  * @brief  Dumps memory contents of a MIFARE Classic PICC.
  *         On success the PICC is halted after dumping the data.
  * @param  piccType : One of the PICC_Type enums.
  * @param  key : Key A used for all sectors.
  */
static void RC522_PICC_DumpMifareClassicToSerial(const PICC_Type piccType, MIFARE_Key *key)
{
  int8_t noOfSectors = 0;
  switch (piccType)
  {
  case PICC_TYPE_MIFARE_MINI:
    // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
    noOfSectors = 5;
    break;
    
  case PICC_TYPE_MIFARE_1K:
    // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
    noOfSectors = 16;
    break;
    
  case PICC_TYPE_MIFARE_4K:
    // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
    noOfSectors = 40;
    break;
    
  default: // Should not happen. Ignore.
    break;
  }
  
  // Dump sectors, highest address first.
  if (noOfSectors > 0) 
  {
    println("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits");
    for (int8_t i = (int8_t)(noOfSectors - 1); i >= 0; i--) 
    {
      RC522_PICC_DumpMifareClassicSectorToSerial(key, i);
    }
  }
  RC522_PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
  RC522_StopCrypto1();
}

/**
  * @brief  Dumps memory contents of a MIFARE Ultralight PICC.
  */
static void RC522_PICC_DumpMifareUltralightToSerial(void)
{
  printf("Page  0  1  2  3");
  // Try the mpages of the original Ultralight. Ultralight C has more pages.
  for (uint8_t page = 0; page < 16; page += 4) 
  {
    // Read returns data for 4 pages at a time.
    // Read pages
    uint8_t buffer[18];
    uint8_t byteCount = sizeof(buffer);
    const StatusCode status = RC522_MIFARE_Read(page, buffer, &byteCount);
    if (status != STAT_OK) 
    {
      printf("MIFARE_Read() failed: ");
      println(RC522_GetStatusCodeName(status));
      break;
    }
    // Dump data
    for (uint8_t offset = 0; offset < 4; offset++) 
    {
      printf(" %2u  ", page + offset);
      for (uint8_t index = 0; index < 4; index++) 
      {
        const size_t i = 4 * offset + index;
        printf("%02X ", buffer[i]);
      }
      println(NULL);
    }
  }
}

/**
  * @brief  Dumps debug info about the selected PICC to Serial.
  *         On success the PICC is halted after dumping the data.
  * @note   For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.  
  */
void RC522_PICC_DumpToSerial(void)
{
  // Dump UID, SAK and Type
  RC522_PICC_DumpDetailsToSerial();
  
  // Dump contents
  const PICC_Type piccType = RC522_PICC_GetType();
  switch (piccType)
  {
  case PICC_TYPE_MIFARE_MINI:
  case PICC_TYPE_MIFARE_1K:
  case PICC_TYPE_MIFARE_4K:
    {
      // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
      MIFARE_Key key;
      for (size_t i = 0; i < MF_KEY_SIZE; i++) 
      {
        key.keyByte[i] = 0xFF;
      }
      RC522_PICC_DumpMifareClassicToSerial(piccType, &key);
      break;
    }
  case PICC_TYPE_MIFARE_ULTRALIGHT:
    RC522_PICC_DumpMifareUltralightToSerial();
    break;
    
  case PICC_TYPE_ISO_14443_4:
  case PICC_TYPE_MIFARE_DESFIRE:
  case PICC_TYPE_ISO_18092:
  case PICC_TYPE_MIFARE_PLUS:
  case PICC_TYPE_TNP3XXX:
    println("Dumping memory contents not implemented for that PICC type.");
    break;
    
  case PICC_TYPE_UNKNOWN:
  case PICC_TYPE_NOT_COMPLETE:
  default:
    break; // No memory dump here
  }
  
  println(NULL);
  RC522_PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
}
#endif // RC522_DUMP_TO_SERIAL_USED

/*############################################################################*/
// Solves problems with Chinese PICC cards
/*############################################################################*/

/**
  * @brief  Performs the "magic sequence" needed to get Chinese UID changeable
  *         Mifare cards to allow writing to sector 0, where the card UID is stored.
  *
  * @note   Note that you do not need to have selected the card through REQA or WUPA,
  *         this sequence works immediately when the card is in the reader vicinity.
  *         This means you can use this method even on "bricked" cards that your reader does
  *         not recognize anymore (see RC522_MIFARE_UnbrickUidSector).
  * 
  * @note   Of course with non-bricked devices, you're free to select them before calling this function.
  */
bool RC522_MIFARE_OpenUidBackdoor(void)
{
  // Magic sequence:
  // > 50 00 57 CD (HALT + CRC)
  // > 40 (7 bits only)
  // < A (4 bits only)
  // > 43
  // < A (4 bits only)
  // Then you can write to sector 0 without authenticating

  RC522_PICC_HaltA(); // 50 00 57 CD
  
  uint8_t cmd = 0x40;
  uint8_t validBits = 7; // Our command is only 7 bits. After receiving card response, this will contain amount of valid response bits.
  uint8_t response[32] = { 0 }; // Card's response is written here.
  uint8_t received = sizeof(response);

  StatusCode status = RC522_TransceiveData(&cmd, 1, 
    response, &received, &validBits, 0, false); // 40
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    println("Card did not respond to 0x40 after HALT command. Are you sure it is a UID changeable one?");
    println("Error name: %s", RC522_GetStatusCodeName(status));
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  if (received != 1 || response[0] != 0x0A) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    println("Got bad response on backdoor 0x40 command: %02X (%u valid bits)", response[0], validBits);
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }

  cmd = 0x43;
  validBits = 8;
  status = RC522_TransceiveData(&cmd, 1, 
    response, &received, &validBits, 0, false); // 43
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    println("Error in communication at command 0x43, after successfully executing 0x40");
    println("Error name: %s", RC522_GetStatusCodeName(status));
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  if (received != 1 || response[0] != 0x0A) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    println("Got bad response on backdoor 0x43 command: %02X (%u valid bits)", response[0], validBits);
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }

  // You can now write to sector 0 without authenticating!
  return true;
}

/**
  * @brief  Reads entire block 0, including all manufacturer data, and overwrites
  *         that block with the new UID, a freshly calculated BCC, and the original
  *         manufacturer data.
  *
  * @note   The common default KEY A is 0xFFFFFFFFFFFF.
  * @note   Make sure to have selected the card before this function is called.
  *
  * @param  newUid :
  * @param  uidSize :
  * @param  key :
  * @param  withBackdoor : 
  */
bool RC522_MIFARE_SetUid(const uint8_t *const newUid, const size_t uidSize, const MIFARE_Key *key, const bool withBackdoor)
{
  // UID + BCC byte can not be larger than 16 together
  if (!newUid || !uidSize || uidSize > 15) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    println("New UID buffer empty, size 0, or size > 15 given");
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }

  // Authenticate for reading
  StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, 0, key);
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    println("Are you sure your KEY A for sector 0 is correct?");
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  
  // Read block 0
  uint8_t block0Buffer[18]; 
  uint8_t byteCount = sizeof(block0Buffer);
  status = RC522_MIFARE_Read(0, block0Buffer, &byteCount);
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    printf("RC522_MIFARE_Read() failed: ");
    println(RC522_GetStatusCodeName(status));
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  
  // Write new UID to the data we just read, and calculate BCC byte.
  uint8_t bcc = 0;
  for (size_t i = 0; i < uidSize; i++) 
  {
    block0Buffer[i] = newUid[i];
    bcc ^= newUid[i];
  }
  
  // Write BCC byte to buffer.
  block0Buffer[uidSize] = bcc;
  
  // Some cards do not need the backdoor. They are writable directly.
  if (withBackdoor) 
  {
    // Stop encrypted traffic so we can send raw bytes.
    RC522_StopCrypto1();
    
    // Activate UID backdoor.
    if (!RC522_MIFARE_OpenUidBackdoor()) 
    {
#if RC522_DUMP_TO_SERIAL_USED
      println("Activating the UID backdoor failed.");
#endif // RC522_DUMP_TO_SERIAL_USED
      return false;
    }
  }
  
  // Write modified block 0 back to card.
  status = RC522_MIFARE_Write(0, block0Buffer, 16);
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    printf("RC522_MIFARE_Write() failed: ");
    println(RC522_GetStatusCodeName(status));
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  
  // Some cards do not need the backdoor. They are writeable directly.
  if (withBackdoor) 
  {
    // Wake the card up again.
    uint8_t atqaAnswer[2];
    uint8_t atqaSize = sizeof(atqaAnswer);
    RC522_PICC_WakeupA(atqaAnswer, &atqaSize);
  }
  
  return true;
}

/**
  * @brief  Resets entire sector 0 to zeroes, so the card can be read again by readers.
  */
bool RC522_MIFARE_UnbrickUidSector(void)
{
  RC522_MIFARE_OpenUidBackdoor();

  const uint8_t block0Buffer[] = { 0x01, 0x02, 0x03, 0x04,
                                   0x04, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00 };
  
  // Write modified block 0 back to card.
  const StatusCode status = RC522_MIFARE_Write(0, block0Buffer, 16);
  if (status != STAT_OK) 
  {
#if RC522_DUMP_TO_SERIAL_USED
    printf("MIFARE_Write() failed: ");
    println(RC522_GetStatusCodeName(status));
#endif // RC522_DUMP_TO_SERIAL_USED
    return false;
  }
  return true;
}
