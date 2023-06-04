/**
  ******************************************************************************
  * @file           : rc522.h
  * @author         : Rusanov M.N.
  * @version        : V1.0.0
  * @date           : 03-June-2023
  * @brief          : Header for rc522.c file.
  *                   This file contains functions for working with MIFARE (RFID)
  *                   cards over the SPI interface.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RC522_H_
#define RC522_H_

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f7xx_hal.h"
#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define RC522_HSPI     hspi2
#define RC522_CS_PORT  RC522_CS_GPIO_Port
#define RC522_CS_PIN   RC522_CS_Pin

#define RC522_DUMP_TO_SERIAL_USED     1 // 1 - used, 0 - not used


// Commands sent to the PICC.
typedef enum
{
  // The commands used by the MFRC522 to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
  PICC_CMD_REQA = 0x26, // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anti-collision or selection. 7 bit frame.
  PICC_CMD_WUPA = 0x52, // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anti-collision or selection. 7 bit frame.
  PICC_CMD_CT = 0x88, // Cascade Tag. Not really a command, but used during anti collision.
  PICC_CMD_SEL_CL1 = 0x93, // Anti-collision/Select, Cascade Level 1
  PICC_CMD_SEL_CL2 = 0x95, // Anti-collision/Select, Cascade Level 2
  PICC_CMD_SEL_CL3 = 0x97, // Anti-collision/Select, Cascade Level 3
  PICC_CMD_HLTA = 0x50, // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
  PICC_CMD_RATS = 0xE0, // Request command for Answer To Reset.
  // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
  // Use RC522_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
  // The read/write commands can also be used for MIFARE Ultralight.
  PICC_CMD_MF_AUTH_KEY_A = 0x60, // Perform authentication with Key A
  PICC_CMD_MF_AUTH_KEY_B = 0x61, // Perform authentication with Key B
  PICC_CMD_MF_READ = 0x30, // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
  PICC_CMD_MF_WRITE = 0xA0, // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
  PICC_CMD_MF_DECREMENT = 0xC0, // Decrements the contents of a block and stores the result in the internal data register.
  PICC_CMD_MF_INCREMENT = 0xC1, // Increments the contents of a block and stores the result in the internal data register.
  PICC_CMD_MF_RESTORE = 0xC2, // Reads the contents of a block into the internal data register.
  PICC_CMD_MF_TRANSFER = 0xB0, // Writes the contents of the internal data register to a block.
  // The commands used for MIFARE Ultralight (from http://www.nxp.com/docs/en/data-sheet/MF0ICU1.pdf, Section 8.6)
  // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
  PICC_CMD_UL_WRITE = 0xA2		// Writes one 4 byte page to the PICC.
} PICC_Command;

typedef enum // MIFARE constants that does not fit anywhere else
{
  MF_ACK = 0x0A, // The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0x0A is NAK.
  MF_KEY_SIZE = 6 // A Mifare Crypto1 key is 6 bytes.
} MIFARE_Misc;

// PICC types we can detect. Remember to update RC522_PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum
{
  PICC_TYPE_UNKNOWN,
  PICC_TYPE_ISO_14443_4, // PICC compliant with ISO/IEC 14443-4. 
  PICC_TYPE_ISO_18092, // PICC compliant with ISO/IEC 18092 (NFC).
  PICC_TYPE_MIFARE_MINI, // MIFARE Classic protocol, 320 bytes.
  PICC_TYPE_MIFARE_1K, // MIFARE Classic protocol, 1KB.
  PICC_TYPE_MIFARE_4K, // MIFARE Classic protocol, 4KB.
  PICC_TYPE_MIFARE_ULTRALIGHT, // MIFARE Ultralight or Ultralight C.
  PICC_TYPE_MIFARE_PLUS, // MIFARE Plus.
  PICC_TYPE_MIFARE_DESFIRE, // MIFARE DESFire.
  PICC_TYPE_TNP3XXX, // Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure.
  PICC_TYPE_NOT_COMPLETE = 0xff // SAK indicates UID is not complete.
} PICC_Type;

// Return codes from the functions. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimizations are triggered
typedef enum
{
  STAT_OK, // Success
  STAT_ERROR, // Error in communication
  STAT_COLLISION, // Collision detected
  STAT_TIMEOUT, // Timeout in communication.
  STAT_NO_ROOM, // A buffer is not big enough.
  STAT_INTERNAL_ERROR, // Internal error in the code. Should not happen ;-)
  STAT_INVALID, // Invalid argument.
  STAT_CRC_WRONG, // The CRC_A does not match
  STAT_MIFARE_NACK = 0xff	// A MIFARE PICC responded with NAK.
} StatusCode;

// Version of chip / firmware.
typedef enum
{
  Version_Counterfeit = 0x12,
  Version_FM17522     = 0x88,
  Version_FM17522_1   = 0xb2,
  Version_FM17522E    = 0x89,
  Version_0_0         = 0x90,
  Version_1_0         = 0x91,
  Version_2_0         = 0x92,
  Version_Unknown     = 0xff,
} RC522_FirmwareVersion;

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the RC522).
// Described in 9.3.3.6 / table 98
typedef enum
{
  RxGain_18db = 0x00, // 5 mm
  RxGain_23db = 0x10, // 10 mm
  RxGain_18db_2 =  0x20, // seems it is a duplicate for RxGain_18db
  RxGain_23db_2 =  0x30, // seems it is a duplicate for RxGain_23db
  RxGain_33db =  0x40, // default  // 20 mm
  RxGain_38db =  0x50, // 30 mm - Best working distance
  RxGain_43db =  0x60, // 32 mm
  RxGain_48db =  0x70, // 33 mm, Not all PICC's work!
  RxGain_min = RxGain_18db,
  RxGain_med = RxGain_33db,
  RxGain_max = RxGain_48db
} RC522_RxGain;

typedef struct {
  uint8_t keyByte[MF_KEY_SIZE];
} MIFARE_Key;


/*############################################################################*/
// Functions for manipulating the MFRC522
/*############################################################################*/
bool RC522_Init(void);
void RC522_Reset(void);
void RC522_AntennaOn(void);
void RC522_AntennaOff(void);
RC522_RxGain RC522_GetAntennaGain(void);
void RC522_SetAntennaGain(const RC522_RxGain gain);
RC522_FirmwareVersion RC522_GetVersion(void);
bool RC522_PerformSelfTest(void);

/*############################################################################*/
// Power control
/*############################################################################*/
void RC522_SoftPowerDown(void);
void RC522_SoftPowerUp(void);

/*############################################################################*/
// Functions for communicating with PICCs
/*############################################################################*/
StatusCode RC522_TransceiveData(uint8_t *const sendData, const size_t sendLen, 
  uint8_t *backData, uint8_t *backLen, uint8_t *validBits, const uint8_t rxAlign, const bool checkCRC);
StatusCode RC522_PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode RC522_PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode RC522_PICC_REQA_or_WUPA(PICC_Command command, uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode RC522_PICC_HaltA(void);

/*############################################################################*/
// Functions for communicating with MIFARE PICCs
/*############################################################################*/
StatusCode RC522_Authenticate(const PICC_Command command, const uint8_t blockAddr, const MIFARE_Key *key);
void RC522_StopCrypto1(void);
StatusCode RC522_MIFARE_Read(const uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize);
StatusCode RC522_MIFARE_Write(const uint8_t blockAddr, const uint8_t *buffer, const size_t bufferSize);
StatusCode RC522_MIFARE_Ultralight_Write(const uint8_t page, const uint8_t *buffer, const size_t bufferSize);
StatusCode RC522_MIFARE_Increment(const uint8_t blockAddr, const int32_t delta);
StatusCode RC522_MIFARE_Decrement(const uint8_t blockAddr, const int32_t delta);
StatusCode RC522_MIFARE_Restore(const uint8_t blockAddr);
StatusCode RC522_MIFARE_Transfer(const uint8_t blockAddr);
StatusCode RC522_MIFARE_GetValue(const uint8_t blockAddr, int32_t *value);
StatusCode RC522_MIFARE_SetValue(const uint8_t blockAddr, const int32_t value);
StatusCode RC522_PCD_NTAG216_AUTH(const uint8_t password[4], uint8_t *pACK);

/*############################################################################*/
// Support functions
/*############################################################################*/
StatusCode RC522_MIFARE_Transceive(const uint8_t *sendData, size_t sendLen, const bool acceptTimeout);
PICC_Type RC522_PICC_GetType(void);

// Advanced functions for MIFARE
void RC522_MIFARE_CalculateAccessBits(uint8_t accessBitBuffer[3], const uint8_t g0, const uint8_t g1, const uint8_t g2, const uint8_t g3);

/*############################################################################*/
// Convenience functions - does not add extra functionality
/*############################################################################*/
bool RC522_PICC_IsNewCardPresent(void);
bool RC522_PICC_ReadCardUid(void);
bool RC522_PICC_SelectCardWithLastUid(void);

/*############################################################################*/
// Solves problems with Chinese PICC cards
/*############################################################################*/
bool RC522_MIFARE_OpenUidBackdoor(void);
bool RC522_MIFARE_SetUid(const uint8_t *const newUid, const size_t uidSize, const MIFARE_Key *key, const bool withBackdoor);
bool RC522_MIFARE_UnbrickUidSector(void);

/*############################################################################*/
// Functions for dump to serial
/*############################################################################*/
#if RC522_DUMP_TO_SERIAL_USED
const char* RC522_PICC_GetTypeName(const PICC_Type piccType);
const char* RC522_GetStatusCodeName(const StatusCode code);
void RC522_DumpVersionToSerial(void);
void RC522_PrintUID(void);
void RC522_PICC_DumpDetailsToSerial(void);
void RC522_PICC_DumpMifareClassicSectorToSerial(const MIFARE_Key *key, const uint8_t sector);
void RC522_PICC_DumpToSerial(void);
#endif // RC522_DUMP_TO_SERIAL_USED

#endif // RC522_H_
