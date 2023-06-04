/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma2d.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STAT_KEY_NOT_TRUE = 0x00,
  STAT_KEY_A_OK = 0x01,
  STAT_KEY_B_OK = 0x02
} StatusAuthenticate;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PICC_MEMORY_SIZE 1024
#define PICC_MEMORY_BLOCK_SIZE 16
#define PICC_MEMORY_BLOCKS (PICC_MEMORY_SIZE / PICC_MEMORY_BLOCK_SIZE)

#define USE_TEST1 0 // Check firmware RC522
#define USE_TEST2 0 // Dump Uid, SAK and Type
#define USE_TEST3 0 // Dump full data to serial
#define USE_TEST4 0 // Change Uid
#define USE_TEST5 0 // Fix a broken Uid
#define USE_TEST6 0 // Setup blocks on a MIFARE Classic
#define USE_TEST7 1 // RFID-Cloner
#define USE_TEST8 0 // Read and Write
#define USE_TEST9 0 // Write personal data
#define USE_TEST10 0 // Read personal data
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Known keys, see: https://code.google.com/p/mfcuk/wiki/MifareClassicDefaultKeys
const uint8_t knownKeys[][MF_KEY_SIZE] =  {
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // FF FF FF FF FF FF = factory default
  {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5}, // A0 A1 A2 A3 A4 A5
  {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5}, // B0 B1 B2 B3 B4 B5
  {0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD}, // 4D 3A 99 C3 51 DD
  {0x1A, 0x98, 0x2C, 0x7E, 0x45, 0x9A}, // 1A 98 2C 7E 45 9A
  {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7}, // D3 F7 D3 F7 D3 F7
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, // AA BB CC DD EE FF
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 00 00 00 00 00 00
  {0xE5, 0x6A, 0xC1, 0x27, 0xDD, 0x45}, // E5 6A C1 27 DD 45
  {0x19, 0xFC, 0x84, 0xA3, 0x78, 0x4B}, // 19 FC 84 A3 78 4B
  {0x77, 0xDA, 0xBC, 0x98, 0x25, 0xE1}, // 77 DA BC 98 25 E1
  {0x26, 0x97, 0x3E, 0xA7, 0x43, 0x21}, // 26 97 3E A7 43 21
  {0xEB, 0x0A, 0x8F, 0xF8, 0x8A, 0xDE}, // EB 0A 8F F8 8A DE
  {0xEA, 0x0F, 0xD7, 0x3C, 0xB1, 0x49}, // EA 0F D7 3C B1 49
  {0xC7, 0x6B, 0xF7, 0x1A, 0x25, 0x09}, // C7 6B F7 1A 25 09
  {0xAC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // AC FF FF FF FF FF
  {0x71, 0xF3, 0xA3, 0x15, 0xAD, 0x26}, // 71 F3 A3 15 AD 26
  {0x72, 0xF9, 0x6B, 0xDD, 0x37, 0x14}, // 72 F9 6B DD 37 14
  {0x04, 0x4C, 0xE1, 0x87, 0x2B, 0xC3}, // 04 4C E1 87 2B C3
  {0xBC, 0x2D, 0x17, 0x91, 0xDE, 0xC1}, // BC 2D 17 91 DE C1
  {0x87, 0x91, 0xB2, 0xCC, 0xB5, 0xC4}, // 87 91 B2 CC B5 C4
  {0x8E, 0x26, 0xE4, 0x5E, 0x7D, 0x65}, // 8E 26 E4 5E 7D 65
  {0x0F, 0x31, 0x81, 0x30, 0xED, 0x18}, // 0F 31 81 30 ED 18
  {0x04, 0x5C, 0xEC, 0xA1, 0x55, 0x35}, // 04 5C EC A1 55 35
  {0x9D, 0x99, 0x3C, 0x5D, 0x4E, 0xF4}, // 9D 99 3C 5D 4E F4
  {0xC6, 0x5D, 0x4E, 0xAA, 0x64, 0x5B}, // C6 5D 4E AA 64 5B
  {0x3A, 0x8A, 0x13, 0x9C, 0x20, 0xB4}, // 3A 8A 13 9C 20 B4
  {0xBA, 0xFF, 0x30, 0x53, 0xB4, 0x96}, // BA FF 30 53 B4 96
  {0x74, 0x13, 0xB5, 0x99, 0xC4, 0xEA}, // 74 13 B5 99 C4 EA
  {0x0C, 0xE7, 0xCD, 0x2C, 0xC7, 0x2B}, // 0C E7 CD 2C C7 2B
  {0x0B, 0xE5, 0xFA, 0xC8, 0xB0, 0x6A}, // 0B E5 FA C8 B0 6A
  {0x0E, 0xB2, 0x3C, 0xC8, 0x11, 0x0B}, // 0E B2 3C C8 11 0B
  {0xBC, 0x45, 0x80, 0xB7, 0xF2, 0x0B}, // BC 45 80 B7 F2 0B
  {0x7A, 0x39, 0x6F, 0x0D, 0x63, 0x3D}, // 7A 39 6F 0D 63 3D
  {0xA3, 0xFA, 0xA6, 0xDA, 0xFF, 0x67}, // A3 FA A6 DA FF 67
  {0xFD, 0x87, 0x05, 0xE7, 0x21, 0xB0}, // FD 87 05 E7 21 B0
  {0x22, 0x05, 0x2B, 0x48, 0x0D, 0x11}, // 22 05 2B 48 0D 11
  {0xA7, 0x14, 0x11, 0x47, 0xD4, 0x30}, // A7 14 11 47 D4 30
  {0x8A, 0x8D, 0x88, 0x15, 0x1A, 0x00}, // 8A 8D 88 15 1A 00
  {0xB2, 0x7A, 0xDD, 0xFB, 0x64, 0xB0}, // B2 7A DD FB 64 B0
  {0x72, 0x59, 0xFA, 0x01, 0x97, 0xC6}  // 72 59 FA 01 97 C6
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CheckFirmwareRC522(void);
void Test2_ReadDataFromPiccToSerial(void);
void Test3_ReadFullDataFromPiccToSerial(void);
void PrintUidAndType(void);
void Test4_ChangeUid(const MIFARE_Key *key);
void Test5_FixABrokenUid(void);
void Test6_SetupBlocksOnMIFARE(const MIFARE_Key *key);
void DumpByteArrayHEX(uint8_t *buffer, size_t bufferSize);
void FormatValueBlock(uint8_t blockAddr);
void Test7_RFID_Cloner(void);
void ReadCard(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE]);
void TestValuesInBlocks(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE]);
void CopyDataInTheNewCard(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE]);
void Test8_ReadAndWrite(MIFARE_Key *key);
void Test9_WritePersonalData(MIFARE_Key *key);
void Test10_ReadPersonalData(MIFARE_Key *key);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  const bool isInitRC522Ok = RC522_Init();
  MIFARE_Key key;

  if (isInitRC522Ok)
  {
#if USE_TEST1
    CheckFirmwareRC522();
#endif // USE_TEST1

#if USE_TEST2
    RC522_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details.
    println("Scan PICC to see UID, SAK, and type");
#endif // USE_TEST2

#if USE_TEST3
    RC522_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details.
    println("Scan PICC to see UID, SAK, type, and data blocks...");
#endif // USE_TEST3

#if USE_TEST4 || USE_TEST5
    // I had no success with this tests.
    // I haven't a card that allows to overwrite the UID
    println("Warning: this example overwrites the UID of your UID changeable card, use with care!");
    println("This example only works with MIFARE Classic cards.");

    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = 0xFF;
    }
#endif // USE_TEST4 || USE_TEST5

#if USE_TEST6
    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = 0xFF;
    }

    println("Scan a MIFARE Classic PICC to demonstrate Value Block mode.");
    printf("Using key (for A and B):");
    DumpByteArrayHEX(key.keyByte, MF_KEY_SIZE);
    println(NULL);
    
    println("BEWARE: Data will be written to the PICC, in sector #1");
#endif // USE_TEST6

#if USE_TEST7
    println("Try the most used default keys to print block 0 to 63 of a MIFARE PICC.");
    println("1.Read card \r\n2.View buffer \r\n3.Copy the data.");
#endif // USE_TEST7

#if USE_TEST8
    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = 0xFF;
    }

    println("Scan a MIFARE Classic PICC to demonstrate read and write.");
    printf("Using key(for A and B) :");
    DumpByteArrayHEX(key.keyByte, MF_KEY_SIZE);
    println(NULL);
    println("BEWARE: Data will be written to the PICC, in sector #1");
#endif // USE_TEST8

#if USE_TEST9
    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = 0xFF;
    }

    println("Write personal data on a MIFARE PICC:");
#endif // USE_TEST9

#if USE_TEST10
    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = 0xFF;
    }

    println("Read personal data on a MIFARE PICC:");
#endif // USE_TEST10
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (isInitRC522Ok)
    {
#if USE_TEST2
      Test2_ReadDataFromPiccToSerial();
#endif // USE_TEST2
      
#if USE_TEST3
      Test3_ReadFullDataFromPiccToSerial();
#endif // USE_TEST3

#if USE_TEST4
      Test4_ChangeUid(&key);
#endif // USE_TEST4

#if USE_TEST5
      Test5_FixABrokenUid();
#endif // USE_TEST5

#if USE_TEST6
      Test6_SetupBlocksOnMIFARE(&key);
#endif // USE_TEST6

#if USE_TEST7
      Test7_RFID_Cloner();
#endif // USE_TEST7

#if USE_TEST8
      Test8_ReadAndWrite(&key);
#endif // USE_TEST8

#if USE_TEST9
      Test9_WritePersonalData(&key);
#endif // USE_TEST9

#if USE_TEST10
      Test10_ReadPersonalData(&key);
#endif // USE_TEST10
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Example to test your firmware.
  *
  * This example test the firmware of your MFRC522 reader module, only known version
  * can be checked. If the test passed it do not mean that your module is faultless!
  * Some modules have bad or broken antennas or the PICC is broken.
  */
void CheckFirmwareRC522(void)
{
  println("*****************************");
  println("MFRC522 Digital self test");
  println("*****************************");
  RC522_DumpVersionToSerial(); // Show version of PCD - MFRC522 Card Reader
  println("-----------------------------");
  println("Only known versions supported");
  println("-----------------------------");
  println("Performing test...");
  const bool result = RC522_PerformSelfTest(); // perform the test
  println("-----------------------------");
  printf("Result: ");
  println((result) ? "OK" : "DEFECT or UNKNOWN");
}

/**
  * @brief Example showing how to read data from a PICC to serial.
  *
  * Example program showing how to read data from a PICC (that is: a RFID Tag or Card)
  * using a MFRC522 based RFID Reader.
  *
  * When you present a PICC (that is: a RFID Tag or Card) at reading distance of
  * the MFRC522 Reader, the serial output will show the ID/UID, type and any data
  * blocks it can read. Note: you may see "Timeout in communication" messages when
  * removing the PICC from reading distance too early.
  *
  * If your reader supports it, this program will read all the PICCs presented
  * (that is: multiple tag reading). So if you stack two or more PICCs on top of
  * each other and present them to the reader, it will first output all details of
  * the first and then the next PICC. Note that this may take some time as all data
  * blocks are dumped, so keep the PICCs at reading distance until complete.
  */
void Test2_ReadDataFromPiccToSerial(void)
{
  if (RC522_PICC_IsNewCardPresent() && RC522_PICC_ReadCardUid()) 
  {
    PrintUidAndType();
  }
}

void Test3_ReadFullDataFromPiccToSerial(void)
{
  if (RC522_PICC_IsNewCardPresent() && RC522_PICC_ReadCardUid()) 
  {
    RC522_PICC_DumpToSerial();
  }
}

/**
  * @brief Example showing how to read data from PICC to serial.
  */
void PrintUidAndType(void)
{
  RC522_PICC_DumpDetailsToSerial();

  // Halt PICC.
  RC522_PICC_HaltA();
  // Stop encryption on PCD.
  RC522_StopCrypto1();
}

/**
  * @brief Example to change UID of changeable MIFARE card.
  *
  * This sample shows how to set the UID on a UID changeable MIFARE card.
  */
void Test4_ChangeUid(const MIFARE_Key *key)
{
  /* Set your new UID here! */
  const uint8_t newUid[] = { 0xDE, 0xAD, 0xBE, 0xEF };

  // Reset the loop if no new card present on the sensor/reader.
  // This saves the entire process when idle. And if present, select one.
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid()) 
  {
    return;
  }
    
  // Dump UID.
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  
  // Set new UID.
  if (RC522_MIFARE_SetUid(newUid, sizeof(newUid), key, true)) 
  {
    println("Wrote new UID to card.");
  }
  
  // Halt PICC and re-select it so DumpToSerial doesn't get confused.
  RC522_PICC_HaltA();
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid()) 
  {
    return;
  }
  
  // Dump the new memory contents.
  println("New UID and contents:");
  RC522_PICC_DumpToSerial();
  
  HAL_Delay(2000);
}

/**
  * @brief Example to fix a broken UID changeable MIFARE cards.
  *
  * This sample shows how to fix a broken UID changeable MIFARE cards that have a corrupted sector 0.
  */
void Test5_FixABrokenUid(void)
{
  // Reset the loop if no new card present on the sensor/reader.
  // This saves the entire process when idle. And if present, select one.
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid()) 
  {
    return;
  }

  if (RC522_MIFARE_UnbrickUidSector()) 
  {
    println("Cleared sector 0, set UID to 1234. Card should be responsive again now.");
  }
  HAL_Delay(1000);
}

/**
* This sample shows how to setup blocks on a MIFARE Classic PICC(= card / tag)
* to be in "Value Block" mode : in this mode the operations Increment / Decrement,
* Restore and Transfer can be used.
* 
* BEWARE : Data will be written to the PICC, in sector # 1(blocks # 4 to # 7).
*/
void Test6_SetupBlocksOnMIFARE(const MIFARE_Key *key)
{
  // Reset the loop if no new card present on the sensor/reader.
  // This saves the entire process when idle. And if present, select one.
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    return;
  }

  // Show some details of the PICC (that is: the tag/card)
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));

  // Check for compatibility
  if (piccType != PICC_TYPE_MIFARE_MINI
      &&  piccType != PICC_TYPE_MIFARE_1K
      &&  piccType != PICC_TYPE_MIFARE_4K) 
  {
    println("This sample only works with MIFARE Classic cards.");
    return;
  }

  // In this sample we use the second sector,
  // that is: sector #1, covering block #4 up to and including block #7
  const uint8_t sector         = 1;
  const uint8_t valueBlockA    = 5;
  const uint8_t valueBlockB    = 6;
  const uint8_t trailerBlock   = 7;

  println("Authenticating using key A...");
  StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // Show the whole sector as it currently is
  println("Current data in sector:");
  RC522_PICC_DumpMifareClassicSectorToSerial(key, sector);
  println(NULL);
    
  // We need a sector trailer that defines blocks 5 and 6 as Value Blocks and enables key B
  // The last block in a sector (block #3 for Mifare Classic 1K) is the Sector Trailer.
  // See http://www.nxp.com/documents/data_sheet/MF1S503x.pdf sections 8.6 and 8.7:
  //      Bytes 0-5:   Key A
  //      Bytes 6-8:   Access Bits
  //      Bytes 9:     User data
  //      Bytes 10-15: Key B (or user data)
  uint8_t trailerBuffer[] = {
        255, 255, 255, 255, 255, 255,       // Keep default key A
        0, 0, 0,
        0,
        255, 255, 255, 255, 255, 255};      // Keep default key B
// The access bits are stored in a peculiar fashion.
// There are four groups:
//      g[0]    Access bits for block 0 (for sectors 0-31)
//              or blocks 0-4 (for sectors 32-39)
//      g[1]    Access bits for block 1 (for sectors 0-31)
//              or blocks 5-9 (for sectors 32-39)
//      g[2]    Access bits for block 2 (for sectors 0-31)
//              or blocks 10-14 (for sectors 32-39)
//      g[3]    Access bits for the Sector Trailer: block 3 (for sectors 0-31)
//              or block 15 (for sectors 32-39)
// Each group has access bits [C1 C2 C3], in this code C1 is MSB and C3 is LSB.
// Determine the bit pattern needed using MIFARE_SetAccessBits:
//      g0=0    access bits for block 0 (of this sector) using [0 0 0] = 000b = 0
//              which means key A|B have r/w for block 0 of this sector
//              which (in this example) translates to block #4 within sector #1;
//              this is the transport configuration (at factory delivery).
//      g1=6    access bits for block 1 (of this sector) using [1 1 0] = 110b = 6
//              which means block 1 (of this sector) is used as a value block,
//              which (in this example) translates to block #5 within sector #1;
//              where key A|B have r, key B has w, key B can increment,
//              and key A|B can decrement, transfer, and restore.
//      g2=6    same thing for block 2 (of this sector): set it to a value block;
//              which (in this example) translates to block #6 within sector #1;
//      g3=3    access bits for block 3 (of this sector): the Sector Trailer here;
//              using [0 1 1] = 011b = 3 which means only key B has r/w access
//              to the Sector Trailer (block 3 of this sector) from now on
//              which (in this example) translates to block #7 within sector #1;
  RC522_MIFARE_CalculateAccessBits(&trailerBuffer[6], 0, 6, 6, 3);

  // Read the sector trailer as it is currently stored on the PICC
  println("Reading sector trailer...");
  uint8_t buffer[18];
  uint8_t size = sizeof(buffer);
  status = RC522_MIFARE_Read(trailerBlock, buffer, &size);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Read() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  // Check if it matches the desired access pattern already;
  // because if it does, we don't need to write it again...
  if (buffer[6] != trailerBuffer[6]
      ||  buffer[7] != trailerBuffer[7]
      ||  buffer[8] != trailerBuffer[8]) 
  {
    // They don't match (yet), so write it to the PICC
    println("Writing new sector trailer...");
    status = RC522_MIFARE_Write(trailerBlock, trailerBuffer, PICC_MEMORY_BLOCK_SIZE);
    if (status != STAT_OK) 
    {
      printf("RC522_MIFARE_Write() failed: ");
      println(RC522_GetStatusCodeName(status));
      return;
    }
  }

  println("Authenticating again using key B...");
  status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // A value block has a 32 bit signed value stored three times
  // and an 8 bit address stored 4 times. Make sure that valueBlockA
  // and valueBlockB have that format (note that it will only format
  // the block when it doesn't comply to the expected format already).
  FormatValueBlock(valueBlockA);
  FormatValueBlock(valueBlockB);

  // Add 1 to the value of valueBlockA and store the result in valueBlockA.
  println("Adding 1 to value of block %u", valueBlockA);
  status = RC522_MIFARE_Increment(valueBlockA, 1);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Increment() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  status = RC522_MIFARE_Transfer(valueBlockA);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Transfer() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  // Show the new value of valueBlockA
  int32_t value;
  status = RC522_MIFARE_GetValue(valueBlockA, &value);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_GetValue() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("New value of value block %u = %li", valueBlockA, value);

  // Decrement 10 from the value of valueBlockB and store the result in valueBlockB.
  println("Subtracting 10 from value of block %u", valueBlockB);
  status = RC522_MIFARE_Decrement(valueBlockB, 10);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Decrement() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  status = RC522_MIFARE_Transfer(valueBlockB);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Transfer() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  // Show the new value of valueBlockB
  status = RC522_MIFARE_GetValue(valueBlockB, &value);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_GetValue() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("New value of value block %u = %li", valueBlockB, value);
  // Check some boundary...
  if (value <= -100) 
  {
    println("Below -100, so resetting it to 255 = 0xFF just for fun...");
    status = RC522_MIFARE_SetValue(valueBlockB, 255);
    if (status != STAT_OK) 
    {
      printf("RC522_MIFARE_SetValue() failed: ");
      println(RC522_GetStatusCodeName(status));
      return;
    }
  }

  // Dump the sector data
  RC522_PICC_DumpMifareClassicSectorToSerial(key, sector);
  println(NULL);

  // Halt PICC
  RC522_PICC_HaltA();
  // Stop encryption on PCD
  RC522_StopCrypto1();
}

/**
 * Helper routine to dump a byte array as HEX values to Serial.
 */
void DumpByteArrayHEX(uint8_t *buffer, size_t bufferSize)
{
  for (size_t i = 0; i < bufferSize; i++) 
  {
    printf("%02X ", buffer[i]);
  }
}

/**
 * Helper routine to dump a byte array as ASCI values to Serial.
 */
void DumpByteArrayASCI(uint8_t *buffer, size_t bufferSize)
{
  for (size_t i = 0; i < bufferSize; i++) 
  {
    printf("%c", ((buffer[i] < ' ') || (buffer[i] > '~')) ? '.' : buffer[i]);
  }
}

/**
 * Ensure that a given block is formatted as a Value Block.
 */
void FormatValueBlock(uint8_t blockAddr)
{
  uint8_t buffer[18];
  uint8_t size = sizeof(buffer);

  println("Reading block %u", blockAddr);
  StatusCode status = RC522_MIFARE_Read(blockAddr, buffer, &size);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Read() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  if (  (buffer[0] == (uint8_t)~buffer[4])
    &&  (buffer[1] == (uint8_t)~buffer[5])
    &&  (buffer[2] == (uint8_t)~buffer[6])
    &&  (buffer[3] == (uint8_t)~buffer[7])

    &&  (buffer[0] == buffer[8])
    &&  (buffer[1] == buffer[9])
    &&  (buffer[2] == buffer[10])
    &&  (buffer[3] == buffer[11])

    &&  (buffer[12] == (uint8_t)~buffer[13])
    &&  (buffer[12] ==           buffer[14])
    &&  (buffer[12] == (uint8_t)~buffer[15])) 
  {
    println("Block has correct Value Block format.");
  }
  else 
  {
    println("Formatting as Value Block...");
    const uint8_t valueBlock[] = {
      0, 0, 0, 0,
      255, 255, 255, 255,
      0, 0, 0, 0,
      blockAddr, ~blockAddr, blockAddr, ~blockAddr };
    status = RC522_MIFARE_Write(blockAddr, valueBlock, PICC_MEMORY_BLOCK_SIZE);
    if (status != STAT_OK) 
    {
      printf("MIFARE_Write() failed: ");
      println(RC522_GetStatusCodeName(status));
    }
  }
}

/**
  * @brief Copy the RFID card data into variables and then
  *        scan the second empty card to copy all the data.
  *
  * Example sketch/program which will try the most used default keys listed in
  * https://code.google.com/p/mfcuk/wiki/MifareClassicDefaultKeys to dump the
  * block 0 of a MIFARE RFID card using a RFID-RC522 reader.
  */
void Test7_RFID_Cloner(void)
{
  static uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE];
  static char choice;
  scanf("%c", &choice);
  
  if (choice == '1')
  {
    println("Read the card");
    ReadCard(dataValue);
  }
  else if (choice == '2')
  {
    println("See what is in the variables");
    TestValuesInBlocks(dataValue);
  }
  else if (choice == '3')
  {
    println("Copying the data on to the new card");
    CopyDataInTheNewCard(dataValue);
  }
}

static void ResetAfterWrongAuthenticate(void)
{
  // http://arduino.stackexchange.com/a/14316
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_SelectCardWithLastUid())
  {
    println("Insert card and try again!");
  }
}

static StatusAuthenticate TryKey(const uint8_t blockAddr, MIFARE_Key *key, const StatusAuthenticate previousStatus)
{
  StatusAuthenticate result = STAT_KEY_NOT_TRUE;
  if ((previousStatus & STAT_KEY_A_OK) == 0)
  {
    const StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, blockAddr, key);
    if (status != STAT_OK)
    {
      ResetAfterWrongAuthenticate();
    }
    else
    {
      result |= STAT_KEY_A_OK;
    }
  }
  if ((previousStatus & STAT_KEY_B_OK) == 0)
  {
    const StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_B, blockAddr, key);
    if (status != STAT_OK)
    {
      ResetAfterWrongAuthenticate();
    }
    else
    {
      result |= STAT_KEY_B_OK;
    }
  }
  return result;
}

static StatusAuthenticate FindTheRightKeys(const uint8_t blockAddr, MIFARE_Key *keyA, MIFARE_Key *keyB)
{
  StatusAuthenticate resultOfAuthenticate = STAT_KEY_NOT_TRUE;
  const size_t numberOfKnownKeys = sizeof(knownKeys) / MF_KEY_SIZE;
  for (size_t k = 0; k < numberOfKnownKeys; k++) 
  {
    MIFARE_Key key;
    for (size_t i = 0; i < MF_KEY_SIZE; i++) 
    {
      key.keyByte[i] = knownKeys[k][i];
    }
    
    const StatusAuthenticate statusAuthenticate = TryKey(blockAddr, &key, resultOfAuthenticate);
    if (statusAuthenticate & STAT_KEY_A_OK)
    {
      resultOfAuthenticate |= STAT_KEY_A_OK;
      for (size_t i = 0; i < MF_KEY_SIZE; i++) 
      {
        (*keyA).keyByte[i] = key.keyByte[i];
      }
    }
    if (statusAuthenticate & STAT_KEY_B_OK)
    {
      resultOfAuthenticate |= STAT_KEY_B_OK;
      for (size_t i = 0; i < MF_KEY_SIZE; i++) 
      {
        (*keyB).keyByte[i] = key.keyByte[i];
      }
    }
    if ((resultOfAuthenticate & STAT_KEY_A_OK) && (resultOfAuthenticate & STAT_KEY_B_OK)) 
    {
      // Found and reported on the key and block,
      // no need to try other keys for this PICC
      break;
    }
  }
  return resultOfAuthenticate;
}

static StatusCode AuthenticateWithRightKeys(StatusAuthenticate resultOfAuthenticate, 
  const uint8_t blockAddr, MIFARE_Key *keyA, MIFARE_Key *keyB)
{
  StatusCode status = STAT_ERROR;
  if (resultOfAuthenticate & STAT_KEY_A_OK)
  {
    status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, blockAddr, keyA);
    if (status != STAT_OK)
    {
      return status;
    }
  }
  if (resultOfAuthenticate & STAT_KEY_B_OK)
  {
    status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_B, blockAddr, keyB);
    if (status != STAT_OK)
    {
      return status;
    }
  }
  return status;
}

static void CopyDataFromBufferToDataValue(const uint8_t blockAddr, MIFARE_Key *keyA, MIFARE_Key *keyB, 
  const StatusAuthenticate resultOfAuthenticate, uint8_t *buffer, uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE])
{
  if ((blockAddr != 0) && (((blockAddr + 1) % 4) == 0))
  {
    for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++)
    {
      if (i < 6)
      {
        dataValue[blockAddr][i] = (resultOfAuthenticate & STAT_KEY_A_OK) ? (*keyA).keyByte[i] : 0;
      }
      else if (i >= 10)
      {
        dataValue[blockAddr][i] = (resultOfAuthenticate & STAT_KEY_B_OK) ? (*keyB).keyByte[i - 10] : 0;
      }
      else
      {
        dataValue[blockAddr][i] = buffer[i];
      }
    }
  }
  else
  {
    for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++)
    {
      dataValue[blockAddr][i] = buffer[i];
    }
  }
}

static void PrintDataBlock(const uint8_t block, 
  uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE], 
  const StatusAuthenticate resultOfAuthenticate)
{
  if ((block % 4) == 0)
  {
    println("Sector: %u", block / 4);
  }
  printf("Block %2u: ", block);
  DumpByteArrayASCI(dataValue[block], PICC_MEMORY_BLOCK_SIZE);
  println(NULL);
  HAL_Delay(100);
  if (resultOfAuthenticate != STAT_KEY_NOT_TRUE)
  {
    printf("          ");
    if ((block != 0) && (((block + 1) % 4) == 0))
    {
      for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++)
      {
        if (i < 6)
        {
          if (resultOfAuthenticate & STAT_KEY_A_OK)
          {
            printf("%02X ", dataValue[block][i]);
          }
          else
          {
            printf("-- ");
          }
        }
        else if (i >= 10)
        {
          if (resultOfAuthenticate & STAT_KEY_B_OK)
          {
            printf("%02X ", dataValue[block][i]);
          }
          else
          {
            printf("-- ");
          }
        }
        else
        {
          printf("%02X ", dataValue[block][i]);
        }
      }
      println(NULL);
      println("          ===== Key A =====|= ACs ==|==|===== Key B =====");
    }
    else 
    {
      DumpByteArrayHEX(dataValue[block], PICC_MEMORY_BLOCK_SIZE);
      println(NULL);
    }
  }
  HAL_Delay(100);
}

/*
 * Try using the PICC (the tag/card) with the given key to access block 0 to 63.
 * On success, it will show the key details, and dump the block data on Serial.
 *
 * @return true when the given key worked, false otherwise.
 */
static StatusCode getDataFromCard(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE])
{
  MIFARE_Key keyA;
  MIFARE_Key keyB;
  StatusAuthenticate resultOfAuthenticate = STAT_KEY_NOT_TRUE;
  for (uint8_t block = 0; block < PICC_MEMORY_BLOCKS; block++) 
  {
    if ((block % 4) == 0)
    {
      resultOfAuthenticate = FindTheRightKeys(block, &keyA, &keyB);
    }
    uint8_t buffer[18] = { 0 };
    if (resultOfAuthenticate != STAT_KEY_NOT_TRUE)
    {
      if ((block % 4) == 0)
      {
        const StatusCode status = AuthenticateWithRightKeys(resultOfAuthenticate, block, &keyA, &keyB);
        if (status != STAT_OK) 
        {
          printf("AuthenticateWithRightKeys() failed: ");
          println(RC522_GetStatusCodeName(status));
        }
      }

      uint8_t byteCount = sizeof(buffer);
      const StatusCode status = RC522_MIFARE_Read(block, buffer, &byteCount);
      if (status != STAT_OK) 
      {
        printf("RC522_MIFARE_Read() failed: ");
        println(RC522_GetStatusCodeName(status));
      }
    }
    CopyDataFromBufferToDataValue(block, &keyA, &keyB, resultOfAuthenticate, buffer, dataValue);
    PrintDataBlock(block, dataValue, resultOfAuthenticate);
  }

  return STAT_OK;
}

void ReadCard(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE])
{
  // Look for new cards and select one of the cards
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    println("Insert card and try again!");
    return;
  }

  // Show some details of the PICC (that is: the tag/card)
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));
    
  getDataFromCard(dataValue);

  println(NULL);
  println("1.Read card \r\n2.View buffer \r\n3.Copy the data.");

  RC522_PICC_HaltA(); // Halt PICC
  RC522_StopCrypto1(); // Stop encryption on RC522
}

void TestValuesInBlocks(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE])
{
  for (uint8_t block = 4; block <= 62; block++) 
  {
    if (((block + 1) % 4) == 0) 
    {
      block++;
    }
  
    println("Writing data into block %u", block);
  
    DumpByteArrayHEX(dataValue[block], PICC_MEMORY_BLOCK_SIZE); 
    println(NULL);
    println(NULL);
  }
  
  println("1.Read card \r\n2.View buffer \r\n3.Copy the data.");
}

void CopyDataInTheNewCard(uint8_t dataValue[PICC_MEMORY_BLOCKS][PICC_MEMORY_BLOCK_SIZE])
{
  println("Insert new card...");
  // Look for new cards and select one of the cards
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    println("Insert card and try again!");
    return;
  }

  // Show some details of the PICC (that is: the tag/card)
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));
    
  MIFARE_Key key;
  for (size_t i = 0; i < 6; i++) 
  {
    key.keyByte[i] = 0xFF;
  }

  for (uint8_t block = 4; block <= 62; block++) 
  {
    // Copy blocks 4 to 62, except all of these below (because they are the authentication blocks)
    if (((block + 1) % 4) == 0) 
    {
      block++;
    }
    
    println("Authenticating using key A...");
    StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, block, &key);
    if (status != STAT_OK) 
    {
      printf("RC522_Authenticate() failed: ");
      println(RC522_GetStatusCodeName(status));
      return;
    }
    
    println("Authenticating again using key B...");
    status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_B, block, &key);
    if (status != STAT_OK) 
    {
      printf("RC522_Authenticate() failed: ");
      println(RC522_GetStatusCodeName(status));
      return;
    }
    
    println("Writing data into block %u", block);
    println(NULL);
          
    DumpByteArrayHEX(dataValue[block], PICC_MEMORY_BLOCK_SIZE); 
          
    status = RC522_MIFARE_Write(block, dataValue[block], PICC_MEMORY_BLOCK_SIZE);
    if (status != STAT_OK) 
    {
      printf("RC522_MIFARE_Write() failed: ");
      println(RC522_GetStatusCodeName(status));
    }
        
    println(NULL);
    println(NULL);
  }
  RC522_PICC_HaltA(); // Halt PICC
  RC522_StopCrypto1(); // Stop encryption on PCD
  
  println("1.Read card \r\n2.View buffer \r\n3.Copy the data.");
}

/*
 * This sample shows how to read and write data blocks on a MIFARE Classic PICC
 * (= card/tag).
 *
 * BEWARE: Data will be written to the PICC, in sector #1 (blocks #4 to #7).
 */
void Test8_ReadAndWrite(MIFARE_Key *key)
{
  // Look for new cards and select one of the cards
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    return;
  }
  // Show some details of the PICC (that is: the tag/card)
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));

  // Check for compatibility
  if (piccType != PICC_TYPE_MIFARE_MINI
      &&  piccType != PICC_TYPE_MIFARE_1K
      &&  piccType != PICC_TYPE_MIFARE_4K) 
  {
    println("This sample only works with MIFARE Classic cards.");
    return;
  }

  // In this sample we use the second sector,
  // that is: sector #1, covering block #4 up to and including block #7
  const uint8_t sector = 1;
  const uint8_t blockAddr = 4;
  uint8_t dataBlock[] = {
    0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
    0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
    0x09, 0x0A, 0xFF, 0x0B, //  9, 10, 255, 11,
    0x0C, 0x0D, 0x0E, 0x0F  // 12, 13,  14, 15
  };

  const uint8_t trailerBlock = 7;
  uint8_t buffer[18];
  uint8_t size = sizeof(buffer);

  // Authenticate using key A
  println("Authenticating using key A...");
  StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // Show the whole sector as it currently is
  println("Current data in sector:");
  RC522_PICC_DumpMifareClassicSectorToSerial(key, sector);
  println(NULL);

  // Read data from the block
  println("Reading data from block %u ...", blockAddr);
  status = RC522_MIFARE_Read(blockAddr, buffer, &size);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Read() failed: ");
    println(RC522_GetStatusCodeName(status));
  }
  println("Data in block %u:", blockAddr);
  DumpByteArrayHEX(buffer, PICC_MEMORY_BLOCK_SIZE);
  println(NULL);
  println(NULL);

  // Authenticate using key B
  println("Authenticating again using key B...");
  status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // Write data to the block
  println("Writing data into block %u ...", blockAddr);
  DumpByteArrayHEX(dataBlock, PICC_MEMORY_BLOCK_SIZE);
  println(NULL);
  status = RC522_MIFARE_Write(blockAddr, dataBlock, PICC_MEMORY_BLOCK_SIZE);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Write() failed: ");
    println(RC522_GetStatusCodeName(status));
  }
  println(NULL);

  // Read data from the block (again, should now be what we have written)
  println("Reading data from block %u ...", blockAddr);
  status = RC522_MIFARE_Read(blockAddr, buffer, &size);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Read() failed: ");
    println(RC522_GetStatusCodeName(status));
  }
  println("Data in block %u:", blockAddr);
  DumpByteArrayHEX(buffer, PICC_MEMORY_BLOCK_SIZE);
  println(NULL);

  // Check that data in block is what we have written
  // by counting the number of bytes that are equal
  println("Checking result...");
  size_t count = 0;
  for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++) 
  {
    // Compare buffer (= what we've read) with dataBlock (= what we've written)
    if (buffer[i] == dataBlock[i])
    {
      count++;
    }
  }
  println("Number of bytes that match = %u", count);
  if (count == PICC_MEMORY_BLOCK_SIZE) 
  {
    println("Success :-)");
  }
  else 
  {
    println("Failure, no match :-(");
    println("  perhaps the write didn't work properly...");
  }
  println(NULL);

  // Dump the sector data
  println("Current data in sector:");
  RC522_PICC_DumpMifareClassicSectorToSerial(key, sector);
  println(NULL);

  RC522_PICC_HaltA(); // Halt PICC
  RC522_StopCrypto1(); // Stop encryption on PCD
}

void Test9_WritePersonalData(MIFARE_Key *key)
{
  // Look for new cards and select one of the cards
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    return;
  }
  // Show some details of the PICC (that is: the tag/card)
  printf("Card UID:");
  RC522_PrintUID();
  println(NULL);
  const PICC_Type piccType = RC522_PICC_GetType();
  printf("PICC type: ");
  println(RC522_PICC_GetTypeName(piccType));

  uint8_t buffer[PICC_MEMORY_BLOCK_SIZE] = {0};

  println("Type Family name");
  scanf("%s", buffer);

  uint8_t block = 1;
  StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, block, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("Authentication success");

  // Write block
  status = RC522_MIFARE_Write(block, buffer, 16);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Write() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("Family name successfully wrote");

  for (size_t i = 0; i < sizeof(buffer); i++)
  {
    buffer[i] = 0;
  }
  println("Type First name");
  scanf("%s", buffer);

  block = 2;
  status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, block, key);
  if (status != STAT_OK) 
  {
    printf("RC522_Authenticate() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("Authentication success");

  // Write block
  status = RC522_MIFARE_Write(block, buffer, 16);
  if (status != STAT_OK) 
  {
    printf("RC522_MIFARE_Write() failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }
  println("First name successfully wrote");
  println(NULL);

  RC522_PICC_HaltA(); // Halt PICC
  RC522_StopCrypto1(); // Stop encryption on PCD
}

void Test10_ReadPersonalData(MIFARE_Key *key)
{
  // Look for new cards and select one of the cards
  if (!RC522_PICC_IsNewCardPresent() || !RC522_PICC_ReadCardUid())
  {
    return;
  }
  println("**Card Detected:**");
  RC522_PICC_DumpDetailsToSerial();

  printf("Name: ");

  // GET FIRST NAME
  uint8_t block = 2;
  StatusCode status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, block, key);
  if (status != STAT_OK) 
  {
    printf("Authentication failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  uint8_t buffer[18];
  uint8_t len = sizeof(buffer);
  status = RC522_MIFARE_Read(block, buffer, &len);
  if (status != STAT_OK) 
  {
    printf("Reading failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // PRINT FIRST NAME
  for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++)
  {
    if (buffer[i] == '\0')
    {
      break;
    }
    printf("%c", buffer[i]);
  }
  printf(" ");


  // GET LAST NAME
  block = 1;
  status = RC522_Authenticate(PICC_CMD_MF_AUTH_KEY_A, block, key);
  if (status != STAT_OK) 
  {
    printf("Authentication failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  status = RC522_MIFARE_Read(block, buffer, &len);
  if (status != STAT_OK) 
  {
    printf("Reading failed: ");
    println(RC522_GetStatusCodeName(status));
    return;
  }

  // PRINT LAST NAME
  for (size_t i = 0; i < PICC_MEMORY_BLOCK_SIZE; i++)
  {
    if (buffer[i] == '\0')
    {
      break;
    }
    printf("%c", buffer[i]);
  }
  println(NULL);
  println("**End Reading**");

  HAL_Delay(1000);

  RC522_PICC_HaltA();
  RC522_StopCrypto1();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: println("Wrong parameters value: file %s on line %d", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
