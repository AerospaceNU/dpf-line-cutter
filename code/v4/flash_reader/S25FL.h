/*
 * S25FLx2.c
 */

#include <string.h>
//#include <S25FLx.h>
SPIClass SPI2(NRF_SPIM2,  PIN_SPI_MISO,  PIN_QSPI_SCK,  PIN_SPI_MOSI);
// Commands
#define WRITE_ENABLE_CMD 0x06
#define WRITE_DISABLE_CMD 0x04
#define READ_STAT_REG_CMD 0x05
#define READ_DATA_CMD 0x13
#define PAGE_PROGRAM_CMD 0x12
#define SECTOR_ERASE_CMD 0xDC
#define CHIP_ERASE_CMD 0xC7

// Timeouts
#define SPI_TX_RX_TIMEOUT_MS 100 // For short transmit-receives that don't require DMA

// Flash properties
#define PAGE_SIZE_BYTES 256
#define SECTOR_SIZE_BYTES 262144
#define FLASH_SIZE_BYTES 0x8000000

int CS0 = 8;

#define CHIP_SELECT digitalWrite(CS0, LOW);
#define CHIP_DESELECT digitalWrite(CS0, HIGH);

class S25FL
{
 public:

  // Disable writing (enable write protect)
  bool write_disable()
  {
    CHIP_SELECT

    uint8_t tx = WRITE_DISABLE_CMD;
    SPI2.transfer(tx);

    CHIP_DESELECT
    return true; // TODO
  }

  // Enable writing (disable write protect)
  bool write_enable()
  {
    CHIP_SELECT
    uint8_t tx = WRITE_ENABLE_CMD;
    SPI2.transfer(tx);
    CHIP_DESELECT
    return true; // TODO
  }

  // Returns true if a write is in progress
  bool is_write_in_progress()
  {
    CHIP_SELECT
    SPI2.transfer(READ_STAT_REG_CMD);
    byte reg = SPI2.transfer(0);
    bool ret = (reg & 0x01) == 0x01; // Check if write-in-progress (the lowest bit) is set
    CHIP_DESELECT
    return ret;
  }

  // Sets stuff up
  void init()
  {
    pinMode(CS0, OUTPUT);
    CHIP_DESELECT
  }

  // Read a given number of bytes from the start, and fill a given array
  // Untested if the data spans multiple pages (blocks of 256? I think?)
  bool read_start(uint32_t startLoc, uint8_t *pData, uint32_t numBytes)
  {
    while (is_write_in_progress()) { delay(1); }
    
    // Check for valid parameters
    // Write cannot be in progress
    if (startLoc + numBytes > FLASH_SIZE_BYTES ||
        pData == NULL ||
        is_write_in_progress())
      return false;

    // Transmit read command and location
    CHIP_SELECT
    SPI2.transfer(READ_DATA_CMD);
    SPI2.transfer(startLoc >> 24);
    SPI2.transfer(startLoc >> 16);
    SPI2.transfer(startLoc >> 8);
    SPI2.transfer(startLoc & 0xFF);

    // Read into given buffer pData
    for (int i = 0; i < numBytes; i++)
    {
      pData[i] = SPI2.transfer(0);
    }

    // Pull CS high if not using DMA because read is complete
    CHIP_DESELECT

    return true;
  }

  // Write a given array of bytes to a start location
  // Users should manage writing to one page at a time because I'm  l a z y
  // And should wait for is_write_in_progress to be false.
  bool write(uint32_t startLoc, uint8_t *data, uint32_t numBytes)
  {
    while(is_write_in_progress()) { delay(1); }

    // Only allows writing to 1 page at a time

    // Check for valid parameters
    if (startLoc + numBytes > FLASH_SIZE_BYTES ||
        startLoc / PAGE_SIZE_BYTES != (startLoc + numBytes) / PAGE_SIZE_BYTES ||
        data == NULL ||
        is_write_in_progress())
      return false;

    // Assume data is erased beforehand.
    // This is up to higher-level code to manage since writing over existing data won't cause an error.

    // Enable writing to flash
    if (!write_enable())
      return false;

    while (is_write_in_progress()) {}

    CHIP_SELECT

    // Fill in TX buffer
    uint8_t txBuf[5];
    txBuf[0] = PAGE_PROGRAM_CMD;
    for (int i = 1; i < 5; i++)
    {
      txBuf[i] = (startLoc >> (8 * (4 - i))) & 0xFF;
    }
    //  memcpy(txBuf + 5, data, numBytes);
    SPI2.transfer(txBuf, 5); // Send the location and page prgm cmd

    for (int i = 0; i < numBytes; i++)
    {
      SPI2.transfer(data[i]);
    }

    CHIP_DESELECT

    return true;
  }

  bool erase_sector_start(uint32_t sectorNum)
  {
    // Check for valid parameters
    if (sectorNum * SECTOR_SIZE_BYTES >= FLASH_SIZE_BYTES)
      return false;

    // Enable writing to flash (also necessary for erasing)
    if (!write_enable())
      return false;

    // Prep erase command
    CHIP_SELECT

    // Fill in TX buffer with the erase command and teh address to erase
    uint8_t txBuf[5];
    txBuf[0] = SECTOR_ERASE_CMD;
    for (int i = 1; i < 5; i++)
    {
      txBuf[i] = ((sectorNum * SECTOR_SIZE_BYTES) >> (8 * (4 - i))) & 0xFF;
    }

    SPI2.transfer(txBuf, 5);

    CHIP_DESELECT

    return true; // TODO
  }

  // Erase the entire chip
  bool erase_chip_start()
  {
    // Enable writing to flash (also necessary for erasing)
    if (!write_enable())
      return false;

    // Perform full erase command. Short enough to avoid DMA
    CHIP_SELECT
    uint8_t txByte = CHIP_ERASE_CMD;
    SPI2.transfer(txByte);
    CHIP_DESELECT

    return true;
  }

  // Check if no writes are in progress. returns !is_write_in_progress
  bool is_write_completed()
  {
    bool ret = is_write_in_progress();
    if (ret)
    {
      delay(1);
    }
    // This delay is here because checking write in progress too quickly causes issues
    // There should be a way to check if ready instead of delaying, but no solution has been found yet
    return !ret;
  }

  // Same functionality as write, but different public function name is clearer for programmers at higher levels
  bool is_erase_complete()
  {
    return is_write_completed();
  }

  // Print manufacturer info
  bool check_connected()
  {
    CHIP_SELECT

    SPI2.transfer(0x9F);
    byte manufacturer_id = SPI2.transfer(0);
    byte mem_type = SPI2.transfer(0);
    byte density = SPI2.transfer(0);
    byte reg4Dh = SPI2.transfer(0);
    byte arch = SPI2.transfer(0);
    byte family = SPI2.transfer(0);

    Serial.print("manufacturer_id ");
    Serial.print(manufacturer_id, HEX);
    Serial.print(" mem_type ");
    Serial.print(mem_type, HEX);
    Serial.print(" density = ");
    Serial.print(density, HEX);
    Serial.print(" reg4Dh = ");
    Serial.print(reg4Dh, HEX);
    Serial.print(" arch = ");
    Serial.print(arch, HEX);
    Serial.print(" family = ");
    Serial.println(family, HEX);

    CHIP_DESELECT
    return true;
  }
};
