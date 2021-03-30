/*
 * S25FL
 */

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
//#include <S25FLx.h>

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
  bool write_disable();
  bool write_enable();
  bool is_write_in_progress();
  void init();
  bool read_start(uint32_t startLoc, uint8_t *pData, uint32_t numBytes);
  bool write(uint32_t startLoc, uint8_t *data, uint32_t numBytes);
  bool erase_sector_start(uint32_t sectorNum);
  bool erase_chip_start();
  bool is_write_completed();
  bool is_erase_complete();
  bool check_connected();
};
