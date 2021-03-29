#include "board_driver_spi.h"
#include "board_driver_spiflash.h"

#define SPIFLASH_WRITEENABLE      0x06        // write enable
#define SPIFLASH_WRITEDISABLE     0x04        // write disable

#define SPIFLASH_BLOCKERASE_4K    0x20        // erase one 4K block of flash memory
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 64K block of flash memory
#define SPIFLASH_CHIPERASE        0x60        // chip erase (may take several seconds depending on size)
                                              // but no actual need to wait for completion (instead need to check the status register BUSY bit)
#define SPIFLASH_STATUSREAD       0x05        // read status register
#define SPIFLASH_STATUSWRITE      0x01        // write status register
#define SPIFLASH_ARRAYREAD        0x0B        // read array (fast, need to add 1 dummy byte after 3 address bytes)
#define SPIFLASH_ARRAYREADLOWFREQ 0x03        // read array (low frequency)

#define SPIFLASH_SLEEP            0xB9        // deep power down
#define SPIFLASH_WAKE             0xAB        // deep power wake up
#define SPIFLASH_BYTEPAGEPROGRAM  0x02        // write (1 to 256bytes)
#define SPIFLASH_IDREAD           0x9F        // read JEDEC manufacturer and device ID (2 bytes, specific bytes for each manufacturer and device)
                                              // Example for Atmel-Adesto 4Mbit AT25DF041A: 0x1F44 (page 27: http://www.adestotech.com/sites/default/files/datasheets/doc3668.pdf)
                                              // Example for Winbond 4Mbit W25X40CL: 0xEF30 (page 14: http://www.winbond.com/NR/rdonlyres/6E25084C-0BFE-4B25-903D-AE10221A0929/0/W25X40CL.pdf)
#define SPIFLASH_MACREAD          0x4B        // read unique ID number (MAC)

// External flash SS pin is PORT B, PIN 2
// From variant.cpp
// 19         | A5               |  PB02  | A5              | EIC/EXTINT[2] *ADC/AIN[10]           PTC/Y[8]  SERCOM5/PAD[0] 

static inline void spiflash_select()
{
  
  PORT->Group[BOARD_SPIFLASH_PORT].OUTCLR.reg |= (1 << BOARD_SPIFLASH_PIN);
}

static inline void spiflash_unselect()
{
  PORT->Group[BOARD_SPIFLASH_PORT].OUTSET.reg |= (1 << BOARD_SPIFLASH_PIN);
  
}

static inline uint8_t spiflash_readStatus()
{
  spiflash_select();
  spi_transfer(SPIFLASH_STATUSREAD);
  uint8_t status = spi_transfer(0);
  spiflash_unselect();
  return status;
}

/// check if the chip is busy erasing/writing
static inline bool spiflash_busy()
{
  return spiflash_readStatus() & 1;
}

static inline void spiflash_command(uint8_t cmd){
  //wait for any write/erase to complete
  //  a time limit cannot really be added here without it being a very large safe limit
  //  that is because some chips can take several seconds to carry out a chip erase or other similar multi block or entire-chip operations
  //  a recommended alternative to such situations where chip can be or not be present is to add a 10k or similar weak pulldown on the
  //  open drain MISO input which can read noise/static and hence return a non 0 status byte, causing the while() to hang when a flash chip is not present
  if (cmd != SPIFLASH_WAKE) while(spiflash_busy());
  spiflash_select();
  spi_transfer(cmd);
}

static inline void spiflash_commandWrite(uint8_t cmd){
  spiflash_command(SPIFLASH_WRITEENABLE); // Write Enable
  spiflash_unselect();
  spiflash_command(cmd);
}

static inline void spiflash_sleep()
{
  spiflash_command(SPIFLASH_SLEEP);
  spiflash_unselect();
}

static inline void spiflash_wakeup()
{
  spiflash_command(SPIFLASH_WAKE);
  spiflash_unselect();
}

static inline uint16_t spiflash_readDeviceId()
{
  spiflash_select();
  spi_transfer(SPIFLASH_IDREAD);
  uint16_t jedecid = spi_transfer(0) << 8;
  jedecid |= spi_transfer(0);
  spiflash_unselect();
  return jedecid;
}

/*  =========================
 *  ===== SPIFLASH API
 *  =========================
 */

uint8_t spiflash_readByte(uint32_t addr)
{
  spiflash_command(SPIFLASH_ARRAYREADLOWFREQ);
  spi_transfer(addr >> 16);
  spi_transfer(addr >> 8);
  spi_transfer(addr);
  uint8_t result = spi_transfer(0);
  spiflash_unselect();
  return result;

}
void spiflash_readBytes(uint32_t addr, void* buf, uint16_t len) 
{
  spiflash_command(SPIFLASH_ARRAYREAD);
  spi_transfer(addr >> 16);
  spi_transfer(addr >> 8);
  spi_transfer(addr);
  spi_transfer(0); //"dont care"
  for (uint16_t i = 0; i < len; ++i)
    ((uint8_t*) buf)[i] = spi_transfer(0);
  spiflash_unselect();
}

void spiflash_init()
{
  PORT->Group[BOARD_SPIFLASH_PORT].DIRSET.reg = (1 << BOARD_SPIFLASH_PIN);
  spi_init(4000000);
  spiflash_unselect();
  spiflash_wakeup();
}
