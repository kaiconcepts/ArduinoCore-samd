/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "board_driver_spi.h"

/*- Definitions -------------------------------------------------------------*/
#define SPI_SERCOM           SERCOM4
#define SPI_SERCOM_GCLK_ID   GCLK_CLKCTRL_ID_SERCOM4_CORE_Val
#define SERCOM_SPI_FREQ_REF 48000000ul
#define PIN_SPI_MISO         PINMUX_PA12D_SERCOM4_PAD0
#define PIN_SPI_MOSI         PINMUX_PB10D_SERCOM4_PAD2
#define PIN_SPI_SCK          PINMUX_PB11D_SERCOM4_PAD3
 
typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;


typedef enum
{
	SPI_SLAVE_OPERATION = 0x2u,
	SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0, // CPOL : 0 | CPHA : 0
	SERCOM_SPI_MODE_1,     // CPOL : 0 | CPHA : 1
	SERCOM_SPI_MODE_2,     // CPOL : 1 | CPHA : 0
	SERCOM_SPI_MODE_3      // CPOL : 1 | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;


typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;


#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static inline void pin_set_peripheral_function(uint32_t pinmux)
{
    /* the variable pinmux consist of two components:
        31:16 is a pad, wich includes:
            31:21 : port information 0->PORTA, 1->PORTB
            20:16 : pin 0-31
        15:00 pin multiplex information
        there are defines for pinmux like: PINMUX_PA09D_SERCOM2_PAD1 
    */
    uint16_t pad = pinmux >> 16;    // get pad (port+pin)
    uint8_t port = pad >> 5;        // get port
    uint8_t pin  = pad & 0x1F;      // get number of pin - no port information anymore
    
    PORT->Group[port].PINCFG[pin].bit.PMUXEN =1;
    
    /* each pinmux register is for two pins! with pin/2 you can get the index of the needed pinmux register
       the p mux resiter is 8Bit   (7:4 odd pin; 3:0 evan bit)  */
    // reset pinmux values.                             VV shift if pin is odd (if evan:  (4*(pin & 1))==0  )
    PORT->Group[port].PMUX[pin/2].reg &= ~( 0xF << (4*(pin & 1)) );
                    //          
    // set new values
    PORT->Group[port].PMUX[pin/2].reg |=  ( (uint8_t)( (pinmux&0xFFFF) <<(4*(pin&1)) ) ); 
}

static inline void initClockNVIC( void )
{
  //Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( SPI_SERCOM_GCLK_ID ) | // Generic Clock 0 (SERCOMx)
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;
}

static inline uint8_t calculateBaudrateSynchronous(uint32_t baudrate) {
  uint16_t b = SERCOM_SPI_FREQ_REF / (2 * baudrate);
  if(b > 0) b--; // Don't -1 on baud calc if already at 0
  return b;
}

/*  =========================
 *  ===== Sercom SPI
 *  =========================
 */
static inline void resetSPI()
{
  //Setting the Software Reset bit to 1
  SPI_SERCOM->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(SPI_SERCOM->SPI.CTRLA.bit.SWRST || SPI_SERCOM->SPI.SYNCBUSY.bit.SWRST);
}

static inline void enableSPI()
{
  //Setting the enable bit to 1
  SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 1;

  while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
}

static inline void disableSPI()
{
  while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 0;
}

static inline void initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
  resetSPI();
  initClockNVIC();

  //Setting the CTRLA register
  SPI_SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  SPI_SERCOM->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

  while( SPI_SERCOM->SPI.SYNCBUSY.bit.CTRLB == 1 );
}


static inline void initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate)
{
  //Extract data from clockMode
  int cpha, cpol;

  if((clockMode & (0x1ul)) == 0 )
    cpha = 0;
  else
    cpha = 1;

  if((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  //Setting the CTRLA register
  SPI_SERCOM->SPI.CTRLA.reg |= ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                           ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  SPI_SERCOM->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
}

static inline bool isBufferOverflowErrorSPI()
{
  return SPI_SERCOM->SPI.STATUS.bit.BUFOVF;
}

static inline bool isDataRegisterEmptySPI()
{
  //DRE : Data Register Empty
  return SPI_SERCOM->SPI.INTFLAG.bit.DRE;
}

/*  =========================
 *  ===== SPI API
 *  =========================
 */

void spi_init(uint32_t baud) {
  pin_set_peripheral_function(PIN_SPI_MISO);
  pin_set_peripheral_function(PIN_SPI_MOSI);
  pin_set_peripheral_function(PIN_SPI_SCK);

/*
#define PIN_SPI_MISO         PINMUX_PA12D_SERCOM4_PAD0
#define PIN_SPI_MOSI         PINMUX_PB10D_SERCOM4_PAD2
#define PIN_SPI_SCK          PINMUX_PB11D_SERCOM4_PAD3
 */

	//Using the WRCONFIG register to bulk configure PB16 for being configured the SERCOM5 SPI MASTER MISO
	PORT->Group[0].WRCONFIG.reg = 
		PORT_WRCONFIG_WRPINCFG |											//Enables the configuration of PINCFG
		PORT_WRCONFIG_WRPMUX |												//Enables the configuration of the PMUX for the selected pins
		PORT_WRCONFIG_PMUXEN |												//Enables the PMUX for the pins
		PORT_WRCONFIG_PMUX(PIN_SPI_MISO) |						//Bulk configuration for PMUX "C" for SERCOM5
		PORT_WRCONFIG_HWSEL |												//Select the correct pin configurations for 16-31
		PORT_WRCONFIG_INEN |												//Enable input on this pin MISO
		PORT_WRCONFIG_PINMASK((uint16_t)((PORT_PA12D_SERCOM4_PAD0) >> 16));				//Selecting which pin is configured  PB16  This bit needs to shift to fit the 16 bit macro requirements
		
	//Using the WRCONFIG register to bulk configure both PB22 and PB23 for being configured the SERCOM5 SPI MASTER MOSI and SCK pins
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |											//Enables the configuration of PINCFG
		PORT_WRCONFIG_WRPMUX |												//Enables the configuration of the PMUX for the selected pins
		PORT_WRCONFIG_PMUX(PIN_SPI_MOSI) |						//Bulk configuration for PMUX
		PORT_WRCONFIG_PMUXEN |												//Enables the PMUX for the pins
		PORT_WRCONFIG_HWSEL |												//Select the correct pin configurations for 16-31
		PORT_WRCONFIG_PINMASK ((uint16_t)((PORT_PB10D_SERCOM4_PAD2 | PORT_PB11D_SERCOM4_PAD3) >> 16));	//Selecting which pin is configured

  disableSPI();
  initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
  initSPIClock(SERCOM_SPI_MODE_0, baud);
  enableSPI();
}

void spi_end() {
  disableSPI();
}

uint8_t spi_transfer(uint8_t data)
{
  SPI_SERCOM->SPI.DATA.bit.DATA = data; // Writing data into Data register
  while(SPI_SERCOM->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception
  return SPI_SERCOM->SPI.DATA.bit.DATA;  // Reading data
}
