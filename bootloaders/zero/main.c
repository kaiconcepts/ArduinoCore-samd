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

#include <stdio.h>
#include <sam.h>
#include "sam_ba_monitor.h"
#include "sam_ba_serial.h"
#include "board_definitions.h"
#include "board_driver_led.h"
#include "board_driver_i2c.h"
#include "board_driver_spiflash.h"
#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"

#define BOOTLOADER_SLOT_SIZE 0x2000
#define SKETCH_SLOT_SIZE     0x1E000
#define NEW_APP_SLOT_ADDR    (BOOTLOADER_SLOT_SIZE + SKETCH_SLOT_SIZE)
#define FIRMWARE_HEADER_SIZE 6

extern uint32_t __sketch_vectors_ptr; // Exported value from linker script
extern void board_init(void);

volatile uint32_t* pulSketch_Start_Address;

static void jump_to_application(void) {

  /* Rebase the Stack Pointer */
  __set_MSP( (uint32_t)(__sketch_vectors_ptr) );

  /* Rebase the vector table base address */
  SCB->VTOR = ((uint32_t)(&__sketch_vectors_ptr) & SCB_VTOR_TBLOFF_Msk);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(*pulSketch_Start_Address));
}

/* 
static uint16_t compute_crc(uint8_t *start, uint32_t len) {
  uint16_t crc = 0;
  for (uint32_t i = 0; i < len; i++) {
    crc = serial_add_crc(start[i], crc);
  }
  return crc;
}
*/

#define NEW_APP_SPIFLASH_ADDR 0x1000
#define READ_CHUNK_SIZE       0x400

static void check_new_application(void) {
  // Check for magic number at end of lower slot, indicating that a new app
  // is present in upper slot.
  volatile uint32_t magic = *((uint32_t *)NEW_APP_SLOT_ADDR - 1);
  if (magic != 0x2150415a) return;

  // Validate checksum on new app
  //uint8_t *app_start = (uint8_t *)(NEW_APP_SLOT_ADDR + FIRMWARE_HEADER_SIZE);
  //uint8_t *newapp = (uint8_t*)NEW_APP_SLOT_ADDR;

  uint8_t appheader[FIRMWARE_HEADER_SIZE];
  uint8_t app_chunk[READ_CHUNK_SIZE];
  spiflash_readBytes(NEW_APP_SPIFLASH_ADDR, appheader, FIRMWARE_HEADER_SIZE);

  uint16_t crc =    appheader[5]  | ((uint32_t)appheader[4] << 8);
  uint32_t crclen = appheader[3]  | ((uint32_t)appheader[2] << 8)
                                  | ((uint32_t)appheader[1] << 16)
                                  | ((uint32_t)appheader[0] << 24);
  if (crclen > SKETCH_SLOT_SIZE) return;

  uint16_t crccomputed = 0;
  for (uint32_t i = 0; i < crclen; i += READ_CHUNK_SIZE){
    spiflash_readBytes(NEW_APP_SPIFLASH_ADDR + i, app_chunk, READ_CHUNK_SIZE);
    for (uint32_t j = 0; j < crclen - i && j < READ_CHUNK_SIZE; ++j){
      crccomputed = serial_add_crc(app_chunk[j], crccomputed);
    }
  }

  //uint16_t crccomputed = compute_crc(app_start, crclen);

  // serial_open();
  // serial_putc(0xa5);
  // serial_putc((crc >> 8) & 0xff);
  // serial_putc((crc >> 0) & 0xff);
  // serial_putc(0x0);
  // serial_putc((crccomputed >> 8) & 0xff);
  // serial_putc((crccomputed >> 0) & 0xff);
  if (crccomputed != crc) return;

  // Wipe lower program slot
  uint32_t addr = BOOTLOADER_SLOT_SIZE; // __sketch_vectors_ptr;
  uint32_t end = addr + SKETCH_SLOT_SIZE;

  const uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
  uint32_t PAGE_SIZE = pageSizes[NVMCTRL->PARAM.bit.PSZ];

  while (addr < end) {
    NVMCTRL->ADDR.reg = addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (NVMCTRL->INTFLAG.bit.READY == 0);
    addr += PAGE_SIZE * 4; // Skip a ROW
  }

  // Copy upper slot to lower slot, erasing magic bytes
  uint32_t size = SKETCH_SLOT_SIZE / 2;
  // new app address + firmware header size + extra length header (4)
  uint32_t src_addr = (NEW_APP_SPIFLASH_ADDR + FIRMWARE_HEADER_SIZE + 4);
  size -= FIRMWARE_HEADER_SIZE / 2;
  uint16_t *dst_addr = (uint16_t*)BOOTLOADER_SLOT_SIZE; //__sketch_vectors_ptr;

  NVMCTRL->CTRLB.bit.MANW = 0;  // Set automatic page write
  while (size) { // Do writes in pages
    // Execute "PBC" Page Buffer Clear
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    while (NVMCTRL->INTFLAG.bit.READY == 0);
    spiflash_readBytes(src_addr, app_chunk, PAGE_SIZE);
    // Fill page buffer
    uint32_t i;
    for (i=0; i<(PAGE_SIZE/2) && i<size; i++) {
      // TODO: Because of the src offset caused by the header, we'll copy
      // a few bytes from outside the flash range. Should make sure this isn't
      // going to cause a problem.
      dst_addr[i] = ((uint16_t *)app_chunk)[i];
    }
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    while (NVMCTRL->INTFLAG.bit.READY == 0);

    // Advance to next page
    dst_addr += i;
    src_addr += i << 1;
    size -= i;
  }
}

static volatile bool main_b_cdc_enable = false;

#ifdef CONFIGURE_PMIC
static volatile bool jump_to_app = false;
#endif

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
//  LED_init();
//  LED_off();

  /*
   * Test sketch stack pointer @ &__sketch_vectors_ptr
   * Stay in SAM-BA if value @ (&__sketch_vectors_ptr) == 0xFFFFFFFF (Erased flash cell value)
   */
  if (__sketch_vectors_ptr == 0xFFFFFFFF)
  {
    /* Stay in bootloader */
    return;
  }

  /*
   * Load the sketch Reset Handler address
   * __sketch_vectors_ptr is exported from linker script and point on first 32b word of sketch vector table
   * First 32b word is sketch stack
   * Second 32b word is sketch entry point: Reset_Handler()
   */
  pulSketch_Start_Address = &__sketch_vectors_ptr ;
  pulSketch_Start_Address++ ;

  /*
   * Test vector table address of sketch @ &__sketch_vectors_ptr
   * Stay in SAM-BA if this function is not aligned enough, ie not valid
   */
  if ( ((uint32_t)(&__sketch_vectors_ptr) & ~SCB_VTOR_TBLOFF_Msk) != 0x00)
  {
    /* Stay in bootloader */
    return;
  }

#if defined(BOOT_DOUBLE_TAP_ADDRESS)
  #define DOUBLE_TAP_MAGIC 0x07738135
  if (PM->RCAUSE.bit.POR)
  {
    /* On power-on initialize double-tap */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
  else
  {
    if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC)
    {
      /* Second tap, stay in bootloader */
      BOOT_DOUBLE_TAP_DATA = 0;
      return;
    }

    /* First tap */
    BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;

    /* Wait 0.5sec to see if the user tap reset again.
     * The loop value is based on SAMD21 default 1MHz clock @ reset.
     */
    for (uint32_t i=0; i<125000; i++) /* 500ms */
      /* force compiler to not optimize this... */
      __asm__ __volatile__("");

    /* Timeout happened, continue boot... */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
#endif

/*
#if defined(BOOT_LOAD_PIN)
  volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
  volatile bool boot_en;

  // Enable the input mode in Boot GPIO Pin
  boot_port->DIRCLR.reg = BOOT_PIN_MASK;
  boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
  boot_port->OUTSET.reg = BOOT_PIN_MASK;
  // Read the BOOT_LOAD_PIN status
  boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;

  // Check the bootloader enable condition
  if (!boot_en)
  {
    // Stay in bootloader
    return;
  }
#endif
*/

//  LED_on();
#ifdef CONFIGURE_PMIC
  jump_to_app = true;
#else
  jump_to_application();
#endif

}

#if DEBUG_ENABLE
#	define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
#	define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
#	define DEBUG_PIN_HIGH 	do{}while(0)
#	define DEBUG_PIN_LOW 	do{}while(0)
#endif

/**
 *  \brief SAMD21 SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  P_USB_CDC pCdc;
#endif
  DEBUG_PIN_HIGH;

  /* We have determined we should stay in the monitor. */
  /* System initialization */
  board_init();
  __enable_irq();

  spiflash_init();

  check_new_application();
  /* Jump in application if condition is satisfied */
  check_start_application();

#ifdef CONFIGURE_PMIC
  configure_pmic();
  if (jump_to_app == true) {
    jump_to_application();
  }
#endif

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  /* UART is enabled in all cases */
  serial_open();
#endif

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  pCdc = usb_init();
#endif

  DEBUG_PIN_LOW;

  /* Initialize LEDs */
  LED_init();
  LEDRX_init();
  LEDRX_off();
  LEDTX_init();
  LEDTX_off();

  /* Start the sys tick (1 ms) */
  SysTick_Config(1000);

  /* Wait for a complete enum on usb or a '#' char on serial line */
  while (1)
  {
    check_new_application();
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    if (pCdc->IsConfigured(pCdc) != 0)
    {
      main_b_cdc_enable = true;
    }

    /* Check if a USB enumeration has succeeded and if comm port has been opened */
    if (main_b_cdc_enable)
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USBCDC);
      /* SAM-BA on USB loop */
      while( 1 )
      {
        sam_ba_monitor_run();
      }
    }
#endif

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    /* Check if a '#' has been received */
    if (!main_b_cdc_enable && serial_sharp_received())
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
      /* SAM-BA on Serial loop */
      while(1)
      {
        sam_ba_monitor_run();
      }
    }
#endif
  }
}

void SysTick_Handler(void)
{
  LED_pulse();

  sam_ba_monitor_sys_tick();
}
