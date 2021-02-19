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

#ifndef _BOARD_DRIVER_SPIFLASH_
#define _BOARD_DRIVER_SPIFLASH_

#include <sam.h>
#include <stdbool.h>
#include "board_definitions.h"

uint8_t spiflash_readByte(uint32_t addr);
void    spiflash_readBytes(uint32_t addr, void* buf, uint16_t len);
void    spiflash_init( void );
{
#endif // _BOARD_DRIVER_FLASHMEM_