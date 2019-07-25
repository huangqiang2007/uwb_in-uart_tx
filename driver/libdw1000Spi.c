/*
 * Driver for decaWave DW1000 802.15.4 UWB radio chip.
 *
 * Copyright (c) 2016 Bitcraze AB
 * Converted to C from  the Decawave DW1000 library for arduino.
 * which is Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "libdw1000Spi.h"
#include "spidrv.h"

void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                void* data, int length) {
	uint8_t header[Header_Size];
	int headerLength;

	header[0] = regid & 0x3f; //set read mode, set sub-index = 0 by default, set register file ID to header buffer 0
	headerLength = 1; //set header length = 1 by default,

	if (address != 0) {  //if sub-address != 0
	header[0] |= 0x40; //set sub-index = 1

	header[1] = address & 0x7f; //set extend-address = 0, set lower order 7 bits sub-address to header buffer 1
	address >>= 7;
	headerLength = 2; //set header length = 2

	if (address != 0) { //if sub-address > 127
	  header[1] |= 0x80; //set extend-address = 1
	  header[2] = address & 0xff; //set higher order 8 bits sub-address to header buffer 2
	  headerLength = 3; //set header length = 3
	}
	}

	//USART_Enable(usart_spi, usartEnable);
	USART1->CMD |= (usartEnable | USART_CMD_MASTEREN);
	SPISendNbytes(header, headerLength);
	SPIRecvNBytes(data, length);
	//USART_Enable(usart_spi, usartDisable);
	USART1->CMD |= usartDisable;
}

uint16_t dwSpiRead16(dwDevice_t *dev, uint8_t regid, uint32_t address)
{
  uint16_t data;

  USART_Enable(usart_spi, usartEnable);
  dwSpiRead(dev, regid, address, &data, sizeof(data));
  USART_Enable(usart_spi, usartDisable);

  return data;
}

uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address)
{
  uint32_t data;

  USART_Enable(usart_spi, usartEnable);
  dwSpiRead(dev, regid, address, &data, sizeof(data));
  USART_Enable(usart_spi, usartDisable);

  return data;
}

void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
	const void* data, int length) {
  //uint8_t header[Header_Size];
  int headerLength;

  txbuffer[0] = regid & 0xbf; //set write mode, set sub-index = 0 by default, set register file ID to header buffer 0
  headerLength = 1; //set header length = 1 by default,

  if (address != 0) { //if sub-address != 0
	txbuffer[0] |= 0x40; //set sub-index = 1

	txbuffer[1] = address & 0x7f; //set extend-address = 0, set lower order 7 bits sub-address to header buffer 1
    address >>= 7;
    headerLength = 2; //set header length = 2

    if (address != 0) { //if sub-address > 127
      txbuffer[1] |= 0x80; //set extend-address = 1
      txbuffer[2] = address & 0xff; //set higher order 8 bits sub-address to header buffer 2
      headerLength = 3; //set header length = 3

    }
  }

  memcpy(&txbuffer[headerLength], data, length+headerLength);
  USART_Enable(usart_spi, usartEnableTx);
  SPISendNbytes(txbuffer, length+headerLength);
  USART_Enable(usart_spi, usartDisable);
}

void dwSpiWrite8(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint8_t data) {
	USART_Enable(usart_spi, usartEnableTx);
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
	USART_Enable(usart_spi, usartDisable);
}

void dwSpiWrite16(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint16_t data) {
	USART_Enable(usart_spi, usartEnableTx);
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
	USART_Enable(usart_spi, usartDisable);
}

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint32_t data) {
	USART_Enable(usart_spi, usartEnableTx);
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
	USART_Enable(usart_spi, usartDisable);
}
