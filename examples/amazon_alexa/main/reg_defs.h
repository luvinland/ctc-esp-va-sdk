/*
* Copyright (c) 2018 Cirrus Logic International (UK) Ltd.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

// Select which device to compile for
//#define FLORIDA		1
//#define LARGO			1
//#define CLEARWATER	1
//#define MOON			1
//#define MARLEY		1
//#define ASHETON		1
//#define GAINES		1
#define COOKE			1
//#define PRINCE		1 

// Select the device address
// 0x0 to 0x0F are consdered SPI, 0x10 and greater are considereed I2C (note that for I2C these are 7bit addresses
//#define DEVICE_ADDR		0x1A // Smart Codec I2C (note that these are 7bit addresses)
//#define DEVICE_ADDR		0x40 // PRINCE I2C  (note that these are 7bit addresses)
#define DEVICE_ADDR		0x0 // SPI chip select (For those devices that support SPI)


// SPI Address, Padding, Data Bytes and RW operation mask
#if defined(COOKE)
#define SPI_ADDRESS_BYTES               4
#define SPI_PADDING_BYTES               4
#define SPI_DATA_BYTES                  4
#define SPI_RW_OPERATION_MASK           0x80
#elif defined(FLORIDA) || defined(LARGO)
#define SPI_ADDRESS_BYTES               4
#define SPI_PADDING_BYTES               2
#define SPI_DATA_BYTES                  2
#define SPI_RW_OPERATION_MASK           0x80
#elif defined(CLEARWATER) || defined(MOON) || defined(MARLEY) || defined(ASHETON) || defined(GAINES) || defined(PRINCE)
#define SPI_ADDRESS_BYTES               4
#define SPI_PADDING_BYTES               2
#define SPI_DATA_BYTES                  4
#define SPI_RW_OPERATION_MASK           0x80
#endif


// Florida or Largo DSP Register addresses
#if defined(FLORIDA) || defined(LARGO)
#define ADSP2								1
/* DSP base addresses */
#define BASE_CORE1_DSP						0x1100
#define BASE_CORE1_MEM						0x100000
#define BASE_CORE2_DSP						0x1200
#define BASE_CORE2_MEM						0x200000
#define BASE_CORE3_DSP						0x1300
#define BASE_CORE3_MEM						0x300000
#define BASE_CORE4_DSP						0x1400
#define BASE_CORE4_MEM						0x400000

#define OFFSET_PM_START						0x00000
#define OFFSET_ZM_START						0x80000
#define OFFSET_XM_UNPACKED_START			0x90000
#define OFFSET_YM_UNPACKED_START			0xA8000
#define OFFSET_XM_PACKED_START				0x90000		// Not used on ADSP2
#define OFFSET_YM_PACKED_START				0xA8000		// Not used on ADSP2

// Not defined for as Florida/Largo as only 4 Core max
#define BASE_CORE5_DSP			0x0
#define BASE_CORE5_MEM			0x0
#define BASE_CORE6_DSP			0x0
#define BASE_CORE6_MEM			0x0
#define BASE_CORE7_DSP			0x0
#define BASE_CORE7_MEM			0x0

#endif

// Clearwater, Moon, Marley DSP Register addresses
#if defined(CLEARWATER) || defined(MOON) || defined(MARLEY) || defined(ASHETON) || defined(GAINES)
#define ADSP2								1
/* DSP base addresses */
#define BASE_CORE1_DSP						0xFFE00
#define BASE_CORE1_MEM						0x80000
#define BASE_CORE2_DSP						0x17FE00
#define BASE_CORE2_MEM						0x100000
#define BASE_CORE3_DSP						0x1FFE00
#define BASE_CORE3_MEM						0x180000
#define BASE_CORE4_DSP						0x27FE00
#define BASE_CORE4_MEM						0x200000
#define BASE_CORE5_DSP						0x2FFE00
#define BASE_CORE5_MEM						0x280000
#define BASE_CORE6_DSP						0x37FE00
#define BASE_CORE6_MEM						0x300000
#define BASE_CORE7_DSP						0x3FFE00
#define BASE_CORE7_MEM						0x380000

#define OFFSET_PM_START						0x00000
#define OFFSET_ZM_START						0x60000

#define OFFSET_XM_UNPACKED_START			0x20000
#define OFFSET_YM_UNPACKED_START			0x40000
#define OFFSET_XM_PACKED_START				0x20000
#define OFFSET_YM_PACKED_START				0x40000


#endif

// Cooke and Prince Registers Addresses 
#if defined(COOKE) || defined(PRINCE)
#define HALOCORE				1
/* DSP base addresses */
#define BASE_CORE1_DSP			0x25C0000  // not sure if this is really the corresponding base location, could argue it is 0x2000000
#define BASE_CORE1_MEM			0x2000000
#define OFFSET_PM_START			0x1800000
#define OFFSET_ZM_START			0		  // Does not exist on Halo Core, but must be defined to avoid compile errors

// These parameters only apply to Halo Core
#define OFFSET_XM_UNPACKED_START	0x800000
#define OFFSET_YM_UNPACKED_START	0x1400000
#define OFFSET_XM_PACKED_START		0x000000
#define OFFSET_YM_PACKED_START		0xC00000

// not defined for Cooke/Prince as they only have 1 core
#define BASE_CORE2_DSP			0x0
#define BASE_CORE2_MEM			0x0
#define BASE_CORE3_DSP			0x0
#define BASE_CORE3_MEM			0x0
#define BASE_CORE4_DSP			0x0
#define BASE_CORE4_MEM			0x0
#define BASE_CORE5_DSP			0x0
#define BASE_CORE5_MEM			0x0
#define BASE_CORE6_DSP			0x0
#define BASE_CORE6_MEM			0x0
#define BASE_CORE7_DSP			0x0
#define BASE_CORE7_MEM			0x0

#endif
