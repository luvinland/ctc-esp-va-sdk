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
#include "reg_defs.h"

#include "app_defs.h"

#pragma once

/*
  Enum: WMFW_STATUS
  Return codes
*/
typedef enum WMFW_STATUS
{
	WMFW_SUCCESS = 0,       ///< Successful completion
	WMFW_END_OF_FILE,       ///< Reached end of file
	WMFW_FILE_OPEN_FAILED,  ///< Couldn't open file
	WMFW_BAD_FILE_FORMAT,   ///< File not in expected format
	WMFW_OUT_OF_MEMORY,     ///< Memory allocation failed
	WMFW_BAD_PARAM,         ///< Bad parameter
} WMFW_STATUS;

/*
  Enum: WMFW_REGION
  Known regions for the WMFW
*/
typedef enum WMFW_REGION
{
	WMFW_PM = 2,                ///< ADSPx program memory
	WMFW_DM = 3,                ///< ADSP1 data memory
	WMFW_XM = 5,                ///< ADSP2 X coefficient memory, unpacked 24bit HaloCore X Memory
	WMFW_YM = 6,                ///< ADSP2 Y coefficient memory, unpacked 24bit HaloCore Y Memory
	WMFW_ZM = 4,                ///< ADSP1 & ADSP2 Z coefficient memory	
	WMFW_PM_P = 16,             ///< HaloCore Packed Program Memory
	WMFW_XM_P = 17,             ///< HaloCore Packed X Memory
	WMFW_YM_P = 18,              ///< HaloCore Packed Y Memory
	WMFW_XM32 = 33,             ///< HaloCore Un-packed 32bit word X Memory
	WMFW_YM32 = 34,             ///< HaloCore Un-packed 32bit word Y Memory
    WMFW_ALGO_INFO_BLOCK = 0xF2,  ///< User Defined Name Text
	WMFW_USER_DEF_NAME = 0xFE,    ///< User Defined Name Text
	WMFW_INFO_STRING = 0xFF,      ///< Informational ASCII string
} WMFW_REGION;



typedef enum WMFW_CORETYPES
{
	CORETYPE_ADSP1 = 1,
	CORETYPE_ADSP2 = 2,
	CORETYPE_HALOCORE = 4,
	CORETYPE_WARP2 = 12,
	CORETYPE_HIFI2 = 22,
} WMFW_CORETYPES;

/*
* Structure defining the header of a WMFW block
*
* Format for a block:
*
*      31       24 23      16 15       8 7        0
*  0   +----------+----------+----------+---------+
*      | type[7:0]|          offset[23:0]         |
*  4   +----------+-------------------------------+
*      |                dataLength                |
*  8   +------------------------------------------+
*      |                   data                   |
*      :                   ....                   :
*      :                                          :
*  The offset/region and data length are little endian.
*  The data is formatted big-endian to facilitate writing
*  straight to the core.
*/
typedef struct WMFWBlockHeader
{
	unsigned int    offset : 24;
	unsigned int    region : 8;
	unsigned int    dataLength;
} WMFWBlockHeader;


#if defined(ADSP2)
typedef struct fwIdBlockFormat
{
	unsigned int	coreID;
	unsigned int	coreRevision;
	unsigned int	firmwareID;
	unsigned int	firmwareRevision;
	unsigned int	firmwareZmBase;
	unsigned int	firmwareXmBase;
	unsigned int	firmwareYmBase;
	unsigned int	algorithmCount;
} fwIdBlockFormat;

typedef struct AlgorithmIDBlockFormat
{
	unsigned int	algorithmID;
	unsigned int	algorithmVersion;
	unsigned int	algorithmZmBase;
	unsigned int	algorithmXmBase;
	unsigned int	algorithmYmBase;
} AlgorithmIDBlockFormat;

#elif defined(HALOCORE)

typedef struct fwIdBlockFormat
{
	unsigned int	coreID;
	unsigned int	coreRevision;  //technically block format rather than core
	unsigned int	vendorID;
	unsigned int	firmwareID;
	unsigned int	firmwareRevision;
	unsigned int	firmwareXmBase;
	unsigned int	firmwareXmSize;
	unsigned int	firmwareYmBase;
	unsigned int	firmwareYmSize;
	unsigned int	algorithmCount;
} fwIdBlockFormat;

typedef struct AlgorithmIDBlockFormat
{
	unsigned int	algorithmID;
	unsigned int	algorithmVersion;
	unsigned int	algorithmXmBase;
	unsigned int	algorithmXmSize;
	unsigned int	algorithmYmBase;
	unsigned int	algorithmYmSize;
} AlgorithmIDBlockFormat;

#endif

// Note this excludes the "WMDR" identifier and header length
typedef struct BinFileHeaderFormat01
{
	unsigned int	firmwareRevision : 24;
	unsigned int	fileFormatVersion : 8;
	unsigned int	coreRevision : 24;
	unsigned int	coreType : 8;	
} BinFileHeaderFormat01;

typedef struct BinCoeffBlockHeader
{
	unsigned int	offset : 16;
	unsigned int	region : 16;
	unsigned int	algorithmId;
	unsigned int	algorithmVersion;
	unsigned int	sampleRate;
	unsigned int	dataLength;
} BinCoeffBlockHeader;

extern size_t dspBase;
extern size_t pmBase;

// Only used on ADSP2
extern size_t zmBase;

// ADSP2 only uses Unpacked format. HaloCore will use a mixture of packed and unpacked
extern size_t xmBaseUnpacked; 
extern size_t xmBasePacked; 
extern size_t ymBaseUnpacked; 
extern size_t ymBasePacked; 


extern AlgorithmIDBlockFormat * algorithmIdBlocks;

#ifdef __cplusplus
extern "C" {
#endif
int ProcessBinFile(const char *filename);
int ProcessWMFWFile(const char *filename);
#if defined(CTC_CS48L32_OKGOOGLE)
int ProcessSEARCHFile(const char *filename);
int ProcessMODELFile(const char *filename);
#endif
#ifdef __cplusplus
}
#endif

