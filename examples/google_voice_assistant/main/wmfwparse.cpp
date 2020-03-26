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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "reg_defs.h"
#include "wmfwparse.h"
//#include "hal.h"
#include <stdint.h>


#include "esp_system.h"

#include "driver/spi_master.h"
extern spi_device_handle_t g_spi;

// Global definitions
const char *wmfw_id_string = "WMFW";
const int wmfw_id_length = 4;
const char *bin_id_string = "WMDR";
const int bin_id_length = 4;


/*
 *  Cirrus firmware (WMFW) files are designed to be simple to parse and load
 *  to a DSP core.  In essence, the structure is as follows:
 * 
 *   - A file header with basic information about the firmware.
 *   - One or more data blocks of different formats.
 *  
 *  The only data blocks which HAVE to be handled are the memory image data
 *  blocks.  These contain a header specifying which area of memory they
 *  are intended for, where they start, and how long they are.  The data in
 *  is pre-formatted for bulk writing to the registers on the device.
 * 
 *  This file shows the minimum necessary to parse the WMFW file, extract the
 *  data blocks and load them to the corresponding memory areas on the DSP
 *  core.
 * 
 *  There are other blocks containing informational strings and metadata, but
 *  these can safely be ignored for loading the firmware image to the DSP core.
 * 
 *  This sample code is as self-contained as possible, but it does require
 *  a couple of platform-specific items:
 * 
 *   - Knowledge of the start addresses of the PM, XM, YM and ZM memory regions
 *     for the DSP code
 *   - A SendDataToDevice function which takes a byte array and a start location and
 *     writes the data in the array to the device as a sequential bytestream.
 * 
 *  Sample versions of these are provided for the purpose of illustration.
 *  They will need to be replaced in a real implementation with versions
 *  designed for the target platform.
 * 
 */


int wmfwFileVersion = 0;
fwIdBlockFormat fwIdBlock;


//
// Function prototypes
//
#ifdef __cplusplus
extern "C" {
#endif
void SwapBufferEndianness(unsigned char * buffer, int length);
void ExpandPacked24Buffer(unsigned char * bufferP, unsigned char * bufferUP, int length);
int ParseFirmwareInfo(unsigned char* buffer, int length);
AlgorithmIDBlockFormat* GetAlgorithmInfo(unsigned int algorithmIndex);
int ProcessWMFWHeader(FILE *wmfwFile);
int ProcessNextWMFWBlock(FILE *wmfwFile);
int ProcessBinHeader(FILE *binFile);
int ProcessUserDefinedNameBlock(FILE *binFile);
int ProcessNextCoeffBlock(FILE *binFile);
void SpiWriteBlock(uint32_t regAddr, uint8_t *buffer, uint32_t length);
#if defined(CTC_CS48L32_OKGOOGLE)
int ProcessSEARCHBlock(FILE *searchFile);
int ProcessMODELBlock(FILE *modelFile);
#endif
#ifdef __cplusplus
}
#endif


static void SwapEndianness(uint8_t* out, uint8_t* in, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		out[size - 1 - i] = in[i];
	}
}

void SpiWriteBlock(uint32_t regAddr, uint8_t *buffer, uint32_t length)
{
	esp_err_t ret = ESP_OK;
	const uint32_t cs48l32_spi_padding = 0x0;

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));

	uint8_t* dataOut = (uint8_t*)malloc(length + 8);
	assert(0 != dataOut);

	t.length = length * 8 + 64;

	SwapEndianness(&dataOut[0], (uint8_t*)&regAddr, 4);
	SwapEndianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
	for(uint16_t i = 0; i < length; i++)
	{
		dataOut[8 + i] = buffer[i];
	}
	
	t.tx_buffer = dataOut;
	
	ret = spi_device_transmit(g_spi, &t);

	if (dataOut)
	{
		free(dataOut);
		dataOut = NULL;
	}

}

/*
*  Function:  SwapBufferEndianness
*
*  @brief  Swaps the endianness of an entire buffer 
*
*  Swaps endianness of each 32-bit word in the buffer
*
*  @param  buffer	   Pointer to buffer to be swapped
*  @param  length	   Length of buffer in bytes
*
*/
void SwapBufferEndianness(unsigned char * buffer, int length)
{
	unsigned char temp;
	if (length % 4 != 0)
	{
		printf("Error: endianness cannot be swapped as buffer is not a multiple of 4 bytes. Cancelling operation.\n");
		return;
	}
	uint8_t* swap = (uint8_t*) buffer;
	for (int i = 0; i < length; i += 4)
	{
		//printf("Before swap: %02x %02x %02x %02x\n", swap[0], swap[1], swap[2], swap[3]);
		temp = swap[3 + i];
		swap[3 + i] = swap[0 + i];
		swap[0 + i] = temp;
		temp = swap[2 + i];
		swap[2 + i] = swap[1 + i];
		swap[1 + i] = temp;
	}

	return;
}

/*
*  Function:  ExpandPacked24Buffer
*
*  @brief  Takes a packed buffer of 24bit samples and returns unpacked 24bit buffer
*
*  Unpack a 24bit buffer
*
*  @param  bufferP	   Pointer to buffer of packed data
*  @param  bufferUP	   Pointer to buffer of unpacked data
*  @param  length	   Length of packed data in bytes
*
*/
void ExpandPacked24Buffer(unsigned char * bufferP, unsigned char * bufferUP, int length)
{
	if (length % 3 != 0) // 
	{
		printf("Error: Buffer cannot be expanded as buffer is not a multiple of 24bit words. Cancelling operation.\n");
		return;
	}
	uint8_t* inptr = (uint8_t*)bufferP;
	uint8_t* outptr = (uint8_t*)bufferUP;
	int k = 0;
	for (int i = 0; i < length; i += 3)
	{
		outptr[0 + k] = inptr[0 + i]; 
		outptr[1 + k] = inptr[1 + i]; 
		outptr[2 + k] = inptr[2 + i]; 
		outptr[3 + k] = 0; // padding byte
		k += 4;
	}

	return;
}


/*
*  Function:  ParseFirmwareInfo
*
*  @brief  Parses the firmware and algorithm information from start of XM
*
*  Extracts firmware and algorithm information from the buffer of data written to start of X memory
*
*  @param  buffer	   Pointer to buffer containing algorithm header info
*  @param  length	   Length of buffer
*
*/
int ParseFirmwareInfo(unsigned char * buffer, int length)
{
	int status = WMFW_SUCCESS;
	//int sizeOfAlgorithmData = 0;
	int sizeOfPackedAlgorithmData = 0;
	AlgorithmIDBlockFormat * eachAlgorithm = NULL;
	int sizeOfPackedFwIdBlock = sizeof(fwIdBlockFormat) / 4 * 3;
	//int algorithmCount = 0;
	unsigned char *temp = NULL;

	// Extract the firmware Info from buffer into the global fwIdBlock structure
	// need endianness swap and potentially unpacking depending on the core type.
	// We do NOT want to swap endianness in the buffer itself
	temp = (unsigned char *)malloc(length);
	if (!temp)
	{
		status = WMFW_OUT_OF_MEMORY;
		goto parseFirmwareInfoDone;
	}
	memcpy(temp, buffer, length);
	SwapBufferEndianness(temp, length);
#if defined(HALOCORE)
	ExpandPacked24Buffer(temp, (unsigned char *)&fwIdBlock, sizeOfPackedFwIdBlock);
#elif defined(ADSP2)
	memcpy(&fwIdBlock, temp, sizeof(fwIdBlockFormat));
#endif

#if defined(ADSP2)
	printf("Firmware and Algorithm Information Found:\n\n");
	printf("\tCore ID \t %08x\n", fwIdBlock.coreID);
	printf("\tCore Rev \t %08x\n", fwIdBlock.coreRevision);
	printf("\tFirmware ID \t %08x\n", fwIdBlock.firmwareID);
	printf("\tFirmware Rev \t %08x\n", fwIdBlock.firmwareRevision);	
	printf("\tSystem ZM Base\t %08x\n", fwIdBlock.firmwareZmBase); 
	printf("\tSystem XM Base\t %08x\n", fwIdBlock.firmwareXmBase);
	printf("\tSystem YM Base\t %08x\n", fwIdBlock.firmwareYmBase);
	printf("\tAlgorithm Count\t %08x\n", fwIdBlock.algorithmCount);
#elif defined(HALOCORE)
	printf("Firmware and Algorithm Information Found:\n\n");
	printf("\tCore ID \t %08x\n", fwIdBlock.coreID);
	printf("\tFormat Rev \t %08x\n", fwIdBlock.coreRevision);
	printf("\tFirmware ID \t %08x\n", fwIdBlock.firmwareID);
	printf("\tFirmware Rev \t %08x\n", fwIdBlock.firmwareRevision);
	printf("\tSystem XM Base\t %08x\n", fwIdBlock.firmwareXmBase);
	printf("\tSystem XM Size\t %08x\n", fwIdBlock.firmwareXmSize);
	printf("\tSystem YM Base\t %08x\n", fwIdBlock.firmwareYmBase);
	printf("\tSystem YM Size\t %08x\n", fwIdBlock.firmwareYmSize);
	printf("\tAlgorithm Count\t %08x\n", fwIdBlock.algorithmCount);
#endif

	// Allocate memory for however many algorithm description blocks are present (according to FWID block) plus one extra entry to store duplicate of the FwId info
	algorithmIdBlocks = (AlgorithmIDBlockFormat*)malloc((fwIdBlock.algorithmCount+1) * sizeof(AlgorithmIDBlockFormat));
	if (!algorithmIdBlocks)
	{
		status = WMFW_OUT_OF_MEMORY;
		goto parseFirmwareInfoDone;
	}

	// Create an additional "algorithm" in the list to store copy of the FwID information
	// This makes look-up easier when parsing the AlgorithmInfo Blocks
#if defined(ADSP2)
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmID = fwIdBlock.firmwareID;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmVersion = fwIdBlock.firmwareRevision;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmZmBase = fwIdBlock.firmwareZmBase;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmXmBase = fwIdBlock.firmwareXmBase;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmYmBase = fwIdBlock.firmwareYmBase;
#elif defined(HALOCORE)
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmID = fwIdBlock.firmwareID;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmVersion = fwIdBlock.firmwareRevision;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmXmBase = fwIdBlock.firmwareXmBase;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmXmSize = fwIdBlock.firmwareXmSize;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmYmBase = fwIdBlock.firmwareYmBase;
	algorithmIdBlocks[fwIdBlock.algorithmCount].algorithmYmSize = fwIdBlock.firmwareYmSize;
#endif 

#ifdef HALOCORE
	sizeOfPackedAlgorithmData = fwIdBlock.algorithmCount*sizeof(AlgorithmIDBlockFormat) * 3 / 4;
	ExpandPacked24Buffer(temp + sizeOfPackedFwIdBlock , (unsigned char *)algorithmIdBlocks, sizeOfPackedAlgorithmData);
#else
	// Copy data from the buffer into newly allocated memory space, then swap endianness for ease of parsing
	sizeOfAlgorithmData = fwIdBlock.algorithmCount*sizeof(AlgorithmIDBlockFormat);
	memcpy(algorithmIdBlocks, temp + sizeof(fwIdBlockFormat), sizeOfAlgorithmData);
#endif

	// Iterate through the algorithm ID blocks and print out all the data for info (skip the firmware ID info which is in last location) 
	for (unsigned int i = 0; i < fwIdBlock.algorithmCount; i++)
	{
		eachAlgorithm = GetAlgorithmInfo(i);
#if defined(ADSP2)
		printf("\tAlgo %d ID \t %08x\n", i, eachAlgorithm->algorithmID);
		printf("\tAlgo %d Version \t %08x\n", i, eachAlgorithm->algorithmVersion);
		printf("\tAlgo %d ZM Base \t %08x\n", i, eachAlgorithm->algorithmZmBase);
		printf("\tAlgo %d XM Base \t %08x\n", i, eachAlgorithm->algorithmXmBase);
		printf("\tAlgo %d YM Base \t %08x\n", i, eachAlgorithm->algorithmYmBase);
#elif defined(HALOCORE)
		printf("\tAlgo %d ID \t %08x\n", i, eachAlgorithm->algorithmID);
		printf("\tAlgo %d Version \t %08x\n", i, eachAlgorithm->algorithmVersion);
		printf("\tAlgo %d XM Base \t %08x\n", i, eachAlgorithm->algorithmXmBase);
		printf("\tAlgo %d XM Size \t %08x\n", i, eachAlgorithm->algorithmXmSize);
		printf("\tAlgo %d YM Base \t %08x\n", i, eachAlgorithm->algorithmYmBase);
		printf("\tAlgo %d YM Size \t %08x\n", i, eachAlgorithm->algorithmYmSize);
#endif
	}

parseFirmwareInfoDone:
	free(temp);
    temp = NULL;
	return status;
}


/*
*  Function:  GetAlgorithmInfo
*
*  @brief  Retrieves the algorithm information structure given an index
*
*  Gets the information from global structures - ParseFirmwareInfo() must have been run on a WMFW first
*  to extract this information
*
*  @param  algorithmIndex	   Index number of algorithm data to retrieve
*
*  @retval NULL				No algorithm found for requested index
*
*/
AlgorithmIDBlockFormat * GetAlgorithmInfo(unsigned int algorithmIndex)
{
	
	if (/*algorithmIndex < 0 ||*/ algorithmIndex > fwIdBlock.algorithmCount+1)
	{
		printf("\tError: algorithm with requested index not found\n");
		return NULL;
	}

	if (algorithmIdBlocks == NULL)
	{
		printf("\tError: Algorithm information not present\n");
		return NULL;
	}

	AlgorithmIDBlockFormat * algorithmInfo;
	algorithmInfo = algorithmIdBlocks + algorithmIndex;

	return algorithmInfo;

}




/*
*  Function:  FindMatchingAlgoBlock
*
*  @brief  Retrieves the algorithm information structure matching the information in a coefficient block header
*
*  @param  blockHeader		Coefficient block header information containing algorithm ID and version numbers
*
*  @retval NULL				No algorithm found for requested index
*
*/
AlgorithmIDBlockFormat * FindMatchingAlgoBlock(BinCoeffBlockHeader blockHeader)
{
	AlgorithmIDBlockFormat * algorithmInfo = NULL;

	// Check if the algortihm ID is non-zero before searching for a match
	if (blockHeader.algorithmId != 0)
	{
		// Cycle through the algorithms that were loaded by the WMFW and 
		// see if there is a match. If so, return a pointer to this algorithm info
		// AlgorithmCount + 1 is used to also include the FwID information which is included as an extra entry at the end of the algorithm list
		for (unsigned int i = 0; i < fwIdBlock.algorithmCount + 1; i++)
		{
			algorithmInfo = GetAlgorithmInfo(i);
			if (algorithmInfo->algorithmID == blockHeader.algorithmId)
			{
				printf("\tAlgorithm ID %08x found\n", blockHeader.algorithmId);

				// Check version number on system versus coefficient data - must match Major version number (minor and patch may differ)
				// Note: WMDR format places the 0-filled byte in LSBs rather than MSBs, so shift by 16 to compare
				if ((algorithmInfo->algorithmVersion & 0xFFFF0000) != ((blockHeader.algorithmVersion >> 8) & 0xFFFF0000))
				{
					printf("\tError: Revision mismatch - cannot safely apply these coefficients\n");
					printf("\tWMFW Revision: %08X\n\tBin  Revision: %08X\n", algorithmInfo->algorithmVersion, (blockHeader.algorithmVersion>>8));
					// Reset the algorithmInfo back to NULL so that we're not returning false info
					algorithmInfo = NULL;
					break;
				}
				else
				{
					// If not version conflict, break the loop leaving algorithmInfo pointing to the correct algorithm block
					break;
				}
			}
			else
			{
				algorithmInfo = NULL;
			}
		}
	}

	if (algorithmInfo == NULL)
	{
		printf("\tError: No matching algorithm found\n");
	}

	return algorithmInfo;
}


/*
*  Function:  FindMatchingAlgoBlock
*
*  @brief  Overload version that only looks for algorithm ID, not matching the version number. This is for use with AlgorithmInfoBlocks
*
*  @param  algorithmId		Algorithm ID number
*
*  @retval NULL				No algorithm found for requested index
*
*/
AlgorithmIDBlockFormat * FindMatchingAlgoBlock(unsigned int algorithmId)
{
	AlgorithmIDBlockFormat * algorithmInfo = NULL;

	// Check if the algortihm ID is non-zero before searching for a match
	if (algorithmId != 0)
	{
		// Cycle through the algorithms that were loaded by the WMFW and 
		// see if there is a match. If so, return a pointer to this algorithm info
		// AlgorithmCount + 1 is used to also include the FwID information which is included as an extra entry at the end of the algorithm list
		for (unsigned int i = 0; i < fwIdBlock.algorithmCount + 1; i++)
		{
			algorithmInfo = GetAlgorithmInfo(i);
			if (algorithmInfo != NULL)
			{
				if (algorithmInfo->algorithmID == algorithmId)
				{
					printf("\tInformation for algorithm ID %08x found\n", algorithmId);
					break;
				}
				else
				{
					algorithmInfo = NULL;
				}
			}
		}
	}

	if (algorithmInfo == NULL)
	{
		printf("\tError: No matching algorithm found\n");
	}

	return algorithmInfo;
}

/*
*  Function:  ExtractStringData
*
*  @brief  Extracts and saves string data from an algorithm information block
*
* Extracts and saves string data from Algorithm Information Data Blocks in WMFW files
*
*  @param  buffer				Pointer that we allocate and save the string data to
*  @param  length				Non-padded string length reported by AlgorithmInfo block
*  @param  numLengthBytes		Number of bytes used to store the length data
*  @param  paddedStringLength	Pointer to location for storing the calculated padded string length
*
*
*/
int ExtractStringData(unsigned char ** outputBuffer, unsigned char ** inputBuffer, unsigned int stringLength, unsigned int numLengthBytes, unsigned int *paddedStringLength)
{
	int status = WMFW_SUCCESS;

	// Length must be at least 3 bytes long and padded such that the name ends on a 4-byte boundary.
	*paddedStringLength = ((stringLength + numLengthBytes + 3) & ~0x03) - numLengthBytes;

	// Print out lengths for debug use if required
	//printf("\tString Length: %d bytes\n", stringLength);
	//printf("\tString Name Length: %d bytes\n", *paddedStringLength);

	// Extract string to outputBuffer
	if (stringLength > 0)
	{
		*outputBuffer = (unsigned char *)malloc(stringLength + 1);
		if (!*outputBuffer)
		{
			status = WMFW_OUT_OF_MEMORY;
			return status;
		}
		for (unsigned int i = 0; i < stringLength; i++)
		{
			(*outputBuffer)[i] = (*inputBuffer)[i];
		}
		(*outputBuffer)[stringLength] = '\0';
	}
	

	return status;
}


/*
*  Function:  ParseAlgorithmInfoBlock
*
*  @brief  Parses Algorithm & Coefficient information from an Algorithm Information Data Block
*
*  Extracts algorithm and coefficient control information from Algorithm Information Data Blocks in WMFW files
*
*  @param  buffer	   Pointer to buffer containing algorithm information data block
*  @param  length	   Length of buffer
*
*/
int ParseAlgorithmInfoBlock(unsigned char * buffer, int length)
{
	int status = WMFW_SUCCESS;
	unsigned int algorithmId;
	unsigned int algorithmNameLength;
	unsigned int paddedAlgorithmNameLength;
	unsigned char *algorithmName = NULL;
	unsigned int algorithmDescriptionLength;
	unsigned int paddedAlgorithmDescriptionLength;
	unsigned char *algorithmDescription = NULL;
	unsigned int algorithmCoefficientCount;
	AlgorithmIDBlockFormat * algorithmIDBlock;

	// Extract Algorithm ID
	algorithmId = (unsigned int)(buffer[0] + (buffer[1]<<8) + (buffer[2]<<16) + (buffer[3]<<24));
	printf("\tAlgo Id: 0x%08x\n", algorithmId);
	buffer += 4;

	// Find algorithm info based upon the ID number, this will be needed to calculate register addresses
	// The WMFW spec does not guarantee that the XM[0] block comes before this, but in practice it is normally the case
	// due to the way WMFW files are generated. 
	// To support cases where the XM[0] block comes after the AlgorithmInfo block, it would be necessary to set a flag
	// and reevaluate the AlgorithmInfo block later once all necessary info is available.
	algorithmIDBlock = FindMatchingAlgoBlock(algorithmId);
	if (!algorithmIDBlock)
	{
		printf("\tError: Cannot find matching algorithm for AlgorithmInfo Block\n");
		goto algorithmInfoParseDone;
	}

	// Extract length of Algorithm Name
	algorithmNameLength = (unsigned int)*buffer;
	buffer += 1;

	if (WMFW_SUCCESS != ExtractStringData(&algorithmName, &buffer, algorithmNameLength, 1, &paddedAlgorithmNameLength))
	{
		status = WMFW_OUT_OF_MEMORY;
		goto algorithmInfoParseDone;
	}

	if (algorithmNameLength > 0)
	{
		printf("\tAlgorithm Name: %s\n", algorithmName);
	}
	else
	{
		printf("\tAlgorithm Name: \n");
	}

	// Always increment by padded length rather than length reported in the file
	buffer += paddedAlgorithmNameLength;

	// Extract length of Description
	algorithmDescriptionLength = (unsigned int)(buffer[0] + (buffer[1] << 8));
	buffer += 2;

	if (WMFW_SUCCESS != ExtractStringData(&algorithmDescription, &buffer, algorithmDescriptionLength, 2, &paddedAlgorithmDescriptionLength))
	{
		status = WMFW_OUT_OF_MEMORY;
		goto algorithmInfoParseDone;
	}

	if (algorithmDescriptionLength > 0)
	{
		printf("\tAlgorithm Description: %s\n", algorithmDescription);
	}
	else
	{
		printf("\tAlgorithm Description: \n");
	}
    
    // Always increment by padded length rather than length reported in the file
	buffer += paddedAlgorithmDescriptionLength;


	algorithmCoefficientCount = (unsigned int)(buffer[0] + (buffer[1]<<8) + (buffer[2]<<16) + (buffer[3]<<24));
	printf("\tCoefficient Descriptor Count: %d\n", algorithmCoefficientCount);
	buffer += 4;

	// Parse each coefficient block
	for (unsigned int i = 0; i < algorithmCoefficientCount; i++)
	{
		unsigned short startOffset;
		unsigned short blockType;
		unsigned int coefficientSize;
		unsigned int coefficientNameLength;
		unsigned int paddedCoefficientNameLength;
		unsigned char * coefficientName = NULL;
		unsigned int coefficientFullNameLength;
		unsigned int paddedCoefficientFullNameLength;
		unsigned char * coefficientFullName = NULL;
		unsigned int coefficientDescriptionLength;
		unsigned int paddedCoefficientDescriptionLength;
		unsigned char * coefficientDescription = NULL;
		unsigned short coefficientType;
		unsigned short coefficientFlags;
		unsigned int controlLength;
		//unsigned char * coefficientInfoBlock = NULL;
		unsigned int registerAddress;

		printf("\n\t\tCoefficient %d\n", i);

		startOffset = (unsigned short)(buffer[0] + (buffer[1]<<8));
		buffer += 2;
		printf("\t\tCoefficient Start Offset: %x\n", startOffset);

		blockType = (unsigned short)(buffer[0] + (buffer[1]<<8));
		buffer += 2;
		printf("\t\tBlock Type: 0x%02x ", blockType);
		switch (blockType)
		{
            case WMFW_PM: printf("(PM)\n"); break;
            case WMFW_ZM: printf("(ZM)\n"); break;
            case WMFW_XM: 
                printf("(XM Unpacked 24)\n");
                registerAddress = xmBaseUnpacked + algorithmIDBlock->algorithmXmBase*4 + startOffset*4;
                printf("\t\tRegister Address: 0x%08x\n", registerAddress);
                break;
            case WMFW_YM: printf("(YM Unpacked 24)\n"); 			
                registerAddress = ymBaseUnpacked + algorithmIDBlock->algorithmYmBase*4 + startOffset*4;
                printf("\t\tRegister Address: 0x%08x\n", registerAddress);
                break;
            case WMFW_PM_P: printf("(PM Packed)\n"); break;
            case WMFW_XM_P: printf("(XM Packed)\n"); break;
            case WMFW_YM_P: printf("(YM Packed)\n"); break;
            case WMFW_XM32: printf("(XM Unpacked 32)\n"); break;
            case WMFW_YM32: printf("(YM Unpacked 32)\n"); break;
            default: printf("Unknown"); break;
		}

		coefficientSize = (unsigned int)(buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24));
		buffer += 4;
		printf("\t\tCoefficient Size: %d bytes\n", coefficientSize);

		coefficientNameLength = (unsigned int)*buffer;
		buffer += 1;

		if (WMFW_SUCCESS != ExtractStringData(&coefficientName, &buffer, coefficientNameLength, 1, &paddedCoefficientNameLength))
		{
			status = WMFW_OUT_OF_MEMORY;
			goto coefficientParseDone;
		}

		if (coefficientNameLength > 0)
		{
			printf("\t\tCoefficient Name: %s\n", coefficientName);
		}
		else
		{
			printf("\t\tCoefficient Name: \n");
		}

		// Always increment by padded length rather than length reported in the file
		buffer += paddedCoefficientNameLength;


		coefficientFullNameLength = (unsigned int)*buffer;
		buffer += 1;
		
		if (WMFW_SUCCESS != ExtractStringData(&coefficientFullName, &buffer, coefficientFullNameLength, 1, &paddedCoefficientFullNameLength))
		{
			status = WMFW_OUT_OF_MEMORY;
			goto coefficientParseDone;
		}

		if (coefficientFullNameLength > 0)
		{
			printf("\t\tCoefficient Full Name: %s\n", coefficientFullName);
		}
		else
		{
			printf("\t\tCoefficient Full Name: \n");
		}

		// Always increment by padded length rather than length reported in the file
		buffer += paddedCoefficientFullNameLength;

		coefficientDescriptionLength = (unsigned int)(buffer[0]+(buffer[1]<<8));
		buffer += 2;
		
		if (WMFW_SUCCESS != ExtractStringData(&coefficientDescription, &buffer, coefficientDescriptionLength, 2, &paddedCoefficientDescriptionLength))
		{
			status = WMFW_OUT_OF_MEMORY;
			goto coefficientParseDone;
		}

		if (coefficientDescriptionLength > 0)
		{
			printf("\t\tCoefficient Description: %s\n", coefficientDescription);
		}
		else
		{
			printf("\t\tCoefficient Description: \n");
		}

        {
    		// Always increment by padded length rather than length reported in the file
    		buffer += paddedCoefficientDescriptionLength;

    		coefficientType = (unsigned short)(buffer[0] + (buffer[1]<<8));
    		buffer += 2;
    		coefficientFlags = (unsigned short)(buffer[0]+(buffer[1]<<8));
    		buffer += 2;
    		controlLength = (unsigned int)(buffer[0] + (buffer[1]<<8));
    		buffer += 4;

    		printf("\t\tCoefficient Type: 0x%04x ", coefficientType);
    		switch (coefficientType)
    		{
    		case 0x0000: printf("(NONE)\n"); break;
    		case 0x0001: printf("(BOOLEAN)\n"); break;
    		case 0x0002: printf("(INTEGER)\n"); break;
    		case 0x0003: printf("(ENUMERATED)\n"); break;
    		case 0x0004: printf("(BYTES)\n"); break;
    		case 0x0005: printf("(IEC958)\n"); break;
    		case 0x0006: printf("(INTEGER64)\n"); break;
    		case 0x1000: printf("(ACKNOWLEDGED_CONTROL)\n"); break;
    		case 0x1001: printf("(EVENT_CONTROL)\n"); break;
    		case 0x1002: printf("(HOST_BUFFER)\n"); break;
    		case 0x1003: printf("(HEADPHONE_IMPEDANCE)\n"); break;
    		case 0x1004: printf("(EVENT_NOTIFIER)\n"); break;
    		case 0x1005: printf("(INDIRECT_BYTES)\n"); break;
    		default: printf("(Unknown)\n"); break;
    		}

    		printf("\t\tCoefficient Flags: 0x%04x\n", coefficientFlags);
    		if (coefficientFlags & 0x8000)
    			printf("\t\t\tSystem Flag\n");
    		if (coefficientFlags & 0x0008)
    			printf("\t\t\tUnsecured Flag\n");
    		if (coefficientFlags & 0x0004)
    			printf("\t\t\tVolatile Flag\n");
    		if (coefficientFlags & 0x0002)
    			printf("\t\t\tWriteable Flag\n");
    		if (coefficientFlags & 0x00001)
    			printf("\t\t\tReadable Flag\n");

    		printf("\t\tControl Length: %d\n", controlLength);

    		// Size of data so far (minus 8-byte header) to be compared with coefficientSize
    		// This will be 12 bytes of non-variable fields plus the variable length parameters
    		unsigned int descriptorSizeSoFar = 12 + paddedCoefficientNameLength + paddedCoefficientFullNameLength + paddedCoefficientDescriptionLength;

    		if (coefficientSize > descriptorSizeSoFar)
    		{
    			printf("\t\tCoefficient Info Block present\n");
    			// Skip over the coefficient info block
    			// Todo: parse coefficient info blocks
    			buffer += (coefficientSize - descriptorSizeSoFar);
    		}
    		else if (coefficientSize == descriptorSizeSoFar)
    		{
    			printf("\t\tNo Coefficient Info Block\n");
    		}
    		else
    		{
    			printf("\t\tError: coefficient descriptor larger than expected length\n");
    			status = WMFW_BAD_FILE_FORMAT;
    			goto coefficientParseDone;
    		}

        }

	coefficientParseDone:

		free(coefficientName);
		free(coefficientFullName);
		free(coefficientDescription);
		if (status != WMFW_SUCCESS)
			goto algorithmInfoParseDone;

	}

algorithmInfoParseDone:
	free(algorithmName);
	free(algorithmDescription);
	return status;
}



/*
*  Function:  ProcessWMFWFile
*
*  @brief  Parses a WMFW file for loading to the device
*
*  This function reads in a WMFW file, processes it, and sends it block by
*  block to the device.
*
*  @param  filename    File to parse
*
*/

int ProcessWMFWFile(const char *filename)
{
	int status = WMFW_SUCCESS;

	printf("\n\n----------------------------------------------------------\n");
	printf("WMFW firmware file processing...\n");
	// Open our WMFW file
	FILE *wmfwFile = fopen(filename, "rb");
	if (!wmfwFile)
	{
		printf("\tError: Failed to open file.\n");
		return WMFW_FILE_OPEN_FAILED;
	}
	else
	{
		printf("\tdetected\n\tProcessing firmware data");
	}
	printf("\n----------------------------------------------------------\n\n");

	// Read and process the header.
	status = ProcessWMFWHeader(wmfwFile);
	if (WMFW_SUCCESS != status)
	{
		printf("Error: ProcessWMFWHeader returned status %d.\n", status);
		return status;
	}

	// Now process all the blocks in the file.
	while (WMFW_SUCCESS == status)
	{
		status = ProcessNextWMFWBlock(wmfwFile);
	}

	if (wmfwFile)
	{
		fclose(wmfwFile);
		wmfwFile = NULL;
	}

	if (WMFW_END_OF_FILE == status)
	{
		status = WMFW_SUCCESS;
	}

	return status;
}



/*
*  Function: ProcessWMFWHeader
*
*  @brief  Processes the header of the WMFW file and skips past it.
*
*  @param  wmfwFile    Open handle to file to process
*
*  @retval WMFW_SUCCESS            Succeeded
*  @retval WMFW_BAD_FILE_FORMAT    Couldn't parse header
*
*/
int ProcessWMFWHeader(FILE *wmfwFile)
{
	WMFW_STATUS     status = WMFW_SUCCESS;
	char            magic[4];       // File identifier, should be 'WMFW'
	char            versionInfo[4]; // Hold the version info of the file
	unsigned int    length;         // Header length
	short			apiRevision;
	char			targetCore;
	unsigned int	memorySizes[4];
	size_t          amountRead = 0;
	time_t			creationTimestamp;
    time_t          dustBin;
	unsigned int	checksum;

	// Read our file identifier. This is the first thing in the file and should be WMFW. 
	amountRead = fread(magic, sizeof(char), 4, wmfwFile);
	if (amountRead < 4)
	{
		printf("Error: WMFW not found at the start of the file.\n");
		return WMFW_BAD_FILE_FORMAT;
	}

	// Check it.
	for (int c = 0; c < wmfw_id_length; c++)
	{
		if (wmfw_id_string[c] != magic[c])
		{
			printf("Error: WMFW not found at the start of the file: %c%c%c%c\n",
				magic[0], magic[1], magic[2], magic[3]
				);
			status = WMFW_BAD_FILE_FORMAT;
			return WMFW_BAD_FILE_FORMAT;
		}
	}

	// Read the length of the header.
	amountRead = fread(&length, sizeof(length), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read full header length: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}
	if (length != 40)
	{
		printf("Header length (%d) does not equal expected 40 bytes\n", length);
		return WMFW_BAD_FILE_FORMAT;
	}

	// Read the Version info.
	amountRead = fread(versionInfo, sizeof(versionInfo), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read full version info: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}

	apiRevision = (int(versionInfo[1]) << 8) + (int(versionInfo[0]));
	targetCore = (int)versionInfo[2];
	wmfwFileVersion = int(versionInfo[3]);
	printf("WMFW header information:\n");
	printf("------------------------\n");
	printf("WMFW API revision: 0x%x\n", apiRevision);
	printf("Firmware file format version: 0x%x\n", wmfwFileVersion);
	printf("Header Length: %d (0x%x) bytes\n", length, length);

	if (targetCore == CORETYPE_ADSP2)
	{
		printf("Target core: ADSP2 (0x%x)\n", targetCore);
	}
	else if (targetCore == CORETYPE_HALOCORE)
	{
		printf("Target core: Halo Core (0x%x)\n", targetCore);
	}
	else
	{
		printf("Target core: Other unsupported core type (0x%x)\n", targetCore);
		return WMFW_BAD_FILE_FORMAT;
	}

	// Memory sizes
	amountRead = fread(memorySizes, sizeof(memorySizes), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read full memory sizes: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}

	printf("Memory sizes:\n");
	printf("\tXM: %d (0x%x)\n", memorySizes[0], memorySizes[0]);
	printf("\tYM: %d (0x%x)\n", memorySizes[1], memorySizes[1]);
	printf("\tPM: %d (0x%x)\n", memorySizes[2], memorySizes[2]);
	printf("\tZM: %d (0x%x)\n", memorySizes[3], memorySizes[3]);


	// Creation timestamp
	amountRead = fread(&creationTimestamp, sizeof(creationTimestamp), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read creation timestamp: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}

	printf("Creation timestamp: %ld\n", creationTimestamp);

	// Throw away dust bin
	amountRead = fread(&dustBin, sizeof(dustBin), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read creation dustBin: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}

	printf("Dust Bin: %ld\n", dustBin);

	// Checksum
	amountRead = fread(&checksum, sizeof(checksum), 1, wmfwFile);
	if (amountRead < 1)
	{
		printf("Couldn't read checksum: %d\n", amountRead);
		return WMFW_BAD_FILE_FORMAT;
	}

	printf("Checksum: %d (0x%x)\n", checksum, checksum);
	printf("---------------------------\n");
	printf("End WMFW header information\n\n");

	return status;
}


/*
*  Function: ProcessNextWMFWBlock
*
*  @brief  Processes a block of the WMFW file and writes it to the device if appropriate.
*
*  @param  wmfwFile    Open handle to file to process
*
*  @retval WMFW_SUCCESS            Succeeded
*  @retval WMFW_BAD_FILE_FORMAT    Couldn't parse block
*  @retval WMFW_COMMS_ERROR        Write failed
*  @retval WMFW_END_OF_FILE        Reached the end of file
*
*/
int ProcessNextWMFWBlock(FILE *wmfwFile)
{
	WMFW_STATUS     status = WMFW_SUCCESS;
	WMFWBlockHeader blockHeader = { 0, 0, 0 };
	unsigned int    regionStart;
	unsigned char   *buffer = NULL;
	unsigned char   registersPerAddress;

	/*
	*  Format for a block:
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

	//  Read the block header. This tells us what type of block it is and how
	//  big it is.
	size_t amountRead = fread(&blockHeader, sizeof(blockHeader), 1, wmfwFile);
	if (amountRead < 1)
	{
		status = WMFW_BAD_FILE_FORMAT;
		goto done;
	}

	// Check it's a block we understand.
	printf("\n");
	switch (blockHeader.region)
	{

#if defined(ADSP2)
	case WMFW_PM:
		printf("PM data block:\n");
		regionStart = pmBase;
		registersPerAddress = 3;
		break;
	case WMFW_XM:
		printf("XM data block:\n");
		regionStart = xmBaseUnpacked;
		registersPerAddress = 2;
		break;
	case WMFW_YM:
		printf("YM data block:\n");
		regionStart = ymBaseUnpacked;
		registersPerAddress = 2;
		break;
	case WMFW_ZM:
		printf("ZM data block:\n");
		regionStart = zmBase;
		registersPerAddress = 2;
		break;
#elif defined(HALOCORE)
	case WMFW_PM_P:
		printf("PM data block:\n");
		regionStart = pmBase;
		registersPerAddress = 5;
		break;
	case WMFW_XM:
		printf("XM_UP24 data block:\n");
		regionStart = xmBaseUnpacked;
		registersPerAddress = 4;
		break;
	case WMFW_YM:
		printf("YM_UP24 data block:\n");
		regionStart = ymBaseUnpacked;
		registersPerAddress = 4;
		break;
	case WMFW_XM_P:
		printf("XM Packed data block:\n");
		regionStart = xmBasePacked;
		registersPerAddress = 3;
		break;
	case WMFW_YM_P:
		printf("YM Packed data block:\n");
		regionStart = ymBasePacked;
		registersPerAddress = 3;
		break;
#endif

	case WMFW_INFO_STRING:  // Informational string
		printf("Info String Block:\n");
		
		// Print out the info string:
		buffer = (unsigned char *)malloc(blockHeader.dataLength+1);
		if (!buffer)
		{
			status = WMFW_OUT_OF_MEMORY;
			goto done;
		}

		// Read in our data from the file.
		amountRead = fread(buffer, 1, blockHeader.dataLength, wmfwFile);
		if (amountRead < blockHeader.dataLength)
		{
			if (feof(wmfwFile))
				printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
			else
				printf("\tError: Couldn't read from file\n");
			status = WMFW_BAD_FILE_FORMAT;
			goto done;
		}

		buffer[blockHeader.dataLength] = '\0';
		printf("\t%s\n", buffer);
		status = WMFW_SUCCESS;
		goto done;

    case WMFW_ALGO_INFO_BLOCK:  // Algorithm information data block
        printf("Found Algo Info Block: 0x%02X\n", blockHeader.region);
        

		if (wmfwFileVersion >= 2)
		{
			buffer = (unsigned char *)malloc(blockHeader.dataLength);
			if (!buffer)
			{
				status = WMFW_OUT_OF_MEMORY;
				goto done;
			}

			// Read in our data from the file.
			amountRead = fread(buffer, 1, blockHeader.dataLength, wmfwFile);
			if (amountRead < blockHeader.dataLength)
			{
				if (feof(wmfwFile))
					printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
				else
					printf("\tError: Couldn't read from file\n");
				status = WMFW_BAD_FILE_FORMAT;
				goto done;
			}

		}
		else
		{
			// No example parsing code provided for earlier WMFW file formats, so skip over block
			printf("\tParsing not supported for file format version %d, skipping block\n", wmfwFileVersion);
			fseek(wmfwFile, (long)blockHeader.dataLength, SEEK_CUR);
		}

        goto done;

	default:                // Unknown block
		//
		// Skip over the rest of the block.
		//
		printf("Unknown block type: 0x%02X\n", blockHeader.region);
		printf("\tSkipping over %d bytes\n",
			blockHeader.dataLength );
		if (0 != fseek(wmfwFile, (long)blockHeader.dataLength, SEEK_CUR))
		{
			status = WMFW_BAD_FILE_FORMAT;
		}

		// And move on.
		goto done;
	}

	// It's a data block for writing to the device.  Allocate a buffer to hold
	// the data.

    while(blockHeader.dataLength > 4080) // default .max_transfer_sz is (4094) byte. - addr(4) + padding(4) = (4086) and divided registersPerAddress remainder '0'.
    {
        printf("\tblockHeader.dataLength: %d bytes\n", blockHeader.dataLength);
        blockHeader.dataLength -= 4080;

    	buffer = (unsigned char *)malloc(4080);
    	if (!buffer)
    	{
    		status = WMFW_OUT_OF_MEMORY;
            printf("\tError: malloc out of memory\n");
    		goto done;
    	}

    	// Read in our data from the file.
    	amountRead = fread(buffer, 1, 4080, wmfwFile);
    	if (amountRead < 4080)
    	{
    		if (feof(wmfwFile))
    			printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
    		else
    			printf("\tError: Couldn't read from file\n");
    		status = WMFW_BAD_FILE_FORMAT;
    		goto done;
    	}

        {   
        	// Work out where to write it to.
        	unsigned int offsetInRegisters = blockHeader.offset * registersPerAddress;
        	unsigned int startAddress = regionStart + offsetInRegisters;


        	printf("\tR%08Xh : 4080 bytes\n", startAddress);

        	// And write the data.
        	SpiWriteBlock(startAddress, buffer, 4080);

			blockHeader.offset += (4080 / registersPerAddress);
        }

    	if (buffer)
    	{
    		free(buffer);
    		buffer = NULL;
    	}

    }

	buffer = (unsigned char *)malloc(blockHeader.dataLength);
	if (!buffer)
	{
		status = WMFW_OUT_OF_MEMORY;
        printf("\tError: malloc out of memory\n");
		goto done;
	}

	// Read in our data from the file.
	amountRead = fread(buffer, 1, blockHeader.dataLength, wmfwFile);
	if (amountRead < blockHeader.dataLength)
	{
		if (feof(wmfwFile))
			printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
		else
			printf("\tError: Couldn't read from file\n");
		status = WMFW_BAD_FILE_FORMAT;
		goto done;
	}

    {   
    	// Work out where to write it to.
    	unsigned int offsetInRegisters = blockHeader.offset * registersPerAddress;
    	unsigned int startAddress = regionStart + offsetInRegisters;


    	printf("\tR%08Xh : %d bytes\n", startAddress, blockHeader.dataLength);

    	// And write the data.
    	SpiWriteBlock(startAddress, buffer, blockHeader.dataLength);


    	// If writing to XM[0], parse this data block to obtain algorithm information
    	// Note that this check means Packed memory start addresses have to be defined, even on ADSP2 devices
    	// that don't have packed memory: on these devices it should be set to the same address as unpacked to avoid 
    	// accidental entry of this if statement.
    	if (startAddress == xmBaseUnpacked || startAddress == xmBasePacked)
    	{
    		ParseFirmwareInfo(buffer, blockHeader.dataLength);
    	}
    }

	// We've finished this block.  Go round for the next one.
done:
	if (buffer)
	{
		free(buffer);
		buffer = NULL;
	}

	// If we hit EOF, assume that's why we got an error.
	if (feof(wmfwFile))
	{
		printf("\nEnd of file\n");
		status = WMFW_END_OF_FILE;
	}

	return status;
}



/*
 *  Function:  ProcessBinFile
 * 
 *  @brief  Parses a WMFW file for loading to the device
 * 
 *  This function reads in a WMFW file, processes it, and sends it block by
 *  block to the device.
 * 
 *  @param  filename    File to parse
 *  
 */
int ProcessBinFile(const char *filename)
{
	int status = WMFW_SUCCESS;

	printf("\n\n----------------------------------------------------------\n");
	printf("Cofficient file processing...\n");

	// Open our WMFW file
	FILE *binFile = fopen(filename, "rb");
	if (!binFile)
	{
		printf("\t%s could not be opened\n\tNo coefficent data will be downloaded\n\n", filename);

		// Note: This is not necessarily a fail condition, as a .bin file may not be required
		// and therefore not present
		return WMFW_SUCCESS;
	}
	else
	{
		printf("\t%s detected\n\tProcessing coefficent data", filename);
	}
	printf("\n----------------------------------------------------------\n\n");

	// Read and process the header.
	status = ProcessBinHeader(binFile);
	if (WMFW_SUCCESS != status)
	{
		printf("Error: ProcessBinHeader returned status %d.\n", status);
		return status;
	}

	// Now process all the blocks in the file.
	while (WMFW_SUCCESS == status)
	{
		status = ProcessNextCoeffBlock(binFile);
	}

	if (binFile)
	{
		fclose(binFile);
		binFile = NULL;
	}

	if (WMFW_END_OF_FILE == status)
	{
		status = WMFW_SUCCESS;
	}

	return status;
}




/*
 *  Function: ProcessBinHeader
 * 
 *  @brief  Processes the header of the WMFW file and skips past it.
 * 
 *  @param  binFile    Open handle to file to process
 * 
 *  @retval WMFW_SUCCESS            Succeeded
 *  @retval WMFW_BAD_FILE_FORMAT    Couldn't parse header
 * 
 */
int ProcessBinHeader(FILE *binFile)
{
	WMFW_STATUS     status = WMFW_SUCCESS;
	char            magic[4];       // File identifier, should be 'WMDR'
	unsigned int    length;         // Header length
	size_t          amountRead = 0;
	unsigned char * headerData = NULL;

	printf("Parsing .bin file header:\n");
	// Read our file identifier. This is the first thing in the file and should be WMDR. 
	amountRead = fread(magic, sizeof(char), 4, binFile);
	if (amountRead < 4)
	{
		printf("Error: WMDR identifier not found at the start of the file.\n");
		status = WMFW_BAD_FILE_FORMAT;
		goto binHeaderDone;
	}

	// Check it.
	for (int c = 0; c < bin_id_length; c++)
	{
		if (bin_id_string[c] != magic[c])
		{
			printf("Error: WMDR identifier not found at the start of the file: %c%c%c%c\n",
				magic[0], magic[1], magic[2], magic[3]
			);
			status = WMFW_BAD_FILE_FORMAT;
			goto binHeaderDone;
		}
	}

	printf("\tWMDR identifier found\n");

	// Read the length of the header.
	amountRead = fread(&length, sizeof(length), 1, binFile);
	if (amountRead < 1)
	{
		printf("Couldn't read full header length: %d\n", amountRead);
		status = WMFW_BAD_FILE_FORMAT;
		goto binHeaderDone;
	}

	printf("\tHeader Length: %d bytes\n", length);

    {   
    	// Length of remaining header data is the length field minus 8 bytes (for WMDR and length)
    	unsigned long headerDataLength = (long)length - 8;

    	// Read rest of the header data. This section is optional.
    	// It is useful as an additional error check, but can be safely skipped over without affecting
    	// the coefficient download process if you are certain the file format version is compatible
    	headerData = (unsigned char*)malloc(headerDataLength);
    	amountRead = fread(headerData, 1, headerDataLength, binFile);
    	if (amountRead < headerDataLength)
    	{
    		printf("Couldn't read full header data: %d\n", amountRead);
    		status = WMFW_BAD_FILE_FORMAT;
    		goto binHeaderDone;
    	}

    	// If the data length matches the recognised file format for version 0x01, try mapping
    	// that format onto the data block
    	if (headerDataLength == sizeof(BinFileHeaderFormat01))
    	{
    		BinFileHeaderFormat01 * fileData;
    		fileData = (BinFileHeaderFormat01*)headerData;
    		
    		// Check that the file format field is compatible with this parser
    		printf("\tFile Format Version: %02X\n", fileData->fileFormatVersion);
    		if (fileData->fileFormatVersion != 0x01)
    		{
    			printf("\tError: File Format %02X not compatible. Exiting.\n", fileData->fileFormatVersion);
    			status = WMFW_BAD_FILE_FORMAT;
    			goto binHeaderDone;
    		}

    		// Check that the firmware version matches the one downloaded by the WMFW
    		printf("\tFirmware Revision: %08X\n", fileData->firmwareRevision);
    		if (fileData->firmwareRevision != fwIdBlock.firmwareRevision)
    		{
    			printf("\nWarning: Firmware Revision Mismatch\n\tBin  File Firmware Revision: %08X\n\tWMFW File Firmware Revision: %08X\n", fileData->firmwareRevision, fwIdBlock.firmwareRevision);
    		}

    		// Check which core type the bin file works with
    		switch (fileData->coreType)
    		{
    		case CORETYPE_ADSP2:
    			printf("\tCore Type: ADSP2\n");
    			printf("\tCore Revision: %08X\n", fileData->coreRevision);
    			// Check that the core revision field reads 0.5.1 (this is the only valid value for ADSP2)
    			if (fileData->coreRevision != 0x000501)
    			{
    				printf("\tError: Core Revision field does not match expected value for ADSP2. Exiting.\n");
    				status = WMFW_BAD_PARAM;
    				goto binHeaderDone;
    			}
    			break;
    		case CORETYPE_HALOCORE:
    			printf("\tCore Type: Halo Core\n");
    			printf("\tCore Revision: 0x%08X\n", fileData->coreRevision);
    			break;
    		default:
    			printf("Error: Core type 0x%02X not compatible. Exiting.\n", fileData->coreType);
    			status = WMFW_BAD_PARAM;
    			goto binHeaderDone;
    		}
    		
    	}
    }
    
	binHeaderDone:
		// Free the header data memory
		free(headerData);
		return status;
}




/*
 *  Function: ProcessNextCoeffBlock
 * 
 *  @brief  Processes a coefficient block of the .bin file and writes it to the device if appropriate.
 * 
 *  @param  binFile    Open handle to file to process
 * 
 *  @retval WMFW_SUCCESS            Succeeded
 *  @retval WMFW_BAD_FILE_FORMAT    Couldn't parse block
 *  @retval WMFW_COMMS_ERROR        Write failed
 *  @retval WMFW_END_OF_FILE        Reached the end of file
 * 
 */
int ProcessNextCoeffBlock(FILE *binFile)
{
	WMFW_STATUS     status = WMFW_SUCCESS;
	BinCoeffBlockHeader blockHeader = { 0, 0, 0, 0, 0, 0 };
	AlgorithmIDBlockFormat *algorithmInfo = NULL;
	unsigned int    regionStart = 0/*NULL*/;
	unsigned int	dataLength;
	unsigned char   *buffer = NULL;
	unsigned char   registersPerAddress;
	//bool algorithmMatchFound = false;

	/*
	 *  Format for a coefficient block:
	 * 
	 *      31       24 23      16 15       8 7        0
	 *  0   +----------+----------+----------+---------+
	 *      |      type[15:0]     |    offset[15:0]    |
	 *  4   +----------+-------------------------------+
	 *		|               Algorithm ID               |
	 *		+------------------------------------------+
	 *		|            Algorithm Version             |
	 *		+------------------------------------------+
	 *		|               Sample Rate                |
	 *		+------------------------------------------+
	 *      |                dataLength                |
	 *  8   +------------------------------------------+
	 *      |                   data                   |
	 *      :                   ....                   :
	 *      :                                          :
	 *  The offset/region and data length are little endian. 
	 *  The data is formatted big-endian to facilitate writing
	 *  straight to the core.
	 */ 

	 //  Read the block header. This tells us what type of block it is and how
	 //  big it is.
	size_t amountRead = fread(&blockHeader, sizeof(blockHeader), 1, binFile);
	if (amountRead < 1)
	{
		status = WMFW_BAD_FILE_FORMAT;
		goto done;
	}

	// Round the datalength up to the nearest multiple of four, as padding bytes may be included in data section
	// but are not included in the datalength count. 
	dataLength = ((blockHeader.dataLength + 3) / 4) * 4;


	// Check it's a block we understand.
	printf("\n");
	switch (blockHeader.region)
	{
#if defined(ADSP2)
	case WMFW_XM:
		printf("XM data block:\n");
		registersPerAddress = 2;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = xmBaseUnpacked + (algorithmInfo->algorithmXmBase * registersPerAddress);
		}
		break;

	case WMFW_YM:
		printf("YM data block:\n");
		registersPerAddress = 2;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = ymBaseUnpacked + (algorithmInfo->algorithmYmBase * registersPerAddress);
		}
		break;	

	case WMFW_ZM:
		printf("ZM data block:\n");
		registersPerAddress = 2;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = zmBase + (algorithmInfo->algorithmZmBase * registersPerAddress);
		}
		break;

#elif defined(HALOCORE)
	case WMFW_XM:
		printf("XM_UP24 data block:\n");
		registersPerAddress =4;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = xmBaseUnpacked + (algorithmInfo->algorithmXmBase * registersPerAddress);
		}
		break;
	case WMFW_YM:
		printf("YM_UP24 data block:\n");
		registersPerAddress = 4;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = ymBaseUnpacked + (algorithmInfo->algorithmYmBase * registersPerAddress);
		}
		break;
	case WMFW_XM_P:
		printf("XM Packed data block:\n");
		registersPerAddress = 3;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = (xmBasePacked + (algorithmInfo->algorithmXmBase * registersPerAddress)) & ~0x3; // ~0x3 is performed to ensure a valid register address
		}
		break;
	case WMFW_YM_P:
		printf("YM Packed data block:\n");
		registersPerAddress = 3;
		algorithmInfo = FindMatchingAlgoBlock(blockHeader);
		if (NULL != algorithmInfo)
		{
			regionStart = (ymBasePacked + (algorithmInfo->algorithmYmBase * registersPerAddress)) & ~0x3;
		}
		break;
#endif

	case (WMFW_USER_DEF_NAME << 8): // Name block string
		printf("User Defined Name Block:\n");
		// Read in the data section and print it out on console.
		// This can be skipped over, as this string data is only for user information
		// If skipping this, use fseek to skip over the data section and move on to the next block
		buffer = (unsigned char *)malloc(dataLength + 1);
		if (!buffer)
		{
			status = WMFW_OUT_OF_MEMORY;
			goto done;
		}
		amountRead = fread(buffer, 1, dataLength, binFile);
		if (amountRead < dataLength)
		{
			if (feof(binFile))
				printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
			else
				printf("\tError: Couldn't read from file\n");
			status = WMFW_BAD_FILE_FORMAT;
			goto done;
		}
		buffer[dataLength] = '\0';
		printf("\t%s\n", buffer);
		
		status = WMFW_SUCCESS;
		goto done;

	default:                // Unknown block
		//
		// Skip over the rest of the block.
		//
		printf("Unknown block type: 0x%04X\n", blockHeader.region);
		printf("\tSkipping over %d bytes\n", dataLength);
		if (0 != fseek(binFile, (long)dataLength, SEEK_CUR))
		{
			status = WMFW_BAD_FILE_FORMAT;
		}

		// And move on.
		goto done;
	}

	// If we got here, then it's an XM/YM/ZM data block for writing to the device.  
	// Allocate a buffer to hold the data.
	buffer = (unsigned char *)malloc(dataLength);
	if (!buffer)
	{
		status = WMFW_OUT_OF_MEMORY;
		goto done;
	}

	// Read in our data from the file.
	amountRead = fread(buffer, 1, dataLength, binFile);
	if (amountRead < dataLength)
	{
		if (feof(binFile))
			printf("\tError: Unexpected end of file after %d bytes of block\n", amountRead);
		else
			printf("\tError: Couldn't read from file\n");
		status = WMFW_BAD_FILE_FORMAT;
		goto done;
	}

	// If we identified an algorithm to write it to, send data to the chip. 
	// Otherwise, just ignore the data buffer we've read and move onto the next block
	if (regionStart != 0/*NULL*/)
	{
		// Work out where to write it to.
		// The blockheader contains a register offset rather than memory offset, so no need to convert to registers
		unsigned int startAddress = regionStart + blockHeader.offset;

		printf("\tR%08Xh : %d bytes\n", startAddress, dataLength);

		// And write the data.
		//HalWriteBlock(startAddress, ADDR_SIZE_4_BYTES, buffer, dataLength, 0);
	}
	// We've finished this block.  Go round for the next one.
	

	done:
		if (buffer)
		{
			free(buffer);
			buffer = NULL;
		}

		// If we hit EOF, assume that's why we got an error.
		if (feof(binFile))
		{
			printf("\nEnd of file\n");
			status = WMFW_END_OF_FILE;
		}

	return status;
}


#if defined(CTC_CS48L32_OKGOOGLE)
/*
*  Function:  ProcessSEARCHFile
*
*  @brief  Parses a SEARCH file for loading to the device
*
*  This function reads in a SEARCH file, processes it, and sends it block by
*  block to the device.
*
*  @param  filename    File to parse
*
*/

int ProcessSEARCHFile(const char *filename)
{
	int status = WMFW_SUCCESS;

	printf("\n\n----------------------------------------------------------\n");
	printf("SEARCH firmware file processing...\n");
	// Open our SEARCH file
	FILE *searchFile = fopen(filename, "rb");
	if (!searchFile)
	{
		printf("\tError: Failed to open file.\n");
		return WMFW_FILE_OPEN_FAILED;
	}
	else
	{
		printf("\tdetected\n\tProcessing firmware data");
	}
	printf("\n----------------------------------------------------------\n");

	// Now process all the blocks in the file.
	while (WMFW_SUCCESS == status)
	{
		status = ProcessSEARCHBlock(searchFile);
	}

	if (searchFile)
	{
		fclose(searchFile);
		searchFile = NULL;
	}

	if (WMFW_END_OF_FILE == status)
	{
		status = WMFW_SUCCESS;
	}

	return status;
}


int ProcessSEARCHBlock(FILE *searchFile)
{
	WMFW_STATUS		status = WMFW_SUCCESS;
	unsigned char	*buffer = NULL;
	unsigned char	buffer_temp[4] = {0, 0, 0, 0};
	int				i = 0;

	// It's a data block for writing to the device.  Allocate a buffer to hold
	// the data.

	buffer = (unsigned char *)malloc(4000);
	if (!buffer)
	{
		status = WMFW_OUT_OF_MEMORY;
        printf("\tError: malloc out of memory\n");
		goto done;
	}

	memset(buffer, 0, 4000);

	// Read in our data from the file.
	while(status == WMFW_SUCCESS)
	{
		size_t amountRead = fread(buffer_temp, 1, 3, searchFile);
		if(amountRead == 3) SwapEndianness(&buffer[(i++ * 4)], (uint8_t*)&buffer_temp, 4);

		if (amountRead < 3)
		{
			if (feof(searchFile))
			{
				printf("\nEnd of file\n");
				status = WMFW_END_OF_FILE;
			}
			else
			{
				printf("\tError: Couldn't read from file\n");
				status = WMFW_BAD_FILE_FORMAT;
				goto done;
			}
		}
	}

    {   
    	// Work out where to write it to.
    	unsigned int startAddress = 0x284A7C0;

    	printf("\tR%08Xh : 4000 bytes\n\n", startAddress);

    	// And write the data.
    	SpiWriteBlock(startAddress, buffer, 4000);
    }

	// fill zero in remaining area
	memset(buffer, 0, 4000);

    {   
    	// Work out where to write it to.
    	unsigned int startAddress = 0x284A7C0 + 4000; // 0x284B760

    	printf("\tR%08Xh : 2144 bytes\n\n", startAddress);

    	// And write the data.
    	SpiWriteBlock(startAddress, buffer, 2144);
    }

	// We've finished this block.  Go round for the next one.
done:
	if (buffer)
	{
		free(buffer);
		buffer = NULL;
	}
	
	return status;
}


/*
*  Function:  ProcessMODELFile
*
*  @brief  Parses a MODEL file for loading to the device
*
*  This function reads in a MODEL file, processes it, and sends it block by
*  block to the device.
*
*  @param  filename    File to parse
*
*/

int ProcessMODELFile(const char *filename)
{
	int status = WMFW_SUCCESS;

	printf("\n\n----------------------------------------------------------\n");
	printf("MODEL firmware file processing...\n");
	// Open our MODEL file
	FILE *modelFile = fopen(filename, "rb");
	if (!modelFile)
	{
		printf("\tError: Failed to open file.\n");
		return WMFW_FILE_OPEN_FAILED;
	}
	else
	{
		printf("\tdetected\n\tProcessing firmware data");
	}
	printf("\n----------------------------------------------------------\n");

	// Now process all the blocks in the file.
	while (WMFW_SUCCESS == status)
	{
		status = ProcessMODELBlock(modelFile);
	}

	if (modelFile)
	{
		fclose(modelFile);
		modelFile = NULL;
	}

	if (WMFW_END_OF_FILE == status)
	{
		status = WMFW_SUCCESS;
	}

	return status;
}


int ProcessMODELBlock(FILE *modelFile)
{
	WMFW_STATUS		status = WMFW_SUCCESS;
	unsigned char	*buffer = NULL;
	unsigned char	buffer_temp[4] = {0, 0, 0, 0};
	unsigned int	modelAreaLength = 360448; // 0x58000
	unsigned int	startAddress = 0x284BFC0;

	// It's a data block for writing to the device.  Allocate a buffer to hold
	// the data.

    while(modelAreaLength > 4000)
    {
        printf("\tmodelAreaLength: %d bytes\n", modelAreaLength);
        modelAreaLength -= 4000;

    	buffer = (unsigned char *)malloc(4000);
    	if (!buffer)
    	{
    		status = WMFW_OUT_OF_MEMORY;
            printf("\tError: malloc out of memory\n");
    		goto done;
    	}

		memset(buffer, 0, 4000);

		if(status != WMFW_END_OF_FILE)
		{
			for(int i = 0; i < 1000; i++)
			{
		    	// Read in our data from the file.
				size_t amountRead = fread(buffer_temp, 1, 3, modelFile);
				if(amountRead == 3) SwapEndianness(&buffer[(i * 4)], (uint8_t*)&buffer_temp, 4);

		    	if (amountRead < 3)
		    	{
		    		if (feof(modelFile))
	    			{
						printf("\t\tEnd of file\n");
						status = WMFW_END_OF_FILE;
						break;
	    			}
		    		else
	    			{
		    			printf("\tError: Couldn't read from file\n");
			    		status = WMFW_BAD_FILE_FORMAT;
			    		goto done;
	    			}
		    	}
			}
		}

        {   
        	// Work out where to write it to.
        	printf("\tR%08Xh : 4000 bytes\n", startAddress);

        	// And write the data.
        	SpiWriteBlock(startAddress, buffer, 4000);

			startAddress += 4000;
        }

    	if (buffer)
    	{
    		free(buffer);
    		buffer = NULL;
    	}

    }

	buffer = (unsigned char *)malloc(4000);
	if (!buffer)
	{
		status = WMFW_OUT_OF_MEMORY;
		printf("\tError: malloc out of memory\n");
		goto done;
	}

	// fill zero in remaining area
	memset(buffer, 0, 4000);

    {   
    	printf("\tR%08Xh : %d bytes\n\n", startAddress, modelAreaLength);

    	// And write the data.
    	SpiWriteBlock(startAddress, buffer, modelAreaLength);
    }

	// We've finished this block.  Go round for the next one.
done:
	if (buffer)
	{
		free(buffer);
		buffer = NULL;
	}
	
	return status;
}

#endif
///////////////////////////////// END OF FILE //////////////////////////////////
