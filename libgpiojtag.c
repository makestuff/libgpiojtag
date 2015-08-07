/*
 * Copyright (C) 2015 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <stdlib.h>
#include <makestuff.h>
#include <liberror.h>
#include <libbuffer.h>
#include <libfpgalink.h>
#include <xsvf.h>
#include "libgpiojtag.h"

// -------------------------------------------------------------------------------------------------
// Declaration of private types & functions
// -------------------------------------------------------------------------------------------------

// Undocumented FPGALink function
FLStatus flLoadSvfAndConvertToCsvf(
	const char *svfFile, struct Buffer *csvfBuf, uint32 *maxBufSize,
	const char **error
);

// Get number of bytes needed to store x bits
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

// Write a hex representation of a uint8 buffer into the supplied char buffer
static void dumpSimple(const uint8 *input, uint32 length, char *buf);

// Compare the received TDO data with the expected data
static bool tdoCompare(
	const uint8 *tdoData, const uint8 *tdoMask, const uint8 *tdoExpected,
	uint32 numBytes
);

// Only shift into TDI, don't bother with sampling TDO
static inline void jShiftInOnly(
	uint32 numBits, const uint8 *inData,
	const struct ParserCallbacks *cb
);

// Shift into TDI, sample TDO
static inline void jShiftInOut(
	uint32 numBits, const uint8 *inData, uint8 *outData,
	const struct ParserCallbacks *cb
);

// Clock the supplied bit-pattern LSB-first into TMS
static inline void jClockFSM(
	uint32 bitPattern, uint8 transitionCount,
	const struct ParserCallbacks *cb
);

// Execute the supplied number of TCK cycles
static inline void jClocks(
	uint32 numClocks,
	const struct ParserCallbacks *cb
);

// Dump a human-readable representation of the .svf file to stdout
#ifdef DEBUG
	static void csvfDump(const uint8 *buffer);
#endif

// -------------------------------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------------------------------

// Play the specified SVF file into the JTAG port.
//
ParserStatus parse(
	const char *progFile, const struct ParserCallbacks *cb, const char **error)
{
	ParserStatus retVal = GJ_SUCCESS;
	uint8 thisByte, numBits;
	uint32 numBytes;
	uint8 *tdoPtr, *tdiPtr;
	uint8 i;
	uint32 xsdrSize = 0;
	uint32 xruntest = 0;
	uint8 tdoMask[BUF_SIZE];
	uint8 tdiData[BUF_SIZE];
	uint8 tdoData[BUF_SIZE];
	uint8 tdoExpected[BUF_SIZE];
	
	char data[BUF_SIZE*2+1];
	char mask[BUF_SIZE*2+1];
	char expected[BUF_SIZE*2+1];
	
	uint8 *tdiAll;
	const uint8 *ptr;

	FLStatus fStatus;
	uint32 maxBufSize;
	struct Buffer csvfBuf = {0,};
	BufferStatus bStatus = bufInitialise(&csvfBuf, 16*1024*1024, 0x00, error);
	CHECK_STATUS(bStatus, GJ_ALLOC, cleanup, "parser()");
	fStatus = flLoadSvfAndConvertToCsvf(progFile, &csvfBuf, &maxBufSize, error);
	CHECK_STATUS(fStatus, GJ_FILE, cleanup, "parser()");
	ptr = csvfBuf.data;
	#ifdef DEBUG
		csvfDump(csvfBuf.data);
	#endif

		jClockFSM(0x0000001F, 6, cb);  // Reset TAP, goto Run-Test/Idle

	thisByte = *ptr++;
	while ( thisByte != XCOMPLETE ) {
		switch ( thisByte ) {
		case XTDOMASK:
			numBytes = bitsToBytes(xsdrSize);
			tdoPtr = tdoMask;
			while ( numBytes ) {
				thisByte = *ptr++;
				*tdoPtr++ = thisByte;
				numBytes--;
			}
			break;

		case XRUNTEST:
			xruntest = *ptr++;
			xruntest <<= 8;
			xruntest |= *ptr++;
			xruntest <<= 8;
			xruntest |= *ptr++;
			xruntest <<= 8;
			xruntest |= *ptr++;
			break;

		case XSIR:
			jClockFSM(0x00000003, 4, cb);  // -> Shift-IR
			numBits = *ptr++;
			numBytes = bitsToBytes((uint32)numBits);
			tdiPtr = tdiData;
			while ( numBytes ) {
				thisByte = *ptr++;
				*tdiPtr++ = thisByte;
				numBytes--;
			}
			jShiftInOnly(numBits, tdiData, cb);  // -> Exit1-DR
			jClockFSM(0x00000001, 2, cb);  // -> Run-Test/Idle
			if ( xruntest ) {
				jClocks(xruntest, cb);
			}
			break;

		case XSDRSIZE:
			xsdrSize = *ptr++;
			xsdrSize <<= 8;
			xsdrSize |= *ptr++;
			xsdrSize <<= 8;
			xsdrSize |= *ptr++;
			xsdrSize <<= 8;
			xsdrSize |= *ptr++;
			break;

		case XSDRTDO:
			numBytes = bitsToBytes(xsdrSize);
			tdiPtr = tdiData;
			tdoPtr = tdoExpected;
			while ( numBytes ) {
				*tdiPtr++ = *ptr++;
				*tdoPtr++ = *ptr++;
				numBytes--;
			}
			numBytes = bitsToBytes(xsdrSize);
			i = 0;
			do {
				jClockFSM(0x00000001, 3, cb);  // -> Shift-DR
				jShiftInOut(xsdrSize, tdiData, tdoData, cb);  // -> Exit1-DR
				jClockFSM(0x0000001A, 6, cb);  // -> Run-Test/Idle
				if ( xruntest ) {
					jClocks(xruntest, cb);
				}
				i++;
			} while ( tdoCompare(tdoData, tdoMask, tdoExpected, numBytes) && i < 32 );

			if ( i == 32 ) {
				dumpSimple(tdoData, numBytes, data);
				dumpSimple(tdoMask, numBytes, mask);
				dumpSimple(tdoExpected, numBytes, expected);
				CHECK_STATUS(
					true, GJ_MISMATCH, cleanup,
					"parse(): XSDRTDO failed:\n  Got: %s\n  Mask: %s\n  Expecting: %s",
					data, mask, expected);
			}
			break;

		case XSDR:
			jClockFSM(0x00000001, 3, cb);  // -> Shift-DR
			numBytes = bitsToBytes(xsdrSize);
			tdiAll = malloc(numBytes);
			tdiPtr = tdiAll;
			while ( numBytes ) {
				*tdiPtr++ = *ptr++;
				numBytes--;
			}
			jShiftInOnly(xsdrSize, tdiAll, cb);  // -> Exit1-DR
			free(tdiAll);
			jClockFSM(0x00000001, 2, cb);  // -> Run-Test/Idle
			if ( xruntest ) {
				jClocks(xruntest, cb);
			}
			break;

		default:
			CHECK_STATUS(
				true, GJ_BADCMD, cleanup,
				"parse(): Unsupported command 0x%02X", thisByte);
		}
		thisByte = *ptr++;
	}
cleanup:
    bufDestroy(&csvfBuf);
	return retVal;
}

// -------------------------------------------------------------------------------------------------
// Implementation of private functions
// -------------------------------------------------------------------------------------------------

static const char *const nibbles = "0123456789ABCDEF";

static void dumpSimple(const uint8 *input, uint32 length, char *p) {
	uint8 upperNibble, lowerNibble;
	while ( length ) {
		upperNibble = lowerNibble = *input++;
		upperNibble >>= 4;
		lowerNibble &= 15;
		*p++ = nibbles[upperNibble];
		*p++ = nibbles[lowerNibble];
		--length;
	}
	*p = '\0';
}

static bool tdoCompare(
	const uint8 *tdoData, const uint8 *tdoMask, const uint8 *tdoExpected, uint32 numBytes)
{
	while ( numBytes ) {
		if ( (*tdoData & *tdoMask) != (*tdoExpected & *tdoMask) ) {
			return true;
		}
		tdoData++;
		tdoExpected++;
		tdoMask++;
		numBytes--;
	}
	return false;
}

#ifdef DEBUG
static void csvfDump(const uint8 *buffer) {
	const uint8 *p, *savePtr;
	uint8 byte;
	uint32 xsdrSize = 0;
	uint32 numBytes, temp;
	p = buffer;
	byte = *p;
	while ( byte != XCOMPLETE ) {
		switch ( byte ) {
		case XTDOMASK:
			printf("XTDOMASK(");
			numBytes = bitsToBytes(xsdrSize);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSDRTDO:
			printf("XSDRTDO(");
			numBytes = temp = bitsToBytes(xsdrSize);
			savePtr = p + 1;
			while ( numBytes ) {
				printf("%02X", *++p);
				p++;
				numBytes--;
			}
			printf(", ");
			while ( temp ) {
				printf("%02X", *++savePtr);
				savePtr++;
				temp--;
			}
			printf(")\n");
			break;
		case XREPEAT:
			printf("XREPEAT(%02X)\n", *++p);
			break;
		case XRUNTEST:
			printf("XRUNTEST(%02X%02X%02X%02X)\n", p[1], p[2], p[3], p[4]);
			p += 4;
			break;
		case XSDRSIZE:
			xsdrSize = *++p;
			xsdrSize <<= 8;
			xsdrSize |= *++p;
			xsdrSize <<= 8;
			xsdrSize |= *++p;
			xsdrSize <<= 8;
			xsdrSize |= *++p;
			printf("XSDRSIZE(%08X)\n", xsdrSize);
			break;
		case XSIR:
			printf("XSIR(");
			byte = *++p;
			printf("%02X, ", byte);
			numBytes = bitsToBytes((uint32)byte);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSDR:
			printf("XSDR(");
			numBytes = bitsToBytes(xsdrSize);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSDRB:
			printf("XSDRB(");
			numBytes = bitsToBytes(xsdrSize);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSDRC:
			printf("XSDRC(");
			numBytes = bitsToBytes(xsdrSize);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSDRE:
			printf("XSDRE(");
			numBytes = bitsToBytes(xsdrSize);
			while ( numBytes ) {
				printf("%02X", *++p);
				numBytes--;
			}
			printf(")\n");
			break;
		case XSTATE:
			printf("XSTATE(%02X)\n", *++p);
			break;
		case XENDIR:
			printf("XENDIR(%02X)\n", *++p);
			break;
		case XENDDR:
			printf("XENDDR(%02X)\n", *++p);
			break;
		default:
			fprintf(stderr, "Unrecognised command %02X\n", byte);
			exit(1);
		}
		byte = *++p;
	}
	printf("XCOMPLETE\n");
}	
#endif

static inline void jShiftInOnly(
	uint32 numBits, const uint8 *inData,
	const struct ParserCallbacks *cb)
{
	uint8 inByte;
	uint32 mask = 1;

	// Do final bit separately
	numBits--;

	// Macro to handle each bit
	#define inOnly() \
		cb->setTDI(inByte & mask); \
		cb->setTCK(true); \
		cb->setTCK(false); \
		mask <<= 1

	// Do all but the final bit
	inByte = *inData;
	while ( numBits ) {
		const uint32 count = (numBits <= 8) ? numBits : 8;
		inByte = *inData;
		mask = 1;
		switch ( count ) {
			case 8: inOnly();
			case 7: inOnly();
			case 6: inOnly();
			case 5: inOnly();
			case 4: inOnly();
			case 3: inOnly();
			case 2: inOnly();
			case 1: inOnly();
		}
		numBits -= count;
		inData++;
	}

	// Final bit
	cb->setTMS(true);
	inOnly();
}

static inline void jShiftInOut(
	uint32 numBits, const uint8 *inData, uint8 *outData,
	const struct ParserCallbacks *cb)
{
	uint8 inByte, outByte = 0;
	uint32 mask = 1;

	// Do final bit separately
	numBits--;

	// Macro to handle each bit
	#define inOut() \
		cb->setTDI(inByte & mask); \
		if ( cb->getTDO() ) { \
			outByte |= mask; \
		} \
		cb->setTCK(true); \
		cb->setTCK(false); \
		mask <<= 1

	// Do all but the final bit
	inByte = *inData;
	outData--;
	while ( numBits ) {
		const uint32 count = (numBits <= 8) ? numBits : 8;
		inByte = *inData;
		outByte = 0;
		mask = 1;
		switch ( count ) {
			case 8: inOut();
			case 7: inOut();
			case 6: inOut();
			case 5: inOut();
			case 4: inOut();
			case 3: inOut();
			case 2: inOut();
			case 1: inOut();
		}
		numBits -= count;
		inData++;
		*++outData = outByte;
	}

	// Final bit
	cb->setTMS(true);
	inOut();
	*outData = outByte;
}

static inline void jClockFSM(
	uint32 bitPattern, uint8 transitionCount,
	const struct ParserCallbacks *cb)
{
	while ( transitionCount ) {
		cb->setTMS(bitPattern & 1); cb->setTCK(true); cb->setTCK(false);
		bitPattern >>= 1;
		transitionCount--;
	}
}

static inline void jClocks(
	uint32 numClocks, const struct ParserCallbacks *cb)
{
	while ( numClocks ) {
		cb->setTCK(true);
		cb->setTCK(false);
		numClocks--;
	}
}
