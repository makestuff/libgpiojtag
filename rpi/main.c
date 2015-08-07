/* 
 * Copyright (C) 2015 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <makestuff.h>
#include <libgpiojtag.h>
#include <wiringPi.h>
#include <liberror.h>

// Pin assignments on the RPi header (see http://wiringpi.com/pins):
//
typedef enum {
	TCK = 7,
	TDO = 0,
	TDI = 9,
	TMS = 8
} Pins;

// Callbacks implementing actual GPIO for JTAG lines
//
void setTCK(bool x) {
	digitalWrite(TCK, x ? HIGH : LOW);
}
void setTDI(bool x) {
	digitalWrite(TDI, x ? HIGH : LOW);
}
void setTMS(bool x) {
	digitalWrite(TMS, x ? HIGH : LOW);
}
bool getTDO(void) {
	return digitalRead(TDO) == HIGH;
}

int main(int argc, const char *argv[]) {
	int retVal;

	// Init library
	wiringPiSetup();

	// Validate command-line args
	if ( argc != 2 ) {
		fprintf(stderr, "Synopsis: %s <svf-file>\n", argv[0]);
		FAIL(GJ_USAGE, exit);
	}

	// Setup pins
	pinMode(TCK, OUTPUT);
	pinMode(TMS, OUTPUT);
	pinMode(TDI, OUTPUT);
	pinMode(TDO, INPUT);

	// Parse .svf file
	const char *error = NULL;
	const struct ParserCallbacks cb = {setTCK, setTMS, setTDI, getTDO};
	ParserStatus pStatus = parse(argv[1], &cb, &error);
	CHECK_STATUS(pStatus, pStatus, cleanup);
	retVal = 0;
cleanup:
    if ( error ) {
        printf("%s\n", error);
        errFree(error);
    }
exit:
	pinMode(TCK, INPUT);
	pinMode(TMS, INPUT);
	pinMode(TDI, INPUT);
	pinMode(TDO, INPUT);
	return retVal;
}
