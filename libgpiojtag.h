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
#ifndef GPIOJTAG_H
#define GPIOJTAG_H

#include <makestuff.h>

// Callback function typedefs
typedef bool (*GetFunc)(void);
typedef void (*SetFunc)(bool);

// Return code of the parse() function
typedef enum {
	GJ_SUCCESS = 0,
	GJ_USAGE,
	GJ_MISMATCH,
	GJ_BADCMD,
	GJ_ALLOC,
	GJ_FILE
} ParserStatus;

// Parser vtable
struct ParserCallbacks {
	SetFunc setTCK;
	SetFunc setTMS;
	SetFunc setTDI;
	GetFunc getTDO;
};

// Parse progFile, executing callback functions to do fundamental JTAG operations
ParserStatus parse(
	const char *progFile, const struct ParserCallbacks *cb, const char **error
) WARN_UNUSED_RESULT;

#endif
