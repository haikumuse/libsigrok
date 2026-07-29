/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Olivier Fauchon <olivier@aixmarseille.com>
 * Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 * Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>
 * Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

#define ANALOG_SAMPLES_PER_PERIOD 20

static const uint8_t pattern_sigrok[] = {
	0x4c, 0x92, 0x92, 0x92, 0x64, 0x00, 0x00, 0x00,
	0x82, 0xfe, 0xfe, 0x82, 0x00, 0x00, 0x00, 0x00,
	0x7c, 0x82, 0x82, 0x92, 0x74, 0x00, 0x00, 0x00,
	0xfe, 0x12, 0x12, 0x32, 0xcc, 0x00, 0x00, 0x00,
	0x7c, 0x82, 0x82, 0x82, 0x7c, 0x00, 0x00, 0x00,
	0xfe, 0x10, 0x28, 0x44, 0x82, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xbe, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t pattern_squid[128][128 / 8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xe0, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe1, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe1, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcf, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x87, 0x03, },
	{ 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xcf, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xcf, 0x03, },
	{ 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0xfe, 0x01, },
	{ 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xfe, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xfc, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0xc0, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x80, 0x01, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x07, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x1c, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x78, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0xe0, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0xc0, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf8, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf0, 0x01, 0x38, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0xf0, 0x1f, 0x1c, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x07, 0x70, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x0f, 0x00, 0xfc, 0x3f, 0x3c, },
	{ 0x80, 0x03, 0x00, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x80, 0xff, 0xff, 0x3f, 0x00, 0xfc, 0x7f, 0x7c, },
	{ 0x00, 0x1e, 0x00, 0x00, 0x1f, 0xc0, 0x01, 0x00, 0xc0, 0xff, 0xff, 0x7f, 0x00, 0xfe, 0xff, 0x7c, },
	{ 0x00, 0xf0, 0x01, 0x00, 0x7c, 0x80, 0x03, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x7c, },
	{ 0x00, 0xc0, 0x0f, 0x00, 0xf0, 0x00, 0x07, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xf8, 0x78, },
	{ 0x00, 0x00, 0x3e, 0x00, 0xc0, 0x03, 0x0e, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0xf0, 0x07, 0x80, 0x07, 0x3c, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0x80, 0x3f, 0x00, 0x1e, 0x78, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xe0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xff, 0x00, 0x7c, 0xe0, 0x01, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xe0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0xc1, 0x07, 0xf0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xc0, 0x03, 0x1f, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0x78, },
	{ 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x1f, 0xfc, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x7f, },
	{ 0xf8, 0x03, 0x00, 0x00, 0xf0, 0x07, 0xfe, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x7f, },
	{ 0x80, 0xff, 0x01, 0x00, 0x80, 0x3f, 0xf0, 0x8f, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x3f, },
	{ 0x00, 0xf0, 0xff, 0x07, 0x00, 0xfc, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x3f, },
	{ 0x00, 0x00, 0xfc, 0x7f, 0x00, 0xe0, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x1f, },
	{ 0x00, 0x00, 0xc0, 0xff, 0x1f, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfc, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf8, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x03, },
	{ 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x03, },
	{ 0x00, 0x10, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7c, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf8, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3c, 0x00, 0x00, },
	{ 0x00, 0x00, 0xc0, 0xff, 0x1f, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x00, 0x00, },
	{ 0x00, 0x00, 0xf8, 0x7f, 0x00, 0xe0, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x00, 0x00, },
	{ 0x00, 0xf0, 0xff, 0x07, 0x00, 0xfc, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x0f, 0x00, 0x00, },
	{ 0x80, 0xff, 0x01, 0x00, 0x80, 0x3f, 0xf0, 0x8f, 0xff, 0xff, 0xff, 0xff, 0x01, 0x0f, 0x00, 0x00, },
	{ 0xf8, 0x03, 0x00, 0x00, 0xf0, 0x07, 0xfe, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x1f, 0xfc, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xc0, 0x07, 0x1f, 0xf0, 0xff, 0xff, 0xff, 0x00, 0x06, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0xc1, 0x07, 0xf0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xff, 0x00, 0x7c, 0xe0, 0x01, 0xf0, 0xff, 0xff, 0xff, 0x01, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0x00, 0x80, 0x3f, 0x00, 0x1e, 0x78, 0x00, 0xf0, 0xff, 0xff, 0xff, 0x01, 0xf0, 0x7f, 0x00, },
	{ 0x00, 0x00, 0xf0, 0x0f, 0x80, 0x07, 0x3c, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x3e, 0x00, 0xc0, 0x03, 0x0e, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfc, 0xff, 0x01, },
	{ 0x00, 0xc0, 0x0f, 0x00, 0xf0, 0x00, 0x07, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfe, 0xff, 0x01, },
	{ 0x00, 0xf0, 0x01, 0x00, 0x7c, 0x80, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x3e, 0x00, 0x00, 0x1f, 0xc0, 0x01, 0x00, 0x80, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xe0, 0x03, },
	{ 0x80, 0x03, 0x00, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x07, 0x70, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf0, 0x01, 0x38, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x0f, 0x80, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf8, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x03, },
	{ 0x00, 0x00, 0xc0, 0x03, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, },
	{ 0x00, 0x00, 0xe0, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x78, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x1c, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x06, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x7f, 0x00, },
	{ 0x00, 0x80, 0x03, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0xc0, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, },
	{ 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
};

SR_PRIV void demo_generate_analog_pattern(struct dev_context *devc)
{
	double t, frequency;
	float amplitude, offset;
	struct analog_pattern *pattern;
	unsigned int num_samples, i;
	float value;
	int last_end;

	num_samples = ANALOG_BUFSIZE / sizeof(float);
	frequency = (double) devc->cur_samplerate / ANALOG_SAMPLES_PER_PERIOD;
	amplitude = DEFAULT_ANALOG_AMPLITUDE;
	offset = DEFAULT_ANALOG_OFFSET;

	/*
	 * FIXME: We actually need only one period. A ringbuffer would be
	 * useful here.
	 * Make sure the number of samples we put out is an integer
	 * multiple of our period size.
	 */

	/* PATTERN_SQUARE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SQUARE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	value = amplitude;
	last_end = 0;
	for (i = 0; i < num_samples; i++) {
		if (i % 5 == 0)
			value = -value;
		if (i % 10 == 0)
			last_end = i;
		pattern->data[i] = value + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SQUARE] = pattern;

	/* Readjusting num_samples for all other patterns. */
	while (num_samples % ANALOG_SAMPLES_PER_PERIOD != 0)
		num_samples--;

	/* PATTERN_SINE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SINE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = sin(2 * G_PI * frequency * t) * amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SINE] = pattern;

	/* PATTERN_TRIANGLE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_TRIANGLE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = (2 / G_PI) * asin(sin(2 * G_PI * frequency * t)) *
			amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_TRIANGLE] = pattern;

	/* PATTERN_SAWTOOTH: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SAWTOOTH]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = 2 * ((t * frequency) - floor(0.5f + t * frequency)) *
			amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SAWTOOTH] = pattern;

	/* PATTERN_ANALOG_RANDOM */
	/* Data not filled here, will be generated in send_analog_packet(). */
	pattern = g_malloc(sizeof(struct analog_pattern));
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_ANALOG_RANDOM] = pattern;
}

SR_PRIV void demo_free_analog_pattern(struct dev_context *devc)
{
	g_free(devc->analog_patterns[PATTERN_SQUARE]);
	g_free(devc->analog_patterns[PATTERN_SINE]);
	g_free(devc->analog_patterns[PATTERN_TRIANGLE]);
	g_free(devc->analog_patterns[PATTERN_SAWTOOTH]);
	g_free(devc->analog_patterns[PATTERN_ANALOG_RANDOM]);
}

static uint64_t encode_number_to_gray(uint64_t nr)
{
	return nr ^ (nr >> 1);
}

static void set_logic_data(uint64_t bits, uint8_t *data, size_t len)
{
	while (len--) {
		*data++ = bits & 0xff;
		bits >>= 8;
	}
}

static void logic_generator(struct sr_dev_inst *sdi, uint64_t size)
{
	struct dev_context *devc;
	uint64_t i, j;
	uint8_t pat;
	uint8_t *sample;
	const uint8_t *image_col;
	size_t col_count, col_height;
	uint64_t gray;

	devc = sdi->priv;

	switch (devc->logic_pattern) {
	case PATTERN_SIGROK:
		memset(devc->logic_data, 0x00, size);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			for (j = 0; j < devc->logic_unitsize; j++) {
				pat = pattern_sigrok[(devc->step + j) % sizeof(pattern_sigrok)] >> 1;
				devc->logic_data[i + j] = ~pat;
			}
			devc->step++;
		}
		break;
	case PATTERN_RANDOM:
		for (i = 0; i < size; i++)
			devc->logic_data[i] = (uint8_t)(rand() & 0xff);
		break;
	case PATTERN_INC:
		for (i = 0; i < size; i++) {
			for (j = 0; j < devc->logic_unitsize; j++)
				devc->logic_data[i + j] = devc->step;
			devc->step++;
		}
		break;
	case PATTERN_WALKING_ONE:
		/* j contains the value of the highest bit */
		j = 1 << (devc->num_logic_channels - 1);
		for (i = 0; i < size; i++) {
			devc->logic_data[i] = devc->step;
			if (devc->step == 0)
				devc->step = 1;
			else
				if (devc->step == j)
					devc->step = 0;
				else
					devc->step <<= 1;
		}
		break;
	case PATTERN_WALKING_ZERO:
		/* Same as walking one, only with inverted output */
		/* j contains the value of the highest bit */
		j = 1 << (devc->num_logic_channels - 1);
		for (i = 0; i < size; i++) {
			devc->logic_data[i] = ~devc->step;
			if (devc->step == 0)
				devc->step = 1;
			else
				if (devc->step == j)
					devc->step = 0;
				else
					devc->step <<= 1;
		}
		break;
	case PATTERN_ALL_LOW:
	case PATTERN_ALL_HIGH:
		/* These were set when the pattern mode was selected. */
		break;
	case PATTERN_SQUID:
		memset(devc->logic_data, 0x00, size);
		col_count = ARRAY_SIZE(pattern_squid);
		col_height = ARRAY_SIZE(pattern_squid[0]);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			sample = &devc->logic_data[i];
			image_col = pattern_squid[devc->step];
			for (j = 0; j < devc->logic_unitsize; j++) {
				pat = image_col[j % col_height];
				sample[j] = pat;
			}
			devc->step++;
			devc->step %= col_count;
		}
		break;
	case PATTERN_GRAYCODE:
		for (i = 0; i < size; i += devc->logic_unitsize) {
			devc->step++;
			devc->step &= devc->all_logic_channels_mask;
			gray = encode_number_to_gray(devc->step);
			gray &= devc->all_logic_channels_mask;
			set_logic_data(gray, &devc->logic_data[i], devc->logic_unitsize);
		}
		break;
	default:
		sr_err("Unknown pattern: %d.", devc->logic_pattern);
		break;
	}
}

/*
 * Fixup a memory image of generated logic data before it gets sent to
 * the session's datafeed. Mask out content from disabled channels.
 *
 * TODO: Need we apply a channel map, and enforce a dense representation
 * of the enabled channels' data?
 */
static void logic_fixup_feed(struct dev_context *devc,
		struct sr_datafeed_logic *logic)
{
	size_t fp_off;
	uint8_t fp_mask;
	size_t off, idx;
	uint8_t *sample;

	fp_off = devc->first_partial_logic_index;
	fp_mask = devc->first_partial_logic_mask;
	if (fp_off == logic->unitsize)
		return;

	for (off = 0; off < logic->length; off += logic->unitsize) {
		sample = logic->data + off;
		sample[fp_off] &= fp_mask;
		for (idx = fp_off + 1; idx < logic->unitsize; idx++)
			sample[idx] = 0x00;
	}
}

/*
 * 对生成的模拟样本施加耦合效果 (模拟真实示波器模拟前端的 AC/DC/GND 耦合)。
 *
 * DSL 等真实硬件驱动通过 I2C 配置耦合继电器, 由硬件物理实现; demo 没有
 * 硬件前端, 必须在软件层模拟。行为参考示波器耦合原理:
 *   GND (0): 信号接地, 输出恒 0 (观察基线噪声/零点)
 *   DC  (1): 直通, 信号原样通过 (保留直流分量)
 *   AC  (2): 高通滤波, 去除直流分量 (一阶 RC, 截止频率 ~ 采样率/200)
 *
 * AC 耦合一阶 RC 高通: y[n] = a*(y[n-1] + x[n] - x[n-1])
 *   a = RC/(RC+dt), dt = 1/samplerate, RC 选 ~200 个样本周期使低频被抑制。
 */
#define DEMO_AC_COUPLING_TAU 200.0f  /* RC 时间常数 (样本数) */

static void apply_analog_coupling(struct analog_gen *ag,
		struct dev_context *devc, float *data, unsigned int count)
{
	uint8_t coupling;
	int aidx;

	if (!ag->ch || !devc)
		return;

	/* 计算模拟通道索引 (与 api.c config_get/set 的 aidx 一致)。 */
	aidx = ag->ch->index - devc->num_logic_channels;
	if (aidx < 0 || aidx >= devc->num_analog_channels || aidx >= DSO_MAX_CHANNELS)
		return;

	coupling = devc->analog_coupling[aidx];

	if (coupling == 1 /* SR_DC_COUPLING */)
		return;  /* DC: 直通, 无处理 */

	if (coupling == 0 /* SR_GND_COUPLING */) {
		/* GND: 强制所有样本为 0。 */
		for (unsigned int i = 0; i < count; i++)
			data[i] = 0.0f;
		/* 重置 AC 滤波器状态避免切换后跳变。 */
		ag->ac_prev_input = 0.0f;
		ag->ac_prev_output = 0.0f;
		return;
	}

	if (coupling == 2 /* SR_AC_COUPLING */) {
		/* 一阶 RC 高通滤波器: y[n] = a*(y[n-1] + x[n] - x[n-1]) */
		const float a = DEMO_AC_COUPLING_TAU / (DEMO_AC_COUPLING_TAU + 1.0f);
		float prev_in = ag->ac_prev_input;
		float prev_out = ag->ac_prev_output;
		for (unsigned int i = 0; i < count; i++) {
			float x = data[i];
			float y = a * (prev_out + x - prev_in);
			data[i] = y;
			prev_in = x;
			prev_out = y;
		}
		ag->ac_prev_input = prev_in;
		ag->ac_prev_output = prev_out;
		return;
	}

	/* 未知耦合值, 按 DC 处理 (直通)。 */
}

/*
 * 对 DSO 样本施加耦合效果。与 apply_analog_coupling 行为一致, 但针对
 * DSO 的 byte-interleaved 缓冲区布局 [ch0_s0, ch1_s0, ch0_s1, ...] 操作。
 *
 * DSO 样本是 8-bit (0..255), 中心值 mid=128 (DSO_SAMPLE_BITS/2 对应的 ADC
 * 中点)。各耦合模式:
 *   GND (0): 样本强制为 mid (基线), 观察零点
 *   DC  (1): 直通
 *   AC  (2): 一阶 RC 高通 (去除直流分量), 围绕 mid 振荡
 *
 * 注意: 此函数必须在 demo_dso_vdiv_scale() 之后调用 (vdiv 缩放改变样本幅度,
 * 耦合应在最终样本值上应用)。每通道 AC 滤波状态保存在
 * devc->dso_ac_prev_input/output, 跨包连续。
 */
static void apply_dso_coupling(struct dev_context *devc, uint8_t *data,
		uint64_t sample_count, uint8_t en_ch_num)
{
	const uint8_t mid = 128;
	const float a = DEMO_AC_COUPLING_TAU / (DEMO_AC_COUPLING_TAU + 1.0f);

	if (!devc || !data || en_ch_num == 0)
		return;

	for (uint8_t ch = 0; ch < en_ch_num && ch < DSO_MAX_CHANNELS; ch++) {
		uint8_t coupling = devc->dso_coupling[ch];

		if (coupling == 1 /* SR_DC_COUPLING */)
			continue;  /* DC: 直通 */

		if (coupling == 0 /* SR_GND_COUPLING */) {
			/* GND: 强制所有样本为 mid (基线)。 */
			for (uint64_t i = 0; i < sample_count; i++)
				data[i * en_ch_num + ch] = mid;
			/* 重置 AC 滤波器状态避免切换后跳变。 */
			devc->dso_ac_prev_input[ch] = 0.0f;
			devc->dso_ac_prev_output[ch] = 0.0f;
			continue;
		}

		if (coupling == 2 /* SR_AC_COUPLING */) {
			/* 一阶 RC 高通: 对 (x - mid) 做高通, 输出加回 mid。
			 * y' = a*(y'[n-1] + (x-mid) - (x_prev-mid))
			 * y  = y' + mid */
			float prev_in = devc->dso_ac_prev_input[ch];
			float prev_out = devc->dso_ac_prev_output[ch];
			for (uint64_t i = 0; i < sample_count; i++) {
				float x = (float)data[i * en_ch_num + ch] - mid;
				float y = a * (prev_out + x - prev_in);
				int v = (int)(y + mid + 0.5f);
				if (v < 0) v = 0;
				if (v > 255) v = 255;
				data[i * en_ch_num + ch] = (uint8_t)v;
				prev_in = x;
				prev_out = y;
			}
			devc->dso_ac_prev_input[ch] = prev_in;
			devc->dso_ac_prev_output[ch] = prev_out;
			continue;
		}
		/* 未知耦合值, 按 DC 处理 (直通)。 */
	}
}

static void send_analog_packet(struct analog_gen *ag,
		struct sr_dev_inst *sdi, uint64_t *analog_sent,
		uint64_t analog_pos, uint64_t analog_todo)
{
	struct sr_datafeed_packet packet;
	struct dev_context *devc;
	struct analog_pattern *pattern;
	uint64_t sending_now, to_avg;
	int ag_pattern_pos;
	unsigned int i;
	float amplitude, offset, value;
	float *data;

	if (!ag->ch || !ag->ch->enabled)
		return;

	devc = sdi->priv;
	packet.type = SR_DF_ANALOG;
	packet.payload = &ag->packet;

	pattern = devc->analog_patterns[ag->pattern];

	ag->packet.meaning->channels = g_slist_append(NULL, ag->ch);
	ag->packet.meaning->mq = ag->mq;
	ag->packet.meaning->mqflags = ag->mq_flags;

	/* Set a unit for the given quantity. */
	if (ag->mq == SR_MQ_VOLTAGE)
		ag->packet.meaning->unit = SR_UNIT_VOLT;
	else if (ag->mq == SR_MQ_CURRENT)
		ag->packet.meaning->unit = SR_UNIT_AMPERE;
	else if (ag->mq == SR_MQ_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_TEMPERATURE)
		ag->packet.meaning->unit = SR_UNIT_CELSIUS;
	else if (ag->mq == SR_MQ_FREQUENCY)
		ag->packet.meaning->unit = SR_UNIT_HERTZ;
	else if (ag->mq == SR_MQ_DUTY_CYCLE)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONTINUITY)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_PULSE_WIDTH)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_SIEMENS;
	else if (ag->mq == SR_MQ_POWER)
		ag->packet.meaning->unit = SR_UNIT_WATT;
	else if (ag->mq == SR_MQ_GAIN)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_SOUND_PRESSURE_LEVEL)
		ag->packet.meaning->unit = SR_UNIT_DECIBEL_SPL;
	else if (ag->mq == SR_MQ_CARBON_MONOXIDE)
		ag->packet.meaning->unit = SR_UNIT_CONCENTRATION;
	else if (ag->mq == SR_MQ_RELATIVE_HUMIDITY)
		ag->packet.meaning->unit = SR_UNIT_HUMIDITY_293K;
	else if (ag->mq == SR_MQ_TIME)
		ag->packet.meaning->unit = SR_UNIT_SECOND;
	else if (ag->mq == SR_MQ_WIND_SPEED)
		ag->packet.meaning->unit = SR_UNIT_METER_SECOND;
	else if (ag->mq == SR_MQ_PRESSURE)
		ag->packet.meaning->unit = SR_UNIT_HECTOPASCAL;
	else if (ag->mq == SR_MQ_PARALLEL_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_PARALLEL_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_PARALLEL_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_SERIES_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_SERIES_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_SERIES_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_DISSIPATION_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_QUALITY_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_PHASE_ANGLE)
		ag->packet.meaning->unit = SR_UNIT_DEGREE;
	else if (ag->mq == SR_MQ_DIFFERENCE)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_COUNT)
		ag->packet.meaning->unit = SR_UNIT_PIECE;
	else if (ag->mq == SR_MQ_POWER_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_APPARENT_POWER)
		ag->packet.meaning->unit = SR_UNIT_VOLT_AMPERE;
	else if (ag->mq == SR_MQ_MASS)
		ag->packet.meaning->unit = SR_UNIT_GRAM;
	else if (ag->mq == SR_MQ_HARMONIC_RATIO)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;

	if (!devc->avg) {
		ag_pattern_pos = analog_pos % pattern->num_samples;
		sending_now = MIN(analog_todo, pattern->num_samples - ag_pattern_pos);
		/* 判断当前通道耦合是否为 DC (直通)。非 DC 时即使 amplitude/offset
		 * 未变也必须走慢速路径 (需修改样本), 不能直接指向 pattern->data。 */
		int aidx_coupling = ag->ch ? ag->ch->index - devc->num_logic_channels : -1;
		uint8_t cur_coupling = (aidx_coupling >= 0
				&& aidx_coupling < devc->num_analog_channels
				&& aidx_coupling < DSO_MAX_CHANNELS)
			? devc->analog_coupling[aidx_coupling] : 1 /* DC */;
		gboolean need_coupling = (cur_coupling != 1 /* DC */);
		if (ag->amplitude != DEFAULT_ANALOG_AMPLITUDE ||
			ag->offset != DEFAULT_ANALOG_OFFSET ||
			ag->pattern == PATTERN_ANALOG_RANDOM || need_coupling) {
			/*
			 * Amplitude or offset changed (or we are generating
			 * random data, or coupling != DC needs per-sample
			 * processing), modify each sample.
			 */
			if (ag->pattern == PATTERN_ANALOG_RANDOM) {
				amplitude = ag->amplitude / 500.0;
				offset = ag->offset - DEFAULT_ANALOG_OFFSET - ag->amplitude;
			} else {
				amplitude = ag->amplitude / DEFAULT_ANALOG_AMPLITUDE;
				offset = ag->offset - DEFAULT_ANALOG_OFFSET;
			}
			/* 耦合处理时用 coupling_buf, 避免污染共享的 pattern->data
			 * 或上一包的 packet.data。DC 耦合保持原逻辑用 packet.data。 */
			data = need_coupling ? ag->coupling_buf : ag->packet.data;
			for (i = 0; i < sending_now; i++) {
				if (ag->pattern == PATTERN_ANALOG_RANDOM) {
					/* Use pre-filled cyclic buffer for stable random
					 * data (repeats like a real captured signal). */
					if (devc->analog_random_buf)
						data[i] = (devc->analog_random_buf[
							(analog_pos + i) % devc->analog_random_buf_len]
							/ 255.0 * 1000) * amplitude + offset;
					else
						data[i] = (rand() % 1000) * amplitude + offset;
				} else
					data[i] = pattern->data[ag_pattern_pos + i] * amplitude + offset;
			}
			/* 应用耦合效果 (GND 置零 / AC 高通; DC 在 apply 内直通返回)。 */
			apply_analog_coupling(ag, devc, data, sending_now);
			ag->packet.data = data;
		} else {
			/* Amplitude and offset unchanged, use the fast way. */
			ag->packet.data = pattern->data + ag_pattern_pos;
		}
		ag->packet.num_samples = sending_now;
		sr_session_send(sdi, &packet);

		/* Whichever channel group gets there first. */
		*analog_sent = MAX(*analog_sent, sending_now);
	} else {
		ag_pattern_pos = analog_pos % pattern->num_samples;
		to_avg = MIN(analog_todo, pattern->num_samples - ag_pattern_pos);
		if (ag->pattern == PATTERN_ANALOG_RANDOM) {
			amplitude = ag->amplitude / 500.0;
			offset = ag->offset - DEFAULT_ANALOG_OFFSET - ag->amplitude;
		} else {
			amplitude = ag->amplitude / DEFAULT_ANALOG_AMPLITUDE;
			offset = ag->offset - DEFAULT_ANALOG_OFFSET;
		}

		for (i = 0; i < to_avg; i++) {
			if (ag->pattern == PATTERN_ANALOG_RANDOM)
				value = (rand() % 1000) * amplitude + offset;
			else
				value = *(pattern->data + ag_pattern_pos + i) * amplitude + offset;
			ag->avg_val = (ag->avg_val + value) / 2;
			ag->num_avgs++;
			/* Time to send averaged data? */
			if ((devc->avg_samples > 0) && (ag->num_avgs >= devc->avg_samples))
				goto do_send;
		}

		if (devc->avg_samples == 0) {
			/*
			 * We're averaging all the samples, so wait with
			 * sending until the very end.
			 */
			*analog_sent = ag->num_avgs;
			return;
		}

do_send:
		/* 对平均值也应用耦合 (avg 模式下每包只发 1 个样本, GND 直接置零,
		 * AC 高通对单点退化为减去直流估计 — 用 prev_output 逼近)。 */
		apply_analog_coupling(ag, devc, &ag->avg_val, 1);
		ag->packet.data = &ag->avg_val;
		ag->packet.num_samples = 1;

		sr_session_send(sdi, &packet);
		*analog_sent = ag->num_avgs;

		ag->num_avgs = 0;
		ag->avg_val = 0.0f;
	}
}

/* Clear DSO config-change flags. demo_send_dso_packet() regenerates the
 * waveform on every call from current dso_pattern/vdiv/offset/timebase, so
 * the flags only need clearing to mark "regeneration consumed". Mirrors old
 * fork demo's dso_wavelength_updata() contract: config_set sets a flag, the
 * next packet send regenerates and clears it. */
SR_PRIV void dso_wavelength_updata(struct dev_context *devc)
{
	if (!devc)
		return;
	devc->dso_vdiv_change = FALSE;
	devc->dso_offset_change = FALSE;
	devc->dso_timebase_change = FALSE;
}

/* Pre-fill the analog random cyclic buffer. Called once at acquisition start
 * so PATTERN_ANALOG_RANDOM streams a stable, repeating noise pattern (like a
 * real captured signal) instead of fresh random noise every packet. Mirrors
 * old fork demo's init_analog_random_data(). */
SR_PRIV int init_analog_random_data(struct dev_context *devc)
{
	size_t i;

	if (!devc)
		return SR_ERR_ARG;
	if (!devc->analog_random_buf) {
		devc->analog_random_buf = g_malloc(ANALOG_RANDOM_BUF_LEN);
		if (!devc->analog_random_buf)
			return SR_ERR_MALLOC;
		devc->analog_random_buf_len = ANALOG_RANDOM_BUF_LEN;
	}
	for (i = 0; i < devc->analog_random_buf_len; i++)
		devc->analog_random_buf[i] = (uint8_t)(rand() & 0xff);
	devc->analog_random_read_pos = 0;
	return SR_OK;
}

/* Generate one 8-bit DSO sample for the given pattern at sample index i.
 * wavelength = samples per period. mid/amp define the vertical range.
 * ch_idx adds a small phase offset per channel so traces don't overlap. */
static uint8_t demo_dso_sample(enum demo_dso_pattern pat, uint64_t i,
		uint8_t mid, uint8_t amp, int ch_idx, uint64_t wavelength)
{
	uint64_t pos;
	double val;

	switch (pat) {
	case DEMO_DSO_PATTERN_RANDOM:
		return (uint8_t)((rand() % 120) + 68);
	case DEMO_DSO_PATTERN_SINE:
		/* Per-channel phase offset so channels don't overlap. */
		val = mid + amp * sin(2.0 * M_PI * (i + ch_idx * (wavelength / 4))
				/ (double)wavelength);
		if (val < 0) val = 0;
		if (val > 255) val = 255;
		return (uint8_t)val;
	case DEMO_DSO_PATTERN_SQUARE:
		pos = (i + ch_idx * (wavelength / 2)) % wavelength;
		return (pos < wavelength / 2) ? (uint8_t)(mid + amp) : (uint8_t)(mid - amp);
	case DEMO_DSO_PATTERN_SAWTOOTH:
		pos = (i + ch_idx * (wavelength / 4)) % wavelength;
		return (uint8_t)(mid - amp + (uint16_t)(2 * amp * pos / wavelength));
	case DEMO_DSO_PATTERN_TRIANGLE:
		pos = (i + ch_idx * (wavelength / 4)) % wavelength;
		if (pos < wavelength / 2)
			return (uint8_t)(mid - amp
				+ (uint16_t)(2 * amp * pos / (wavelength / 2)));
		else
			return (uint8_t)(mid + amp
				- (uint16_t)(2 * amp * (pos - wavelength / 2) / (wavelength / 2)));
	default:
		return mid;
	}
}

/*
 * Send one DSO frame (one SR_DF_DSO packet) for the demo device.
 *
 * Generates waveform data based on devc->dso_pattern (random/sine/square/
 * sawtooth/triangle) for each enabled DSO channel, interleaved as
 * [ch0_s0, ch1_s0, ch0_s1, ch1_s1, ...] which matches the layout expected
 * by DsoSnapshot::append_data(). Per-channel vdiv/offset/trig_value apply
 * vertical shifts so the traces look distinct.
 *
 * In instant mode, sends a progressive slice based on elapsed time rather
 * than the full frame, so the GUI shows a live scrolling waveform.
 * After sending the packet the acquisition is stopped — DSO mode delivers
 * one complete frame per acquisition (single-shot). The caller may restart
 * acquisition for the next frame (repeat/loop mode handled in
 * dev_acquisition_stop).
 */
SR_PRIV int demo_send_dso_packet(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_dso dso;
	struct sr_channel *ch;
	GSList *l;
	uint8_t en_ch_num;
	uint8_t *p;
	uint64_t i;
	uint64_t sending_samples;
	uint64_t wavelength;
	uint8_t mid, amp;
	int ch_idx;

	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;

	devc = sdi->priv;

	/* Consume any pending config-change flags (regeneration is implicit
	 * — we build the waveform from current config on every call). */
	dso_wavelength_updata(devc);

	/* Count enabled DSO channels. */
	en_ch_num = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch && ch->type == SR_CHANNEL_DSO && ch->enabled)
			en_ch_num++;
	}
	if (en_ch_num == 0)
		return SR_ERR;

	/* In instant mode, send a progressive slice based on how many samples
	 * have already been sent, so the GUI shows a live scrolling waveform.
	 * Otherwise send the full frame (single-shot DSO). */
	sending_samples = DSO_PACKET_LEN;
	if (devc->instant && devc->dso_sent_samples > 0) {
		/* Send a fraction of the remaining frame each tick. */
		uint64_t remaining = (devc->dso_sent_samples >= DSO_PACKET_LEN)
			? 0 : (DSO_PACKET_LEN - devc->dso_sent_samples);
		if (remaining > 0)
			sending_samples = MIN(remaining, DSO_PACKET_LEN / 10);
		else
			sending_samples = DSO_PACKET_LEN; /* wrap: start a new frame */
	}

	/* (Re)allocate the DSO buffer if needed (full-frame sized). */
	if (!devc->dso_buf) {
		devc->dso_buf = g_malloc(DSO_PACKET_LEN * en_ch_num);
		if (!devc->dso_buf)
			return SR_ERR_MALLOC;
	}

	/* Waveform geometry: ~4 periods across the screen. mid=128 (8-bit
	 * center). amp scales with the first enabled channel's vdiv so
	 * changing vdiv visibly changes amplitude. */
	wavelength = DSO_PACKET_LEN / 4;
	if (wavelength < 20)
		wavelength = 20;
	mid = 128;
	amp = 60;

	/*
	 * Fill interleaved samples using the selected DSO pattern. Each
	 * channel gets a phase offset (via ch_idx) so traces don't overlap.
	 * Per-channel trigger-value shifts the baseline vertically.
	 */
	p = devc->dso_buf;
	for (i = devc->dso_sent_samples; i < devc->dso_sent_samples + sending_samples; i++) {
		ch_idx = 0;
		for (l = sdi->channels; l; l = l->next) {
			ch = l->data;
			if (!ch || ch->type != SR_CHANNEL_DSO || !ch->enabled)
				continue;
			uint8_t v = demo_dso_sample(devc->dso_pattern, i,
					mid, amp, ch_idx, wavelength);
			/* Apply per-channel trigger-value offset. */
			if (ch_idx < DSO_MAX_CHANNELS)
				v = (uint8_t)(v + (uint8_t)(devc->dso_trig_value[ch_idx] - 128));
			*p++ = v;
			ch_idx++;
		}
	}

	/* Build and send the DSO packet. */
	dso.data = devc->dso_buf;
	dso.num_samples = sending_samples;
	dso.trig_flag = 1;            /* Trigger found in this packet. */
	dso.trig_ch = 0;              /* First enabled DSO channel. */
	dso.en_ch_num = en_ch_num;
	dso.sample_bits = devc->dso_unit_bits;
	dso.trig_offset = (int16_t)(sending_samples / 2);  /* Trigger at center. */
	dso.packet_len = sending_samples * en_ch_num;
	dso.samplerate_tog = (uint32_t)devc->cur_samplerate;

	/* Apply per-channel vdiv scaling before sending — mirrors old fork
	 * demo's receive_data_dso vdiv transform. The dso_buf layout is
	 * interleaved [ch0_s0, ch1_s0, ch0_s1, ...] so ch_idx = i % 2. */
	demo_dso_vdiv_scale(devc, devc->dso_buf, dso.packet_len);

	/* Apply per-channel coupling (GND/DC/AC) on the final scaled samples.
	 * Without this, switching DSO coupling from DC to GND in the header
	 * had no visible effect — the waveform kept showing the live signal
	 * instead of collapsing to the mid baseline. ANALOG channels already
	 * had this via apply_analog_coupling(); DSO was missing it. */
	apply_dso_coupling(devc, devc->dso_buf, sending_samples, en_ch_num);

	/* Diagnostic: log factor/coupling state every ~50 frames to verify
	 * config changes reach the driver. Remove after debugging. */
	{
		static int _dso_cfg_dbg = 0;
		if ((++_dso_cfg_dbg % 20) == 0) {
			sr_warn("[DSO-CFG] vf[0]=%llu vf[1]=%llu coup[0]=%u coup[1]=%u "
			       "en[0]=%d en[1]=%d amp=%u",
			       (unsigned long long)devc->dso_vfactor[0],
			       (unsigned long long)devc->dso_vfactor[1],
			       devc->dso_coupling[0], devc->dso_coupling[1],
			       (int)devc->dso_enabled[0], (int)devc->dso_enabled[1],
			       amp);
		}
	}

	packet.type = SR_DF_DSO;
	packet.payload = &dso;
	sr_session_send(sdi, &packet);

	/* Update DSO measurement stats (max/min/cycle/acc_*) after sending
	 * — mirrors old fork dso_status_update() call in receive_data_dso. */
	demo_dso_status_update(devc, devc->dso_buf, dso.packet_len);

	devc->dso_sent_samples += sending_samples;

	return SR_OK;
}

/* Callback handling data */
SR_PRIV int demo_prepare_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct analog_gen *ag;
	GHashTableIter iter;
	void *value;
	uint64_t samples_todo, logic_done, analog_done, analog_sent, sending_now;
	int64_t elapsed_us, limit_us, todo_us;
	int64_t trigger_offset;
	int pre_trigger_samples;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;

	/* .demo file replay: if a non-random sample source with an open
	 * archive is set, route to the mode-specific receive_data_*_file
	 * function instead of the math-generator streaming path below. */
	if (devc->sample_generator != DEMO_GEN_RANDOM && devc->archive) {
		switch (devc->device_mode) {
		case DEMO_MODE_LOGIC:
			return demo_receive_data_logic_decoder(sdi);
		case DEMO_MODE_DSO:
			return demo_receive_data_dso_file(sdi);
		case DEMO_MODE_ANALOG:
			return demo_receive_data_analog_file(sdi);
		default:
			break;
		}
	}

	static int _demo_tick_count = 0;
	_demo_tick_count++;
	/* Reduced to sr_dbg: with 25ms tick interval this fires 40x/sec,
	 * sr_info would flood the log. */
	sr_dbg("demo_prepare_data TICK #%d: stl=%p, trigger_fired=%d, "
		"limit_samples=%" PRIu64 ", limit_msec=%" PRIu64 ", "
		"sent_samples=%" PRIu64 ", spent_us=%" PRId64 ", "
		"cur_samplerate=%" PRIu64 ", num_logic=%zu, num_analog=%zu, "
		"num_dso=%zu, enabled_logic=%zu, enabled_analog=%zu",
		_demo_tick_count, (void*)devc->stl, (int)devc->trigger_fired,
		devc->limit_samples, devc->limit_msec,
		devc->sent_samples, devc->spent_us,
		devc->cur_samplerate, devc->num_logic_channels,
		devc->num_analog_channels, (size_t)devc->num_dso_channels,
		devc->enabled_logic_channels, devc->enabled_analog_channels);

	/*
	 * DSO mode: send one complete frame per acquisition tick.
	 * DSO channels coexist with logic/analog in the channel list, but in
	 * DSO work mode only DSO channels are enabled — detect that and skip
	 * the streaming logic/analog path entirely.
	 *
	 * In single-shot mode (non-instant, non-loop): send one full frame then
	 * stop. In instant mode: send progressive slices (live scrolling). In
	 * loop mode: send full frames continuously (timer stays alive).
	 */
	if (devc->num_dso_channels > 0) {
		gboolean has_enabled_dso = FALSE;
		gboolean has_enabled_other = FALSE;
		for (GSList *l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = l->data;
			if (!ch || !ch->enabled)
				continue;
			if (ch->type == SR_CHANNEL_DSO)
				has_enabled_dso = TRUE;
			else
				has_enabled_other = TRUE;
		}
		if (has_enabled_dso && !has_enabled_other) {
			/* Pure DSO acquisition. */
			demo_send_dso_packet(sdi);
			/* When a full frame has been sent, reset the counter so the
			 * next tick starts a fresh frame (instant mode wraps mid-frame). */
			if (devc->dso_sent_samples >= DSO_PACKET_LEN)
				devc->dso_sent_samples = 0;
			/* DSO 连续采集语义（与 DSView 一致）：
			 *   - instant=TRUE: 单帧滚动采集，发完一帧后停止
			 *     （应用层 Instant Stop 按钮触发）
			 *   - instant=FALSE: 持续采集，每 tick 发送一帧刷新波形，
			 *     不自动停止（应用层 Run/Stop 按钮触发，用户点 Stop 才停）
			 *
			 * 原代码在 !instant && !loop_mode 时单帧停止，导致 DSO 模式
			 * 点 Run/Stop 后只采一帧就停了。DSView 的 demo 驱动在非 instant
			 * 模式下持续发送帧不停止，由应用层 stop_capture 控制停止。
			 * loop_mode 逻辑已包含在"非 instant 持续发送"中，无需单独判断。
			 */
			if (devc->instant) {
				/* Instant: single frame, then stop. */
				sr_dev_acquisition_stop(sdi);
			}
			/* Non-instant: timer stays alive, next tick sends next frame. */
			return G_SOURCE_CONTINUE;
		}
	}

	/* Just in case. */
	if (devc->cur_samplerate <= 0
			|| (devc->num_logic_channels <= 0
			&& devc->num_analog_channels <= 0)) {
		sr_info("demo_prepare_data: EARLY STOP (samplerate=%" PRIu64
			", num_logic=%zu, num_analog=%zu)",
			devc->cur_samplerate, devc->num_logic_channels,
			devc->num_analog_channels);
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* What time span should we send samples for? */
	elapsed_us = g_get_monotonic_time() - devc->start_us;
	limit_us = 1000 * devc->limit_msec;
	if (limit_us > 0 && limit_us < elapsed_us)
		todo_us = MAX(0, limit_us - devc->spent_us);
	else
		todo_us = MAX(0, elapsed_us - devc->spent_us);

	/* How many samples are outstanding since the last round? */
	samples_todo = (todo_us * devc->cur_samplerate + G_USEC_PER_SEC - 1)
			/ G_USEC_PER_SEC;

	if (devc->limit_samples > 0) {
		if (devc->limit_samples < devc->sent_samples)
			samples_todo = 0;
		else if (devc->limit_samples - devc->sent_samples < samples_todo)
			samples_todo = devc->limit_samples - devc->sent_samples;
	}

	sr_dbg("demo_prepare_data: elapsed_us=%" PRId64 ", limit_us=%" PRId64
		", todo_us=%" PRId64 ", samples_todo=%" PRIu64,
		elapsed_us, limit_us, todo_us, samples_todo);

	if (samples_todo == 0)
		return G_SOURCE_CONTINUE;

	if (devc->limit_frames) {
		/* Never send more samples than a frame can fit... */
		samples_todo = MIN(samples_todo, SAMPLES_PER_FRAME);
		/* ...or than we need to finish the current frame. */
		samples_todo = MIN(samples_todo,
			SAMPLES_PER_FRAME - devc->sent_frame_samples);
	}

	/* Calculate the actual time covered by this run back from the sample
	 * count, rounded towards zero. This avoids getting stuck on a too-low
	 * time delta with no samples being sent due to round-off.
	 */
	todo_us = samples_todo * G_USEC_PER_SEC / devc->cur_samplerate;

	logic_done = devc->num_logic_channels > 0 ? 0 : samples_todo;
	if (!devc->enabled_logic_channels)
		logic_done = samples_todo;

	analog_done = devc->num_analog_channels > 0 ? 0 : samples_todo;
	if (!devc->enabled_analog_channels)
		analog_done = samples_todo;

	while (logic_done < samples_todo || analog_done < samples_todo) {
		/* Logic */
		if (logic_done < samples_todo) {
			sending_now = MIN(samples_todo - logic_done,
					LOGIC_BUFSIZE / devc->logic_unitsize);
			logic_generator(sdi, sending_now * devc->logic_unitsize);

			trigger_offset = 0;
			/* Trigger check: runs in BOTH modes. In Stream mode the trigger
			 * just inserts a SR_DF_TRIGGER marker into the stream (via
			 * std_session_send_df_trigger inside soft_trigger_logic_check).
			 * In Buffer mode it gates data emission until fire. */
			if (devc->stl && (!devc->trigger_fired)) {
			/* Per-tick trigger check log — sr_dbg to avoid flooding at 25ms ticks. */
			sr_dbg("demo trigger check: sending_now=%" PRIu64
				", logic_unitsize=%zu, data[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x"
				", stl->count=%d, op_mode=%d",
				sending_now, devc->logic_unitsize,
				devc->logic_data[0], devc->logic_data[1],
				devc->logic_data[2], devc->logic_data[3],
				devc->logic_data[4], devc->logic_data[5],
				devc->logic_data[6], devc->logic_data[7],
				devc->stl->count, devc->op_mode);
				trigger_offset = soft_trigger_logic_check(devc->stl,
						devc->logic_data, sending_now * devc->logic_unitsize,
						&pre_trigger_samples);
			sr_dbg("demo trigger check: soft_trigger_logic_check returned %d"
				", pre_trigger_samples=%d",
				(int)trigger_offset, pre_trigger_samples);
				if (trigger_offset > -1) {
					devc->trigger_fired = TRUE;
					sr_info("demo trigger FIRED at offset %d (op_mode=%d)",
						(int)trigger_offset, devc->op_mode);
					/* In Buffer mode, reset logic_done to pre_trigger_samples
					 * so pre-trigger data (already sent by soft_trigger_logic_check)
					 * isn't double-counted. In Stream mode we already sent the
					 * full buffer above, so just mark fired and continue. */
					if (devc->op_mode == DEMO_OP_BUFFER)
						logic_done = pre_trigger_samples;
				}
			}

			/* Send logic samples */
			packet.type = SR_DF_LOGIC;
			packet.payload = &logic;
			logic.unitsize = devc->logic_unitsize;

			if (!devc->stl) {
				/* No trigger defined: always send full buffer (both modes). */
				logic.length = sending_now * devc->logic_unitsize;
				logic.data = devc->logic_data;
				logic_fixup_feed(devc, &logic);
				sr_session_send(sdi, &packet);
				logic_done += sending_now;
			} else if (devc->op_mode == DEMO_OP_STREAM) {
				/* Stream mode: send the full buffer regardless of trigger
				 * state. The trigger marker was already emitted by
				 * soft_trigger_logic_check (SR_DF_TRIGGER). The frontend
				 * ring buffer will display pre/post-trigger data together. */
				logic.length = sending_now * devc->logic_unitsize;
				logic.data = devc->logic_data;
				logic_fixup_feed(devc, &logic);
				sr_session_send(sdi, &packet);
				logic_done += sending_now;
			} else {
				/* Buffer mode: only send AFTER trigger fires. Pre-trigger
				 * samples are buffered inside soft_trigger_logic and sent
				 * by soft_trigger_logic_check itself when the trigger fires. */
				if (devc->trigger_fired && (trigger_offset < (int)sending_now)) {
					logic.length = (sending_now - trigger_offset) * devc->logic_unitsize;
					logic.data = devc->logic_data + trigger_offset * devc->logic_unitsize;
					logic_fixup_feed(devc, &logic);
					sr_session_send(sdi, &packet);
					logic_done += sending_now - trigger_offset;
				} else if (!devc->trigger_fired) {
					/* Trigger not yet fired: send nothing. logic_done still
					 * advances so we don't loop forever, but sent_samples
					 * is NOT accumulated (see below). */
					logic_done += sending_now;
				}
			}
		}

		/* Analog, one channel at a time */
		if (analog_done < samples_todo) {
			analog_sent = 0;

			g_hash_table_iter_init(&iter, devc->ch_ag);
			while (g_hash_table_iter_next(&iter, NULL, &value)) {
				send_analog_packet(value, sdi, &analog_sent,
						devc->sent_samples + analog_done,
						samples_todo - analog_done);
			}
			analog_done += analog_sent;
		}
	}

	uint64_t min = MIN(logic_done, analog_done);
	/* sent_samples accumulation policy:
	 * - No trigger (stl==NULL): always accumulate (both modes).
	 * - Stream mode: always accumulate — data is being sent continuously,
	 *   trigger marker or not. limit_samples is the stop condition.
	 * - Buffer mode + trigger not fired: do NOT accumulate — keeps the
	 *   session alive waiting for trigger. spent_us still accumulates so
	 *   limit_msec acts as a timeout. */
	if (!devc->stl || devc->op_mode == DEMO_OP_STREAM || devc->trigger_fired) {
		devc->sent_samples += min;
	}
	devc->sent_frame_samples += min;
	devc->spent_us += todo_us;

	if (devc->limit_frames && devc->sent_frame_samples >= SAMPLES_PER_FRAME) {
		std_session_send_df_frame_end(sdi);
		devc->sent_frame_samples = 0;
		devc->limit_frames--;
		if (!devc->limit_frames) {
			sr_dbg("Requested number of frames reached.");
			sr_dev_acquisition_stop(sdi);
		}
	}

	if ((devc->limit_samples > 0 && devc->sent_samples >= devc->limit_samples)
			|| (limit_us > 0 && devc->spent_us >= limit_us)) {

		if (devc->loop_mode) {
			/* Loop mode: wrap counters and keep streaming instead of
			 * stopping. The session timer stays alive so data flows
			 * continuously until the user presses stop. */
			sr_info("demo_prepare_data: LOOP wrap (sent_samples=%" PRIu64
				" -> 0, spent_us=%" PRId64 " -> 0)",
				devc->sent_samples, devc->spent_us);
			devc->sent_samples = 0;
			devc->spent_us = 0;
		} else {
			/* If we're averaging everything - now is the time to send data */
			if (devc->avg && devc->avg_samples == 0) {
				g_hash_table_iter_init(&iter, devc->ch_ag);
				while (g_hash_table_iter_next(&iter, NULL, &value)) {
					ag = value;
					packet.type = SR_DF_ANALOG;
					packet.payload = &ag->packet;
					/* 应用耦合 (与 do_send 路径一致)。 */
					apply_analog_coupling(ag, devc, &ag->avg_val, 1);
					ag->packet.data = &ag->avg_val;
					ag->packet.num_samples = 1;
					sr_session_send(sdi, &packet);
				}
			}
			sr_info("demo_prepare_data: STOP condition met (sent_samples=%" PRIu64
				", limit_samples=%" PRIu64 ", spent_us=%" PRId64
				", limit_us=%" PRId64 ")",
				devc->sent_samples, devc->limit_samples,
				devc->spent_us, limit_us);
			sr_dev_acquisition_stop(sdi);
		}
	} else if (devc->limit_frames) {
		if (devc->sent_frame_samples == 0)
			std_session_send_df_frame_begin(sdi);
	}

	return G_SOURCE_CONTINUE;
}

/* =====================================================================
 * .demo file replay support (ported from old fork demo driver).
 * The .demo file format is a zip archive containing:
 *   - 'header'        : INI metadata (samplerate / total samples / total
 *                       blocks / total probes / probeN names)
 *   - L-<ch>/<block> : per-channel per-block logic data (byte-interleaved)
 *   - O-<ch>/0       : per-channel single-block DSO data
 *   - A-0/0          : single-channel single-block analog data
 * Files live under <user_config_dir>/demo/<mode>/<name>.demo.
 * ===================================================================== */

/* Return the demo file root directory: <user_config_dir>/demo/.
 * Caller owns the returned string (g_free). Mirrors old fork's
 * DS_USR_PATH/demo/ layout. */
SR_PRIV char *demo_get_root_dir(void)
{
	const gchar *cfg_dir;
	char *path;

	cfg_dir = g_get_user_config_dir();
	if (!cfg_dir)
		cfg_dir = ".";
	path = g_build_filename(cfg_dir, "demo", NULL);
	return path;
}

/* Close any open zip archive. Safe to call when archive is NULL. */
SR_PRIV void demo_close_archive(struct dev_context *devc)
{
	if (!devc)
		return;
	if (devc->archive) {
		unzClose(devc->archive);
		devc->archive = NULL;
	}
}

/* Scan <root>/<sub_dir>/ for *.demo files and store their basenames
 * (without the .demo suffix) into info->patterns[]. Index 0 is always
 * the static literal "random" (DEMO_GEN_RANDOM). Returns SR_OK even when
 * the directory does not exist (only "random" available in that case). */
SR_PRIV int demo_get_pattern_mode_from_file(const char *sub_dir,
	struct demo_mode_pattern *info, int max_count)
{
	char *root, *dir_path, *full, *name;
	GDir *dir;
	const char *filename;
	int num = 1;

	if (!sub_dir || !info || max_count < 1)
		return SR_ERR_ARG;

	/* Index 0 is always the literal "random" — never freed by clear_helper
	 * (which only frees indices >= 1). */
	info->patterns[0] = (char *)"random";
	info->count = 1;

	root = demo_get_root_dir();
	dir_path = g_build_filename(root, sub_dir, NULL);
	g_free(root);

	dir = g_dir_open(dir_path, 0, NULL);
	if (!dir) {
		g_free(dir_path);
		return SR_OK;  /* directory missing — only random available */
	}

	while ((filename = g_dir_read_name(dir)) != NULL && num < max_count) {
		full = g_build_filename(dir_path, filename, NULL);
		if (!g_file_test(full, G_FILE_TEST_IS_DIR) &&
		    g_str_has_suffix(filename, ".demo")) {
			/* Strip the ".demo" suffix (5 chars). */
			name = g_strndup(filename, strlen(filename) - 5);
			info->patterns[num] = name;
			num++;
		}
		g_free(full);
	}
	g_dir_close(dir);
	g_free(dir_path);
	info->count = num;
	return SR_OK;
}

/* Find the index of str in devc->demo_pattern_array[mode].patterns[].
 * Returns the matching index, or -1 if not found. */
SR_PRIV int demo_get_pattern_mode_index_by_string(struct dev_context *devc,
	int mode, const char *str)
{
	int i;
	struct demo_mode_pattern *info;

	if (!devc || !str || mode < 0 || mode > 2)
		return -1;

	info = &devc->demo_pattern_array[mode];
	for (i = 0; i < info->count; i++) {
		if (info->patterns[i] && strcmp(info->patterns[i], str) == 0)
			return i;
	}
	return -1;
}

/* Build the absolute path to the .demo file for the given pattern_mode and
 * store it in devc->demo_file_path (g_free'd before re-alloc). Returns
 * SR_ERR if pattern_mode is out of range; SR_OK otherwise. */
SR_PRIV int demo_reset_dsl_path(struct sr_dev_inst *sdi, uint8_t pattern_mode)
{
	struct dev_context *devc = sdi->priv;
	static const char *const mode_names[] = { "logic", "dso", "analog" };
	struct demo_mode_pattern *info;
	char *root, *mode_dir, *file_path = NULL;
	int mode = (int)devc->device_mode;

	g_free(devc->demo_file_path);

	if (pattern_mode != DEMO_GEN_RANDOM) {
		if (mode < 0 || mode > 2) {
			devc->demo_file_path = g_strdup("");
			return SR_ERR;
		}
		info = &devc->demo_pattern_array[mode];
		if (pattern_mode >= (uint8_t)info->count || !info->patterns[pattern_mode]) {
			devc->demo_file_path = g_strdup("");
			return SR_ERR;
		}
		root = demo_get_root_dir();
		mode_dir = g_build_filename(root, mode_names[mode], NULL);
		file_path = g_strdup_printf("%s/%s.demo",
			mode_dir, info->patterns[pattern_mode]);
		g_free(mode_dir);
		g_free(root);
		devc->demo_file_path = file_path;
	} else {
		/* Math-generated mode — no file. */
		devc->demo_file_path = g_strdup("");
	}
	return SR_OK;
}

/* First-open initialization: scan all three mode subdirectories for .demo
 * files, then pick the default LOGIC pattern (DEFAULT_LOGIC_FILE). */
SR_PRIV void demo_scan_dsl_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int dex;

	if (!devc->b_load_directory) {
		demo_get_pattern_mode_from_file("logic",
			&devc->demo_pattern_array[DEMO_MODE_LOGIC], PATTERN_COUNT);
		demo_get_pattern_mode_from_file("dso",
			&devc->demo_pattern_array[DEMO_MODE_DSO], PATTERN_COUNT);
		demo_get_pattern_mode_from_file("analog",
			&devc->demo_pattern_array[DEMO_MODE_ANALOG], PATTERN_COUNT);
		devc->b_load_directory = TRUE;
	}

	dex = demo_get_pattern_mode_index_by_string(devc, DEMO_MODE_LOGIC,
		DEFAULT_LOGIC_FILE);
	if (dex == -1)
		dex = DEMO_GEN_RANDOM;
	devc->sample_generator = (uint8_t)dex;
	devc->device_mode = DEMO_MODE_LOGIC;
	demo_reset_dsl_path(sdi, devc->sample_generator);
}

/* Open the .demo zip archive and parse its 'header' INI to populate
 * devc->cur_samplerate / total_samples / limit_samples / num_blocks /
 * num_probes. Channels are NOT rebuilt (the new design toggles ch->enabled
 * instead — see api.c config_set SR_CONF_DEVICE_MODE). If the archive
 * cannot be opened or has no header, falls back to RANDOM mode with
 * mode-specific defaults. Sets devc->load_data = TRUE on success so the
 * receive_data_*_file functions know data is ready to be read. */
SR_PRIV int demo_load_virtual_device_session(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char *metafile = NULL;
	GKeyFile *kf = NULL;
	char **sections = NULL, **keys = NULL, *val = NULL;
	gsize ngroups, nkeys;
	gsize i, j;

	demo_close_archive(devc);
	devc->cur_block = 0;

	if (devc->sample_generator != DEMO_GEN_RANDOM &&
	    devc->demo_file_path && devc->demo_file_path[0] != '\0') {
		/* Open the .demo zip archive. */
		devc->archive = unzOpen64(devc->demo_file_path);
		if (!devc->archive) {
			sr_warn("demo: failed to open '%s', falling back to random.",
				devc->demo_file_path);
			devc->sample_generator = DEMO_GEN_RANDOM;
			goto set_defaults;
		}

		/* Locate the 'header' file inside the zip. */
		if (unzLocateFile(devc->archive, "header", 0) != UNZ_OK) {
			sr_warn("demo: no 'header' in '%s', falling back to random.",
				devc->demo_file_path);
			demo_close_archive(devc);
			devc->sample_generator = DEMO_GEN_RANDOM;
			goto set_defaults;
		} else {
			unz_file_info64 info;
			if (unzGetCurrentFileInfo64(devc->archive, &info, NULL, 0,
					NULL, 0, NULL, 0) != UNZ_OK) {
				sr_warn("demo: unzGetCurrentFileInfo64 failed, "
					"falling back to random.");
				demo_close_archive(devc);
				devc->sample_generator = DEMO_GEN_RANDOM;
				goto set_defaults;
			}
			metafile = g_malloc(info.uncompressed_size + 1);
			if (!metafile) {
				demo_close_archive(devc);
				devc->sample_generator = DEMO_GEN_RANDOM;
				goto set_defaults;
			}
			unzOpenCurrentFile(devc->archive);
			unzReadCurrentFile(devc->archive, metafile,
				info.uncompressed_size);
			metafile[info.uncompressed_size] = '\0';
			unzCloseCurrentFile(devc->archive);
		}

		/* Parse the header INI. probeN keys are skipped — the new
		 * design does not rebuild channels on file load. */
		kf = g_key_file_new();
		if (g_key_file_load_from_data(kf, metafile, strlen(metafile),
				G_KEY_FILE_NONE, NULL)) {
			sections = g_key_file_get_groups(kf, &ngroups);
			for (i = 0; i < ngroups; i++) {
				keys = g_key_file_get_keys(kf, sections[i], &nkeys, NULL);
				for (j = 0; j < nkeys; j++) {
					val = g_key_file_get_value(kf, sections[i],
						keys[j], NULL);
					if (!val)
						continue;
					if (!strcmp(keys[j], "samplerate")) {
						devc->cur_samplerate =
							g_ascii_strtoull(val, NULL, 10);
					} else if (!strcmp(keys[j], "total samples")) {
						devc->total_samples =
							g_ascii_strtoull(val, NULL, 10);
						devc->limit_samples = devc->total_samples;
					} else if (!strcmp(keys[j], "total blocks")) {
						devc->num_blocks = (int)g_ascii_strtoull(
							val, NULL, 10);
					} else if (!strcmp(keys[j], "total probes")) {
						devc->num_probes = (int)g_ascii_strtoull(
							val, NULL, 10);
					}
					/* probe0/probe1/... channel-name keys
					 * intentionally ignored. */
					g_free(val);
					val = NULL;
				}
				g_strfreev(keys);
				keys = NULL;
			}
			g_strfreev(sections);
			sections = NULL;
		} else {
			sr_warn("demo: failed to parse header INI, "
				"using parsed values where available.");
		}
		g_key_file_free(kf);
		g_free(metafile);
	}

set_defaults:
	/* RANDOM fallback (or explicit random mode): set mode-specific
	 * defaults so the math generators have a sensible capture depth. */
	if (devc->sample_generator == DEMO_GEN_RANDOM) {
		switch (devc->device_mode) {
		case DEMO_MODE_LOGIC:
			devc->cur_samplerate = SR_MHZ(1);
			devc->total_samples = SR_MHZ(1);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 0;
			break;
		case DEMO_MODE_DSO:
			devc->cur_samplerate = SR_MHZ(100);
			devc->total_samples = SR_KHZ(10);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 1;
			break;
		case DEMO_MODE_ANALOG:
			devc->cur_samplerate = SR_KHZ(200);
			devc->total_samples = SR_KHZ(10);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 1;
			break;
		default:
			break;
		}
	}
	devc->load_data = TRUE;
	return SR_OK;
}

/* DSO measurement calculation (ported from old fork dso_status_update).
 * Walks the interleaved [ch0,ch1,...] DSO buffer and computes per-channel
 * max/min/high_level/low_level/cycle stats/accumulated mean/square into
 * devc->mstatus. ch0 and ch1 share the same values (the old fork's
 * behavior). Called after each SR_DF_DSO packet by demo_send_dso_packet
 * and demo_receive_data_dso_file. */
SR_PRIV void demo_dso_status_update(struct dev_context *devc,
	const uint8_t *buf, uint32_t len)
{
	struct demo_dso_measure *st;
	uint8_t ch_max = DSO_MID_VAL;
	uint8_t ch_min = DSO_MID_VAL;
	uint8_t val;
	uint32_t i;
	uint64_t total_val = 0;
	gboolean first_plevel = FALSE;
	gboolean temp_plevel = FALSE;
	uint32_t temp_llen = 0;

	if (!devc || !buf || len == 0)
		return;
	st = &devc->mstatus;

	/* Step 1: find max/min over ch0 samples (every other byte). */
	for (i = 0; i < len; i += 2) {
		val = buf[i];
		if (val > ch_max)
			ch_max = val;
		if (val < ch_min)
			ch_min = val;
	}
	st->ch0_max = st->ch1_max = ch_max;
	st->ch0_min = st->ch1_min = ch_min;
	st->ch0_high_level = st->ch1_high_level = ch_max;
	st->ch0_low_level = st->ch1_low_level = ch_min;

	/* Step 2: clear cycle stats. */
	st->ch0_cyc_tlen = st->ch1_cyc_tlen = 0;
	st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
	st->ch0_cyc_cnt = st->ch1_cyc_cnt = 0;
	st->ch0_cyc_rlen = st->ch1_cyc_rlen = 0;
	st->ch0_cyc_flen = st->ch1_cyc_flen = 0;
	st->ch0_cyc_plen = st->ch1_cyc_plen = 0;
	st->ch0_level_valid = st->ch1_level_valid = TRUE;
	st->ch0_acc_mean = st->ch1_acc_mean = 0;

	/* Step 3: scan for cycle stats. ch0 and ch1 share the same values
	 * (mirrors old fork behavior where both channels are updated in lockstep). */
	for (i = 0; i < len; i += 2) {
		val = buf[i];
		if (first_plevel) {
			if (val <= DSO_MID_VAL) {
				st->ch0_cyc_plen++;
				st->ch1_cyc_plen++;
			}
			temp_llen++;
			if (temp_plevel) {
				/* currently high — count rising edge */
				st->ch0_cyc_rlen++;
				st->ch1_cyc_rlen++;
				if (val == ch_max) {
					temp_plevel = !temp_plevel;
					if (st->ch0_plevel == temp_plevel) {
						st->ch0_cyc_cnt++;
						st->ch1_cyc_cnt++;
						st->ch0_cyc_tlen += temp_llen;
						st->ch1_cyc_tlen += temp_llen;
						st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
						temp_llen = 0;
					} else {
						st->ch0_cyc_llen = st->ch1_cyc_llen = temp_llen;
					}
				}
			} else {
				/* currently low — count falling edge */
				st->ch0_cyc_flen++;
				st->ch1_cyc_flen++;
				if (val == ch_min) {
					temp_plevel = !temp_plevel;
					if (st->ch0_plevel == temp_plevel) {
						st->ch0_cyc_cnt++;
						st->ch1_cyc_cnt++;
						st->ch0_cyc_tlen += temp_llen;
						st->ch1_cyc_tlen += temp_llen;
						st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
						temp_llen = 0;
					} else {
						st->ch0_cyc_llen = st->ch1_cyc_llen = temp_llen;
					}
				}
			}
		} else {
			/* Find first extremum to seed plevel. */
			if (val == ch_max || val == ch_min) {
				if (val == ch_max) {
					temp_plevel = FALSE;
					st->ch0_plevel = st->ch1_plevel = FALSE;
				} else {
					temp_plevel = TRUE;
					st->ch0_plevel = st->ch1_plevel = TRUE;
				}
				first_plevel = TRUE;
			}
		}
		total_val += val;
	}

	/* Step 4: rlen/flen halve (each cycle counted twice: rising+falling). */
	st->ch0_cyc_rlen /= 2;
	st->ch1_cyc_rlen = st->ch0_cyc_rlen;
	st->ch0_cyc_flen /= 2;
	st->ch1_cyc_flen = st->ch0_cyc_flen;

	/* Step 5: acc_mean / acc_square. For non-RANDOM mode the waveform is
	 * a uniform mid value, so acc_mean = mid * num_ch0_samples. For
	 * RANDOM mode, acc_mean = sum of all ch0 samples. acc_square is a
	 * rough estimate (ch_max * num_samples * 7) matching old fork. */
	if (devc->sample_generator != DEMO_GEN_RANDOM) {
		st->ch0_acc_mean = st->ch1_acc_mean =
			(uint32_t)(DSO_MID_VAL * (uint64_t)(len / 2));
	} else {
		st->ch0_acc_mean = st->ch1_acc_mean = (uint32_t)total_val;
	}
	st->ch0_acc_square = st->ch1_acc_square =
		(uint64_t)ch_max * (uint64_t)(len / 2) * 7;
	st->measure_valid = TRUE;
}

/* DSO per-channel vdiv scaling (ported from old fork receive_data_dso).
 * Walks the interleaved [ch0,ch1,...] DSO buffer and applies per-channel
 * linear compression (vdiv > 200mV) or expansion-with-clamp (vdiv <= 200mV)
 * using devc->dso_vdiv[] and devc->dso_offset[]. Called by
 * demo_send_dso_packet before sending the SR_DF_DSO packet, and by
 * demo_receive_data_dso_file after loading file data. */
SR_PRIV void demo_dso_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len)
{
	uint32_t i;
	int ch_idx;

	if (!devc || !buf || len == 0)
		return;

	for (i = 0; i < len; i++) {
		uint64_t vdiv;
		uint8_t temp_val = buf[i];
		uint16_t val, tem;

		ch_idx = (int)(i % 2);  /* 0=ch0, 1=ch1 */
		vdiv = devc->dso_vdiv[ch_idx];

		if (vdiv > SR_mV(200)) {
			/* Normal range: linear compress around mid. */
			if (temp_val > DSO_MID_VAL) {
				val = (uint8_t)(temp_val - DSO_MID_VAL);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				temp_val = (uint8_t)(DSO_MID_VAL + tem);
			} else if (temp_val < DSO_MID_VAL) {
				val = (uint8_t)(DSO_MID_VAL - temp_val);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				temp_val = (uint8_t)(DSO_MID_VAL - tem);
			}
			buf[i] = temp_val;
		} else {
			/* High sensitivity range: expand to 16-bit then clamp
			 * into the [DSO_MAX_VAL, DSO_MIN_VAL] window around
			 * (expand_mid - offset). */
			uint16_t high_gate, low_gate;
			uint16_t expand_mid;
			uint16_t expand_scale;
			/* (SR_mV(200) / vdiv) * 256 — old fork's
			 * DSO_EXPAND_MID_VAL() macro expands the 8-bit value into
			 * the 16-bit space centered on this point. */
			expand_scale = (uint16_t)(SR_mV(200) / vdiv);
			expand_mid = (uint16_t)(expand_scale * 256);
			if (temp_val > DSO_MID_VAL) {
				val = (uint8_t)(temp_val - DSO_MID_VAL);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				tem = (uint16_t)(expand_mid + tem);
			} else if (temp_val < DSO_MID_VAL) {
				val = (uint8_t)(DSO_MID_VAL - temp_val);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				tem = (uint16_t)(expand_mid - tem);
			} else {
				tem = expand_mid;
			}
			high_gate = (uint16_t)(expand_mid - devc->dso_offset[ch_idx]);
			low_gate = (uint16_t)(high_gate + DSO_LIMIT);
			if (tem <= high_gate)
				tem = DSO_MAX_VAL;
			else if (tem >= low_gate)
				tem = DSO_MIN_VAL;
			else
				tem = (uint16_t)(tem - high_gate);
			buf[i] = (uint8_t)tem;
		}
	}
}

/* ANALOG per-channel vdiv scaling (ported from old fork receive_data_analog).
 * Applies linear compression around ANALOG_MID_VAL using
 * devc->analog_vdiv[ch_idx]. Called by send_analog_packet (math mode) and
 * demo_receive_data_analog_file (file mode) before sending SR_DF_ANALOG. */
SR_PRIV void demo_analog_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len, int ch_idx)
{
	uint32_t i;
	uint64_t vdiv;

	if (!devc || !buf || len == 0 || ch_idx < 0 || ch_idx >= DSO_MAX_CHANNELS)
		return;

	vdiv = devc->analog_vdiv[ch_idx];
	if (vdiv == 0)
		return;

	for (i = 0; i < len; i++) {
		uint8_t temp_value = buf[i];
		uint8_t val;
		uint16_t tem;

		if (temp_value > ANALOG_MID_VAL) {
			val = (uint8_t)(temp_value - ANALOG_MID_VAL);
			tem = (uint16_t)((uint16_t)val *
				(uint16_t)ANALOG_DEFAULT_VDIV / (uint16_t)vdiv);
			if (tem >= ANALOG_MID_VAL)
				temp_value = ANALOG_MIN_VAL;
			else
				temp_value = (uint8_t)(ANALOG_MID_VAL + tem);
		} else if (temp_value < ANALOG_MID_VAL) {
			val = (uint8_t)(ANALOG_MID_VAL - temp_value);
			tem = (uint16_t)((uint16_t)val *
				(uint16_t)ANALOG_DEFAULT_VDIV / (uint16_t)vdiv);
			if (tem >= ANALOG_MID_VAL)
				temp_value = ANALOG_MAX_VAL;
			else
				temp_value = (uint8_t)(ANALOG_MID_VAL - tem);
		}
		buf[i] = temp_value;
	}
}

/* =====================================================================
 * .demo zip data replay callbacks.
 * Each function reads per-channel block data from the open zip archive
 * (devc->archive), byte-interleaves it into devc->packet_buffer->post_buf,
 * and sends a SR_DF_LOGIC / SR_DF_DSO / SR_DF_ANALOG packet. Called from
 * demo_prepare_data when sample_generator != DEMO_GEN_RANDOM && archive.
 * Returns G_SOURCE_CONTINUE to keep the timer alive, or stops the
 * acquisition (sr_dev_acquisition_stop) when all blocks have been read.
 * ===================================================================== */

/* Count enabled channels of the given type and fill ch_list[] with pointers
 * to them in sdi->channels order. Returns the count (0..max). */
static int demo_collect_enabled_channels(const struct sr_dev_inst *sdi,
	int channel_type, struct sr_channel **ch_list, int max)
{
	struct sr_channel *ch;
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l && n < max; l = l->next) {
		ch = l->data;
		if (ch && ch->type == channel_type && ch->enabled)
			ch_list[n++] = ch;
	}
	return n;
}

/* Ensure devc->packet_buffer exists and its post_buf is sized for
 * post_buf_len bytes. Allocates / reallocates as needed. Returns NULL on
 * allocation failure. */
static struct demo_packet_buffer *demo_ensure_packet_buffer(
	struct dev_context *devc, uint64_t post_buf_len)
{
	struct demo_packet_buffer *pb;
	int i;

	if (!devc->packet_buffer) {
		devc->packet_buffer = g_new0(struct demo_packet_buffer, 1);
		if (!devc->packet_buffer)
			return NULL;
	}
	pb = devc->packet_buffer;
	if (pb->post_buf_len != post_buf_len) {
		g_free(pb->post_buf);
		pb->post_buf = g_malloc(post_buf_len);
		if (!pb->post_buf) {
			pb->post_buf_len = 0;
			return NULL;
		}
		pb->post_buf_len = post_buf_len;
		pb->post_len = 0;
		for (i = 0; i < MAX_PROBE_NUM; i++) {
			g_free(pb->block_bufs[i]);
			pb->block_bufs[i] = NULL;
			pb->block_read_positions[i] = 0;
		}
		pb->block_data_len = 0;
		pb->block_chan_read_pos = 0;
	}
	return pb;
}

/* Read one per-channel block (file "L-<ch>/<block>") from the zip into
 * pb->block_bufs[ch]. Allocates block_bufs as needed. Returns SR_OK or
 * SR_ERR on zip failure. */
static int demo_read_logic_block(struct dev_context *devc,
	struct demo_packet_buffer *pb, int chan_num, int block_idx)
{
	char file_name[32];
	unz_file_info64 info;
	int ch_index, malloc_idx;
	int ret;

	for (ch_index = 0; ch_index < chan_num; ch_index++) {
		snprintf(file_name, sizeof(file_name), "L-%d/%d",
			ch_index, block_idx);
		if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
			sr_err("demo: can't locate zip entry '%s'.", file_name);
			return SR_ERR;
		}
		if (unzGetCurrentFileInfo64(devc->archive, &info,
				NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
			sr_err("demo: unzGetCurrentFileInfo64 failed for '%s'.",
				file_name);
			return SR_ERR;
		}
		if (ch_index == 0) {
			/* First channel sets the block size; allocate all
			 * channel block_bufs to that size. */
			if (info.uncompressed_size > pb->block_data_len) {
				for (malloc_idx = 0; malloc_idx < chan_num; malloc_idx++) {
					g_free(pb->block_bufs[malloc_idx]);
					pb->block_bufs[malloc_idx] =
						g_malloc(info.uncompressed_size + 1);
					if (!pb->block_bufs[malloc_idx])
						return SR_ERR_MALLOC;
				}
				pb->block_data_len = info.uncompressed_size;
			}
		} else if (info.uncompressed_size != pb->block_data_len) {
			sr_err("demo: block size mismatch for '%s'.", file_name);
			return SR_ERR;
		}
		if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
			sr_err("demo: can't open zip entry '%s'.", file_name);
			return SR_ERR;
		}
		ret = unzReadCurrentFile(devc->archive,
			pb->block_bufs[ch_index], pb->block_data_len);
		unzCloseCurrentFile(devc->archive);
		if (ret < 0) {
			sr_err("demo: read error for '%s'.", file_name);
			return SR_ERR;
		}
		pb->block_read_positions[ch_index] = 0;
	}
	pb->block_chan_read_pos = 0;
	return SR_OK;
}

/* LOGIC file replay: reads L-<ch>/<block> from zip, byte-interleaves 8
 * bytes per channel per unit into post_buf (LA_CROSS_DATA format), sends
 * SR_DF_LOGIC. Stops acquisition when all blocks are consumed. */
SR_PRIV int demo_receive_data_logic_decoder(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	int chan_num;
	const uint8_t byte_align = 8;  /* LOGIC: 8 bytes per channel per unit */
	uint64_t post_buf_len;
	int read_chan_idx, ret;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_LOGIC,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled logic channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* post_buf holds byte_align bytes per channel per interleave unit.
	 * Use a chunk of ~4KB worth of units (512 units per channel). */
	post_buf_len = (uint64_t)byte_align * chan_num * 64;
	pb = demo_ensure_packet_buffer(devc, post_buf_len);
	if (!pb) {
		sr_err("demo: packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* If the current block has been fully interleaved, advance to the
	 * next block (or stop if all blocks are done). */
	if (pb->block_chan_read_pos >= pb->block_data_len) {
		if (devc->cur_block >= devc->num_blocks) {
			/* All blocks read — end of stream. */
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		ret = demo_read_logic_block(devc, pb, chan_num, devc->cur_block);
		if (ret != SR_OK) {
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		devc->cur_block++;
	}

	/* Byte-interleave one chunk: byte_align bytes from each channel in
	 * turn, until post_buf is full or the block is exhausted. */
	pb->post_len = 0;
	read_chan_idx = 0;
	while (pb->post_len + byte_align <= pb->post_buf_len &&
	       pb->block_chan_read_pos + byte_align <= pb->block_data_len) {
		uint8_t *dst = (uint8_t *)pb->post_buf + pb->post_len;
		uint8_t *src = (uint8_t *)pb->block_bufs[read_chan_idx] +
			pb->block_read_positions[read_chan_idx];
		memcpy(dst, src, byte_align);
		pb->post_len += byte_align;
		pb->block_read_positions[read_chan_idx] += byte_align;
		read_chan_idx++;
		if (read_chan_idx == chan_num) {
			read_chan_idx = 0;
			pb->block_chan_read_pos += byte_align;
		}
	}

	if (pb->post_len >= (uint64_t)byte_align * chan_num) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.unitsize = 0;  /* unused for LA_CROSS_DATA */
		logic.format = LA_CROSS_DATA;
		logic.length = pb->post_len;
		logic.data = pb->post_buf;
		sr_session_send(sdi, &packet);
	}

	return G_SOURCE_CONTINUE;
}

/* DSO file replay: reads O-<ch>/0 from zip into post_buf (byte-interleaved
 * [ch0,ch1,...]), applies per-channel vdiv scaling, sends SR_DF_DSO, and
 * updates measurement stats. The data is read once (single block) and
 * re-sent each tick until limit_samples is reached. */
SR_PRIV int demo_receive_data_dso_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_dso dso;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	char file_name[32];
	unz_file_info64 info;
	int chan_num, ch_index, ret;
	uint64_t post_buf_len;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_DSO,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled DSO channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	post_buf_len = DSO_PACKET_LEN;
	pb = demo_ensure_packet_buffer(devc, post_buf_len);
	if (!pb) {
		sr_err("demo: packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Load the DSO data from the zip on first call (load_data==TRUE).
	 * DSO .demo files contain a single block per channel (O-<ch>/0). */
	if (devc->load_data) {
		for (ch_index = 0; ch_index < chan_num; ch_index++) {
			snprintf(file_name, sizeof(file_name), "O-%d/0", ch_index);
			if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
				sr_err("demo: can't locate zip entry '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (unzGetCurrentFileInfo64(devc->archive, &info,
					NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
				sr_err("demo: unzGetCurrentFileInfo64 failed.");
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (ch_index == 0) {
				if (info.uncompressed_size * chan_num > pb->post_buf_len) {
					/* Resize post_buf to fit all channels' data. */
					g_free(pb->post_buf);
					pb->post_buf = g_malloc(info.uncompressed_size * chan_num);
					if (!pb->post_buf) {
						pb->post_buf_len = 0;
						sr_dev_acquisition_stop(sdi);
						return G_SOURCE_CONTINUE;
					}
					pb->post_buf_len = info.uncompressed_size * chan_num;
				}
				pb->block_data_len = info.uncompressed_size;
			} else if (info.uncompressed_size != pb->block_data_len) {
				sr_err("demo: DSO block size mismatch for '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
				sr_err("demo: can't open zip entry '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			/* Read into post_buf at offset ch_index (byte-interleaved:
			 * [ch0_s0, ch1_s0, ch0_s1, ch1_s1, ...]). We read the whole
			 * channel block contiguously first, then de-interleave below. */
			{
				uint8_t *tmp = g_malloc(pb->block_data_len);
				if (!tmp) {
					unzCloseCurrentFile(devc->archive);
					sr_dev_acquisition_stop(sdi);
					return G_SOURCE_CONTINUE;
				}
				unzReadCurrentFile(devc->archive, tmp, pb->block_data_len);
				unzCloseCurrentFile(devc->archive);
				/* Scatter into interleaved post_buf: sample i goes to
				 * position i*chan_num + ch_index. */
				for (uint64_t i = 0; i < pb->block_data_len; i++) {
					((uint8_t *)pb->post_buf)[i * chan_num + ch_index] = tmp[i];
				}
				g_free(tmp);
			}
		}
		pb->post_len = pb->block_data_len * chan_num;
		/* Apply per-channel vdiv scaling once on load. Subsequent ticks
		 * re-send the already-scaled data. If vdiv/offset change mid-
		 * stream the user must restart acquisition to re-scale. */
		demo_dso_vdiv_scale(devc, (uint8_t *)pb->post_buf, pb->post_len);
		devc->load_data = FALSE;
	}

	/* Build and send the DSO packet. */
	dso.data = pb->post_buf;
	dso.num_samples = pb->post_len / chan_num;
	dso.trig_flag = 1;
	dso.trig_ch = 0;
	dso.en_ch_num = (uint8_t)chan_num;
	dso.sample_bits = devc->dso_unit_bits;
	dso.trig_offset = (int16_t)(dso.num_samples / 2);
	dso.packet_len = (uint32_t)pb->post_len;
	dso.samplerate_tog = (uint32_t)devc->cur_samplerate;

	packet.type = SR_DF_DSO;
	packet.payload = &dso;
	sr_session_send(sdi, &packet);

	/* Update measurement stats. */
	demo_dso_status_update(devc, (uint8_t *)pb->post_buf, pb->post_len);

	/* Single-shot: stop after one frame. Loop/instant handled by caller. */
	if (!devc->instant && !devc->loop_mode)
		sr_dev_acquisition_stop(sdi);

	return G_SOURCE_CONTINUE;
}

/* ANALOG file replay: reads A-0/0 from zip into data_buf (with vdiv scaling
 * applied per-channel), then sends a chunk cyclically each tick as
 * SR_DF_ANALOG. Stops acquisition when limit_samples is reached. */
SR_PRIV int demo_receive_data_analog_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	char file_name[32];
	unz_file_info64 info;
	int chan_num, ret;
	uint64_t chunk_len, back_len, front_len;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_ANALOG,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled analog channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Load A-0/0 from zip on first call. Expand the cyclic byte pattern
	 * to total_samples length (per-channel), apply per-channel vdiv
	 * scaling, store in devc->data_buf. */
	if (devc->load_data) {
		uint8_t *cycle_data;
		uint64_t total_buf_len;
		uint64_t per_block_after_expand;
		uint64_t cur_l;
		int ch_idx;

		snprintf(file_name, sizeof(file_name), "A-0/0");
		if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
			sr_err("demo: can't locate zip entry '%s'.", file_name);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		if (unzGetCurrentFileInfo64(devc->archive, &info,
				NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
			sr_err("demo: unzGetCurrentFileInfo64 failed.");
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		cycle_data = g_malloc(ANALOG_DATA_LEN_PER_CYCLE);
		if (!cycle_data) {
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
			g_free(cycle_data);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		ret = unzReadCurrentFile(devc->archive, cycle_data,
			ANALOG_DATA_LEN_PER_CYCLE);
		unzCloseCurrentFile(devc->archive);
		if (ret < 0) {
			g_free(cycle_data);
			sr_err("demo: read error for A-0/0.");
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}

		/* Expand: each cycle byte is repeated per_block_after_expand
		 * times, interleaved per channel. Mirrors old fork. */
		total_buf_len = (uint64_t)(ANALOG_CYCLE_RATIO *
			(double)devc->total_samples * chan_num);
		if (total_buf_len % ANALOG_DATA_LEN_PER_CYCLE != 0)
			total_buf_len = total_buf_len / ANALOG_DATA_LEN_PER_CYCLE
				* ANALOG_DATA_LEN_PER_CYCLE;
		g_free(devc->data_buf);
		devc->data_buf = g_malloc0(total_buf_len);
		if (!devc->data_buf) {
			g_free(cycle_data);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		devc->data_buf_len = total_buf_len;
		per_block_after_expand = total_buf_len / ANALOG_DATA_LEN_PER_CYCLE;

		/* Apply per-channel vdiv scaling on the cycle bytes and expand
		 * into data_buf with proper channel interleaving. */
		for (uint64_t i = 0; i < ANALOG_DATA_LEN_PER_CYCLE; i++) {
			uint8_t scaled = cycle_data[i];
			ch_idx = (int)(i % chan_num);
			demo_analog_vdiv_scale(devc, &scaled, 1, ch_idx);
			for (uint64_t j = 0; j < per_block_after_expand; j++) {
				if (i % chan_num == 0)
					cur_l = i * per_block_after_expand + j * chan_num;
				else
					cur_l = (i % chan_num) +
						(i / chan_num) * chan_num * per_block_after_expand +
						j * chan_num;
				if (cur_l < total_buf_len)
					((uint8_t *)devc->data_buf)[cur_l] = scaled;
			}
		}
		g_free(cycle_data);
		devc->load_data = FALSE;
		devc->packet_buffer = NULL;  /* will be allocated below */
	}

	/* Send a chunk cyclically from data_buf. */
	chunk_len = MIN(ANALOG_BUFSIZE, devc->data_buf_len);
	pb = demo_ensure_packet_buffer(devc, chunk_len);
	if (!pb) {
		sr_err("demo: analog packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Track read position in pb->block_chan_read_pos (reused for analog). */
	if (pb->block_chan_read_pos + chunk_len >= devc->data_buf_len) {
		back_len = devc->data_buf_len - pb->block_chan_read_pos;
		front_len = chunk_len - back_len;
		memcpy(pb->post_buf,
			(uint8_t *)devc->data_buf + pb->block_chan_read_pos, back_len);
		memcpy((uint8_t *)pb->post_buf + back_len,
			devc->data_buf, front_len);
		pb->block_chan_read_pos = front_len;
	} else {
		memcpy(pb->post_buf,
			(uint8_t *)devc->data_buf + pb->block_chan_read_pos,
			chunk_len);
		pb->block_chan_read_pos += chunk_len;
	}

	/* Send SR_DF_ANALOG with byte encoding (uint8_t samples). */
	sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	encoding.unitsize = 1;
	encoding.is_signed = FALSE;
	encoding.is_float = FALSE;
	meaning.mq = SR_MQ_VOLTAGE;
	meaning.unit = SR_UNIT_VOLT;
	meaning.mqflags = SR_MQFLAG_AC;
	meaning.channels = NULL;
	for (int i = 0; i < chan_num; i++)
		meaning.channels = g_slist_append(meaning.channels, ch_list[i]);
	analog.data = pb->post_buf;
	analog.num_samples = chunk_len / chan_num;

	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_session_send(sdi, &packet);

	g_slist_free(meaning.channels);

	/* Stop when limit_samples is reached. */
	devc->sent_samples += chunk_len / chan_num;
	if (devc->limit_samples > 0 &&
	    devc->sent_samples >= devc->limit_samples) {
		sr_dev_acquisition_stop(sdi);
	}

	return G_SOURCE_CONTINUE;
}

