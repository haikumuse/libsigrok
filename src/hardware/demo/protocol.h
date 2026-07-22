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

#ifndef LIBSIGROK_HARDWARE_DEMO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_DEMO_PROTOCOL_H

#include <stdint.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "demo"

/* The size in bytes of chunks to send through the session bus. */
#define LOGIC_BUFSIZE			4096
/* Size of the analog pattern space per channel. */
#define ANALOG_BUFSIZE			4096
/* This is a development feature: it starts a new frame every n samples. */
#define SAMPLES_PER_FRAME		1000UL
#define DEFAULT_LIMIT_FRAMES		0

#define DEFAULT_ANALOG_ENCODING_DIGITS	4
#define DEFAULT_ANALOG_SPEC_DIGITS		4
#define DEFAULT_ANALOG_AMPLITUDE		10
#define DEFAULT_ANALOG_OFFSET			0.

/* DSO defaults (ported from PXView fork demo). */
#define DEFAULT_NUM_DSO_CHANNELS		2
#define DSO_PACKET_LEN			20000
#define DSO_SAMPLE_BITS			8
#define DSO_DEFAULT_VDIV			1000
#define DSO_DEFAULT_HW_OFFSET		128
#define DSO_DEFAULT_OFFSET			128
#define DSO_DEFAULT_TRIG_VAL		128
#define DSO_DEFAULT_COUPLING		1  /* SR_DC_COUPLING */
#define DSO_MAX_CHANNELS			16

/* Logic patterns we can generate. */
enum logic_pattern_type {
	/**
	 * Spells "sigrok" across 8 channels using '0's (with '1's as
	 * "background") when displayed using the 'bits' output format.
	 * The pattern is repeated every 8 channels, shifted to the right
	 * in time by one bit.
	 */
	PATTERN_SIGROK,

	/** Pseudo-random values on all channels. */
	PATTERN_RANDOM,

	/**
	 * Incrementing number across 8 channels. The pattern is repeated
	 * every 8 channels, shifted to the right in time by one bit.
	 */
	PATTERN_INC,

	/**
	 * Single bit "walking" across all logic channels by being
	 * shifted across data lines, restarting after the last line
	 * was used. An all-zero (all-one) state is inserted to prevent
	 * repetitive patterns (e.g. with 8 data lines, every 8th state
	 * would show the same line state)
	 */
	PATTERN_WALKING_ONE,
	PATTERN_WALKING_ZERO,

	/** All channels have a low logic state. */
	PATTERN_ALL_LOW,

	/** All channels have a high logic state. */
	PATTERN_ALL_HIGH,

	/**
	 * Mimics a cable squid. Derived from the "works with" logo
	 * to occupy a larger number of channels yet "painting"
	 * something that can get recognized.
	 */
	PATTERN_SQUID,

	/** Gray encoded data, like rotary encoder signals. */
	PATTERN_GRAYCODE,
};

/* Analog patterns we can generate. */
enum analog_pattern_type {
	PATTERN_SQUARE,
	PATTERN_SINE,
	PATTERN_TRIANGLE,
	PATTERN_SAWTOOTH,
	PATTERN_ANALOG_RANDOM,
};

/* Operation mode — mirrors pxlogic's OP_BUFFER/OP_STREAM so the same
 * DeviceAgent::is_stream_mode() / samplingbar code path works for demo.
 * - OP_BUFFER: trigger-unhit buffers pre-trigger data inside soft_trigger_logic,
 *   waits for trigger to fire, then sends post-trigger data until limit_samples.
 * - OP_STREAM: continuously sends data regardless of trigger state; trigger
 *   (if any) just inserts a SR_DF_TRIGGER marker into the stream. limit_samples
 *   still acts as the stop condition (user-chosen capture depth). */
enum demo_op_mode {
	DEMO_OP_BUFFER = 0,
	DEMO_OP_STREAM = 1,
};

static const char *analog_pattern_str[] = {
	"square",
	"sine",
	"triangle",
	"sawtooth",
	"random",
};

struct analog_pattern {
	float data[ANALOG_BUFSIZE];
	unsigned int num_samples;
};

struct dev_context {
	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t limit_msec;
	uint64_t limit_frames;
	/* Operation mode: DEMO_OP_BUFFER (default) or DEMO_OP_STREAM.
	 * Set via SR_CONF_OPERATION_MODE config_set. Controls whether
	 * demo_prepare_data sends data continuously (stream) or waits
	 * for trigger (buffer). */
	enum demo_op_mode op_mode;
	uint64_t sent_samples;
	uint64_t sent_frame_samples; /* Number of samples that were sent for current frame. */
	int64_t start_us;
	int64_t spent_us;
	uint64_t step;
	/* Logic */
	int32_t num_logic_channels;
	size_t logic_unitsize;
	uint64_t all_logic_channels_mask;
	/* There is only ever one logic channel group, so its pattern goes here. */
	enum logic_pattern_type logic_pattern;
	uint8_t logic_data[LOGIC_BUFSIZE];
	/* Analog */
	struct analog_pattern *analog_patterns[ARRAY_SIZE(analog_pattern_str)];
	int32_t num_analog_channels;
	GHashTable *ch_ag;
	/* DAQ per-channel probe config (vdiv/coupling for ANALOG channels). */
	uint64_t analog_vdiv[DSO_MAX_CHANNELS];
	uint8_t analog_coupling[DSO_MAX_CHANNELS];
	gboolean avg; /* True if averaging is enabled */
	uint64_t avg_samples;
	size_t enabled_logic_channels;
	size_t enabled_analog_channels;
	size_t first_partial_logic_index;
	uint8_t first_partial_logic_mask;
	/* Triggers */
	uint64_t capture_ratio;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;
	/* DSO */
	int32_t num_dso_channels;
	size_t enabled_dso_channels;
	uint64_t dso_timebase;
	uint64_t dso_max_timebase;
	uint64_t dso_min_timebase;
	uint8_t dso_unit_bits;
	uint32_t dso_ref_min;
	uint32_t dso_ref_max;
	uint8_t dso_trig_hrate;   /* horizontal trigger position percentage */
	uint8_t dso_trig_source;
	uint8_t dso_trig_slope;
	/* per-channel DSO config (indexed by DSO channel 0..n) */
	uint64_t dso_vdiv[DSO_MAX_CHANNELS];
	uint64_t dso_vfactor[DSO_MAX_CHANNELS];
	uint16_t dso_offset[DSO_MAX_CHANNELS];
	uint16_t dso_hw_offset[DSO_MAX_CHANNELS];
	uint8_t dso_coupling[DSO_MAX_CHANNELS];
	uint8_t dso_trig_value[DSO_MAX_CHANNELS];
	gboolean dso_enabled[DSO_MAX_CHANNELS];
	/* DSO data buffer */
	uint8_t *dso_buf;
	uint64_t dso_sent_samples;
};

struct analog_gen {
	struct sr_channel *ch;
	enum sr_mq mq;
	enum sr_mqflag mq_flags;
	enum sr_unit unit;
	enum analog_pattern_type pattern;
	float amplitude;
	float offset;
	struct sr_datafeed_analog packet;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float avg_val; /* Average value */
	unsigned int num_avgs; /* Number of samples averaged */
};

SR_PRIV void demo_generate_analog_pattern(struct dev_context *devc);
SR_PRIV void demo_free_analog_pattern(struct dev_context *devc);
SR_PRIV int demo_prepare_data(int fd, int revents, void *cb_data);
SR_PRIV int demo_send_dso_packet(const struct sr_dev_inst *sdi);

#endif
