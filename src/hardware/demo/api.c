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

#define DEFAULT_NUM_LOGIC_CHANNELS		32
#define DEFAULT_LOGIC_PATTERN			PATTERN_SIGROK

#define DEFAULT_NUM_ANALOG_CHANNELS		5

/* Note: No spaces allowed because of sigrok-cli. */
static const char *logic_pattern_str[] = {
	"sigrok",
	"random",
	"incremental",
	"walking-one",
	"walking-zero",
	"all-low",
	"all-high",
	"squid",
	"graycode",
	"i2c",
	"mixed",
};

/* Operation mode strings for config_get/set/list. Indexed by enum demo_op_mode.
 * Uses the SAME string values as pxlogic ("Buffer Mode"/"Stream Mode") so
 * DeviceAgent::get_hardware_operation_mode() recognizes them without changes. */
static const char *demo_op_mode_strs[] = {
	[DEMO_OP_BUFFER] = "Buffer Mode",
	[DEMO_OP_STREAM] = "Stream Mode",
};

static const uint32_t scanopts[] = {
	SR_CONF_NUM_LOGIC_CHANNELS,
	SR_CONF_NUM_ANALOG_CHANNELS,
	SR_CONF_LIMIT_FRAMES,
};

static const uint32_t drvopts[] = {
	SR_CONF_DEMO_DEV,
	SR_CONF_LOGIC_ANALYZER,
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_AVERAGING | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_AVG_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	/* Operation mode: Buffer Mode (default) / Stream Mode.
	 * Mirrors pxlogic's SR_CONF_OPERATION_MODE so DeviceAgent's
	 * is_stream_mode() / samplingbar dropdown work unchanged. */
	SR_CONF_OPERATION_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	/* Advanced trigger configuration (PXView-local extension keys).
	 * Demo driver accepts and stores them for consistency with pxlogic,
	 * enabling API-level testing without hardware. */
	SR_CONF_TRIGGER_ADV_MODE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_ADV_ENABLE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_ADV_STAGES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_ADV_CONFIG | SR_CONF_GET | SR_CONF_SET,
	/* DSO device-level options */
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_MAX_TIMEBASE | SR_CONF_GET,
	SR_CONF_MIN_TIMEBASE | SR_CONF_GET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_INSTANT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_HW_DEPTH | SR_CONF_GET,
	SR_CONF_VLD_CH_NUM | SR_CONF_GET,
	SR_CONF_UNIT_BITS | SR_CONF_GET,
	SR_CONF_REF_MIN | SR_CONF_GET,
	SR_CONF_REF_MAX | SR_CONF_GET,
	SR_CONF_MAX_HEIGHT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* DSO max height byte value (companion to MAX_HEIGHT string). Bound as
	 * enum in deviceoptions.cpp:118 so the GUI can show the numeric value. */
	SR_CONF_MAX_HEIGHT_VALUE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* Loop capture toggle. capturemanager.cpp:197 calls set_config_bool. When
	 * TRUE, dev_acquisition_stop re-arms acquisition instead of finalizing. */
	SR_CONF_LOOP_MODE | SR_CONF_GET | SR_CONF_SET,
	/* Logic channel mode: selects channel count / max samplerate tradeoff.
	 * Mirrors pxlogic's SR_CONF_CHANNEL_MODE so the DeviceOptionsDock builds
	 * the same radio-button group. config_set enables/disables channels based
	 * on the selected mode's num_channels. */
	SR_CONF_CHANNEL_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* Logic pattern mode (sigrok/random/incremental/...). Device-level (cg=NULL)
	 * access so the DeviceOptionsDock Mode section shows a pattern dropdown.
	 * DeviceAgent::get_demo_operation_mode() reads this to detect pattern
	 * changes; set_session() writes it to restore saved state. */
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* Probe config key list (for ProbeOptions binding — DAQ/DSO map_*). */
	SR_CONF_PROBE_CONFIGS | SR_CONF_LIST,
	/* Per-channel probe keys also advertised at device level so upstream
	 * libsigrok's check_key() (hwdriver.c) accepts get_config_list(NULL,
	 * key) from ProbeOptions binding. Fork libsigrok4DSL had no check_key
	 * guard; upstream 0.6.0 requires the key appear in devopts. */
	SR_CONF_PROBE_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* PROBE_OFFSET / PROBE_HW_OFFSET advertised at device level so
	 * DeviceAgent::get_config's fork-only-key pre-check (which queries
	 * device-level devopts with cg=NULL) accepts GET calls. Without this,
	 * get_probe_offset/get_probe_hw_offset silently return false for ALL
	 * channels (DSO and ANALOG), causing DsoSignal/AnalogSignal to fall
	 * back to mid-range defaults instead of reading the driver value. */
	SR_CONF_PROBE_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_HW_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_DEFAULT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_UNIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_MAP_MIN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_MAX | SR_CONF_GET | SR_CONF_SET,
	/* DSO max sample rate (per-channel). Used by SamplingBar to clamp the
	 * timebase-derived sample rate in commit_hori_res(). */
	SR_CONF_MAX_DSO_SAMPLERATE | SR_CONF_GET,
	/* Device working mode (LOGIC/DSO/ANALOG). Mirrors fork SR_CONF_DEVICE_MODE.
	 * set toggles ch->enabled flags; get returns current mode. */
	SR_CONF_DEVICE_MODE | SR_CONF_GET | SR_CONF_SET,
	/* Capture probe count. Sets devc->num_probes for .demo replay. */
	SR_CONF_CAPTURE_NUM_PROBES | SR_CONF_SET,
	/* Load decoder flag: TRUE when sample_generator != DEMO_GEN_RANDOM. */
	SR_CONF_LOAD_DECODER | SR_CONF_GET,
	/* Zero calibration ability (always FALSE for demo). */
	SR_CONF_HAVE_ZERO | SR_CONF_GET,
	/* File block count for .demo replay. */
	SR_CONF_NUM_BLOCKS | SR_CONF_GET | SR_CONF_SET,
	/* Max DSO sample limits (SR_KHZ(20) = 20000). */
	SR_CONF_MAX_DSO_SAMPLELIMITS | SR_CONF_GET,
	/* PXLogic-compatible test keys (for UI testing without hardware).
	 * These mirror the pxlogic driver's Mode section so the user can test
	 * all device-options dock controls with the demo device. */
	SR_CONF_VTH             | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_FILTER          | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_CLOCK_EDGE      | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_CLOCK_TYPE      | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_OUT     | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_RLE             | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_EX_TRIGGER_MATCH| SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_THRESHOLD       | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFER_OPTIONS  | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BANDWIDTH_LIMIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PWM0_EN         | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PWM0_FREQ       | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PWM0_DUTY       | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PWM1_EN         | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PWM1_FREQ       | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PWM1_DUTY       | SR_CONF_GET | SR_CONF_SET,
};

static const uint32_t devopts_cg_logic[] = {
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const uint32_t devopts_cg_analog_group[] = {
	SR_CONF_AMPLITUDE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
};

static const uint32_t devopts_cg_analog_channel[] = {
	SR_CONF_MEASURED_QUANTITY | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_AMPLITUDE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	/* DAQ per-channel enable toggle (signalmodel.cpp calls set_config_bool
	 * with channel group). Mirrors DSO PROBE_EN. */
	SR_CONF_PROBE_EN | SR_CONF_GET | SR_CONF_SET,
	/* DAQ probe attenuation factor (1x/10x/100x). deviceoptions.cpp:122
	 * binds as enum. */
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET,
	/* DAQ probe controls (coupling / vdiv / scale mapping). */
	SR_CONF_PROBE_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	/* PROBE_OFFSET / PROBE_HW_OFFSET: ANALOG (DAQ) channels have no hardware
	 * offset, but AnalogSignal::set_zero_ratio and SignalModel::set_zero_offset
	 * / set_hw_offset unconditionally call set_config_uint16 for these keys.
	 * Without them in devopts_cg_analog_channel, hwdriver.c check_key() rejects
	 * with SR_ERR_ARG before the driver's config_set can accept-and-ignore.
	 * config_set returns 0 for ANALOG; config_get returns 0 for ANALOG. */
	SR_CONF_PROBE_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_HW_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_DEFAULT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_UNIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_MAP_MIN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_MAX | SR_CONF_GET | SR_CONF_SET,
};

static const uint32_t devopts_cg_dso_channel[] = {
	SR_CONF_PROBE_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_HW_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_VALUE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET,
	/* DSO per-channel enable toggle. signalmodel.cpp:159,303 calls
	 * set_config_bool(SR_CONF_PROBE_EN, enabled, ch, NULL). */
	SR_CONF_PROBE_EN | SR_CONF_GET | SR_CONF_SET,
	/* DSO pattern selection (random/sine/square/sawtooth/triangle).
	 * Bound as enum in deviceoptions.cpp:114. config_list returns
	 * dso_pattern_strs[]; config_set selects the DSO waveform generator. */
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_MAP_DEFAULT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_UNIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_MAP_MIN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_MAP_MAX | SR_CONF_GET | SR_CONF_SET,
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static const uint64_t samplerates[] = {
	SR_HZ(1),
	SR_GHZ(1),
	SR_HZ(1),
};

/* DSO vdiv list (10mV to 2V). */
static const uint64_t dso_vdivs[] = {
	SR_mV(10), SR_mV(20), SR_mV(50), SR_mV(100),
	SR_mV(200), SR_mV(500), SR_V(1), SR_V(2),
};

/* DSO/analog coupling modes. MUST be int32_t to match sr_key_info_config
 * SR_T_INT32 for SR_CONF_PROBE_COUPLING. Using uint8_t caused
 * sr_variant_type_check to reject SET calls (expected "i", got "y") →
 * coupling changes silently lost. The static_assert below enforces this at
 * compile time so a future edit cannot reintroduce the bug. */
static const int32_t dso_couplings[] = { 0 /*GND*/, 1 /*DC*/, 2 /*AC*/ };
_Static_assert(sizeof(dso_couplings[0]) == sizeof(int32_t),
	"dso_couplings must be int32_t to match SR_T_INT32 for SR_CONF_PROBE_COUPLING");

/* DSO timebase list (ns). */
static const uint64_t dso_timebases[] = {
	SR_NS(10), SR_NS(20), SR_NS(50), SR_NS(100),
	SR_NS(200), SR_NS(500), SR_US(1), SR_US(2),
	SR_US(5), SR_US(10), SR_US(20), SR_US(50),
	SR_MS(1), SR_MS(2), SR_MS(5), SR_MS(10),
	SR_MS(20), SR_MS(50), SR_MS(100), SR_MS(200),
	SR_MS(500),
};

/* DSO max height strings. */
static const char *dso_max_heights[] = { "1X", "2X", "3X", "4X", "5X" };

/* DSO max height byte values (companion to dso_max_heights[]). Returned by
 * SR_CONF_MAX_HEIGHT_VALUE — bound as enum in deviceoptions.cpp:118 so the
 * GUI can show the numeric value alongside the string. Indexes match
 * dso_max_heights[]: 1X->0x33, 2X->0x66, 3X->0x99, 4X->0xCC, 5X->0xFF. */
static const uint8_t dso_max_height_values[] = { 0x33, 0x66, 0x99, 0xCC, 0xFF };

/* DSO probe map units. */
static const char *dso_map_units[] = { "V", "A", "°C", "°F", "g", "m", "m/s" };

/* DSO pattern strings for SR_CONF_PATTERN_MODE on the DSO channel group.
 * Indexed by enum demo_dso_pattern. Mirrors old fork demo's DSO pattern set
 * so the GUI's pattern-mode dropdown shows the same options for DSO as for
 * LOGIC/ANALOG channels. */
static const char *dso_pattern_strs[] = {
	"random",
	"sine",
	"square",
	"sawtooth",
	"triangle",
};

/* --- PXLogic-compatible string arrays for UI testing --- */
/* Clock edge strings (matches pxlogic/dslogic signal_edges[]). */
static const char *demo_signal_edges[] = { "rising", "falling" };

/* Filter mode strings (matches pxlogic filter_modes[]). */
static const char *demo_filter_modes[] = { "None", "1 Sample Clock" };

/* External trigger match strings (matches pxlogic extern_trigger_match_strs[]). */
static const char *demo_extern_trig_strs[] = {
	"Close", "Rising", "One", "Falling", "Zero", "Edge",
};

/* Threshold select strings (matches pxlogic/dslogic threshold selections). */
static const char *demo_threshold_strs[] = {
	"1.8V", "2.5V", "3.3V", "5.0V",
};

/* Buffer option strings. */
static const char *demo_buffer_options_strs[] = {
	"Buffer Mode", "Stream Mode",
};

/* Bandwidth limit strings. */
static const char *demo_bw_limit_strs[] = {
	"20M", "30M", "40M", "50M", "60M", "70M", "80M", "90M", "100M",
};

/* Logic channel-mode descriptor table. Each entry trades channel count for
 * max samplerate. SR_CONF_CHANNEL_MODE (string config) selects among these.
 * Mirrors old fork demo's logic_channel_modes[] but simplified to the fields
 * the new GUI actually consumes (descr string + max_samplerate for clamp).
 * The string is what config_list returns via g_variant_new_strv and what
 * config_get returns for the current mode. */
struct demo_logic_channel_mode {
	enum demo_logic_channel_id id;
	enum demo_logic_channel_index index;
	uint16_t num_channels;
	uint64_t max_samplerate;
	const char *descr;
};
/* Logic channel-mode descriptor table. Mirrors pxlogic's buffer-mode
 * channel modes so the demo device can simulate real hardware tradeoffs
 * between channel count and max samplerate. SR_CONF_CHANNEL_MODE (string
 * config) selects among these. */
static const struct demo_logic_channel_mode logic_channel_modes[] = {
	{ DEMO_LOGIC250x32,  LOGIC250x32,  32, SR_MHZ(250), "Use 32 Channels (Max 250MHz)" },
	{ DEMO_LOGIC250x16,  LOGIC250x16,  16, SR_MHZ(250), "Use 16 Channels (Max 250MHz)" },
	{ DEMO_LOGIC500x16,  LOGIC500x16,  16, SR_MHZ(500), "Use 16 Channels (Max 500MHz)" },
	{ DEMO_LOGIC1000x8,  LOGIC1000x8,   8, SR_GHZ(1),   "Use 8 Channels (Max 1000MHz)" },
};

/* String array view of logic_channel_modes[].descr for std_str_idx validation
 * in config_set and g_variant_new_strv in config_list. */
static const char *logic_channel_mode_strs[ARRAY_SIZE(logic_channel_modes)] = {
	[LOGIC250x32]  = "Use 32 Channels (Max 250MHz)",
	[LOGIC250x16]  = "Use 16 Channels (Max 250MHz)",
	[LOGIC500x16]  = "Use 16 Channels (Max 500MHz)",
	[LOGIC1000x8]  = "Use 8 Channels (Max 1000MHz)",
};

/* Probe config keys exposed via SR_CONF_PROBE_CONFIGS for ProbeOptions
 * binding. Applies to both DSO and ANALOG (DAQ) channels — the binding
 * creates widgets for VDIV/COUPLING/MAP_* keys. */
static const int32_t probe_configs[] = {
SR_CONF_PROBE_VDIV,
SR_CONF_PROBE_COUPLING,
SR_CONF_PROBE_MAP_DEFAULT,
SR_CONF_PROBE_MAP_UNIT,
SR_CONF_PROBE_MAP_MIN,
SR_CONF_PROBE_MAP_MAX,
SR_CONF_PATTERN_MODE,
};

/* Clamp cur_samplerate to the current logic channel-mode's max. Called from
 * config_set on SR_CONF_CHANNEL_MODE so switching to fewer channels (higher
 * max samplerate) or more channels (lower max) keeps the rate in range. */
SR_PRIV void logic_adjust_samplerate(struct dev_context *devc)
{
	uint64_t max_sr;

	if (!devc)
		return;
	max_sr = logic_channel_modes[devc->logic_ch_mode_index].max_samplerate;
	if (devc->cur_samplerate > max_sr) {
		sr_info("demo: clamp cur_samplerate %" PRIu64 " -> %" PRIu64
			" (channel mode '%s')",
			devc->cur_samplerate, max_sr,
			logic_channel_modes[devc->logic_ch_mode_index].descr);
		devc->cur_samplerate = max_sr;
	}
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_channel *ch;
	struct sr_channel_group *cg, *acg;
	struct sr_config *src;
	struct analog_gen *ag;
	GSList *l;
	int num_logic_channels, num_analog_channels, num_dso_channels, pattern, i;
	uint64_t limit_frames;
	char channel_name[16];

	num_logic_channels = DEFAULT_NUM_LOGIC_CHANNELS;
	num_analog_channels = DEFAULT_NUM_ANALOG_CHANNELS;
	num_dso_channels = DEFAULT_NUM_DSO_CHANNELS;
	limit_frames = DEFAULT_LIMIT_FRAMES;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_NUM_LOGIC_CHANNELS:
			num_logic_channels = g_variant_get_int32(src->data);
			break;
		case SR_CONF_NUM_ANALOG_CHANNELS:
			num_analog_channels = g_variant_get_int32(src->data);
			break;
		case SR_CONF_LIMIT_FRAMES:
			limit_frames = g_variant_get_uint64(src->data);
			break;
		}
	}

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->status = SR_ST_INACTIVE;
	sdi->model = g_strdup("Demo device");

	devc = g_malloc0(sizeof(struct dev_context));
	devc->cur_samplerate = SR_KHZ(200);
	devc->num_logic_channels = num_logic_channels;
	devc->logic_unitsize = (devc->num_logic_channels + 7) / 8;
	devc->all_logic_channels_mask = 1UL << 0;
	devc->all_logic_channels_mask <<= devc->num_logic_channels;
	devc->all_logic_channels_mask--;
	devc->logic_pattern = DEFAULT_LOGIC_PATTERN;
	devc->num_analog_channels = num_analog_channels;
	devc->limit_frames = limit_frames;
	devc->capture_ratio = 20;
	devc->trig_adv_mode = 0;      /* Simple */
	devc->trig_adv_enable = FALSE;
	devc->trig_adv_stages = 0;
	devc->trig_adv_config = NULL;
	devc->stl = NULL;
	/* Default to Buffer Mode (trigger-aware, finite capture). User can
	 * switch to Stream Mode via the samplingbar OPERATION_MODE dropdown. */
	devc->op_mode = DEMO_OP_BUFFER;

	/* DSO initialization. */
	devc->num_dso_channels = num_dso_channels;
	devc->enabled_dso_channels = 0;
	devc->dso_unit_bits = DSO_SAMPLE_BITS;
	devc->dso_ref_min = 1;
	devc->dso_ref_max = 255;
	devc->dso_timebase = SR_NS(500);
	devc->dso_max_timebase = SR_MS(500);
	devc->dso_min_timebase = SR_NS(10);
	devc->dso_trig_hrate = 0;
	devc->dso_trig_source = 0;
	devc->dso_trig_slope = 0;
	devc->dso_buf = NULL;
	devc->dso_sent_samples = 0;
	/* DSO pattern + config-change regen flags (Tier A). Default pattern is
	 * random so demo behavior matches old fork demo's default until the user
	 * picks sine/square/sawtooth/triangle via the DSO PATTERN_MODE dropdown.
	 * Per-channel: each DSO channel can have a different waveform shape. */
	for (int i = 0; i < DSO_MAX_CHANNELS; i++)
		devc->dso_pattern[i] = DEMO_DSO_PATTERN_RANDOM;
	devc->dso_vdiv_change = FALSE;
	devc->dso_offset_change = FALSE;
	devc->dso_timebase_change = FALSE;
	devc->instant = FALSE;
	devc->loop_mode = FALSE;
	/* Logic channel-mode default: 32ch@250M (widest channel count, mirrors
	 * pxlogic's BUFFER_LOGIC250x32 default). The GUI shows a radio-button
	 * group built from SR_CONF_CHANNEL_MODE config_list; user can switch to
	 * 16ch@250M / 16ch@500M / 8ch@1G for higher max samplerate. */
	devc->logic_ch_mode = DEMO_LOGIC250x32;
	devc->logic_ch_mode_index = LOGIC250x32;
	/* Analog random cyclic buffer (allocated lazily by init_analog_random_data
	 * when PATTERN_ANALOG_RANDOM is first selected on an ANALOG channel). */
	devc->analog_random_buf = NULL;
	devc->analog_random_buf_len = 0;
	devc->analog_random_read_pos = 0;
	/* DAQ (ANALOG) per-channel defaults. */
	for (i = 0; i < num_analog_channels && i < DSO_MAX_CHANNELS; i++) {
		devc->analog_vdiv[i] = DSO_DEFAULT_VDIV;
		devc->analog_coupling[i] = DSO_DEFAULT_COUPLING;
		devc->analog_map_default[i] = TRUE;  /* map fields disabled until user unchecks */
	}
	for (i = 0; i < num_dso_channels && i < DSO_MAX_CHANNELS; i++) {
		devc->dso_vdiv[i] = DSO_DEFAULT_VDIV;
		devc->dso_vfactor[i] = 1;  /* 1x probe attenuation factor (not vdiv). */
		devc->dso_offset[i] = DSO_DEFAULT_OFFSET;
		devc->dso_hw_offset[i] = DSO_DEFAULT_HW_OFFSET;
		devc->dso_coupling[i] = DSO_DEFAULT_COUPLING;
		devc->dso_trig_value[i] = DSO_DEFAULT_TRIG_VAL;
		devc->dso_enabled[i] = TRUE;
	}

	/* .demo replay + device-mode state init (Tier B). g_malloc0 already
	 * zeroed everything; explicit init here keeps the contract clear.
	 * demo_file_path MUST be g_strdup("") rather than NULL because
	 * demo_reset_dsl_path safe_free's it before re-allocating. */
	devc->device_mode = DEMO_MODE_LOGIC;
	devc->sample_generator = DEMO_GEN_RANDOM;
	devc->num_blocks = 0;
	devc->cur_block = 0;
	devc->total_samples = 0;
	devc->trig_pos = 0;
	devc->data_buf = NULL;
	devc->data_buf_len = 0;
	devc->load_data = TRUE;
	devc->vdiv_change = FALSE;
	devc->offset_change = FALSE;
	devc->timebase_change = FALSE;
	devc->b_load_directory = FALSE;
	devc->packet_buffer = NULL;
	devc->demo_file_path = g_strdup("");
	devc->num_probes = num_logic_channels;
	devc->archive = NULL;
	devc->mstatus.measure_valid = TRUE;

	/* --- PXLogic-compatible test field defaults --- */
	devc->vth = 2.0;
	devc->filter = 0;
	devc->clock_edge = 0;
	devc->clock_type = FALSE;
	devc->trig_out_en = FALSE;
	devc->rle = FALSE;
	devc->ext_trig_mode = 0;
	devc->threshold_sel = 2; /* 3.3V */
	devc->buffer_options = 0;
	devc->bw_limit = 0;
	devc->pwm0_en = FALSE;
	devc->pwm0_freq = 1000.0;
	devc->pwm0_duty = 50.0;
	devc->pwm1_en = FALSE;
	devc->pwm1_freq = 1000.0;
	devc->pwm1_duty = 50.0;

	/* Simulated hardware memory depth: 4 billion total samples (matches
	 * pxlogic's hw_depth = SR_Gn(4)). HW_DEPTH returns this / ch_num,
	 * bounding the sample depth dropdown like real hardware. */
	devc->simulated_hw_depth = (uint64_t)4000000000ULL;

	/* Seed xorshift32 PRNG with a non-zero value (xorshift requires
	 * a non-zero seed). Use wall-clock time for per-run variability,
	 * falling back to a fixed seed if time is 0. */
	devc->prng_state = (uint32_t)(g_get_monotonic_time() & 0xFFFFFFFF);
	if (devc->prng_state == 0)
		devc->prng_state = 0x12345678;

	if (num_logic_channels > 0) {
		/* Logic channels, all in one channel group. */
		cg = sr_channel_group_new(sdi, "Logic", NULL);
		for (i = 0; i < num_logic_channels; i++) {
			sprintf(channel_name, "D%d", i);
			ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
			cg->channels = g_slist_append(cg->channels, ch);
		}
	}

	/* Analog channels, channel groups and pattern generators. */
	devc->ch_ag = g_hash_table_new(g_direct_hash, g_direct_equal);
	if (num_analog_channels > 0) {
		/*
		 * Have the waveform for analog patterns pre-generated. It's
		 * supposed to be periodic, so the generator just needs to
		 * access the prepared sample data (DDS style).
		 */
		demo_generate_analog_pattern(devc);

		/* An "Analog" channel group with all analog channels in it. */
		acg = sr_channel_group_new(sdi, "Analog", NULL);

		for (i = 0; i < num_analog_channels; i++) {
			/* Default per-channel patterns: ch0=sine, ch1=square so the
			 * user sees channel 1 as sine and channel 2 as square wave.
			 * Channels 2+ cycle through triangle/sawtooth/random. */
			if (i == 0)
				pattern = PATTERN_SINE;
			else if (i == 1)
				pattern = PATTERN_SQUARE;
			else
				pattern = i % ARRAY_SIZE(analog_pattern_str);

			snprintf(channel_name, 16, "A%d", i);
			ch = sr_channel_new(sdi, i + num_logic_channels, SR_CHANNEL_ANALOG,
					TRUE, channel_name);
			acg->channels = g_slist_append(acg->channels, ch);

			/* Every analog channel gets its own channel group as well. */
			cg = sr_channel_group_new(sdi, channel_name, NULL);
			cg->channels = g_slist_append(NULL, ch);

			/* Every channel gets a generator struct. */
			ag = g_malloc(sizeof(struct analog_gen));
			ag->ch = ch;
			ag->mq = SR_MQ_VOLTAGE;
			ag->mq_flags = SR_MQFLAG_DC;
			ag->unit = SR_UNIT_VOLT;
			ag->amplitude = DEFAULT_ANALOG_AMPLITUDE;
			ag->offset = DEFAULT_ANALOG_OFFSET;
			sr_analog_init(&ag->packet, &ag->encoding, &ag->meaning, &ag->spec, 2);
			ag->packet.meaning->channels = cg->channels;
			ag->packet.meaning->mq = ag->mq;
			ag->packet.meaning->mqflags = ag->mq_flags;
			ag->packet.meaning->unit = ag->unit;
			ag->packet.encoding->digits = DEFAULT_ANALOG_ENCODING_DIGITS;
			ag->packet.spec->spec_digits = DEFAULT_ANALOG_SPEC_DIGITS;
			ag->packet.data = devc->analog_patterns[pattern];
			ag->pattern = pattern;
			ag->avg_val = 0.0f;
			ag->num_avgs = 0;
			ag->ac_prev_input = 0.0f;
			ag->ac_prev_output = 0.0f;
			g_hash_table_insert(devc->ch_ag, ch, ag);
		}
	}

	/* DSO channels, each in its own channel group for per-channel config. */
	if (num_dso_channels > 0) {
		for (i = 0; i < num_dso_channels; i++) {
			snprintf(channel_name, 16, "O%d", i);
			ch = sr_channel_new(sdi,
				i + num_logic_channels + num_analog_channels,
				SR_CHANNEL_DSO, TRUE, channel_name);
			/* Each DSO channel gets its own channel group so
			 * per-channel vdiv/coupling/offset can be addressed. */
			cg = sr_channel_group_new(sdi, channel_name, NULL);
			cg->channels = g_slist_append(NULL, ch);
		}
	}

	sdi->priv = devc;

	return std_scan_complete(di, g_slist_append(NULL, sdi));
}

static void clear_helper(struct dev_context *devc)
{
	GHashTableIter iter;
	void *value;

	demo_free_analog_pattern(devc);

	/* Analog generators. */
	g_hash_table_iter_init(&iter, devc->ch_ag);
	while (g_hash_table_iter_next(&iter, NULL, &value))
		g_free(value);
	g_hash_table_unref(devc->ch_ag);

	/* DSO scratch buffer. */
	g_free(devc->dso_buf);
	devc->dso_buf = NULL;

	/* Analog random cyclic buffer (allocated by init_analog_random_data). */
	g_free(devc->analog_random_buf);
	devc->analog_random_buf = NULL;
	devc->analog_random_buf_len = 0;
	devc->analog_random_read_pos = 0;

	/* .demo file replay resources (Tier B). */
	demo_close_archive(devc);
	g_free(devc->demo_file_path);
	g_free(devc->data_buf);
	if (devc->packet_buffer) {
		int i;
		g_free(devc->packet_buffer->post_buf);
		for (i = 0; i < MAX_PROBE_NUM; i++)
			g_free(devc->packet_buffer->block_bufs[i]);
		g_free(devc->packet_buffer);
	}
	/* demo_pattern_array: per-mode pattern file name list. Index 0 is the
	 * static literal "random" (not g_strdup'd); indices >=1 are g_strdup'd
	 * by demo_get_pattern_mode_from_file(). */
	for (int m = 0; m < 3; m++) {
		for (int i = 1; i < devc->demo_pattern_array[m].count; i++)
			g_free(devc->demo_pattern_array[m].patterns[i]);
	}
}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di, (std_dev_clear_callback)clear_helper);
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	struct analog_gen *ag;
	GVariant *mq_arr[2];
	GSList *l;
	int pattern;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES: {
		/* 纯 DSO 模式下帧大小固定为 DSO_PACKET_LEN（demo_send_dso_packet 每帧
		 * 发送 DSO_PACKET_LEN 样本）。返回帧大小而非 devc->limit_samples，
		 * 让前端 get_sample_limit()/cur_samplelimits 返回正确的帧大小，
		 * 使 cur_snap_sampletime = 帧时间，get_max_offset 基于帧时间计算
		 * offset 范围。这与 DSView DSL 硬件驱动 DSO 模式下强制
		 * limit_samples = dso_depth/num_channels（帧大小）的做法一致。
		 *
		 * 但在 MSO 模式（DSO + logic/analog 同时启用）下，analog/logic 通道
		 * 按 devc->limit_samples 发送样本。若此处仍返回 DSO_PACKET_LEN，
		 * AnalogSnapshot 的 _total_sample_count 会小于实际接收的样本数，
		 * 导致 ring buffer 回绕后 _sample_count 变成回绕余数（如 32），
		 * envelope 仅计算极少样本，模拟波形无法显示。
		 *
		 * 注意：不能用 devc->device_mode 判断，因为 PXView 的 set_work_mode
		 * 对非 DSL 设备（demo/file）不调 SR_CONF_DEVICE_MODE SET，所以
		 * devc->device_mode 永远是默认 DEMO_MODE_LOGIC。改用检查是否有
		 * enabled DSO channel 来判断当前是否 DSO 模式（SigSession::
		 * switch_work_mode 切换模式时会 enable 对应类型通道）。 */
		gboolean dso_active = FALSE;
		gboolean other_active = FALSE;
		for (GSList *l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = l->data;
			if (!ch->enabled)
				continue;
			if (ch->type == SR_CHANNEL_DSO)
				dso_active = TRUE;
			else
				other_active = TRUE;
		}
		/* 仅纯 DSO 模式（DSO 是唯一启用的通道类型）返回 DSO_PACKET_LEN。
		 * MSO 模式（DSO + logic/analog）返回 limit_samples，让 analog/logic
		 * snapshot 的 ring buffer 大小匹配实际发送的样本数。 */
		if (dso_active && !other_active)
			*data = g_variant_new_uint64(DSO_PACKET_LEN);
		else
			*data = g_variant_new_uint64(devc->limit_samples);
		break;
	}
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(devc->limit_msec);
		break;
	case SR_CONF_LIMIT_FRAMES:
		*data = g_variant_new_uint64(devc->limit_frames);
		break;
	case SR_CONF_OPERATION_MODE:
		/* Return current op_mode as string ("Buffer Mode"/"Stream Mode").
		 * DeviceAgent::get_hardware_operation_mode() converts back to
		 * LO_OP_BUFFER/LO_OP_STREAM for is_stream_mode() checks. */
		*data = g_variant_new_string(demo_op_mode_strs[devc->op_mode]);
		break;
	case SR_CONF_AVERAGING:
		*data = g_variant_new_boolean(devc->avg);
		break;
	case SR_CONF_AVG_SAMPLES:
		*data = g_variant_new_uint64(devc->avg_samples);
		break;
	case SR_CONF_MEASURED_QUANTITY:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		/* Any channel in the group will do. */
		ch = cg->channels->data;
		if (ch->type != SR_CHANNEL_ANALOG)
			return SR_ERR_ARG;
		ag = g_hash_table_lookup(devc->ch_ag, ch);
		mq_arr[0] = g_variant_new_uint32(ag->mq);
		mq_arr[1] = g_variant_new_uint64(ag->mq_flags);
		*data = g_variant_new_tuple(mq_arr, 2);
		break;
	case SR_CONF_PATTERN_MODE:
		if (!cg) {
			/* Device-level query: return the current pattern based on
			 * the active work mode (LOGIC/DSO/ANALOG). This lets the
			 * DeviceOptionsDock Mode dropdown show DSO patterns when
			 * in DSO mode, logic patterns when in LOGIC mode, etc.
			 * DeviceAgent::get_demo_operation_mode() also reads this. */
			if (devc->device_mode == DEMO_MODE_DSO)
				*data = g_variant_new_string(
					dso_pattern_strs[devc->dso_pattern[0]]);
			else if (devc->device_mode == DEMO_MODE_ANALOG) {
				/* Return first analog channel's pattern as representative. */
				for (l = sdi->channels; l; l = l->next) {
					ch = l->data;
					if (ch && ch->type == SR_CHANNEL_ANALOG) {
						ag = g_hash_table_lookup(devc->ch_ag, ch);
						if (ag) {
							*data = g_variant_new_string(
									analog_pattern_str[ag->pattern]);
							break;
						}
					}
				}
			} else
				*data = g_variant_new_string(
					logic_pattern_str[devc->logic_pattern]);
			break;
		}
		/* Any channel in the group will do. */
		ch = cg->channels->data;
		if (ch->type == SR_CHANNEL_LOGIC) {
			pattern = devc->logic_pattern;
			*data = g_variant_new_string(logic_pattern_str[pattern]);
		} else if (ch->type == SR_CHANNEL_ANALOG) {
			ag = g_hash_table_lookup(devc->ch_ag, ch);
			pattern = ag->pattern;
			*data = g_variant_new_string(analog_pattern_str[pattern]);
		} else if (ch->type == SR_CHANNEL_DSO) {
			/* DSO pattern (random/sine/square/sawtooth/triangle).
			 * Per-channel: each DSO channel can have its own waveform shape.
			 * Compute the DSO-specific index by subtracting logic and analog
			 * channel counts, same as PROBE_VDIV/etc. The old code used the
			 * global channel list position (which includes 32+ logic/analog
			 * channels), so dso_idx was always >= DSO_MAX_CHANNELS and got
			 * clamped to 0 — all DSO channels read channel 0's pattern. */
			int dso_idx = ch->index - devc->num_logic_channels
				- devc->num_analog_channels;
			if (dso_idx < 0 || dso_idx >= DSO_MAX_CHANNELS)
				dso_idx = 0;
			*data = g_variant_new_string(dso_pattern_strs[devc->dso_pattern[dso_idx]]);
		} else
			return SR_ERR_BUG;
		break;
	case SR_CONF_AMPLITUDE:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		/* Any channel in the group will do. */
		ch = cg->channels->data;
		if (ch->type != SR_CHANNEL_ANALOG)
			return SR_ERR_ARG;
		ag = g_hash_table_lookup(devc->ch_ag, ch);
		*data = g_variant_new_double(ag->amplitude);
		break;
	case SR_CONF_OFFSET:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		/* Any channel in the group will do. */
		ch = cg->channels->data;
		if (ch->type != SR_CHANNEL_ANALOG)
			return SR_ERR_ARG;
		ag = g_hash_table_lookup(devc->ch_ag, ch);
		*data = g_variant_new_double(ag->offset);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_TRIGGER_ADV_MODE:
		*data = g_variant_new_byte(devc->trig_adv_mode);
		break;
	case SR_CONF_TRIGGER_ADV_ENABLE:
		*data = g_variant_new_boolean(devc->trig_adv_enable);
		break;
	case SR_CONF_TRIGGER_ADV_STAGES:
		*data = g_variant_new_byte(devc->trig_adv_stages);
		break;
	case SR_CONF_TRIGGER_ADV_CONFIG:
		*data = g_variant_new_string(devc->trig_adv_config ? devc->trig_adv_config : "");
		break;
	/* --- DSO device-level config --- */
	case SR_CONF_TIMEBASE:
		*data = g_variant_new_uint64(devc->dso_timebase);
		break;
	case SR_CONF_MAX_TIMEBASE:
		*data = g_variant_new_uint64(devc->dso_max_timebase);
		break;
	case SR_CONF_MIN_TIMEBASE:
		*data = g_variant_new_uint64(devc->dso_min_timebase);
		break;
	case SR_CONF_TRIGGER_SOURCE:
		*data = g_variant_new_byte(devc->dso_trig_source);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		*data = g_variant_new_byte(devc->dso_trig_slope);
		break;
	case SR_CONF_HORIZ_TRIGGERPOS:
		/* hwdriver.c declares this key as SR_T_FLOAT (GVariant 'd' double).
		 * Other drivers (yokogawa-dlm, siglent-sds, rigol-ds) all use double.
		 * The value is a percentage 0-100 stored in dso_trig_hrate (uint8_t). */
		*data = g_variant_new_double((double)devc->dso_trig_hrate);
		break;
	case SR_CONF_INSTANT:
		/* Instant mode: when TRUE, demo_send_dso_packet sends progressive
		 * packets based on elapsed time rather than one full frame per
		 * tick. Set via deviceoptions.cpp:156 bool binding. */
		*data = g_variant_new_boolean(devc->instant);
		break;
	case SR_CONF_LOOP_MODE:
		/* Loop capture toggle. capturemanager.cpp:197 calls set_config_bool.
		 * When TRUE, dev_acquisition_stop re-arms acquisition. */
		*data = g_variant_new_boolean(devc->loop_mode);
		break;
	case SR_CONF_CHANNEL_MODE:
		/* Logic channel-mode string. deviceoptions.cpp/mainwindow.cpp/
		 * session_service.cpp call get_config_string. */
		*data = g_variant_new_string(
			logic_channel_modes[devc->logic_ch_mode_index].descr);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		/* Byte value companion to MAX_HEIGHT string. deviceoptions.cpp:118
		 * binds as enum. Demo default is 1X (index 0). */
		*data = g_variant_new_byte(dso_max_height_values[0]);
		break;
	case SR_CONF_HW_DEPTH:
		/* Simulated hardware storage depth (samples per channel).
		 * - DSO: DSO_PACKET_LEN (20k samples per frame)
		 * - Logic: simulated_hw_depth / ch_num, matching pxlogic's
		 *   hw_depth / ch_num formula. This bounds the sample depth
		 *   dropdown by the simulated hardware memory (4G samples),
		 *   not by the current limit_samples (which was a circular
		 *   dependency — the dropdown upper bound changed with the
		 *   current selection). In Stream mode the frontend uses
		 *   SR_CONF_STREAM_MEM_BUFF instead, so this value is only
		 *   used in Buffer mode (same as pxlogic). */
		if (devc->num_dso_channels > 0 && devc->num_logic_channels == 0
				&& devc->num_analog_channels == 0)
			*data = g_variant_new_uint64(DSO_PACKET_LEN);
		else {
			size_t ch_num = devc->enabled_logic_channels;
			if (ch_num == 0)
				ch_num = devc->num_logic_channels;
			if (ch_num == 0)
				ch_num = 1;
			*data = g_variant_new_uint64(
				devc->simulated_hw_depth / ch_num);
		}
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int32(devc->num_dso_channels);
		break;
	case SR_CONF_UNIT_BITS:
		*data = g_variant_new_byte(devc->dso_unit_bits);
		break;
	case SR_CONF_REF_MIN:
		*data = g_variant_new_uint32(devc->dso_ref_min);
		break;
	case SR_CONF_REF_MAX:
		*data = g_variant_new_uint32(devc->dso_ref_max);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(dso_max_heights[0]);
		break;
	case SR_CONF_MAX_DSO_SAMPLERATE:
		/* Per-channel max DSO sample rate. Demo device advertises 200 MHz
		 * (matches libsigrok4DSL demo driver). Used by SamplingBar to
		 * clamp commit_hori_res() output. */
		*data = g_variant_new_uint64(SR_MHZ(200));
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_int16((int16_t)devc->device_mode);
		break;
	case SR_CONF_LOAD_DECODER:
		*data = g_variant_new_boolean(devc->sample_generator != DEMO_GEN_RANDOM);
		break;
	case SR_CONF_HAVE_ZERO:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_NUM_BLOCKS:
		*data = g_variant_new_uint64((uint64_t)devc->num_blocks);
		break;
	case SR_CONF_MAX_DSO_SAMPLELIMITS:
		*data = g_variant_new_uint64(SR_KHZ(20));
		break;
	/* --- DSO per-channel config --- */
	case SR_CONF_PROBE_VDIV:
	case SR_CONF_PROBE_COUPLING:
	case SR_CONF_TRIGGER_VALUE:
	case SR_CONF_PROBE_OFFSET:
	case SR_CONF_PROBE_HW_OFFSET:
	case SR_CONF_PROBE_FACTOR:
	case SR_CONF_PROBE_EN:
	case SR_CONF_PROBE_MAP_DEFAULT:
	case SR_CONF_PROBE_MAP_UNIT:
	case SR_CONF_PROBE_MAP_MIN:
	case SR_CONF_PROBE_MAP_MAX:
	{
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ch = cg->channels->data;
		/* MAP keys apply to both DSO (oscilloscope) and ANALOG (DAQ)
		 * channels. DSO-only keys (VDIV/COUPLING/etc.) are rejected
		 * below; MAP keys fall through to the shared default-value
		 * branch so DAQ mode gets the same scale-mapping controls. */
		gboolean is_dso = (ch->type == SR_CHANNEL_DSO);
		gboolean is_analog = (ch->type == SR_CHANNEL_ANALOG);
		if (!is_dso && !is_analog)
			return SR_ERR_ARG;
		int idx = -1;
		int aidx = -1;
		if (is_dso) {
			idx = ch->index - devc->num_logic_channels
				- devc->num_analog_channels;
			if (idx < 0 || idx >= devc->num_dso_channels)
				return SR_ERR_ARG;
		}
		if (is_analog) {
			aidx = ch->index - devc->num_logic_channels;
			if (aidx < 0 || aidx >= devc->num_analog_channels
					|| aidx >= DSO_MAX_CHANNELS)
				return SR_ERR_ARG;
		}
		switch (key) {
		case SR_CONF_PROBE_VDIV:
			if (is_dso)
				*data = g_variant_new_uint64(devc->dso_vdiv[idx]);
			else
				*data = g_variant_new_uint64(devc->analog_vdiv[aidx]);
			break;
		case SR_CONF_PROBE_COUPLING:
		/* 返回 int32 ("i") 以匹配 sr_key_info_config SR_T_INT32。
		 * sr_variant_type_check 在 SET 时验证类型, byte 会被拒绝。
		 * ProbeOptions::print_coupling 改用 g_variant_get(gvar,"i",...)。 */
		if (is_dso)
			*data = g_variant_new_int32((int32_t)devc->dso_coupling[idx]);
		else
			*data = g_variant_new_int32((int32_t)devc->analog_coupling[aidx]);
		break;
		case SR_CONF_TRIGGER_VALUE:
			if (!is_dso) return SR_ERR_ARG;
			*data = g_variant_new_int32((int32_t)devc->dso_trig_value[idx]);
			break;
		case SR_CONF_PROBE_OFFSET:
		/* ANALOG (DAQ) has no hardware offset — return 0 (consistent with
		 * config_set accepting-but-ignoring for ANALOG). */
		if (!is_dso) {
			*data = g_variant_new_uint16(0);
			break;
		}
		*data = g_variant_new_uint16(devc->dso_offset[idx]);
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		/* ANALOG (DAQ) has no hardware offset — return 0. */
		if (!is_dso) {
			*data = g_variant_new_uint16(0);
			break;
		}
		*data = g_variant_new_uint16(devc->dso_hw_offset[idx]);
		break;
		case SR_CONF_PROBE_FACTOR:
			/* DSO uses dso_vfactor[]; ANALOG (DAQ) returns fixed 1x. */
			if (is_dso)
				*data = g_variant_new_uint64(devc->dso_vfactor[idx]);
			else
				*data = g_variant_new_uint64(1);
			break;
		case SR_CONF_PROBE_EN:
			/* Per-channel enable toggle. signalmodel.cpp:159,303 calls
			 * set_config_bool with channel group. DSO uses dso_enabled[];
			 * ANALOG always enabled (no per-channel disable state stored). */
			if (is_dso)
				*data = g_variant_new_boolean(devc->dso_enabled[idx]);
			else
				*data = g_variant_new_boolean(TRUE);
			break;
		case SR_CONF_PROBE_MAP_DEFAULT:
			/* 返回每通道存储的状态, 使 UI 能在用户取消 "map default"
			 * 复选框后启用 map unit/min/max 字段。DSO 通道无独立存储,
			 * 保留旧行为 (恒 TRUE)。 */
			if (is_analog)
				*data = g_variant_new_boolean(devc->analog_map_default[aidx]);
			else
				*data = g_variant_new_boolean(TRUE);
			break;
		case SR_CONF_PROBE_MAP_UNIT:
			*data = g_variant_new_string(dso_map_units[0]);
			break;
		case SR_CONF_PROBE_MAP_MIN:
			*data = g_variant_new_double(-5.0);
			break;
		case SR_CONF_PROBE_MAP_MAX:
			*data = g_variant_new_double(5.0);
			break;
		}
		break;
	}
	/* --- PXLogic-compatible test keys (device-level) --- */
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_string(demo_filter_modes[devc->filter]);
		break;
	case SR_CONF_CLOCK_EDGE:
		*data = g_variant_new_string(demo_signal_edges[devc->clock_edge]);
		break;
	case SR_CONF_CLOCK_TYPE:
		*data = g_variant_new_boolean(devc->clock_type);
		break;
	case SR_CONF_TRIGGER_OUT:
		*data = g_variant_new_boolean(devc->trig_out_en);
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(devc->rle);
		break;
	case SR_CONF_EX_TRIGGER_MATCH:
		*data = g_variant_new_string(demo_extern_trig_strs[devc->ext_trig_mode]);
		break;
	case SR_CONF_THRESHOLD:
		*data = g_variant_new_string(demo_threshold_strs[devc->threshold_sel]);
		break;
	case SR_CONF_BUFFER_OPTIONS:
		*data = g_variant_new_string(demo_buffer_options_strs[devc->buffer_options]);
		break;
	case SR_CONF_BANDWIDTH_LIMIT:
		*data = g_variant_new_string(demo_bw_limit_strs[devc->bw_limit]);
		break;
	case SR_CONF_PWM0_EN:
		*data = g_variant_new_boolean(devc->pwm0_en);
		break;
	case SR_CONF_PWM0_FREQ:
		*data = g_variant_new_double(devc->pwm0_freq);
		break;
	case SR_CONF_PWM0_DUTY:
		*data = g_variant_new_double(devc->pwm0_duty);
		break;
	case SR_CONF_PWM1_EN:
		*data = g_variant_new_boolean(devc->pwm1_en);
		break;
	case SR_CONF_PWM1_FREQ:
		*data = g_variant_new_double(devc->pwm1_freq);
		break;
	case SR_CONF_PWM1_DUTY:
		*data = g_variant_new_double(devc->pwm1_duty);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* Type-check helper for config_set entry points. Verifies the GVariant type
 * matches the expected type string (e.g. "i" for int32, "t" for uint64).
 * sr_variant_type_check in hwdriver.c already validates types before the
 * driver's config_set runs, but this provides a driver-level diagnostic
 * with the key name and expected/actual types for immediate identification
 * of type mismatches during development. Returns SR_OK on match, SR_ERR_ARG
 * on mismatch. */
static int demo_check_gvar_type(uint32_t key, GVariant *data,
	const char *expected_type)
{
	const GVariantType *actual;

	if (!data || !expected_type)
		return SR_ERR_ARG;

	actual = g_variant_get_type(data);
	if (!actual) {
		sr_err("demo: config_set key %u: GVariant has no type", key);
		return SR_ERR_ARG;
	}

	if (!g_variant_type_equal(actual, G_VARIANT_TYPE(expected_type))) {
		gchar *actual_str = g_variant_type_dup_string(actual);
		sr_err("demo: config_set key %u: type mismatch — expected '%s', got '%s'",
			key, expected_type, actual_str);
		g_free(actual_str);
		return SR_ERR_ARG;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct analog_gen *ag;
	struct sr_channel *ch;
	GVariant *mq_tuple_child;
	GSList *l;
	int logic_pattern, analog_pattern;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		devc->cur_samplerate = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_msec = 0;
		devc->limit_samples = g_variant_get_uint64(data);
		/* Align to 64 bytes to match pxlogic's hardware sample alignment.
		 * pxlogic does: (limit_samples + 63) & ~63. Without this, tests
		 * that set non-aligned sample counts get different actual counts
		 * between demo and pxlogic (e.g. 1000 → pxlogic=1024, demo=1000). */
		devc->limit_samples = (devc->limit_samples + 63) & ~63;
		break;
	case SR_CONF_LIMIT_MSEC:
		devc->limit_msec = g_variant_get_uint64(data);
		devc->limit_samples = 0;
		break;
	case SR_CONF_LIMIT_FRAMES:
		devc->limit_frames = g_variant_get_uint64(data);
		break;
	case SR_CONF_OPERATION_MODE: {
		/* Validate via std_str_idx against demo_op_mode_strs[].
		 * Returns DEMO_OP_BUFFER=0 / DEMO_OP_STREAM=1. */
		int idx = std_str_idx(data, ARRAY_AND_SIZE(demo_op_mode_strs));
		if (idx < 0)
			return SR_ERR_ARG;
		devc->op_mode = (enum demo_op_mode)idx;
		sr_info("demo: set OPERATION_MODE='%s' (op_mode=%d)",
		        demo_op_mode_strs[idx], idx);
		break;
	}
	case SR_CONF_AVERAGING:
		devc->avg = g_variant_get_boolean(data);
		sr_dbg("%s averaging", devc->avg ? "Enabling" : "Disabling");
		break;
	case SR_CONF_AVG_SAMPLES:
		devc->avg_samples = g_variant_get_uint64(data);
		sr_dbg("Setting averaging rate to %" PRIu64, devc->avg_samples);
		break;
	case SR_CONF_MEASURED_QUANTITY:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		for (l = cg->channels; l; l = l->next) {
			ch = l->data;
			if (ch->type != SR_CHANNEL_ANALOG)
				return SR_ERR_ARG;
			ag = g_hash_table_lookup(devc->ch_ag, ch);
			mq_tuple_child = g_variant_get_child_value(data, 0);
			ag->mq = g_variant_get_uint32(mq_tuple_child);
			mq_tuple_child = g_variant_get_child_value(data, 1);
			ag->mq_flags = g_variant_get_uint64(mq_tuple_child);
			g_variant_unref(mq_tuple_child);
		}
		break;
	case SR_CONF_PATTERN_MODE:
		if (!cg) {
			/* Device-level set: set the pattern based on current work
			 * mode. In DSO mode, sets the DSO waveform pattern
			 * (sine/square/sawtooth/triangle/random). In LOGIC mode,
			 * sets the logic pattern. In ANALOG mode, sets all analog
			 * channels' pattern. */
			if (devc->device_mode == DEMO_MODE_DSO) {
				int dso_pat = std_str_idx(data,
					ARRAY_AND_SIZE(dso_pattern_strs));
				if (dso_pat < 0)
					return SR_ERR_ARG;
				sr_dbg("Setting DSO pattern to %s",
						dso_pattern_strs[dso_pat]);
				for (int i = 0; i < DSO_MAX_CHANNELS; i++)
					devc->dso_pattern[i] = (enum demo_dso_pattern)dso_pat;
				devc->dso_vdiv_change = TRUE;
			} else if (devc->device_mode == DEMO_MODE_ANALOG) {
				analog_pattern = std_str_idx(data,
					ARRAY_AND_SIZE(analog_pattern_str));
				if (analog_pattern < 0)
					return SR_ERR_ARG;
				sr_dbg("Setting analog pattern to %s (first channel only, "
						"preserving per-channel defaults for others)",
						analog_pattern_str[analog_pattern]);
				/* Only set the first analog channel's pattern at device
				 * level. Each analog channel has its own default pattern
				 * (square/sine/triangle/sawtooth/random) set during scan.
				 * Setting all channels to the same pattern would override
				 * these defaults. Per-channel pattern changes should use
				 * the channel-group SET path (cg != NULL). */
				for (l = sdi->channels; l; l = l->next) {
					ch = l->data;
					if (ch && ch->type == SR_CHANNEL_ANALOG) {
						ag = g_hash_table_lookup(devc->ch_ag, ch);
						if (ag)
							ag->pattern = analog_pattern;
						break;  /* only first channel */
					}
				}
			} else {
				logic_pattern = std_str_idx(data,
					ARRAY_AND_SIZE(logic_pattern_str));
				if (logic_pattern < 0)
					return SR_ERR_ARG;
				sr_dbg("Setting logic pattern to %s",
						logic_pattern_str[logic_pattern]);
				devc->logic_pattern = logic_pattern;
				if (logic_pattern == PATTERN_ALL_LOW)
					memset(devc->logic_data, 0x00, LOGIC_BUFSIZE);
				else if (logic_pattern == PATTERN_ALL_HIGH)
					memset(devc->logic_data, 0xff, LOGIC_BUFSIZE);
			}
			break;
		}
		logic_pattern = std_str_idx(data, ARRAY_AND_SIZE(logic_pattern_str));
		analog_pattern = std_str_idx(data, ARRAY_AND_SIZE(analog_pattern_str));
		{
			/* DSO pattern string lookup (random/sine/square/sawtooth/triangle).
			 * Validated separately because the DSO cg only carries DSO channels. */
			int dso_pattern = std_str_idx(data, ARRAY_AND_SIZE(dso_pattern_strs));
			if (logic_pattern < 0 && analog_pattern < 0 && dso_pattern < 0)
				return SR_ERR_ARG;
			/* If any DSO channel is in this group, treat as DSO pattern set. */
			ch = cg->channels->data;
			if (ch->type == SR_CHANNEL_DSO) {
				if (dso_pattern < 0)
					return SR_ERR_ARG;
				/* Compute DSO-specific index (same fix as config_get).
				 * Old code used global channel list position, which was
				 * always >= DSO_MAX_CHANNELS → clamped to 0 → all writes
				 * went to dso_pattern[0]. */
				int dso_idx = ch->index - devc->num_logic_channels
					- devc->num_analog_channels;
				if (dso_idx < 0 || dso_idx >= DSO_MAX_CHANNELS)
					dso_idx = 0;
				sr_dbg("Setting DSO ch%d pattern to %s", dso_idx, dso_pattern_strs[dso_pattern]);
				devc->dso_pattern[dso_idx] = (enum demo_dso_pattern)dso_pattern;
				/* Trigger waveform regeneration on next packet send. */
				devc->dso_vdiv_change = TRUE;
				break;
			}
		}
		for (l = cg->channels; l; l = l->next) {
			ch = l->data;
			if (ch->type == SR_CHANNEL_LOGIC) {
				if (logic_pattern == -1)
					return SR_ERR_ARG;
				sr_dbg("Setting logic pattern to %s",
						logic_pattern_str[logic_pattern]);
				devc->logic_pattern = logic_pattern;
				/* Might as well do this now, these are static. */
				if (logic_pattern == PATTERN_ALL_LOW)
					memset(devc->logic_data, 0x00, LOGIC_BUFSIZE);
				else if (logic_pattern == PATTERN_ALL_HIGH)
					memset(devc->logic_data, 0xff, LOGIC_BUFSIZE);
			} else if (ch->type == SR_CHANNEL_ANALOG) {
				if (analog_pattern == -1)
					return SR_ERR_ARG;
				sr_dbg("Setting analog pattern for channel %s to %s",
						ch->name, analog_pattern_str[analog_pattern]);
				ag = g_hash_table_lookup(devc->ch_ag, ch);
				ag->pattern = analog_pattern;
			} else
				return SR_ERR_BUG;
		}
		break;
	case SR_CONF_AMPLITUDE:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		for (l = cg->channels; l; l = l->next) {
			ch = l->data;
			if (ch->type != SR_CHANNEL_ANALOG)
				return SR_ERR_ARG;
			ag = g_hash_table_lookup(devc->ch_ag, ch);
			ag->amplitude = g_variant_get_double(data);
		}
		break;
	case SR_CONF_OFFSET:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		for (l = cg->channels; l; l = l->next) {
			ch = l->data;
			if (ch->type != SR_CHANNEL_ANALOG)
				return SR_ERR_ARG;
			ag = g_hash_table_lookup(devc->ch_ag, ch);
			ag->offset = g_variant_get_double(data);
		}
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_TRIGGER_ADV_MODE:
		devc->trig_adv_mode = g_variant_get_byte(data);
		break;
	case SR_CONF_TRIGGER_ADV_ENABLE:
		devc->trig_adv_enable = g_variant_get_boolean(data);
		break;
	case SR_CONF_TRIGGER_ADV_STAGES:
		devc->trig_adv_stages = g_variant_get_byte(data);
		break;
	case SR_CONF_TRIGGER_ADV_CONFIG:
		g_free(devc->trig_adv_config);
		{
			const char *cfg_str = g_variant_get_string(data, NULL);
			devc->trig_adv_config = g_strdup(cfg_str ? cfg_str : "");
		}
		break;
	/* --- DSO device-level config --- */
	case SR_CONF_TIMEBASE:
		devc->dso_timebase = g_variant_get_uint64(data);
		/* Mark DSO waveform for regeneration on next packet send. */
		devc->dso_timebase_change = TRUE;
		break;
	case SR_CONF_TRIGGER_SOURCE:
		devc->dso_trig_source = g_variant_get_byte(data);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		devc->dso_trig_slope = g_variant_get_byte(data);
		break;
	case SR_CONF_HORIZ_TRIGGERPOS:
		/* Accept double (SR_T_FLOAT) from the GUI, store as uint8_t 0-100. */
		devc->dso_trig_hrate = (uint8_t)g_variant_get_double(data);
		break;
	case SR_CONF_INSTANT:
		/* Store instant flag — demo_send_dso_packet reads it to switch
		 * between progressive time-based sending (instant=TRUE) and
		 * one-full-frame-per-tick sending (instant=FALSE). */
		devc->instant = g_variant_get_boolean(data);
		sr_dbg("demo: set INSTANT=%d", devc->instant);
		break;
	case SR_CONF_LOOP_MODE:
		/* Loop capture toggle. capturemanager.cpp:197 calls set_config_bool.
		 * When TRUE, dev_acquisition_stop re-arms acquisition instead of
		 * finalizing, so capture repeats until user stops. */
		devc->loop_mode = g_variant_get_boolean(data);
		sr_info("demo: set LOOP_MODE=%d", devc->loop_mode);
		break;
	case SR_CONF_DEVICE_MODE: {
		int new_mode = g_variant_get_int16(data);
		struct sr_channel *ch;
		GSList *l;
		/* Toggle channel enabled flags based on new mode. For LOGIC mode,
		 * respect the current channel-mode's num_channels limit (e.g. if
		 * the user selected "Use 16 Channels", only 16 of 32 logic channels
		 * should be enabled). */
		uint16_t logic_limit = (new_mode == DEMO_MODE_LOGIC)
			? logic_channel_modes[devc->logic_ch_mode_index].num_channels
			: 0;
		int logic_seen = 0;
		for (l = sdi->channels; l; l = l->next) {
			ch = l->data;
			switch (new_mode) {
			case DEMO_MODE_LOGIC:
				if (ch->type == SR_CHANNEL_LOGIC) {
					ch->enabled = (logic_seen < logic_limit);
					logic_seen++;
				} else {
					ch->enabled = FALSE;
				}
				break;
			case DEMO_MODE_DSO:
				ch->enabled = (ch->type == SR_CHANNEL_DSO);
				break;
			case DEMO_MODE_ANALOG:
				ch->enabled = (ch->type == SR_CHANNEL_ANALOG);
				break;
			}
		}
		devc->device_mode = (enum demo_device_mode)new_mode;
		/* Try to find default .demo file for the new mode. */
		{
			int nv = demo_get_pattern_mode_index_by_string(devc, new_mode,
				new_mode == DEMO_MODE_LOGIC ? DEFAULT_LOGIC_FILE :
				new_mode == DEMO_MODE_DSO ? DEFAULT_DSO_FILE : DEFAULT_ANALOG_FILE);
			if (nv != -1)
				devc->sample_generator = (uint8_t)nv;
			else
				devc->sample_generator = DEMO_GEN_RANDOM;
		}
		demo_reset_dsl_path((struct sr_dev_inst *)sdi, devc->sample_generator);
		demo_load_virtual_device_session((struct sr_dev_inst *)sdi);
		break;
	}
	case SR_CONF_CAPTURE_NUM_PROBES:
		devc->num_probes = (int)g_variant_get_uint64(data);
		break;
	case SR_CONF_NUM_BLOCKS:
		devc->num_blocks = (int)g_variant_get_uint64(data);
		sr_dbg("Setting block number to %d.", devc->num_blocks);
		break;
	case SR_CONF_CHANNEL_MODE: {
		/* Logic channel-mode selection (string). Validate via std_str_idx
		 * against logic_channel_mode_strs[]; set index + id, then clamp
		 * cur_samplerate to the mode's max via logic_adjust_samplerate().
		 *
		 * Only toggle logic channel enabled flags when in LOGIC mode. In
		 * DSO/ANALOG mode, config restore may set CHANNEL_MODE from a
		 * saved .pxc file — re-enabling logic channels here would break
		 * the DSO data path (demo_prepare_data checks has_enabled_other
		 * and skips SR_DF_DSO if any non-DSO channel is enabled). The
		 * mode is stored for when the user switches back to LOGIC. */
		int idx = std_str_idx(data, ARRAY_AND_SIZE(logic_channel_mode_strs));
		if (idx < 0)
			return SR_ERR_ARG;
		devc->logic_ch_mode_index = (enum demo_logic_channel_index)idx;
		devc->logic_ch_mode = logic_channel_modes[idx].id;
		sr_info("demo: set CHANNEL_MODE='%s' (index=%d, device_mode=%d)",
		        logic_channel_modes[idx].descr, idx, devc->device_mode);
		if (devc->device_mode == DEMO_MODE_LOGIC) {
			uint16_t mode_channels = logic_channel_modes[idx].num_channels;
			int ch_idx = 0;
			for (GSList *l = sdi->channels; l; l = l->next) {
				struct sr_channel *ch = l->data;
				if (ch->type == SR_CHANNEL_LOGIC) {
					ch->enabled = (ch_idx < mode_channels);
					ch_idx++;
				}
			}
		}
		logic_adjust_samplerate(devc);
		break;
	}
	case SR_CONF_MAX_HEIGHT:
		/* Accept but ignore; demo always uses 1X. */
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		/* Accept byte value; demo stores index 0 (1X) as the only real
		 * state. The GUI binds this as enum alongside MAX_HEIGHT string. */
	sr_dbg("demo: set MAX_HEIGHT_VALUE (ignored, demo uses 1X)");
	break;
/* --- Probe mapping keys (accept but ignore; demo uses fixed map values) --- */
/* Separated from the DSO per-channel block below so the audit script
 * does not misattribute g_variant_get_* calls from other keys. */
case SR_CONF_PROBE_MAP_UNIT:
case SR_CONF_PROBE_MAP_MIN:
case SR_CONF_PROBE_MAP_MAX:
	break;
/* --- Probe map default toggle (needs channel setup) --- */
case SR_CONF_PROBE_MAP_DEFAULT:
	{
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ch = cg->channels->data;
		if (ch->type == SR_CHANNEL_ANALOG) {
			int aidx = ch->index - devc->num_logic_channels;
			if (aidx < 0 || aidx >= devc->num_analog_channels
					|| aidx >= DSO_MAX_CHANNELS)
				return SR_ERR_ARG;
			/* 存储每通道 map_default 状态, 使后续 GET 返回用户选择。 */
			devc->analog_map_default[aidx] = g_variant_get_boolean(data);
		}
		/* DSO 通道仍忽略 (无独立存储)。 */
		break;
	}
/* --- DSO per-channel config --- */
case SR_CONF_PROBE_VDIV:
case SR_CONF_PROBE_COUPLING:
case SR_CONF_TRIGGER_VALUE:
case SR_CONF_PROBE_OFFSET:
case SR_CONF_PROBE_HW_OFFSET:
case SR_CONF_PROBE_FACTOR:
case SR_CONF_PROBE_EN:
	{
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ch = cg->channels->data;
		/* MAP keys apply to DSO and ANALOG (DAQ) channels. */
		gboolean is_dso = (ch->type == SR_CHANNEL_DSO);
		gboolean is_analog = (ch->type == SR_CHANNEL_ANALOG);
		if (!is_dso && !is_analog)
			return SR_ERR_ARG;
		int idx = -1;
		int aidx = -1;
		if (is_dso) {
			idx = ch->index - devc->num_logic_channels
				- devc->num_analog_channels;
			if (idx < 0 || idx >= devc->num_dso_channels)
				return SR_ERR_ARG;
		}
		if (is_analog) {
			aidx = ch->index - devc->num_logic_channels;
			if (aidx < 0 || aidx >= devc->num_analog_channels
					|| aidx >= DSO_MAX_CHANNELS)
				return SR_ERR_ARG;
		}
		switch (key) {
		case SR_CONF_PROBE_VDIV:
			if (is_dso) {
				devc->dso_vdiv[idx] = g_variant_get_uint64(data);
				/* Trigger DSO waveform regeneration (vdiv affects amplitude). */
				devc->dso_vdiv_change = TRUE;
			} else
				devc->analog_vdiv[aidx] = g_variant_get_uint64(data);
			break;
		case SR_CONF_PROBE_COUPLING:
		/* PROBE_COUPLING expects int32 ("i") per sr_key_info_config SR_T_INT32.
		 * demo_check_gvar_type provides a driver-level diagnostic if the type
		 * is wrong (sr_variant_type_check in hwdriver.c should catch it first,
		 * but this gives an immediate driver-context error message). */
		if (demo_check_gvar_type(key, data, "i") != SR_OK)
			return SR_ERR_ARG;
		{
			int32_t cv = g_variant_get_int32(data);
			if (is_dso)
				devc->dso_coupling[idx] = (uint8_t)cv;
			else
				devc->analog_coupling[aidx] = (uint8_t)cv;
		}
		break;
		case SR_CONF_TRIGGER_VALUE:
			if (!is_dso) return SR_ERR_ARG;
			devc->dso_trig_value[idx] = (uint8_t)g_variant_get_int32(data);
			break;
		case SR_CONF_PROBE_OFFSET:
		/* DSO stores dso_offset[] + triggers regeneration; ANALOG (DAQ) has
		 * no hardware offset concept — accept but ignore (consistent with
		 * PROBE_MAP_UNIT/MIN/MAX). Without this, AnalogSignal::set_zero_ratio
		 * and commit_settings() would log SR_ERR_ARG warnings for every
		 * ANALOG channel on every drag. */
		if (!is_dso) break;
		devc->dso_offset[idx] = g_variant_get_uint16(data);
		/* Trigger DSO waveform regeneration (offset affects vertical shift). */
		devc->dso_offset_change = TRUE;
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		/* DSO stores dso_hw_offset[]; ANALOG accepts but ignores. */
		if (!is_dso) break;
		devc->dso_hw_offset[idx] = g_variant_get_uint16(data);
		break;
		case SR_CONF_PROBE_FACTOR:
			/* DSO uses dso_vfactor[]; ANALOG (DAQ) accepts but ignores
			 * (no per-channel factor state stored for analog). */
			if (is_dso)
				devc->dso_vfactor[idx] = g_variant_get_uint64(data);
			break;
		case SR_CONF_PROBE_EN:
			/* Per-channel enable toggle. signalmodel.cpp:159,303 calls
			 * set_config_bool. DSO stores in dso_enabled[]; ANALOG accepts
			 * silently (channels are enabled at scan time). */
			if (is_dso)
				devc->dso_enabled[idx] = g_variant_get_boolean(data);
			break;
		}
		break;
	}
	/* --- PXLogic-compatible test keys (device-level) --- */
	case SR_CONF_VTH:
		devc->vth = g_variant_get_double(data);
		break;
	case SR_CONF_FILTER: {
		int fidx = std_str_idx(data, ARRAY_AND_SIZE(demo_filter_modes));
		if (fidx < 0)
			return SR_ERR_ARG;
		devc->filter = fidx;
		break;
	}
	case SR_CONF_CLOCK_EDGE: {
		int eidx = std_str_idx(data, ARRAY_AND_SIZE(demo_signal_edges));
		if (eidx < 0)
			return SR_ERR_ARG;
		devc->clock_edge = eidx;
		break;
	}
	case SR_CONF_CLOCK_TYPE:
		devc->clock_type = g_variant_get_boolean(data);
		break;
	case SR_CONF_TRIGGER_OUT:
		devc->trig_out_en = g_variant_get_boolean(data);
		break;
	case SR_CONF_RLE:
		devc->rle = g_variant_get_boolean(data);
		break;
	case SR_CONF_EX_TRIGGER_MATCH: {
		int tidx = std_str_idx(data, ARRAY_AND_SIZE(demo_extern_trig_strs));
		if (tidx < 0)
			return SR_ERR_ARG;
		devc->ext_trig_mode = tidx;
		break;
	}
	case SR_CONF_THRESHOLD: {
		int thidx = std_str_idx(data, ARRAY_AND_SIZE(demo_threshold_strs));
		if (thidx < 0)
			return SR_ERR_ARG;
		devc->threshold_sel = thidx;
		break;
	}
	case SR_CONF_BUFFER_OPTIONS: {
		int bidx = std_str_idx(data, ARRAY_AND_SIZE(demo_buffer_options_strs));
		if (bidx < 0)
			return SR_ERR_ARG;
		devc->buffer_options = bidx;
		break;
	}
	case SR_CONF_BANDWIDTH_LIMIT: {
		int bwidx = std_str_idx(data, ARRAY_AND_SIZE(demo_bw_limit_strs));
		if (bwidx < 0)
			return SR_ERR_ARG;
		devc->bw_limit = bwidx;
		break;
	}
	case SR_CONF_PWM0_EN:
		devc->pwm0_en = g_variant_get_boolean(data);
		break;
	case SR_CONF_PWM0_FREQ:
		devc->pwm0_freq = g_variant_get_double(data);
		break;
	case SR_CONF_PWM0_DUTY:
		devc->pwm0_duty = g_variant_get_double(data);
		break;
	case SR_CONF_PWM1_EN:
		devc->pwm1_en = g_variant_get_boolean(data);
		break;
	case SR_CONF_PWM1_FREQ:
		devc->pwm1_freq = g_variant_get_double(data);
		break;
	case SR_CONF_PWM1_DUTY:
		devc->pwm1_duty = g_variant_get_double(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct sr_channel *ch;
	struct dev_context *devc;

	if (!sdi)
		return SR_ERR_ARG;
	devc = sdi->priv;

	if (!cg) {
		switch (key) {
		case SR_CONF_SCAN_OPTIONS:
		case SR_CONF_DEVICE_OPTIONS:
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		case SR_CONF_SAMPLERATE:
			*data = std_gvar_samplerates_steps(ARRAY_AND_SIZE(samplerates));
			break;
		case SR_CONF_OPERATION_MODE:
			/* Expose Buffer/Stream strings for the samplingbar dropdown.
			 * Same string values as pxlogic so DeviceAgent code is shared. */
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_op_mode_strs));
			break;
		case SR_CONF_TRIGGER_MATCH:
			*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
			break;
		case SR_CONF_TIMEBASE:
			*data = std_gvar_array_u64(ARRAY_AND_SIZE(dso_timebases));
			break;
		case SR_CONF_MAX_HEIGHT:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(dso_max_heights));
			break;
		case SR_CONF_MAX_HEIGHT_VALUE:
			/* Byte values companion to MAX_HEIGHT strings. Bound as enum
			 * in deviceoptions.cpp:118. */
			*data = g_variant_new_fixed_array(G_VARIANT_TYPE("y"),
				dso_max_height_values, ARRAY_SIZE(dso_max_height_values),
				sizeof(uint8_t));
			break;
		case SR_CONF_CHANNEL_MODE:
			/* Logic channel-mode string list. deviceoptions.cpp:380 reads
			 * via g_variant_get_strv and builds a radio-button group.
			 * deviceoptionsdock.cpp does the same for the dock variant. */
			*data = g_variant_new_strv(ARRAY_AND_SIZE(logic_channel_mode_strs));
			break;
		case SR_CONF_PROBE_CONFIGS:
			/* Returns the list of probe-config keys supported by
			 * this driver. ProbeOptions binding iterates this list
			 * and creates widgets for VDIV/COUPLING/MAP_* keys —
			 * used by both DSO (oscilloscope) and ANALOG (DAQ)
			 * channels. */
			*data = std_gvar_array_i32(ARRAY_AND_SIZE(probe_configs));
			break;
		case SR_CONF_PROBE_VDIV:
			/* Return dict {"vdivs": [uint64...]} — ProbeOptions binding
			 * extracts via g_variant_lookup_value("vdivs"). */
			{
				GVariantBuilder gvb;
				g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
				GVariant *gv = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
					dso_vdivs, ARRAY_SIZE(dso_vdivs), sizeof(uint64_t));
				g_variant_builder_add(&gvb, "{sv}", "vdivs", gv);
				*data = g_variant_builder_end(&gvb);
			}
			break;
		case SR_CONF_PROBE_COUPLING:
		/* Return dict {"coupling": [int32...]} — ProbeOptions binding
		 * extracts via g_variant_lookup_value("coupling"). int32 matches
		 * sr_key_info_config SR_T_INT32 so SET passes type check. */
		{
			GVariantBuilder gvb;
			g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
			GVariant *gv = g_variant_new_fixed_array(G_VARIANT_TYPE("i"),
				dso_couplings, ARRAY_SIZE(dso_couplings), sizeof(int32_t));
			g_variant_builder_add(&gvb, "{sv}", "coupling", gv);
			*data = g_variant_builder_end(&gvb);
		}
		break;
		case SR_CONF_PROBE_MAP_UNIT:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(dso_map_units));
			break;
		case SR_CONF_PATTERN_MODE:
			/* Device-level: return pattern strings based on current
			 * work mode so the Mode dropdown shows the correct list.
			 * DSO mode shows dso_pattern_strs (sine/square/...),
			 * ANALOG mode shows analog_pattern_str,
			 * LOGIC mode shows logic_pattern_str. */
			if (devc->device_mode == DEMO_MODE_DSO)
				*data = g_variant_new_strv(
					ARRAY_AND_SIZE(dso_pattern_strs));
			else if (devc->device_mode == DEMO_MODE_ANALOG)
				*data = g_variant_new_strv(
					ARRAY_AND_SIZE(analog_pattern_str));
			else
				*data = g_variant_new_strv(
					ARRAY_AND_SIZE(logic_pattern_str));
			break;
		/* --- PXLogic-compatible list keys --- */
		case SR_CONF_FILTER:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_filter_modes));
			break;
		case SR_CONF_CLOCK_EDGE:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_signal_edges));
			break;
		case SR_CONF_EX_TRIGGER_MATCH:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_extern_trig_strs));
			break;
		case SR_CONF_THRESHOLD:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_threshold_strs));
			break;
		case SR_CONF_BUFFER_OPTIONS:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_buffer_options_strs));
			break;
		case SR_CONF_BANDWIDTH_LIMIT:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(demo_bw_limit_strs));
			break;
		default:
			return SR_ERR_NA;
		}
	} else {
		ch = cg->channels->data;
		switch (key) {
		case SR_CONF_DEVICE_OPTIONS:
			if (ch->type == SR_CHANNEL_LOGIC)
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_logic));
			else if (ch->type == SR_CHANNEL_ANALOG) {
				if (strcmp(cg->name, "Analog") == 0)
					*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_group));
				else
					*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_channel));
			}
			else if (ch->type == SR_CHANNEL_DSO)
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_dso_channel));
			else
				return SR_ERR_BUG;
			break;
		case SR_CONF_PATTERN_MODE:
			/* The analog group (with all 4 channels) shall not have a pattern property. */
			if (strcmp(cg->name, "Analog") == 0)
				return SR_ERR_NA;

			if (ch->type == SR_CHANNEL_LOGIC)
				*data = g_variant_new_strv(ARRAY_AND_SIZE(logic_pattern_str));
			else if (ch->type == SR_CHANNEL_ANALOG)
				*data = g_variant_new_strv(ARRAY_AND_SIZE(analog_pattern_str));
			else if (ch->type == SR_CHANNEL_DSO)
				/* DSO pattern list (random/sine/square/sawtooth/triangle). */
				*data = g_variant_new_strv(ARRAY_AND_SIZE(dso_pattern_strs));
			else
				return SR_ERR_BUG;
			break;
		case SR_CONF_PROBE_VDIV:
			/* VDIV applies to both DSO and ANALOG (DAQ) channels.
			 * Return dict {"vdivs": [uint64...]} for ProbeOptions. */
			if (ch->type != SR_CHANNEL_DSO && ch->type != SR_CHANNEL_ANALOG)
				return SR_ERR_ARG;
			{
				GVariantBuilder gvb;
				g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
				GVariant *gv = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
					dso_vdivs, ARRAY_SIZE(dso_vdivs), sizeof(uint64_t));
				g_variant_builder_add(&gvb, "{sv}", "vdivs", gv);
				*data = g_variant_builder_end(&gvb);
			}
			break;
		case SR_CONF_PROBE_COUPLING:
		/* COUPLING applies to both DSO and ANALOG (DAQ) channels.
		 * Return dict {"coupling": [int32...]} for ProbeOptions. */
		if (ch->type != SR_CHANNEL_DSO && ch->type != SR_CHANNEL_ANALOG)
			return SR_ERR_ARG;
		{
			GVariantBuilder gvb;
			g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
			GVariant *gv = g_variant_new_fixed_array(G_VARIANT_TYPE("i"),
				dso_couplings, ARRAY_SIZE(dso_couplings), sizeof(int32_t));
			g_variant_builder_add(&gvb, "{sv}", "coupling", gv);
			*data = g_variant_builder_end(&gvb);
			}
			break;
		case SR_CONF_PROBE_MAP_UNIT:
		/* MAP_UNIT applies to both DSO and ANALOG (DAQ) channels. */
		if (ch->type != SR_CHANNEL_DSO && ch->type != SR_CHANNEL_ANALOG)
			return SR_ERR_ARG;
		*data = g_variant_new_strv(ARRAY_AND_SIZE(dso_map_units));
		break;
		default:
			return SR_ERR_NA;
		}
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	GSList *l;
	struct sr_channel *ch;
	int bitpos;
	uint8_t mask;
	struct sr_trigger *trigger;

	devc = sdi->priv;
	devc->sent_samples = 0;
	devc->sent_frame_samples = 0;
	devc->dso_sent_samples = 0;

	/* Loop mode: keep limit_samples non-zero so the app's ring buffer
	 * (get_ring_sample_count() -> limit_samples) sizes to the selected
	 * duration window, and the LOOP capture cycles within it (ruler does
	 * not grow unbounded). The driver-level stop condition already guards
	 * on !loop_mode, so loop keeps streaming forever. Aligns with
	 * PXView-1.5.8. */

	/* Setup triggers */
	if ((trigger = sr_session_trigger_get(sdi->session))) {
		int pre_trigger_samples = 0;
		if (devc->limit_samples > 0)
			pre_trigger_samples = (devc->capture_ratio * devc->limit_samples) / 100;
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;

		/* Disable all analog channels since using them when there are logic
		 * triggers set up would require having pre-trigger sample buffers
		 * for analog sample data.
		 */
		for (l = sdi->channels; l; l = l->next) {
			ch = l->data;
			if (ch->type == SR_CHANNEL_ANALOG)
				ch->enabled = FALSE;
		}
	}
	devc->trigger_fired = FALSE;

	/*
	 * Determine the numbers of logic and analog channels that are
	 * involved in the acquisition. Determine an offset and a mask to
	 * remove excess logic data content before datafeed submission.
	 */
	devc->enabled_logic_channels = 0;
	devc->enabled_analog_channels = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->type == SR_CHANNEL_ANALOG) {
			devc->enabled_analog_channels++;
			continue;
		}
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		/*
		 * TODO: Need we create a channel map here, such that the
		 * session datafeed packets will have a dense representation
		 * of the enabled channels' data? For example store channels
		 * D3 and D5 in bit positions 0 and 1 respectively, when all
		 * other channels are disabled? The current implementation
		 * generates a sparse layout, might provide data for logic
		 * channels that are disabled while it might suppress data
		 * from enabled channels at the same time.
		 */
		devc->enabled_logic_channels++;
	}
	/* Tight cross encoding: unitsize reflects only ENABLED logic channels,
	 * so the LA_CROSS_DATA stream packs exactly the enabled channels. */
	devc->logic_unitsize = (devc->enabled_logic_channels + 7) / 8;
	if (devc->logic_unitsize < 1)
		devc->logic_unitsize = 1;
	devc->first_partial_logic_index = devc->enabled_logic_channels / 8;
	bitpos = devc->enabled_logic_channels % 8;
	mask = (1 << bitpos) - 1;
	devc->first_partial_logic_mask = mask;
	sr_dbg("num logic %zu, partial off %zu, mask 0x%02x.",
		devc->enabled_logic_channels,
		devc->first_partial_logic_index,
		devc->first_partial_logic_mask);

	sr_info("demo dev_acquisition_start: cur_samplerate=%" PRIu64
		", limit_samples=%" PRIu64 ", limit_msec=%" PRIu64
		", limit_frames=%" PRIu64 ", capture_ratio=%" PRIu64
		", num_logic=%zu, num_analog=%zu, num_dso=%zu"
		", enabled_logic=%zu, enabled_analog=%zu"
		", stl=%p, logic_unitsize=%zu",
		devc->cur_samplerate, devc->limit_samples, devc->limit_msec,
		devc->limit_frames, devc->capture_ratio,
		(size_t)devc->num_logic_channels, (size_t)devc->num_analog_channels,
		(size_t)devc->num_dso_channels,
		devc->enabled_logic_channels, devc->enabled_analog_channels,
		(void*)devc->stl, devc->logic_unitsize);

	/* Tick interval for demo_prepare_data. 25ms = 40 packets/sec gives
	 * smooth progress-bar updates (the old 100ms = 10 FPS looked
	 * stuttery). samples_todo is derived from real elapsed time via
	 * g_get_monotonic_time(), so shorter ticks just produce smaller,
	 * more frequent batches — no data-integrity impact. */
	const int demo_tick_ms = 25;
	int _src_ret = sr_session_source_add(sdi->session, -1, 0, demo_tick_ms,
			demo_prepare_data, (struct sr_dev_inst *)sdi);
	sr_info("demo dev_acquisition_start: sr_session_source_add returned %d (tick=%dms)",
		_src_ret, demo_tick_ms);

	std_session_send_df_header(sdi);

	if (devc->limit_frames > 0)
		std_session_send_df_frame_begin(sdi);

	/* We use this timestamp to decide how many more samples to send. */
	devc->start_us = g_get_monotonic_time();
	devc->spent_us = 0;
	devc->step = 0;

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	sr_info("demo dev_acquisition_stop: CALLED (sent_samples=%" PRIu64
		", spent_us=%" PRId64 ", trigger_fired=%d)",
		((struct dev_context *)sdi->priv)->sent_samples,
		((struct dev_context *)sdi->priv)->spent_us,
		(int)((struct dev_context *)sdi->priv)->trigger_fired);

	sr_session_source_remove(sdi->session, -1);

	devc = sdi->priv;
	if (devc->limit_frames > 0)
		std_session_send_df_frame_end(sdi);

	std_session_send_df_end(sdi);

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}

	/* Free DSO scratch buffer (re-allocated on next send_dso_packet). */
	if (devc->dso_buf) {
		g_free(devc->dso_buf);
		devc->dso_buf = NULL;
	}

	return SR_OK;
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;

	ret = std_dummy_dev_open(sdi);
	if (ret != SR_OK)
		return ret;

	/* Scan .demo files on first open. */
	demo_scan_dsl_file(sdi);

	return SR_OK;
}

static struct sr_dev_driver demo_driver_info = {
	.name = "demo",
	.longname = "Demo driver and pattern generator",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = std_dummy_dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(demo_driver_info);
