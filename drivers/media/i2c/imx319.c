// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#define IMX319_REG_MODE_SELECT		0x0100
#define IMX319_MODE_STANDBY		0x00
#define IMX319_MODE_STREAMING		0x01

/* Chip ID */
#define IMX319_REG_CHIP_ID		0x0016
#define IMX319_CHIP_ID			0x0319

/* V_TIMING internal */
#define IMX319_REG_FLL			0x0340
#define IMX319_FLL_MAX			0xffff

/* Exposure control */
#define IMX319_REG_EXPOSURE		0x0202
#define IMX319_EXPOSURE_MIN		1
#define IMX319_EXPOSURE_STEP		1
#define IMX319_EXPOSURE_DEFAULT		0x04f6

/*
 *  the digital control register for all color control looks like:
 *  +-----------------+------------------+
 *  |      [7:0]      |       [15:8]     |
 *  +-----------------+------------------+
 *  |	  0x020f      |       0x020e     |
 *  --------------------------------------
 *  it is used to calculate the digital gain times value(integral + fractional)
 *  the [15:8] bits is the fractional part and [7:0] bits is the integral
 *  calculation equation is:
 *      gain value (unit: times) = REG[15:8] + REG[7:0]/0x100
 *  Only value in 0x0100 ~ 0x0FFF range is allowed.
 *  Analog gain use 10 bits in the registers and allowed range is 0 ~ 960
 */
/* Analog gain control */
#define IMX319_REG_ANALOG_GAIN		0x0204
#define IMX319_ANA_GAIN_MIN		0
#define IMX319_ANA_GAIN_MAX		960
#define IMX319_ANA_GAIN_STEP		1
#define IMX319_ANA_GAIN_DEFAULT		0

/* Digital gain control */
#define IMX319_REG_DPGA_USE_GLOBAL_GAIN	0x3ff9
#define IMX319_REG_DIG_GAIN_GLOBAL	0x020e
#define IMX319_DGTL_GAIN_MIN		256
#define IMX319_DGTL_GAIN_MAX		4095
#define IMX319_DGTL_GAIN_STEP		1
#define IMX319_DGTL_GAIN_DEFAULT	256

/* Test Pattern Control */
#define IMX319_REG_TEST_PATTERN		0x0600
#define IMX319_TEST_PATTERN_DISABLED		0
#define IMX319_TEST_PATTERN_SOLID_COLOR		1
#define IMX319_TEST_PATTERN_COLOR_BARS		2
#define IMX319_TEST_PATTERN_GRAY_COLOR_BARS	3
#define IMX319_TEST_PATTERN_PN9			4

/* Flip Control */
#define IMX319_REG_ORIENTATION		0x0101

/* default link frequency and external clock */
#define IMX319_LINK_FREQ_DEFAULT	482400000LL
#define IMX319_EXT_CLK			19200000
#define IMX319_LINK_FREQ_INDEX		0

struct imx319_reg {
	u16 address;
	u8 val;
};

struct imx319_reg_list {
	u32 num_of_regs;
	const struct imx319_reg *regs;
};

/* Full unbinned register-list readout coordinate space. */
#define IMX319_NATIVE_WIDTH	3280
#define IMX319_NATIVE_HEIGHT	2464

/* Mode : resolution and related config&values */
struct imx319_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
	struct v4l2_rect analogue_crop;

	/* V-timing */
	u32 fll_def;
	u32 fll_min;

	/* H-timing */
	u32 llp;

	/* index of link frequency */
	u32 link_freq_index;

	/* Default register values */
	struct imx319_reg_list reg_list;
};

struct imx319_hwcfg {
	unsigned long link_freq_bitmap;
};

struct imx319_profile {
	const struct imx319_mode *modes;
	unsigned int num_modes;
	const struct imx319_reg_list *global_setting;
	const s64 *link_freqs;
	unsigned int num_link_freqs;
	u64 pixel_rate;
};

struct imx319 {
	struct device *dev;
	const struct imx319_profile *profile;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator *vana, *vdig, *vif;
	bool managed_power;
	bool vana_enabled, vdig_enabled, vif_enabled, clock_enabled;
	bool power_fault;
	bool removing;
	/* Exactly one owned runtime-PM reference while streaming is true. */
	bool streaming;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	/* Current mode */
	const struct imx319_mode *cur_mode;

	struct imx319_hwcfg *hwcfg;

	/*
	 * Mutex for serialized access:
	 * Protect sensor set pad format and start/stop streaming safely.
	 * Protect access to sensor v4l2 controls.
	 */
	struct mutex mutex;

	/* True if the device has been identified */
	bool identified;
};

static const struct imx319_reg imx319_global_regs[] = {
	{ 0x0136, 0x13 },
	{ 0x0137, 0x33 },
	{ 0x3c7e, 0x05 },
	{ 0x3c7f, 0x07 },
	{ 0x4d39, 0x0b },
	{ 0x4d41, 0x33 },
	{ 0x4d43, 0x0c },
	{ 0x4d49, 0x89 },
	{ 0x4e05, 0x0b },
	{ 0x4e0d, 0x33 },
	{ 0x4e0f, 0x0c },
	{ 0x4e15, 0x89 },
	{ 0x4e49, 0x2a },
	{ 0x4e51, 0x33 },
	{ 0x4e53, 0x0c },
	{ 0x4e59, 0x89 },
	{ 0x5601, 0x4f },
	{ 0x560b, 0x45 },
	{ 0x562f, 0x0a },
	{ 0x5643, 0x0a },
	{ 0x5645, 0x0c },
	{ 0x56ef, 0x51 },
	{ 0x586f, 0x33 },
	{ 0x5873, 0x89 },
	{ 0x5905, 0x33 },
	{ 0x5907, 0x89 },
	{ 0x590d, 0x33 },
	{ 0x590f, 0x89 },
	{ 0x5915, 0x33 },
	{ 0x5917, 0x89 },
	{ 0x5969, 0x1c },
	{ 0x596b, 0x72 },
	{ 0x5971, 0x33 },
	{ 0x5973, 0x89 },
	{ 0x5975, 0x33 },
	{ 0x5977, 0x89 },
	{ 0x5979, 0x1c },
	{ 0x597b, 0x72 },
	{ 0x5985, 0x33 },
	{ 0x5987, 0x89 },
	{ 0x5999, 0x1c },
	{ 0x599b, 0x72 },
	{ 0x59a5, 0x33 },
	{ 0x59a7, 0x89 },
	{ 0x7485, 0x08 },
	{ 0x7487, 0x0c },
	{ 0x7489, 0xc7 },
	{ 0x748b, 0x8b },
	{ 0x9004, 0x09 },
	{ 0x9200, 0x6a },
	{ 0x9201, 0x22 },
	{ 0x9202, 0x6a },
	{ 0x9203, 0x23 },
	{ 0x9204, 0x5f },
	{ 0x9205, 0x23 },
	{ 0x9206, 0x5f },
	{ 0x9207, 0x24 },
	{ 0x9208, 0x5f },
	{ 0x9209, 0x26 },
	{ 0x920a, 0x5f },
	{ 0x920b, 0x27 },
	{ 0x920c, 0x5f },
	{ 0x920d, 0x29 },
	{ 0x920e, 0x5f },
	{ 0x920f, 0x2a },
	{ 0x9210, 0x5f },
	{ 0x9211, 0x2c },
	{ 0xbc22, 0x1a },
	{ 0xf01f, 0x04 },
	{ 0xf021, 0x03 },
	{ 0xf023, 0x02 },
	{ 0xf03d, 0x05 },
	{ 0xf03f, 0x03 },
	{ 0xf041, 0x02 },
	{ 0xf0af, 0x04 },
	{ 0xf0b1, 0x03 },
	{ 0xf0b3, 0x02 },
	{ 0xf0cd, 0x05 },
	{ 0xf0cf, 0x03 },
	{ 0xf0d1, 0x02 },
	{ 0xf13f, 0x04 },
	{ 0xf141, 0x03 },
	{ 0xf143, 0x02 },
	{ 0xf15d, 0x05 },
	{ 0xf15f, 0x03 },
	{ 0xf161, 0x02 },
	{ 0xf1cf, 0x04 },
	{ 0xf1d1, 0x03 },
	{ 0xf1d3, 0x02 },
	{ 0xf1ed, 0x05 },
	{ 0xf1ef, 0x03 },
	{ 0xf1f1, 0x02 },
	{ 0xf287, 0x04 },
	{ 0xf289, 0x03 },
	{ 0xf28b, 0x02 },
	{ 0xf2a5, 0x05 },
	{ 0xf2a7, 0x03 },
	{ 0xf2a9, 0x02 },
	{ 0xf2b7, 0x04 },
	{ 0xf2b9, 0x03 },
	{ 0xf2bb, 0x02 },
	{ 0xf2d5, 0x05 },
	{ 0xf2d7, 0x03 },
	{ 0xf2d9, 0x02 },
};

static const struct imx319_reg_list imx319_global_setting = {
	.num_of_regs = ARRAY_SIZE(imx319_global_regs),
	.regs = imx319_global_regs,
};

static const struct imx319_reg mode_3264x2448_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x80 },
	{ 0x0340, 0x0c },
	{ 0x0341, 0xaa },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x9f },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x08 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x08 },
	{ 0x040c, 0x0c },
	{ 0x040d, 0xc0 },
	{ 0x040e, 0x09 },
	{ 0x040f, 0x90 },
	{ 0x034c, 0x0c },
	{ 0x034d, 0xc0 },
	{ 0x034e, 0x09 },
	{ 0x034f, 0x90 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0x48 },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x01 },
	{ 0x3f79, 0x18 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x00 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x0a },
	{ 0x0203, 0x7a },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_3280x2464_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x80 },
	{ 0x0340, 0x0c },
	{ 0x0341, 0xaa },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x9f },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x0c },
	{ 0x040d, 0xd0 },
	{ 0x040e, 0x09 },
	{ 0x040f, 0xa0 },
	{ 0x034c, 0x0c },
	{ 0x034d, 0xd0 },
	{ 0x034e, 0x09 },
	{ 0x034f, 0xa0 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0x48 },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x01 },
	{ 0x3f79, 0x18 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x00 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x0a },
	{ 0x0203, 0x7a },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1936x1096_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x80 },
	{ 0x0340, 0x0c },
	{ 0x0341, 0xaa },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x02 },
	{ 0x0347, 0xac },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x06 },
	{ 0x034b, 0xf3 },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x02 },
	{ 0x0409, 0xa0 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x07 },
	{ 0x040d, 0x90 },
	{ 0x040e, 0x04 },
	{ 0x040f, 0x48 },
	{ 0x034c, 0x07 },
	{ 0x034d, 0x90 },
	{ 0x034e, 0x04 },
	{ 0x034f, 0x48 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0x48 },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x01 },
	{ 0x3f79, 0x18 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x00 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x05 },
	{ 0x0203, 0x34 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1920x1080_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x80 },
	{ 0x0340, 0x0c },
	{ 0x0341, 0xaa },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x02 },
	{ 0x0347, 0xb4 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x06 },
	{ 0x034b, 0xeb },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x02 },
	{ 0x0409, 0xa8 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x07 },
	{ 0x040d, 0x80 },
	{ 0x040e, 0x04 },
	{ 0x040f, 0x38 },
	{ 0x034c, 0x07 },
	{ 0x034d, 0x80 },
	{ 0x034e, 0x04 },
	{ 0x034f, 0x38 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0x48 },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x01 },
	{ 0x3f79, 0x18 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x00 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x05 },
	{ 0x0203, 0x34 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1640x1232_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x08 },
	{ 0x0343, 0x20 },
	{ 0x0340, 0x18 },
	{ 0x0341, 0x2a },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x9f },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x02 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x06 },
	{ 0x040d, 0x68 },
	{ 0x040e, 0x04 },
	{ 0x040f, 0xd0 },
	{ 0x034c, 0x06 },
	{ 0x034d, 0x68 },
	{ 0x034e, 0x04 },
	{ 0x034f, 0xd0 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0xba },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x00 },
	{ 0x3f79, 0x34 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x04 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x04 },
	{ 0x0203, 0xf6 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1640x922_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x08 },
	{ 0x0343, 0x20 },
	{ 0x0340, 0x18 },
	{ 0x0341, 0x2a },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x01 },
	{ 0x0347, 0x30 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x08 },
	{ 0x034b, 0x6f },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x02 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x02 },
	{ 0x040c, 0x06 },
	{ 0x040d, 0x68 },
	{ 0x040e, 0x03 },
	{ 0x040f, 0x9a },
	{ 0x034c, 0x06 },
	{ 0x034d, 0x68 },
	{ 0x034e, 0x03 },
	{ 0x034f, 0x9a },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0xba },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x00 },
	{ 0x3f79, 0x34 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x04 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x04 },
	{ 0x0203, 0xf6 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1296x736_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x08 },
	{ 0x0343, 0x20 },
	{ 0x0340, 0x18 },
	{ 0x0341, 0x2a },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x01 },
	{ 0x0347, 0xf0 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x07 },
	{ 0x034b, 0xaf },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x02 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0xac },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x05 },
	{ 0x040d, 0x10 },
	{ 0x040e, 0x02 },
	{ 0x040f, 0xe0 },
	{ 0x034c, 0x05 },
	{ 0x034d, 0x10 },
	{ 0x034e, 0x02 },
	{ 0x034f, 0xe0 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0xba },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x00 },
	{ 0x3f79, 0x34 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x04 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x04 },
	{ 0x0203, 0xf6 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const struct imx319_reg mode_1280x720_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x03 },
	{ 0x0342, 0x08 },
	{ 0x0343, 0x20 },
	{ 0x0340, 0x18 },
	{ 0x0341, 0x2a },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x02 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x07 },
	{ 0x034b, 0x9f },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x0902, 0x0a },
	{ 0x3140, 0x02 },
	{ 0x3141, 0x00 },
	{ 0x3f0d, 0x0a },
	{ 0x3f14, 0x01 },
	{ 0x3f3c, 0x02 },
	{ 0x3f4d, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x4254, 0x7f },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0xb4 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x05 },
	{ 0x040d, 0x00 },
	{ 0x040e, 0x02 },
	{ 0x040f, 0xd0 },
	{ 0x034c, 0x05 },
	{ 0x034d, 0x00 },
	{ 0x034e, 0x02 },
	{ 0x034f, 0xd0 },
	{ 0x3261, 0x00 },
	{ 0x3264, 0x00 },
	{ 0x3265, 0x10 },
	{ 0x0301, 0x05 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0x92 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x02 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xfa },
	{ 0x0310, 0x00 },
	{ 0x0820, 0x0f },
	{ 0x0821, 0x13 },
	{ 0x0822, 0x33 },
	{ 0x0823, 0x33 },
	{ 0x3e20, 0x01 },
	{ 0x3e37, 0x00 },
	{ 0x3e3b, 0x01 },
	{ 0x38a3, 0x01 },
	{ 0x38a8, 0x00 },
	{ 0x38a9, 0x00 },
	{ 0x38aa, 0x00 },
	{ 0x38ab, 0x00 },
	{ 0x3234, 0x00 },
	{ 0x3fc1, 0x00 },
	{ 0x3235, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3143, 0x04 },
	{ 0x360a, 0x00 },
	{ 0x0b00, 0x00 },
	{ 0x0106, 0x00 },
	{ 0x0b05, 0x01 },
	{ 0x0b06, 0x01 },
	{ 0x3230, 0x00 },
	{ 0x3602, 0x01 },
	{ 0x3607, 0x01 },
	{ 0x3c00, 0x00 },
	{ 0x3c01, 0xba },
	{ 0x3c02, 0xc8 },
	{ 0x3c03, 0xaa },
	{ 0x3c04, 0x91 },
	{ 0x3c05, 0x54 },
	{ 0x3c06, 0x26 },
	{ 0x3c07, 0x20 },
	{ 0x3c08, 0x51 },
	{ 0x3d80, 0x00 },
	{ 0x3f50, 0x00 },
	{ 0x3f56, 0x00 },
	{ 0x3f57, 0x30 },
	{ 0x3f78, 0x00 },
	{ 0x3f79, 0x34 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x3fba, 0x00 },
	{ 0x3fbb, 0x00 },
	{ 0xa081, 0x04 },
	{ 0xe014, 0x00 },
	{ 0x0202, 0x04 },
	{ 0x0203, 0xf6 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0210, 0x01 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x01 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x01 },
	{ 0x0215, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x3614, 0x00 },
	{ 0x3616, 0x0d },
	{ 0x3617, 0x56 },
	{ 0xb612, 0x20 },
	{ 0xb613, 0x20 },
	{ 0xb614, 0x20 },
	{ 0xb615, 0x20 },
	{ 0xb616, 0x0a },
	{ 0xb617, 0x0a },
	{ 0xb618, 0x20 },
	{ 0xb619, 0x20 },
	{ 0xb61a, 0x20 },
	{ 0xb61b, 0x20 },
	{ 0xb61c, 0x0a },
	{ 0xb61d, 0x0a },
	{ 0xb666, 0x30 },
	{ 0xb667, 0x30 },
	{ 0xb668, 0x30 },
	{ 0xb669, 0x30 },
	{ 0xb66a, 0x14 },
	{ 0xb66b, 0x14 },
	{ 0xb66c, 0x20 },
	{ 0xb66d, 0x20 },
	{ 0xb66e, 0x20 },
	{ 0xb66f, 0x20 },
	{ 0xb670, 0x10 },
	{ 0xb671, 0x10 },
	{ 0x3237, 0x00 },
	{ 0x3900, 0x00 },
	{ 0x3901, 0x00 },
	{ 0x3902, 0x00 },
	{ 0x3904, 0x00 },
	{ 0x3905, 0x00 },
	{ 0x3906, 0x00 },
	{ 0x3907, 0x00 },
	{ 0x3908, 0x00 },
	{ 0x3909, 0x00 },
	{ 0x3912, 0x00 },
	{ 0x3930, 0x00 },
	{ 0x3931, 0x00 },
	{ 0x3933, 0x00 },
	{ 0x3934, 0x00 },
	{ 0x3935, 0x00 },
	{ 0x3936, 0x00 },
	{ 0x3937, 0x00 },
	{ 0x30ac, 0x00 },
};

static const char * const imx319_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

/*
 * When adding more than the one below, make sure the disallowed ones will
 * actually be disabled in the LINK_FREQ control.
 */
static const s64 link_freq_menu_items[] = {
	IMX319_LINK_FREQ_DEFAULT,
};

/* Mode configs */
static const struct imx319_mode supported_modes[] = {
	{
		.width = 3280,
		.height = 2464,
		.analogue_crop = {
			.left = 0, .top = 0,
			.width = 3280, .height = 2464,
		},
		.fll_def = 3242,
		.fll_min = 3242,
		.llp = 3968,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3280x2464_regs),
			.regs = mode_3280x2464_regs,
		},
	},
	{
		.width = 3264,
		.height = 2448,
		.analogue_crop = {
			.left = 0, .top = 0,
			.width = 3280, .height = 2464,
		},
		.fll_def = 3242,
		.fll_min = 3242,
		.llp = 3968,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3264x2448_regs),
			.regs = mode_3264x2448_regs,
		},
	},
	{
		.width = 1936,
		.height = 1096,
		.analogue_crop = {
			.left = 0, .top = 684,
			.width = 3280, .height = 1096,
		},
		.fll_def = 3242,
		.fll_min = 3242,
		.llp = 3968,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1936x1096_regs),
			.regs = mode_1936x1096_regs,
		},
	},
	{
		.width = 1920,
		.height = 1080,
		.analogue_crop = {
			.left = 0, .top = 692,
			.width = 3280, .height = 1080,
		},
		.fll_def = 3242,
		.fll_min = 3242,
		.llp = 3968,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	},
	{
		.width = 1640,
		.height = 1232,
		.analogue_crop = {
			.left = 0, .top = 0,
			.width = 3280, .height = 2464,
		},
		.fll_def = 5146,
		.fll_min = 5146,
		.llp = 2500,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1640x1232_regs),
			.regs = mode_1640x1232_regs,
		},
	},
	{
		.width = 1640,
		.height = 922,
		.analogue_crop = {
			.left = 0, .top = 304,
			.width = 3280, .height = 1856,
		},
		.fll_def = 5146,
		.fll_min = 5146,
		.llp = 2500,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1640x922_regs),
			.regs = mode_1640x922_regs,
		},
	},
	{
		.width = 1296,
		.height = 736,
		.analogue_crop = {
			.left = 0, .top = 496,
			.width = 3280, .height = 1472,
		},
		.fll_def = 5146,
		.fll_min = 5146,
		.llp = 2500,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1296x736_regs),
			.regs = mode_1296x736_regs,
		},
	},
	{
		.width = 1280,
		.height = 720,
		.analogue_crop = {
			.left = 0, .top = 512,
			.width = 3280, .height = 1440,
		},
		.fll_def = 5146,
		.fll_min = 5146,
		.llp = 2500,
		.link_freq_index = IMX319_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_regs),
			.regs = mode_1280x720_regs,
		},
	},
};

#include "imx319-222mhz-tables.h"

static const struct imx319_reg_list imx319_222mhz_global_setting = {
	.regs = imx319_222mhz_global_regs,
	.num_of_regs = ARRAY_SIZE(imx319_222mhz_global_regs),
};
static const struct imx319_mode imx319_222mhz_modes[] = {
	{
		.width = 1632, .height = 1224,
		.analogue_crop = {
			.left = 0, .top = 8,
			.width = 3280, .height = 2448,
		},
		.fll_def = 3350, .fll_min = 3350, .llp = 2080,
		.link_freq_index = 0,
		.reg_list = { ARRAY_SIZE(imx319_222mhz_mode_regs), imx319_222mhz_mode_regs },
	},
};
/* Firmware/PLL-derived rates, to be checked against native SOF timestamps. */
static const s64 imx319_222mhz_link_freqs[] = { 222000000 };
static const struct imx319_profile imx319_482mhz_profile = {
	.modes = supported_modes, .num_modes = ARRAY_SIZE(supported_modes),
	.global_setting = &imx319_global_setting,
	.link_freqs = link_freq_menu_items, .num_link_freqs = ARRAY_SIZE(link_freq_menu_items),
	.pixel_rate = IMX319_LINK_FREQ_DEFAULT * 2 * 4 / 10,
};
static const struct imx319_profile imx319_222mhz_profile = {
	.modes = imx319_222mhz_modes, .num_modes = ARRAY_SIZE(imx319_222mhz_modes),
	.global_setting = &imx319_222mhz_global_setting,
	.link_freqs = imx319_222mhz_link_freqs, .num_link_freqs = ARRAY_SIZE(imx319_222mhz_link_freqs),
	.pixel_rate = 209066666,
};

static inline struct imx319 *to_imx319(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx319, sd);
}

/* Standard regulator consumers supply dependency ordering. */
static int imx319_get_power_resources(struct imx319 *imx319)
{
	struct device *dev = imx319->dev;

	if (!dev->of_node)
		return 0;
	imx319->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx319->reset_gpio))
		return PTR_ERR(imx319->reset_gpio);
	imx319->vana = devm_regulator_get_optional(dev, "vana");
	if (IS_ERR(imx319->vana))
		return PTR_ERR(imx319->vana);
	imx319->vdig = devm_regulator_get_optional(dev, "vdig");
	if (IS_ERR(imx319->vdig))
		return PTR_ERR(imx319->vdig);
	imx319->vif = devm_regulator_get_optional(dev, "vif");
	if (IS_ERR(imx319->vif))
		return PTR_ERR(imx319->vif);
	imx319->managed_power = true;
	return 0;
}

static int imx319_power_off(struct imx319 *imx319)
{
	int ret, first = 0;

	if (!imx319->managed_power)
		return 0;
	if (imx319->clock_enabled) {
		clk_disable_unprepare(imx319->clk);
		imx319->clock_enabled = false;
	}
	usleep_range(1000, 1100);
	ret = gpiod_set_value_cansleep(imx319->reset_gpio, 1);
	if (ret)
		first = ret;
	imx319->identified = false;
	usleep_range(1000, 1100);
	if (imx319->vif_enabled) {
		ret = regulator_disable(imx319->vif);
		if (!ret)
			imx319->vif_enabled = false;
		else if (!first)
			first = ret;
	}
	if (imx319->vdig_enabled) {
		ret = regulator_disable(imx319->vdig);
		if (!ret)
			imx319->vdig_enabled = false;
		else if (!first)
			first = ret;
	}
	if (imx319->vana_enabled) {
		ret = regulator_disable(imx319->vana);
		if (!ret)
			imx319->vana_enabled = false;
		else if (!first)
			first = ret;
	}
	ret = pinctrl_pm_select_sleep_state(imx319->dev);
	if (ret && !first)
		first = ret;
	if (first)
		imx319->power_fault = true;
	return first;
}

static int imx319_power_on(struct imx319 *imx319)
{
	int ret, cleanup_ret;

	if (!imx319->managed_power)
		return 0;
	if (imx319->power_fault || imx319->vana_enabled || imx319->vdig_enabled ||
	    imx319->vif_enabled || imx319->clock_enabled)
		return -EIO;
	ret = pinctrl_pm_select_default_state(imx319->dev);
	if (ret) {
		imx319->power_fault = true;
		return ret;
	}
	ret = gpiod_set_value_cansleep(imx319->reset_gpio, 1);
	if (ret) {
		imx319->power_fault = true;
		return ret;
	}
	usleep_range(1000, 1100);
	ret = regulator_enable(imx319->vana);
	if (ret)
		goto failed;
	imx319->vana_enabled = true;
	usleep_range(1000, 1100);
	ret = regulator_enable(imx319->vdig);
	if (ret)
		goto failed;
	imx319->vdig_enabled = true;
	usleep_range(1000, 1100);
	ret = regulator_enable(imx319->vif);
	if (ret)
		goto failed;
	imx319->vif_enabled = true;
	ret = clk_prepare_enable(imx319->clk);
	if (ret)
		goto failed;
	imx319->clock_enabled = true;
	usleep_range(1000, 1100);
	ret = gpiod_set_value_cansleep(imx319->reset_gpio, 0);
	if (ret)
		goto failed;
	usleep_range(18000, 19000);
	return 0;
failed:
	imx319->power_fault = true;
	cleanup_ret = imx319_power_off(imx319);
	if (cleanup_ret)
		dev_err(imx319->dev, "power rollback incomplete: %d\n", cleanup_ret);
	return ret;
}

static int imx319_runtime_resume(struct device *dev)
{
	return imx319_power_on(to_imx319(dev_get_drvdata(dev)));
}

static int imx319_runtime_suspend(struct device *dev)
{
	return imx319_power_off(to_imx319(dev_get_drvdata(dev)));
}

static const struct dev_pm_ops imx319_pm_ops = {
	SET_RUNTIME_PM_OPS(imx319_runtime_suspend, imx319_runtime_resume, NULL)
};

/* Get bayer order based on flip setting. */
static u32 imx319_get_format_code(struct imx319 *imx319)
{
	/*
	 * Only one bayer order is supported.
	 * It depends on the flip settings.
	 */
	u32 code;
	static const u32 codes[2][2] = {
		{ MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SGRBG10_1X10, },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SBGGR10_1X10, },
	};

	lockdep_assert_held(&imx319->mutex);
	code = codes[imx319->vflip->val][imx319->hflip->val];

	return code;
}

/* Read registers up to 4 at a time */
static int imx319_read_reg(struct imx319 *imx319, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx319->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = { 0 };
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 4 at a time */
static int imx319_write_reg(struct imx319 *imx319, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx319->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx319_write_regs(struct imx319 *imx319,
			     const struct imx319_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = imx319_write_reg(imx319, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(imx319->dev,
					    "write reg 0x%4.4x return err %d",
					    regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int imx319_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx319 *imx319 = to_imx319(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);

	mutex_lock(&imx319->mutex);

	/* Initialize try_fmt */
	try_fmt->width = imx319->cur_mode->width;
	try_fmt->height = imx319->cur_mode->height;
	try_fmt->code = imx319_get_format_code(imx319);
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx319->mutex);

	return 0;
}

static int imx319_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx319 *imx319 = container_of(ctrl->handler,
					     struct imx319, ctrl_handler);
	s64 max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx319->cur_mode->height + ctrl->val - 18;
		__v4l2_ctrl_modify_range(imx319->exposure,
					 imx319->exposure->minimum,
					 max, imx319->exposure->step, max);
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (imx319->power_fault)
		return -EIO;
	ret = pm_runtime_get_if_in_use(imx319->dev);
	if (ret <= 0)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/* Analog gain = 1024/(1024 - ctrl->val) times */
		ret = imx319_write_reg(imx319, IMX319_REG_ANALOG_GAIN, 2,
				       ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx319_write_reg(imx319, IMX319_REG_DIG_GAIN_GLOBAL, 2,
				       ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx319_write_reg(imx319, IMX319_REG_EXPOSURE, 2,
				       ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* Update FLL that meets expected vertical blanking */
		ret = imx319_write_reg(imx319, IMX319_REG_FLL, 2,
				       imx319->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx319_write_reg(imx319, IMX319_REG_TEST_PATTERN,
				       2, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx319_write_reg(imx319, IMX319_REG_ORIENTATION, 1,
				       imx319->hflip->val |
				       imx319->vflip->val << 1);
		break;
	default:
		ret = -EINVAL;
		dev_info(imx319->dev, "ctrl(id:0x%x,val:0x%x) is not handled",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(imx319->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx319_ctrl_ops = {
	.s_ctrl = imx319_set_ctrl,
};

static int imx319_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx319 *imx319 = to_imx319(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&imx319->mutex);
	code->code = imx319_get_format_code(imx319);
	mutex_unlock(&imx319->mutex);

	return 0;
}

static int imx319_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx319 *imx319 = to_imx319(sd);

	if (fse->index >= imx319->profile->num_modes)
		return -EINVAL;

	mutex_lock(&imx319->mutex);
	if (fse->code != imx319_get_format_code(imx319)) {
		mutex_unlock(&imx319->mutex);
		return -EINVAL;
	}
	mutex_unlock(&imx319->mutex);

	fse->min_width = imx319->profile->modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx319->profile->modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx319_update_pad_format(struct imx319 *imx319,
				     const struct imx319_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = imx319_get_format_code(imx319);
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx319_do_get_pad_format(struct imx319 *imx319,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx319_update_pad_format(imx319, imx319->cur_mode, fmt);
	}

	return 0;
}

static int imx319_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx319 *imx319 = to_imx319(sd);
	int ret;

	mutex_lock(&imx319->mutex);
	ret = imx319_do_get_pad_format(imx319, sd_state, fmt);
	mutex_unlock(&imx319->mutex);

	return ret;
}

static int imx319_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct imx319 *imx319 = to_imx319(sd);
	const struct imx319_mode *mode;
	struct v4l2_mbus_framefmt *format;

	if (sel->pad || (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE &&
			 sel->which != V4L2_SUBDEV_FORMAT_TRY))
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = (struct v4l2_rect) {
			.left = 0, .top = 0,
			.width = IMX319_NATIVE_WIDTH,
			.height = IMX319_NATIVE_HEIGHT,
		};
		return 0;
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&imx319->mutex);
		if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
			format = v4l2_subdev_state_get_format(sd_state, sel->pad);
			mode = v4l2_find_nearest_size(imx319->profile->modes,
						      imx319->profile->num_modes,
						      width, height,
						      format->width, format->height);
		} else {
			mode = imx319->cur_mode;
		}
		/* Report the analogue window before binning/digital cropping. */
		sel->r = mode->analogue_crop;
		mutex_unlock(&imx319->mutex);
		return 0;
	default:
		return -EINVAL;
	}
}

static int
imx319_set_pad_format(struct v4l2_subdev *sd,
		      struct v4l2_subdev_state *sd_state,
		      struct v4l2_subdev_format *fmt)
{
	struct imx319 *imx319 = to_imx319(sd);
	const struct imx319_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	s32 vblank_def;
	s32 vblank_min;
	s64 h_blank;
	u64 pixel_rate;
	u32 height;

	mutex_lock(&imx319->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && imx319->streaming) {
		mutex_unlock(&imx319->mutex);
		return -EBUSY;
	}

	/*
	 * Only one bayer order is supported.
	 * It depends on the flip settings.
	 */
	fmt->format.code = imx319_get_format_code(imx319);

	mode = v4l2_find_nearest_size(imx319->profile->modes,
				      imx319->profile->num_modes,
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx319_update_pad_format(imx319, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx319->cur_mode = mode;
		pixel_rate = imx319->profile->pixel_rate;
		__v4l2_ctrl_s_ctrl_int64(imx319->pixel_rate, pixel_rate);
		/* Update limits and set FPS to default */
		height = imx319->cur_mode->height;
		vblank_def = imx319->cur_mode->fll_def - height;
		vblank_min = imx319->cur_mode->fll_min - height;
		height = IMX319_FLL_MAX - height;
		__v4l2_ctrl_modify_range(imx319->vblank, vblank_min, height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(imx319->vblank, vblank_def);
		h_blank = mode->llp - imx319->cur_mode->width;
		/*
		 * Currently hblank is not changeable.
		 * So FPS control is done only by vblank.
		 */
		__v4l2_ctrl_modify_range(imx319->hblank, h_blank,
					 h_blank, 1, h_blank);
	}

	mutex_unlock(&imx319->mutex);

	return 0;
}

/* Verify chip ID */
static int imx319_identify_module(struct imx319 *imx319)
{
	int ret;
	u32 val;

	if (imx319->identified)
		return 0;

	ret = imx319_read_reg(imx319, IMX319_REG_CHIP_ID, 2, &val);
	if (ret)
		return ret;

	if (val != IMX319_CHIP_ID) {
		dev_err(imx319->dev, "chip id mismatch: %x!=%x",
			IMX319_CHIP_ID, val);
		return -EIO;
	}

	imx319->identified = true;

	return 0;
}

/* Start streaming */
static int imx319_start_streaming(struct imx319 *imx319)
{
	const struct imx319_reg_list *reg_list;
	int ret;

	ret = imx319_identify_module(imx319);
	if (ret)
		return ret;

	/* Global Setting */
	reg_list = imx319->profile->global_setting;
	ret = imx319_write_regs(imx319, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(imx319->dev, "failed to set global settings");
		return ret;
	}

	/* Apply default values of current mode */
	reg_list = &imx319->cur_mode->reg_list;
	ret = imx319_write_regs(imx319, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(imx319->dev, "failed to set mode");
		return ret;
	}

	/* set digital gain control to all color mode */
	ret = imx319_write_reg(imx319, IMX319_REG_DPGA_USE_GLOBAL_GAIN, 1, 1);
	if (ret)
		return ret;

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx319->sd.ctrl_handler);
	if (ret)
		return ret;

	return imx319_write_reg(imx319, IMX319_REG_MODE_SELECT,
				1, IMX319_MODE_STREAMING);
}

/* Stop streaming */
static int imx319_stop_streaming(struct imx319 *imx319)
{
	return imx319_write_reg(imx319, IMX319_REG_MODE_SELECT,
				1, IMX319_MODE_STANDBY);
}

static int imx319_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx319 *imx319 = to_imx319(sd);
	int ret = 0, cleanup_ret;

	enable = !!enable;
	mutex_lock(&imx319->mutex);
	if (enable && imx319->removing) {
		ret = -ENODEV;
		goto unlock;
	}
	if (enable == imx319->streaming)
		goto unlock;
	if (enable) {
		if (imx319->power_fault) {
			ret = -EIO;
			goto unlock;
		}
		ret = pm_runtime_resume_and_get(imx319->dev);
		if (ret < 0)
			goto unlock;
		ret = imx319_start_streaming(imx319);
		if (ret) {
			if (!imx319->managed_power) {
				pm_runtime_put(imx319->dev);
				goto unlock;
			}
			/* Keep the stream's PM reference while attempting standby. */
			cleanup_ret = imx319->identified ? imx319_stop_streaming(imx319) : 0;
			if (cleanup_ret)
				imx319->power_fault = true;
			cleanup_ret = pm_runtime_put_sync_suspend(imx319->dev);
			if (cleanup_ret < 0)
				imx319->power_fault = true;
			goto unlock;
		}
		imx319->streaming = true;
	} else {
		ret = imx319_stop_streaming(imx319);
		if (ret && imx319->managed_power)
			imx319->power_fault = true;
		/* Exactly one reference is owned, even when the stop write fails. */
		imx319->streaming = false;
		if (imx319->managed_power)
			cleanup_ret = pm_runtime_put_sync_suspend(imx319->dev);
		else {
			pm_runtime_put(imx319->dev);
			ret = 0;
			cleanup_ret = 0;
		}
		if (cleanup_ret < 0) {
			imx319->power_fault = true;
			if (!ret)
				ret = cleanup_ret;
		}
	}
	__v4l2_ctrl_grab(imx319->vflip, enable);
	__v4l2_ctrl_grab(imx319->hflip, enable);
unlock:
	mutex_unlock(&imx319->mutex);
	return ret;
}


static const struct v4l2_subdev_core_ops imx319_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx319_video_ops = {
	.s_stream = imx319_set_stream,
};

static const struct v4l2_subdev_pad_ops imx319_pad_ops = {
	.enum_mbus_code = imx319_enum_mbus_code,
	.get_fmt = imx319_get_pad_format,
	.set_fmt = imx319_set_pad_format,
	.get_selection = imx319_get_selection,
	.enum_frame_size = imx319_enum_frame_size,
};

static const struct v4l2_subdev_ops imx319_subdev_ops = {
	.core = &imx319_subdev_core_ops,
	.video = &imx319_video_ops,
	.pad = &imx319_pad_ops,
};

static const struct media_entity_operations imx319_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops imx319_internal_ops = {
	.open = imx319_open,
};

/* Initialize control handlers */
static int imx319_init_controls(struct imx319 *imx319)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max;
	s64 vblank_def;
	s64 vblank_min;
	s64 hblank;
	u64 pixel_rate;
	const struct imx319_mode *mode;
	u32 max;
	int ret;

	ret = v4l2_fwnode_device_parse(imx319->dev, &props);
	if (ret)
		return ret;

	ctrl_hdlr = &imx319->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx319->mutex;
	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx319_ctrl_ops, &props);
	if (ret)
		goto error;
	max = imx319->profile->num_link_freqs - 1;
	imx319->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx319_ctrl_ops,
						   V4L2_CID_LINK_FREQ, max, 0,
						   imx319->profile->link_freqs);
	if (imx319->link_freq)
		imx319->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Array timing rate is independent of the CSI transport capacity. */
	pixel_rate = imx319->profile->pixel_rate;
	/* By default, PIXEL_RATE is read only */
	imx319->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	/* Initial vblank/hblank/exposure parameters based on current mode */
	mode = imx319->cur_mode;
	vblank_def = mode->fll_def - mode->height;
	vblank_min = mode->fll_min - mode->height;
	imx319->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_min,
					   IMX319_FLL_MAX - mode->height,
					   1, vblank_def);

	hblank = mode->llp - mode->width;
	imx319->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx319->hblank)
		imx319->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* fll >= exposure time + adjust parameter (default value is 18) */
	exposure_max = mode->fll_def - 18;
	imx319->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX319_EXPOSURE_MIN, exposure_max,
					     IMX319_EXPOSURE_STEP,
					     IMX319_EXPOSURE_DEFAULT);

	imx319->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx319->hflip)
		imx319->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx319->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx319->vflip)
		imx319->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX319_ANA_GAIN_MIN, IMX319_ANA_GAIN_MAX,
			  IMX319_ANA_GAIN_STEP, IMX319_ANA_GAIN_DEFAULT);

	/* Digital gain */
	v4l2_ctrl_new_std(ctrl_hdlr, &imx319_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX319_DGTL_GAIN_MIN, IMX319_DGTL_GAIN_MAX,
			  IMX319_DGTL_GAIN_STEP, IMX319_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx319_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx319_test_pattern_menu) - 1,
				     0, 0, imx319_test_pattern_menu);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx319->dev, "control init failed: %d", ret);
		goto error;
	}

	imx319->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static struct imx319_hwcfg *imx319_get_hwcfg(struct imx319 *imx319)
{
	struct device *dev = imx319->dev;
	const struct imx319_profile *profile;
	struct imx319_hwcfg *cfg;
	unsigned int i;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	int ret;

	if (!fwnode)
		return NULL;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return NULL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	if (ret)
		goto out_err;

	if (dev->of_node && bus_cfg.bus.mipi_csi2.num_data_lanes != 4)
		goto out_err;

	cfg = devm_kzalloc(dev, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		goto out_err;

	profile = NULL;
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		if (bus_cfg.link_frequencies[i] == IMX319_LINK_FREQ_DEFAULT)
			profile = &imx319_482mhz_profile;
		else if (bus_cfg.link_frequencies[i] == 222000000)
			profile = &imx319_222mhz_profile;
		else
			continue;
		break;
	}
	if (!profile)
		goto out_err;
	imx319->profile = profile;

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       profile->link_freqs,
				       profile->num_link_freqs,
				       &cfg->link_freq_bitmap);
	if (ret)
		goto out_err;

	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(ep);
	return cfg;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(ep);
	return NULL;
}

static int imx319_probe(struct i2c_client *client)
{
	struct imx319 *imx319;
	bool full_power = false, probe_reference = false;
	unsigned long freq;
	int ret, cleanup_ret;

	imx319 = devm_kzalloc(&client->dev, sizeof(*imx319), GFP_KERNEL);
	if (!imx319)
		return -ENOMEM;
	imx319->dev = &client->dev;
	mutex_init(&imx319->mutex);
	imx319->clk = devm_v4l2_sensor_clk_get(imx319->dev, NULL);
	if (IS_ERR(imx319->clk)) {
		ret = PTR_ERR(imx319->clk);
		goto error_probe;
	}
	freq = clk_get_rate(imx319->clk);
	if (freq != IMX319_EXT_CLK) {
		ret = -EINVAL;
		goto error_probe;
	}
	v4l2_i2c_subdev_init(&imx319->sd, client, &imx319_subdev_ops);
	ret = imx319_get_power_resources(imx319);
	if (ret)
		goto error_probe;
	imx319->hwcfg = imx319_get_hwcfg(imx319);
	if (!imx319->hwcfg) {
		ret = -ENODEV;
		goto error_probe;
	}
	imx319->cur_mode = &imx319->profile->modes[0];
	ret = imx319_init_controls(imx319);
	if (ret)
		goto error_probe;
	imx319->sd.internal_ops = &imx319_internal_ops;
	imx319->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	imx319->sd.entity.ops = &imx319_subdev_entity_ops;
	imx319->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx319->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx319->sd.entity, 1, &imx319->pad);
	if (ret)
		goto error_handler;
	if (imx319->managed_power) {
		ret = imx319_power_on(imx319);
		if (ret)
			goto error_entity;
		full_power = true;
	} else {
		full_power = acpi_dev_state_d0(imx319->dev);
	}
	if (full_power) {
		ret = imx319_identify_module(imx319);
		if (ret)
			goto error_power;
		ret = pm_runtime_set_active(imx319->dev);
		if (ret)
			goto error_power;
	}
	if (imx319->managed_power) {
		pm_runtime_get_noresume(imx319->dev);
		probe_reference = true;
		pm_runtime_set_autosuspend_delay(imx319->dev, 1000);
		pm_runtime_use_autosuspend(imx319->dev);
	}
	pm_runtime_enable(imx319->dev);
	ret = v4l2_async_register_subdev_sensor(&imx319->sd);
	if (ret)
		goto error_pm;
	if (probe_reference) {
		pm_runtime_mark_last_busy(imx319->dev);
		pm_runtime_put_autosuspend(imx319->dev);
	} else {
		pm_runtime_idle(imx319->dev);
	}
	return 0;
error_pm:
	pm_runtime_disable(imx319->dev);
	if (imx319->managed_power)
		pm_runtime_dont_use_autosuspend(imx319->dev);
	if (probe_reference)
		pm_runtime_put_noidle(imx319->dev);
error_power:
	cleanup_ret = imx319_power_off(imx319);
	if (cleanup_ret)
		dev_err(imx319->dev, "probe power rollback incomplete: %d\n", cleanup_ret);
	else if (full_power)
		pm_runtime_set_suspended(imx319->dev);
error_entity:
	media_entity_cleanup(&imx319->sd.entity);
error_handler:
	v4l2_ctrl_handler_free(&imx319->ctrl_handler);
error_probe:
	mutex_destroy(&imx319->mutex);
	return dev_err_probe(&client->dev, ret, "sensor probe failed\n");
}

static void imx319_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx319 *imx319 = to_imx319(sd);
	int ret;

	v4l2_async_unregister_subdev(sd);
	mutex_lock(&imx319->mutex);
	imx319->removing = true;
	mutex_unlock(&imx319->mutex);
	imx319_set_stream(sd, 0);
	pm_runtime_disable(imx319->dev);
	if (imx319->managed_power)
		pm_runtime_dont_use_autosuspend(imx319->dev);
	ret = imx319_power_off(imx319);
	if (ret)
		dev_err(imx319->dev, "remove power rollback incomplete: %d\n", ret);
	else if (!pm_runtime_status_suspended(imx319->dev))
		pm_runtime_set_suspended(imx319->dev);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	mutex_destroy(&imx319->mutex);
}


static const struct acpi_device_id imx319_acpi_ids[] __maybe_unused = {
	{ "SONY319A" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, imx319_acpi_ids);

static const struct of_device_id imx319_of_match[] = {
	{ .compatible = "sony,imx319" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx319_of_match);

static struct i2c_driver imx319_i2c_driver = {
	.driver = {
		.name = "imx319",
		.acpi_match_table = ACPI_PTR(imx319_acpi_ids),
		.of_match_table = imx319_of_match,
		.pm = pm_ptr(&imx319_pm_ops),
	},
	.probe = imx319_probe,
	.remove = imx319_remove,
	.flags = I2C_DRV_ACPI_WAIVE_D0_PROBE,
};
module_i2c_driver(imx319_i2c_driver);

MODULE_AUTHOR("Qiu, Tianshu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Rapolu, Chiranjeevi");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Yang, Hyungwoo");
MODULE_DESCRIPTION("Sony imx319 sensor driver");
MODULE_LICENSE("GPL v2");
