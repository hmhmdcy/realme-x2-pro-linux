// SPDX-License-Identifier: GPL-2.0-only
/* Samsung S5KGW1 RAW10 C-PHY sensor, 19.2 MHz input clock. */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5KGW1_CHIP_ID		0x0971
#define S5KGW1_MCLK		19200000
#define S5KGW1_CODE		MEDIA_BUS_FMT_SGBRG10_1X10
#define S5KGW1_FRAME_MAX		0xffff
#define S5KGW1_EXPOSURE_MIN	6
#define S5KGW1_EXPOSURE_MARGIN	6

/* OP PLL: 19.2 MHz / 3 * 312 / 2; LINK_FREQ is half the symbol rate.
 * VT PLL candidate: 19.2 MHz / 3 * 150 * 2. Physical timing is unverified.
 */
static const s64 s5kgw1_link_freqs[] = { 499200000 };
#define S5KGW1_PIXEL_RATE		1920000000LL

struct s5kgw1_reg {
	u16 address;
	u16 value;
	u8 value_bytes;
	const u16 *burst;
	u16 count;
	u32 delay_us;
};

#include "s5kgw1-tables.h"

struct s5kgw1_mode {
	u32 width;
	u32 height;
	u32 line_length;
	u32 frame_length;
	struct v4l2_rect crop;
	const struct s5kgw1_reg *regs;
	unsigned int num_regs;
};

/* 2x2 binning followed by eight binned rows of top/bottom trim (0x0352).
 * Full-resolution and high-speed/HDR modes remain to be integrated.
 */
static const struct s5kgw1_mode s5kgw1_modes[] = {
	{
		.width = 4608, .height = 3456,
		.line_length = 17824, .frame_length = 3584,
		.crop = { 32, 16, 9216, 6912 },
		.regs = s5kgw1_mode2_regs,
		.num_regs = ARRAY_SIZE(s5kgw1_mode2_regs),
	}, {
		.width = 4608, .height = 2592,
		.line_length = 17824, .frame_length = 3584,
		.crop = { 32, 880, 9216, 5184 },
		.regs = s5kgw1_mode4_regs,
		.num_regs = ARRAY_SIZE(s5kgw1_mode4_regs),
	},
};

enum s5kgw1_supply {
	S5KGW1_VANA,
	S5KGW1_VDIG,
	S5KGW1_VIO,
	S5KGW1_VAF,
	S5KGW1_NUM_SUPPLIES,
};

static const char * const s5kgw1_supply_names[] = {
	"vdda", "vddd", "vddio", "afvdd",
};

struct s5kgw1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clock;
	struct gpio_desc *reset;
	struct regulator *supplies[S5KGW1_NUM_SUPPLIES];
	bool supply_enabled[S5KGW1_NUM_SUPPLIES];
	bool clock_enabled;
	bool power_fault;
	bool streaming;
	bool removing;
	unsigned int burst_bytes;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	const struct s5kgw1_mode *mode;
};

static inline struct s5kgw1 *to_s5kgw1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5kgw1, sd);
}

static int s5kgw1_write_table(struct s5kgw1 *sensor,
			    const struct s5kgw1_reg *regs, unsigned int count)
{
	u8 payload[8];
	unsigned int i, offset, words, j;
	int ret;

	for (i = 0; i < count; i++) {
		const struct s5kgw1_reg *r = &regs[i];

		if (!r->burst) {
			ret = cci_write(sensor->regmap,
					r->value_bytes == 1 ? CCI_REG8(r->address) :
					CCI_REG16(r->address),
					r->value, NULL);
			if (ret)
				return ret;
		} else {
			/* 0x6f12 is an indirect RAM FIFO, not consecutive registers.
			 * Every chunk must repeat this register address. The sensor's
			 * internal pointer advances, including between I2C transfers.
			 */
			if (r->address != 0x6f12 || !r->count)
				return -EINVAL;
			for (offset = 0; offset < r->count; offset += words) {
				words = min_t(unsigned int, r->count - offset,
						 sensor->burst_bytes / 2);
				for (j = 0; j < words; j++)
					put_unaligned_be16(r->burst[offset + j],
							   &payload[2 * j]);
				ret = regmap_raw_write(sensor->regmap, r->address,
						       payload, 2 * words);
				if (ret)
					return ret;
			}
		}
		if (r->delay_us)
			usleep_range(r->delay_us, r->delay_us + 100);
	}
	return 0;
}

static int s5kgw1_power_off(struct device *dev)
{
	struct s5kgw1 *sensor = to_s5kgw1(dev_get_drvdata(dev));
	static const unsigned int order[] = {
		S5KGW1_VIO, S5KGW1_VDIG, S5KGW1_VAF, S5KGW1_VANA,
	};
	unsigned int i, index;
	int ret, first = 0;

	if (sensor->clock_enabled) {
		clk_disable_unprepare(sensor->clock);
		sensor->clock_enabled = false;
	}
	usleep_range(1000, 1100);
	ret = gpiod_set_value_cansleep(sensor->reset, 1);
	if (ret)
		first = ret;
	usleep_range(1000, 1100);
	for (i = 0; i < ARRAY_SIZE(order); i++) {
		index = order[i];
		if (sensor->supply_enabled[index]) {
			ret = regulator_disable(sensor->supplies[index]);
			if (ret && !first)
				first = ret;
			if (!ret)
				sensor->supply_enabled[index] = false;
		}
		if (index == S5KGW1_VAF)
			usleep_range(1000, 1100);
	}
	ret = pinctrl_pm_select_sleep_state(dev);
	if (ret && !first)
		first = ret;
	if (first)
		sensor->power_fault = true;
	return first;
}

static int s5kgw1_power_on(struct device *dev)
{
	struct s5kgw1 *sensor = to_s5kgw1(dev_get_drvdata(dev));
	unsigned int i;
	int ret, cleanup_ret;

	if (sensor->power_fault || sensor->clock_enabled)
		return -EIO;
	for (i = 0; i < S5KGW1_NUM_SUPPLIES; i++)
		if (sensor->supply_enabled[i])
			return -EIO;
	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		goto failed;
	ret = gpiod_set_value_cansleep(sensor->reset, 1);
	if (ret)
		goto failed;
	usleep_range(1000, 1100);
	/* OEM order: VANA, VDIG, VIO, VAF; optional VAF is a module rail. */
	for (i = 0; i < S5KGW1_NUM_SUPPLIES; i++) {
		if (sensor->supplies[i]) {
			ret = regulator_enable(sensor->supplies[i]);
			if (ret)
				goto failed;
			sensor->supply_enabled[i] = true;
		}
		if (i != S5KGW1_VIO)
			usleep_range(1000, 1100);
	}
	ret = gpiod_set_value_cansleep(sensor->reset, 0);
	if (ret)
		goto failed;
	usleep_range(1000, 1100);
	ret = clk_prepare_enable(sensor->clock);
	if (ret)
		goto failed;
	sensor->clock_enabled = true;
	usleep_range(5000, 5500);
	return 0;
failed:
	cleanup_ret = s5kgw1_power_off(dev);
	if (cleanup_ret)
		dev_err(dev, "power rollback incomplete: %d\n", cleanup_ret);
	return ret;
}

static void s5kgw1_cleanup_power(void *data)
{
	struct s5kgw1 *sensor = data;
	unsigned int i;
	bool enabled = sensor->clock_enabled;
	int ret;

	for (i = 0; i < S5KGW1_NUM_SUPPLIES; i++)
		enabled |= sensor->supply_enabled[i];
	if (!enabled)
		return;
	ret = s5kgw1_power_off(sensor->dev);
	if (ret)
		dev_err(sensor->dev, "sensor power cleanup incomplete: %d\n", ret);
}

static int s5kgw1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgw1 *sensor = container_of(ctrl->handler, struct s5kgw1,
					    controls);
	s64 maximum;
	int ret, release_ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		maximum = sensor->mode->height + ctrl->val -
			  S5KGW1_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(sensor->exposure, S5KGW1_EXPOSURE_MIN,
					 maximum, 1, min_t(s64, maximum,
					 sensor->exposure->default_value));
	}
	if (sensor->removing || sensor->power_fault)
		return -EIO;
	ret = pm_runtime_get_if_in_use(sensor->dev);
	if (ret <= 0)
		return ret;

	ret = cci_write(sensor->regmap, CCI_REG8(0x0104), 1, NULL);
	if (ret)
		goto release;
	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, CCI_REG16(0x0204), ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, CCI_REG16(0x0202), ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, CCI_REG16(0x0340),
			  sensor->mode->height + ctrl->val, &ret);
		cci_write(sensor->regmap, CCI_REG16(0x0202),
			  sensor->exposure->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	/* Release hold even when a preceding I2C write failed. */
release:
	release_ret = cci_write(sensor->regmap, CCI_REG8(0x0104), 0, NULL);
	if (!ret)
		ret = release_ret;
	if (release_ret)
		sensor->power_fault = true;
	pm_runtime_put(sensor->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5kgw1_ctrl_ops = {
	.s_ctrl = s5kgw1_set_ctrl,
};

static int s5kgw1_init_controls(struct s5kgw1 *sensor)
{
	struct v4l2_ctrl_handler *h = &sensor->controls;
	const struct s5kgw1_mode *mode = sensor->mode;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u32 hb = mode->line_length - mode->width;
	u32 vb = mode->frame_length - mode->height;
	int ret;

	v4l2_ctrl_handler_init(h, 9);
	ctrl = v4l2_ctrl_new_int_menu(h, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      s5kgw1_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(h, NULL, V4L2_CID_PIXEL_RATE, S5KGW1_PIXEL_RATE,
			  S5KGW1_PIXEL_RATE, 1, S5KGW1_PIXEL_RATE);
	sensor->hblank = v4l2_ctrl_new_std(h, NULL, V4L2_CID_HBLANK,
					 hb, hb, 1, hb);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->vblank = v4l2_ctrl_new_std(h, &s5kgw1_ctrl_ops,
		V4L2_CID_VBLANK, vb, S5KGW1_FRAME_MAX - mode->height, 1, vb);
	sensor->exposure = v4l2_ctrl_new_std(h, &s5kgw1_ctrl_ops,
		V4L2_CID_EXPOSURE, S5KGW1_EXPOSURE_MIN,
		mode->frame_length - S5KGW1_EXPOSURE_MARGIN, 1,
		mode->frame_length - S5KGW1_EXPOSURE_MARGIN);
	/* Native code = floor(gain * 32), 1..32x for the normal modes. */
	v4l2_ctrl_new_std(h, &s5kgw1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  32, 1024, 1, 32);
	ret = h->error;
	if (ret)
		goto failed;
	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto failed;
	ret = v4l2_ctrl_new_fwnode_properties(h, &s5kgw1_ctrl_ops, &props);
	if (ret)
		goto failed;
	sensor->sd.ctrl_handler = h;
	return 0;
failed:
	v4l2_ctrl_handler_free(h);
	return ret;
}

static int s5kgw1_enable_streams(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	struct s5kgw1 *sensor = to_s5kgw1(sd);
	int ret, cleanup_ret;

	if (sensor->removing || sensor->power_fault)
		return -EIO;
	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;
	ret = s5kgw1_write_table(sensor, s5kgw1_init_regs,
				ARRAY_SIZE(s5kgw1_init_regs));
	if (ret)
		goto failed;
	ret = s5kgw1_write_table(sensor, sensor->mode->regs,
				sensor->mode->num_regs);
	if (ret)
		goto failed;
	ret = __v4l2_ctrl_handler_setup(&sensor->controls);
	if (ret)
		goto failed;
	ret = cci_write(sensor->regmap, CCI_REG8(0x0100), 1, NULL);
	if (ret)
		goto failed;
	sensor->streaming = true;
	return 0;
failed:
	cleanup_ret = cci_write(sensor->regmap, CCI_REG8(0x0100), 0, NULL);
	if (cleanup_ret)
		dev_warn(sensor->dev, "stream rollback needs power off: %d\n",
			 cleanup_ret);
	cleanup_ret = pm_runtime_put_sync_suspend(sensor->dev);
	if (cleanup_ret < 0)
		sensor->power_fault = true;
	return ret;
}

static int s5kgw1_disable_streams(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	struct s5kgw1 *sensor = to_s5kgw1(sd);
	int ret, suspend_ret;

	if (!sensor->streaming)
		return 0;
	ret = cci_write(sensor->regmap, CCI_REG8(0x0100), 0, NULL);
	/* Suspend asserts reset and removes the sensor's supply references. */
	sensor->streaming = false;
	suspend_ret = pm_runtime_put_sync_suspend(sensor->dev);
	if (suspend_ret < 0) {
		sensor->power_fault = true;
		if (!ret)
			ret = suspend_ret;
	}
	return ret;
}

static void s5kgw1_format(const struct s5kgw1_mode *mode,
			 struct v4l2_mbus_framefmt *fmt)
{
	*fmt = (struct v4l2_mbus_framefmt) {
		.width = mode->width, .height = mode->height,
		.code = S5KGW1_CODE, .field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
}

static int s5kgw1_set_format(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state, struct v4l2_subdev_format *fmt)
{
	struct s5kgw1 *sensor = to_s5kgw1(sd);
	const struct s5kgw1_mode *mode;
	u32 hb, vb;
	int ret;

	if (fmt->pad || fmt->stream)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sensor->streaming)
		return -EBUSY;
	mode = v4l2_find_nearest_size(s5kgw1_modes, ARRAY_SIZE(s5kgw1_modes),
			width, height, fmt->format.width, fmt->format.height);
	s5kgw1_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->mode = mode;
		hb = mode->line_length - mode->width;
		vb = mode->frame_length - mode->height;
		__v4l2_ctrl_modify_range(sensor->hblank, hb, hb, 1, hb);
		__v4l2_ctrl_modify_range(sensor->vblank, vb,
				 S5KGW1_FRAME_MAX - mode->height, 1, vb);
		ret = __v4l2_ctrl_s_ctrl(sensor->vblank, vb);
		if (ret)
			return ret;
	}
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	return 0;
}

static int s5kgw1_enum_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->stream || code->index)
		return -EINVAL;
	code->code = S5KGW1_CODE;
	return 0;
}

static int s5kgw1_enum_size(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_frame_size_enum *fse)
{
	const struct s5kgw1_mode *mode;

	if (fse->pad || fse->stream || fse->code != S5KGW1_CODE ||
	    fse->index >= ARRAY_SIZE(s5kgw1_modes))
		return -EINVAL;
	mode = &s5kgw1_modes[fse->index];
	fse->min_width = fse->max_width = mode->width;
	fse->min_height = fse->max_height = mode->height;
	return 0;
}

static int s5kgw1_get_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *fmt;
	const struct s5kgw1_mode *mode;

	if (sel->pad || sel->stream)
		return -EINVAL;
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect) { 0, 0, 9280, 6944 };
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = s5kgw1_modes[0].crop;
		return 0;
	case V4L2_SEL_TGT_CROP:
		fmt = v4l2_subdev_state_get_format(state, 0);
		mode = v4l2_find_nearest_size(s5kgw1_modes,
				ARRAY_SIZE(s5kgw1_modes), width, height,
				fmt->width, fmt->height);
		sel->r = mode->crop;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5kgw1_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	s5kgw1_format(&s5kgw1_modes[0], v4l2_subdev_state_get_format(state, 0));
	return 0;
}

static const struct v4l2_subdev_video_ops s5kgw1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5kgw1_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5kgw1_set_format,
	.enum_mbus_code = s5kgw1_enum_code,
	.enum_frame_size = s5kgw1_enum_size,
	.get_selection = s5kgw1_get_selection,
	.enable_streams = s5kgw1_enable_streams,
	.disable_streams = s5kgw1_disable_streams,
};

static const struct v4l2_subdev_ops s5kgw1_ops = {
	.video = &s5kgw1_video_ops,
	.pad = &s5kgw1_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kgw1_internal_ops = {
	.init_state = s5kgw1_init_state,
};

static const struct media_entity_operations s5kgw1_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5kgw1_check_endpoint(struct s5kgw1 *sensor)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_CPHY };
	struct fwnode_handle *fwnode;
	unsigned int i;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!fwnode)
		return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return ret;
	ret = -EINVAL;
	if (ep.bus.mipi_csi2.num_data_lanes != 3 || ep.nr_of_link_frequencies != 1 ||
	    ep.link_frequencies[0] != s5kgw1_link_freqs[0])
		goto out;
	for (i = 0; i < 3; i++)
		if (ep.bus.mipi_csi2.data_lanes[i] != i ||
		    ep.bus.mipi_csi2.line_orders[i] != V4L2_MBUS_CSI2_CPHY_LINE_ORDER_ABC)
			goto out;
	ret = 0;
out:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int s5kgw1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5kgw1 *sensor;
	unsigned int i;
	u64 id;
	int ret, off_ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->dev = dev;
	v4l2_i2c_subdev_init(&sensor->sd, client, &s5kgw1_ops);
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap), "CCI unavailable\n");
	sensor->burst_bytes = 8;
	if (client->adapter->quirks && client->adapter->quirks->max_write_len) {
		unsigned int limit = client->adapter->quirks->max_write_len;

		if (limit < 4)
			return -EOPNOTSUPP;
		sensor->burst_bytes = min(8U, (limit - 2) & ~1U);
	}
	sensor->clock = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->clock))
		return dev_err_probe(dev, PTR_ERR(sensor->clock), "MCLK unavailable\n");
	if (clk_get_rate(sensor->clock) != S5KGW1_MCLK)
		return dev_err_probe(dev, -EINVAL, "19.2 MHz MCLK required\n");
	ret = s5kgw1_check_endpoint(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "three-trio C-PHY endpoint required\n");
	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset), "reset unavailable\n");
	for (i = 0; i < S5KGW1_NUM_SUPPLIES; i++) {
		sensor->supplies[i] = devm_regulator_get_optional(dev,
						      s5kgw1_supply_names[i]);
		if (IS_ERR(sensor->supplies[i])) {
			ret = PTR_ERR(sensor->supplies[i]);
			if (i == S5KGW1_VAF && ret == -ENODEV) {
				sensor->supplies[i] = NULL;
				continue;
			}
			return dev_err_probe(dev, ret, "supply %s unavailable\n",
					     s5kgw1_supply_names[i]);
		}
	}
	ret = devm_add_action_or_reset(dev, s5kgw1_cleanup_power, sensor);
	if (ret)
		return ret;
	ret = s5kgw1_power_on(dev);
	if (ret)
		return ret;
	/* No initialization or mode writes before checking the physical chip ID. */
	ret = cci_read(sensor->regmap, CCI_REG16(0x0000), &id, NULL);
	if (!ret && id != S5KGW1_CHIP_ID)
		ret = -ENODEV;
	off_ret = s5kgw1_power_off(dev);
	if (ret)
		return dev_err_probe(dev, ret, "GW1 identification failed\n");
	if (off_ret)
		return off_ret;
	sensor->mode = &s5kgw1_modes[0];
	ret = s5kgw1_init_controls(sensor);
	if (ret)
		return ret;
	sensor->sd.state_lock = sensor->controls.lock;
	sensor->sd.internal_ops = &s5kgw1_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.ops = &s5kgw1_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto free_controls;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto cleanup_entity;
	ret = pm_runtime_set_suspended(dev);
	if (ret)
		goto cleanup_subdev;
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		pm_runtime_disable(dev);
		goto cleanup_subdev;
	}
	dev_info(dev, "S5KGW1 identified (RAW10 C-PHY)\n");
	return 0;
cleanup_subdev:
	v4l2_subdev_cleanup(&sensor->sd);
cleanup_entity:
	media_entity_cleanup(&sensor->sd.entity);
free_controls:
	v4l2_ctrl_handler_free(&sensor->controls);
	return ret;
}

static void s5kgw1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgw1 *sensor = to_s5kgw1(sd);
	struct v4l2_subdev_state *state;
	int ret;

	v4l2_async_unregister_subdev(sd);
	state = v4l2_subdev_lock_and_get_active_state(sd);
	sensor->removing = true;
	if (sensor->streaming)
		s5kgw1_disable_streams(sd, state, 0, BIT_ULL(0));
	v4l2_subdev_unlock_state(state);
	pm_runtime_disable(sensor->dev);
	ret = s5kgw1_power_off(sensor->dev);
	if (ret)
		dev_err(sensor->dev, "remove power cleanup incomplete: %d\n", ret);
	else
		pm_runtime_set_suspended(sensor->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->controls);
}

static const struct dev_pm_ops s5kgw1_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kgw1_power_off, s5kgw1_power_on, NULL)
};

static const struct of_device_id s5kgw1_of_match[] = {
	{ .compatible = "samsung,s5kgw1" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5kgw1_of_match);

static struct i2c_driver s5kgw1_driver = {
	.driver = {
		.name = "s5kgw1",
		.of_match_table = s5kgw1_of_match,
		.pm = &s5kgw1_pm_ops,
	},
	.probe = s5kgw1_probe,
	.remove = s5kgw1_remove,
};
module_i2c_driver(s5kgw1_driver);

MODULE_DESCRIPTION("Samsung S5KGW1 RAW10 C-PHY sensor");
MODULE_LICENSE("GPL");
