// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung SOFEF03F_M, Realme X2 Pro RMX1931.
 * Initialization and power sequencing from the handset's stock live DT and
 * realmeX2Pro-kernel-source, commit 9668fcdc6ec15be7a10d66f7b93c347829e0fdb6.
 * Vendor sequences: Copyright (c) 2018, The Linux Foundation.
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <video/mipi_display.h>

struct sofef03f {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset;
	struct gpio_desc *vci;
	struct gpio_desc *vddd;
	bool supply_enabled[2];
	bool prefer_90;
	bool active_90;
	struct delayed_work backlight_work;
	unsigned int backlight_retries;
	bool post_on_sent;
	bool backlight_reapplied;
	unsigned int last_brightness;
	bool hbm_enabled;
	struct mutex lock;
};

static const struct regulator_bulk_data sofef03f_supplies[] = {
	{ .supply = "vddio", .init_load_uA = 62000 },
	{ .supply = "vdda", .init_load_uA = 100000 },
};

static inline struct sofef03f *to_sofef03f(struct drm_panel *panel)
{
	return container_of(panel, struct sofef03f, panel);
}

/* The stock Samsung DDIC receives PPS as DCS command 0x9e. */
static const u8 sofef03f_stock_pps[128] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38, 0x00, 0x1e, 0x02, 0x1c, 0x02, 0x1c,
	0x02, 0x00, 0x02, 0x0e, 0x00, 0x20, 0x02, 0xe3, 0x00, 0x07, 0x00, 0x0c, 0x03, 0x50, 0x03, 0x64,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static int sofef03f_refresh = 90;
module_param_named(refresh, sofef03f_refresh, int, 0444);
MODULE_PARM_DESC(refresh, "Preferred panel refresh rate (60 or 90)");

static bool sofef03f_active_90hz(struct sofef03f *ctx);

static int sofef03f_write_brightness_locked(struct sofef03f *ctx)
{
	struct backlight_device *bl = ctx->panel.backlight;
	unsigned int mode_flags = ctx->dsi->mode_flags;
	unsigned int brightness = backlight_get_brightness(bl);
	bool use_hbm = brightness > 1023;
	u8 hbm_value;
	int ret;

	if (!ctx->panel.prepared || !READ_ONCE(ctx->panel.backlight_ready))
		return 0;

	if (!ctx->post_on_sent) {
		/* Stock post-on-backlight is dsi_hs_mode. */
		ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
		ctx->dsi->mode_flags = mode_flags;
		if (ret < 0) {
			dev_warn(&ctx->dsi->dev,
				 "Post-on display-on failed: %d\n", ret);
			return ret;
		}
		usleep_range(20000, 21000);
		ctx->post_on_sent = true;
		dev_info(&ctx->dsi->dev, "post-on display-on completed\n");
	}

	if (use_hbm && !ctx->hbm_enabled) {
		hbm_value = 0xe0;
		ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					 &hbm_value, sizeof(hbm_value));
		ctx->dsi->mode_flags = mode_flags;
		if (ret < 0) {
			dev_warn(&ctx->dsi->dev,
				 "HBM enable failed: %d\n", ret);
			return ret;
		}
		ctx->hbm_enabled = true;
	}

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, brightness);
	ctx->dsi->mode_flags = mode_flags;
	if (ret < 0) {
		dev_warn(&ctx->dsi->dev,
			 "Brightness write %u failed: %d\n", brightness, ret);
		return ret;
	}
	if (brightness != ctx->last_brightness) {
		dev_info(&ctx->dsi->dev, "brightness wrote %u\n", brightness);
		ctx->last_brightness = brightness;
	}

	if (!use_hbm && ctx->hbm_enabled) {
		hbm_value = 0x20;
		ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					 &hbm_value, sizeof(hbm_value));
		ctx->dsi->mode_flags = mode_flags;
		if (ret < 0) {
			dev_warn(&ctx->dsi->dev,
				 "HBM disable failed: %d\n", ret);
			return ret;
		}
		ctx->hbm_enabled = false;
	}
	return 0;
}

static void sofef03f_backlight_work(struct work_struct *work)
{
	struct sofef03f *ctx = container_of(work, struct sofef03f,
					    backlight_work.work);
	int ret;

	mutex_lock(&ctx->lock);
	if (!ctx->panel.prepared) {
		mutex_unlock(&ctx->lock);
		return;
	}
	if (!READ_ONCE(ctx->panel.backlight_ready)) {
		if (ctx->backlight_retries++ < 5)
			schedule_delayed_work(&ctx->backlight_work,
					      msecs_to_jiffies(100));
		mutex_unlock(&ctx->lock);
		return;
	}

	ret = sofef03f_write_brightness_locked(ctx);
	if (ret < 0) {
		if (ctx->backlight_retries++ < 5)
			schedule_delayed_work(&ctx->backlight_work,
					      msecs_to_jiffies(100));
	} else if (!ctx->backlight_reapplied) {
		ctx->backlight_reapplied = true;
		schedule_delayed_work(&ctx->backlight_work,
				      msecs_to_jiffies(400));
	}
	mutex_unlock(&ctx->lock);
}

static int sofef03f_on(struct sofef03f *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	u8 command[1 + sizeof(pps)];

	ctx->active_90 = sofef03f_active_90hz(ctx);
	dev_info(&ctx->dsi->dev, "initializing panel at %d Hz\n",
		 ctx->active_90 ? 90 : 60);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	if (memcmp(&pps, sofef03f_stock_pps, sizeof(pps))) {
		dev_err(&ctx->dsi->dev, "Calculated DSC PPS differs from stock\n");
		print_hex_dump(KERN_ERR, "sofef03f PPS: ", DUMP_PREFIX_OFFSET,
			       16, 1, &pps, sizeof(pps), false);
		return -EINVAL;
	}
	command[0] = 0x9e;
	memcpy(command + 1, &pps, sizeof(pps));

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9d, 0x01);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, command, sizeof(command));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x00, 0x00, 0x04, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x00, 0x00, 0x09, 0x5f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	if (ctx->active_90)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x53);
	else
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x5b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x00, 0xc2, 0xc2, 0x42);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x5d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd3, 0x29, 0x00, 0x00, 0x0d, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x78);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd3, 0x4f, 0x00, 0x00, 0x06, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0xaf);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd3, 0x29, 0x00, 0x00, 0x0d, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd9, 0x72);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	if (ctx->active_90)
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x10);
	else
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_msleep(&dsi_ctx, 70);
	return dsi_ctx.accum_err;
}

/* Keep our regulator references balanced even when disable fails. */
static int sofef03f_supply_enable(struct sofef03f *ctx, unsigned int index)
{
	int ret;

	if (ctx->supply_enabled[index])
		return 0;
	ret = regulator_enable(ctx->supplies[index].consumer);
	if (!ret)
		ctx->supply_enabled[index] = true;
	return ret;
}

static void sofef03f_supply_disable(struct sofef03f *ctx, unsigned int index)
{
	int ret;

	if (!ctx->supply_enabled[index])
		return;
	ret = regulator_disable(ctx->supplies[index].consumer);
	if (ret)
		dev_err(&ctx->dsi->dev, "Failed to disable %s: %d\n",
			ctx->supplies[index].supply, ret);
	else
		ctx->supply_enabled[index] = false;
}

static void sofef03f_power_off(struct sofef03f *ctx)
{
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(22000, 22500);
	gpiod_set_value_cansleep(ctx->vddd, 0);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->vci, 0);
	usleep_range(1000, 1100);
	sofef03f_supply_disable(ctx, 1);
	sofef03f_supply_disable(ctx, 0);
}

/* Configure outputs only after KMS has stopped the firmware scanout. */
static int sofef03f_init_gpios(struct sofef03f *ctx)
{
	int ret;

	ret = gpiod_direction_output(ctx->reset, 1);
	if (ret)
		return ret;
	ret = gpiod_direction_output(ctx->vddd, 0);
	if (ret)
		return ret;
	return gpiod_direction_output(ctx->vci, 0);
}

static int sofef03f_prepare(struct drm_panel *panel)
{
	struct sofef03f *ctx = to_sofef03f(panel);
	int ret;

	ret = sofef03f_init_gpios(ctx);
	if (ret)
		return ret;

	ret = sofef03f_supply_enable(ctx, 0);
	if (ret)
		return ret;
	msleep(20);
	ret = sofef03f_supply_enable(ctx, 1);
	if (ret) {
		sofef03f_supply_disable(ctx, 0);
		return ret;
	}
	gpiod_set_value_cansleep(ctx->vci, 1);
	/* Stock OPLUS values are microseconds, not milliseconds. */
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->vddd, 1);
	usleep_range(1000, 1100);
	/* Logical reset is active low: stock raw levels 1/0/1, 5/10/10 ms. */
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(5000, 5100);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(10000, 10100);
	ret = sofef03f_on(ctx);
	if (ret)
		sofef03f_power_off(ctx);
	else
		dev_info(&ctx->dsi->dev, "panel initialization complete\n");
	return ret;
}

static int sofef03f_enable(struct drm_panel *panel)
{
	struct sofef03f *ctx = to_sofef03f(panel);

	/*
	 * Stock 19781 keeps display-on out of the LP on-command and sends it
	 * from post-on-backlight in HS mode after the first frame.
	 */
	mutex_lock(&ctx->lock);
	ctx->backlight_retries = 0;
	ctx->post_on_sent = false;
	ctx->backlight_reapplied = false;
	ctx->last_brightness = ~0U;
	ctx->hbm_enabled = false;
	mutex_unlock(&ctx->lock);
	schedule_delayed_work(&ctx->backlight_work,
			      msecs_to_jiffies(100));
	return 0;
}

static int sofef03f_disable(struct drm_panel *panel)
{
	struct sofef03f *ctx = to_sofef03f(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct mipi_dsi_multi_context pre_off = { .dsi = ctx->dsi };

	cancel_delayed_work_sync(&ctx->backlight_work);
	mutex_lock(&ctx->lock);
	ctx->backlight_retries = 0;
	ctx->post_on_sent = false;
	ctx->backlight_reapplied = false;
	/*
	 * Stock off-command is display-on, display-off, sleep-in.  Keep the
	 * leading display-on even though the panel is already on; the DDIC
	 * needs this sequence before the next prepare/sleep-out.
	 */
	mipi_dsi_dcs_set_display_on_multi(&pre_off);
	mipi_dsi_msleep(&pre_off, 20);
	if (pre_off.accum_err)
		dev_warn(&ctx->dsi->dev, "Pre-off display-on failed: %d\n",
			 pre_off.accum_err);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	/* Shutdown must progress to unprepare, including after a DCS timeout. */
	msleep(22);
	if (dsi_ctx.accum_err)
		dev_warn(&ctx->dsi->dev, "Display-off failed: %d; forcing teardown\n",
			 dsi_ctx.accum_err);
	mutex_unlock(&ctx->lock);
	return 0;
}

static int sofef03f_unprepare(struct drm_panel *panel)
{
	struct sofef03f *ctx = to_sofef03f(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mutex_lock(&ctx->lock);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	/* Preserve the stock wait even if the command bus is unresponsive. */
	msleep(120);
	sofef03f_power_off(ctx);
	if (dsi_ctx.accum_err)
		dev_warn(&ctx->dsi->dev, "Sleep failed: %d; panel reset and power removed\n",
			 dsi_ctx.accum_err);
	/* Reset and VCI/VDDD are off: DRM must allow the next prepare/enable. */
	mutex_unlock(&ctx->lock);
	return 0;
}

static const struct drm_display_mode sofef03f_mode_60 = {
	.clock = (1080 + 32 + 16 + 16) * (2400 + 12 + 4 + 12) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 32,
	.hsync_end = 1080 + 32 + 16,
	.htotal = 1080 + 32 + 16 + 16,
	.vdisplay = 2400,
	.vsync_start = 2400 + 12,
	.vsync_end = 2400 + 12 + 4,
	.vtotal = 2400 + 12 + 4 + 12,
	.width_mm = 68,
	.height_mm = 152,
	.type = DRM_MODE_TYPE_DRIVER,
};

/* Stock 90 Hz timing: vertical front porch shrinks from 12 to 4 lines. */
static const struct drm_display_mode sofef03f_mode_90 = {
	.clock = (1080 + 32 + 16 + 16) * (2400 + 12 + 4 + 4) * 90 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 32,
	.hsync_end = 1080 + 32 + 16,
	.htotal = 1080 + 32 + 16 + 16,
	.vdisplay = 2400,
	.vsync_start = 2400 + 12,
	.vsync_end = 2400 + 12 + 4,
	.vtotal = 2400 + 12 + 4 + 4,
	.width_mm = 68,
	.height_mm = 152,
	.type = DRM_MODE_TYPE_DRIVER,
};

static bool sofef03f_active_90hz(struct sofef03f *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;

	if (!connector || !connector->state || !connector->state->crtc ||
	    !connector->state->crtc->state)
		return ctx->prefer_90;

	crtc_state = connector->state->crtc->state;
	if (drm_mode_match(&crtc_state->mode, &sofef03f_mode_60,
			   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
		return false;
	if (drm_mode_match(&crtc_state->mode, &sofef03f_mode_90,
			   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
		return true;

	dev_warn(&ctx->dsi->dev,
		 "unknown mode " DRM_MODE_FMT ", using %d Hz panel command\n",
		 DRM_MODE_ARG(&crtc_state->mode), ctx->prefer_90 ? 90 : 60);
	return ctx->prefer_90;
}

static int sofef03f_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct sofef03f *ctx = to_sofef03f(panel);
	const struct drm_display_mode *modes[] = {
		ctx->prefer_90 ? &sofef03f_mode_90 : &sofef03f_mode_60,
		ctx->prefer_90 ? &sofef03f_mode_60 : &sofef03f_mode_90,
	};
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		mode = drm_mode_duplicate(connector->dev, modes[i]);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);
		mode->type |= DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = modes[0]->width_mm;
	connector->display_info.height_mm = modes[0]->height_mm;
	ctx->connector = connector;

	return ARRAY_SIZE(modes);
}

static const struct drm_panel_funcs sofef03f_funcs = {
	.prepare = sofef03f_prepare,
	.enable = sofef03f_enable,
	.disable = sofef03f_disable,
	.unprepare = sofef03f_unprepare,
	.get_modes = sofef03f_get_modes,
};

static int sofef03f_bl_update(struct backlight_device *bl)
{
	struct sofef03f *ctx = bl_get_data(bl);
	int ret;

	mutex_lock(&ctx->lock);
	ret = sofef03f_write_brightness_locked(ctx);
	mutex_unlock(&ctx->lock);
	return ret;
}

static const struct backlight_ops sofef03f_bl_ops = {
	.update_status = sofef03f_bl_update,
};

static int sofef03f_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sofef03f *ctx;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW, .brightness = 400, .max_brightness = 2047,
		.power = BACKLIGHT_POWER_OFF,
	};
	int ret;

	if (sofef03f_refresh != 60 && sofef03f_refresh != 90)
		return dev_err_probe(dev, -EINVAL, "refresh must be 60 or 90\n");

	ctx = devm_drm_panel_alloc(dev, struct sofef03f, panel,
				   &sofef03f_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	ctx->dsi = dsi;
	mutex_init(&ctx->lock);
	INIT_DELAYED_WORK(&ctx->backlight_work, sofef03f_backlight_work);
	ctx->prefer_90 = sofef03f_refresh == 90;
	ctx->active_90 = ctx->prefer_90;
	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(sofef03f_supplies),
					    sofef03f_supplies, &ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");
	/* mipi_dsi_attach() must quiesce the boot frame before panel reset. */
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "Failed to get reset\n");
	ctx->vci = devm_gpiod_get(dev, "vci-enable", GPIOD_ASIS);
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci), "Failed to get VCI enable\n");
	ctx->vddd = devm_gpiod_get(dev, "vddd-enable", GPIOD_ASIS);
	if (IS_ERR(ctx->vddd))
		return dev_err_probe(dev, PTR_ERR(ctx->vddd), "Failed to get VDDD enable\n");
	ctx->panel.backlight = devm_backlight_device_register(dev, dev_name(dev),
				dev, ctx, &sofef03f_bl_ops, &props);
	if (IS_ERR(ctx->panel.backlight))
		return PTR_ERR(ctx->panel.backlight);
	ctx->panel.prepare_prev_first = true;
	ctx->panel.delay_backlight_until_first_frame = true;
	mipi_dsi_set_drvdata(dsi, ctx);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Stock host config disables EOT append and forced HS clock. */
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_count = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	dsi->dsc = &ctx->dsc;
	drm_panel_add(&ctx->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&ctx->panel);
	return ret;
}

static void sofef03f_remove(struct mipi_dsi_device *dsi)
{
	struct sofef03f *ctx = mipi_dsi_get_drvdata(dsi);

	cancel_delayed_work_sync(&ctx->backlight_work);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id sofef03f_match[] = {
	{ .compatible = "samsung,sofef03f-m" },
	{ }
};
MODULE_DEVICE_TABLE(of, sofef03f_match);

static struct mipi_dsi_driver sofef03f_driver = {
	.probe = sofef03f_probe,
	.remove = sofef03f_remove,
	.driver = { .name = "panel-samsung-sofef03f", .of_match_table = sofef03f_match },
};
module_mipi_dsi_driver(sofef03f_driver);
MODULE_DESCRIPTION("Samsung SOFEF03F_M DSC command mode panel");
MODULE_LICENSE("GPL");
