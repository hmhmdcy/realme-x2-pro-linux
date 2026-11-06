// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019, Linaro Limited

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/wcd934x/registers.h>
#include <linux/mfd/wcd934x/wcd934x.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slimbus.h>

#define WCD934X_REGMAP_IRQ_REG(_irq, _off, _mask)		\
	[_irq] = {						\
		.reg_offset = (_off),				\
		.mask = (_mask),				\
		.type = {					\
			.type_reg_offset = (_off),		\
			.types_supported = IRQ_TYPE_EDGE_BOTH,	\
			.type_reg_mask  = (_mask),		\
			.type_level_low_val = (_mask),		\
			.type_level_high_val = (_mask),		\
			.type_falling_val = 0,			\
			.type_rising_val = 0,			\
		},						\
	}

struct wcd934x_priv {
	struct wcd934x_ddata ddata;
	/* Serializes UP, DOWN and removal of status-scoped resources. */
	struct mutex lock;
	bool up;
	bool ready;
};

static struct wcd934x_priv *wcd934x_get_private(struct slim_device *sdev)
{
	struct wcd934x_ddata *ddata = dev_get_drvdata(&sdev->dev);

	if (!ddata)
		return NULL;

	return container_of(ddata, struct wcd934x_priv, ddata);
}

static const struct mfd_cell wcd934x_devices[] = {
	{
		.name = "wcd934x-codec",
	}, {
		.name = "wcd934x-gpio",
		.of_compatible = "qcom,wcd9340-gpio",
	}, {
		.name = "wcd934x-soundwire",
		.of_compatible = "qcom,soundwire-v1.3.0",
	},
};

static const struct regmap_irq wcd934x_irqs[] = {
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_SLIMBUS, 0, BIT(0)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_HPH_PA_OCPL_FAULT, 0, BIT(2)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_HPH_PA_OCPR_FAULT, 0, BIT(3)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_MBHC_SW_DET, 1, BIT(0)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_MBHC_ELECT_INS_REM_DET, 1, BIT(1)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_MBHC_BUTTON_PRESS_DET, 1, BIT(2)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_MBHC_BUTTON_RELEASE_DET, 1, BIT(3)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_MBHC_ELECT_INS_REM_LEG_DET, 1, BIT(4)),
	WCD934X_REGMAP_IRQ_REG(WCD934X_IRQ_SOUNDWIRE, 2, BIT(4)),
};

static const unsigned int wcd934x_config_regs[] = {
	WCD934X_INTR_LEVEL0,
};

static const struct regmap_irq_chip wcd934x_regmap_irq_chip = {
	.name = "wcd934x_irq",
	.status_base = WCD934X_INTR_PIN1_STATUS0,
	.mask_base = WCD934X_INTR_PIN1_MASK0,
	.ack_base = WCD934X_INTR_PIN1_CLEAR0,
	.num_regs = 4,
	.irqs = wcd934x_irqs,
	.num_irqs = ARRAY_SIZE(wcd934x_irqs),
	.config_base = wcd934x_config_regs,
	.num_config_bases = ARRAY_SIZE(wcd934x_config_regs),
	.num_config_regs = 4,
	.set_type_config = regmap_irq_set_type_config_simple,
};

static bool wcd934x_is_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WCD934X_INTR_PIN1_STATUS0...WCD934X_INTR_PIN2_CLEAR3:
	case WCD934X_SWR_AHB_BRIDGE_RD_DATA_0:
	case WCD934X_SWR_AHB_BRIDGE_RD_DATA_1:
	case WCD934X_SWR_AHB_BRIDGE_RD_DATA_2:
	case WCD934X_SWR_AHB_BRIDGE_RD_DATA_3:
	case WCD934X_SWR_AHB_BRIDGE_ACCESS_STATUS:
	case WCD934X_ANA_MBHC_RESULT_3:
	case WCD934X_ANA_MBHC_RESULT_2:
	case WCD934X_ANA_MBHC_RESULT_1:
	case WCD934X_ANA_MBHC_MECH:
	case WCD934X_ANA_MBHC_ELECT:
	case WCD934X_ANA_MBHC_ZDET:
	case WCD934X_ANA_MICB2:
	case WCD934X_ANA_RCO:
	case WCD934X_ANA_BIAS:
		return true;
	default:
		return false;
	}
};

static const struct regmap_range_cfg wcd934x_ranges[] = {
	{	.name = "WCD934X",
		.range_min =  0x0,
		.range_max =  WCD934X_MAX_REGISTER,
		.selector_reg = WCD934X_SEL_REGISTER,
		.selector_mask = WCD934X_SEL_MASK,
		.selector_shift = WCD934X_SEL_SHIFT,
		.window_start = WCD934X_WINDOW_START,
		.window_len = WCD934X_WINDOW_LENGTH,
	},
};

static const struct regmap_config wcd934x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_MAPLE,
	.max_register = 0xffff,
	.can_multi_write = true,
	.ranges = wcd934x_ranges,
	.num_ranges = ARRAY_SIZE(wcd934x_ranges),
	.volatile_reg = wcd934x_is_volatile_register,
};

static int wcd934x_bring_up(struct wcd934x_ddata *ddata)
{
	struct regmap *regmap = ddata->regmap;
	u16 id_minor, id_major;
	int ret;

	ret = regmap_bulk_read(regmap, WCD934X_CHIP_TIER_CTRL_CHIP_ID_BYTE0,
			       (u8 *)&id_minor, sizeof(u16));
	if (ret)
		return ret;

	ret = regmap_bulk_read(regmap, WCD934X_CHIP_TIER_CTRL_CHIP_ID_BYTE2,
			       (u8 *)&id_major, sizeof(u16));
	if (ret)
		return ret;

	dev_info(ddata->dev, "WCD934x chip id major 0x%x, minor 0x%x\n",
		 id_major, id_minor);

	ret = regmap_write(regmap, WCD934X_CODEC_RPM_RST_CTL, 0x01);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_SIDO_NEW_VOUT_A_STARTUP, 0x19);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_SIDO_NEW_VOUT_D_STARTUP, 0x15);
	if (ret)
		return ret;
	/* Add 1msec delay for VOUT to settle */
	usleep_range(1000, 1100);
	ret = regmap_write(regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x5);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x7);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_CODEC_RPM_RST_CTL, 0x3);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_CODEC_RPM_RST_CTL, 0x7);
	if (ret)
		return ret;
	ret = regmap_write(regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x3);
	if (ret)
		return ret;

	return 0;
}

static int wcd934x_slim_status_up(struct slim_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct wcd934x_priv *priv = wcd934x_get_private(sdev);
	struct wcd934x_ddata *ddata = &priv->ddata;
	struct regmap *regmap;
	int ret;

	if (priv->up)
		return 0;

	regmap = regmap_init_slimbus(sdev, &wcd934x_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Error allocating slim regmap\n");
	ddata->regmap = regmap;

	ret = wcd934x_bring_up(ddata);
	if (ret) {
		dev_err(dev, "Failed to bring up WCD934X: err = %d\n", ret);
		goto err_regmap;
	}

	ret = regmap_add_irq_chip(regmap, ddata->irq, IRQF_TRIGGER_HIGH, 0,
				  &wcd934x_regmap_irq_chip, &ddata->irq_data);
	if (ret) {
		dev_err(dev, "Failed to add IRQ chip: err = %d\n", ret);
		goto err_regmap;
	}

	ret = mfd_add_devices(dev, PLATFORM_DEVID_AUTO, wcd934x_devices,
			      ARRAY_SIZE(wcd934x_devices), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to add child devices: err = %d\n", ret);
		goto err_irq;
	}
	priv->up = true;

	return 0;

err_irq:
	regmap_del_irq_chip(ddata->irq, ddata->irq_data);
	ddata->irq_data = NULL;
err_regmap:
	regmap_exit(regmap);
	ddata->regmap = NULL;

	return ret;
}

static void wcd934x_slim_status_down(struct slim_device *sdev)
{
	struct wcd934x_priv *priv = wcd934x_get_private(sdev);
	struct wcd934x_ddata *ddata = &priv->ddata;

	if (!priv->up)
		return;

	/* Stop and synchronize the parent thread before child teardown can sleep. */
	disable_irq(ddata->irq);

	/*
	 * The transport is already gone on DOWN. IRQ frees must only update
	 * software state; this map is discarded, never synced back to hardware.
	 * An ordinary driver removal while UP still performs real register I/O.
	 */
	if (sdev->status == SLIM_DEVICE_STATUS_DOWN)
		regcache_cache_only(ddata->regmap, true);

	/* Children must release their virtual IRQs before the domain is freed. */
	mfd_remove_devices(&sdev->dev);
	regmap_del_irq_chip(ddata->irq, ddata->irq_data);
	ddata->irq_data = NULL;
	regmap_exit(ddata->regmap);
	ddata->regmap = NULL;
	priv->up = false;
}

static int wcd934x_slim_status(struct slim_device *sdev,
			       enum slim_device_status status)
{
	struct wcd934x_priv *priv = wcd934x_get_private(sdev);
	int ret = 0;

	if (!priv)
		return -EPROBE_DEFER;

	mutex_lock(&priv->lock);
	if (!priv->ready) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	switch (status) {
	case SLIM_DEVICE_STATUS_UP:
		ret = wcd934x_slim_status_up(sdev);
		break;
	case SLIM_DEVICE_STATUS_DOWN:
		wcd934x_slim_status_down(sdev);
		break;
	default:
		ret = -EINVAL;
		break;
	}
unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int wcd934x_finish_probe(struct slim_device *sdev)
{
	struct wcd934x_priv *priv = wcd934x_get_private(sdev);
	int ret;

	/* Address assignment can call device_status: do not hold priv->lock. */
	ret = slim_get_logical_addr(sdev);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);
	priv->ready = true;
	ret = wcd934x_slim_status_up(sdev);
	mutex_unlock(&priv->lock);

	return ret;
}

static int wcd934x_reset_addresses(struct slim_device *sdev)
{
	struct device_node *np;
	struct slim_device *sidev;

	np = of_parse_phandle(sdev->dev.of_node, "slim-ifc-dev", 0);
	if (!np)
		return -EINVAL;
	sidev = of_slim_get_device(sdev->ctrl, np);
	of_node_put(np);
	if (!sidev)
		return -EPROBE_DEFER;

	/* Reset affects both the primary and interface logical devices. */
	slim_report_absent(sidev);
	slim_report_absent(sdev);
	put_device(&sidev->dev);

	return 0;
}

static void wcd934x_stop_action(void *data)
{
	struct slim_device *sdev = data;
	struct wcd934x_priv *priv = wcd934x_get_private(sdev);

	if (!priv)
		return;
	mutex_lock(&priv->lock);
	priv->ready = false;
	wcd934x_slim_status_down(sdev);
	mutex_unlock(&priv->lock);
}

static void wcd934x_disable_supplies(void *data)
{
	struct wcd934x_ddata *ddata = data;

	regulator_bulk_disable(WCD934X_MAX_SUPPLY, ddata->supplies);
}

static int wcd934x_slim_probe(struct slim_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct device_node *np = dev->of_node;
	struct wcd934x_ddata *ddata;
	struct wcd934x_priv *priv;
	struct gpio_desc *reset_gpio;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	ddata = &priv->ddata;
	ddata->dev = dev;
	mutex_init(&priv->lock);

	ddata->irq = of_irq_get(np, 0);
	if (ddata->irq < 0)
		return dev_err_probe(ddata->dev, ddata->irq,
				     "Failed to get IRQ\n");

	ddata->extclk = devm_clk_get(dev, "extclk");
	if (IS_ERR(ddata->extclk))
		return dev_err_probe(dev, PTR_ERR(ddata->extclk),
				     "Failed to get extclk");

	ddata->supplies[0].supply = "vdd-buck";
	ddata->supplies[1].supply = "vdd-buck-sido";
	ddata->supplies[2].supply = "vdd-tx";
	ddata->supplies[3].supply = "vdd-rx";
	ddata->supplies[4].supply = "vdd-io";

	ret = devm_regulator_bulk_get(dev, WCD934X_MAX_SUPPLY, ddata->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	ret = regulator_bulk_enable(WCD934X_MAX_SUPPLY, ddata->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable supplies\n");
	ret = devm_add_action_or_reset(dev, wcd934x_disable_supplies, ddata);
	if (ret)
		return ret;

	/*
	 * For WCD934X, it takes about 600us for the Vout_A and
	 * Vout_D to be ready after BUCK_SIDO is powered up.
	 * SYS_RST_N shouldn't be pulled high during this time
	 */
	usleep_range(600, 650);
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to get reset gpio\n");
	msleep(20);
	gpiod_set_value(reset_gpio, 1);
	msleep(20);

	ret = wcd934x_reset_addresses(sdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to invalidate SLIM addresses\n");

	dev_set_drvdata(dev, ddata);
	ret = devm_add_action_or_reset(dev, wcd934x_stop_action, sdev);
	if (ret)
		return ret;

	return wcd934x_finish_probe(sdev);
}

static void wcd934x_slim_remove(struct slim_device *sdev)
{
	wcd934x_stop_action(sdev);
}

static const struct slim_device_id wcd934x_slim_id[] = {
	{ SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9340,
	  SLIM_DEV_IDX_WCD9340, SLIM_DEV_INSTANCE_ID_WCD9340 },
	{}
};
MODULE_DEVICE_TABLE(slim, wcd934x_slim_id);

static struct slim_driver wcd934x_slim_driver = {
	.driver = {
		.name = "wcd934x-slim",
	},
	.probe = wcd934x_slim_probe,
	.remove = wcd934x_slim_remove,
	.device_status = wcd934x_slim_status,
	.id_table = wcd934x_slim_id,
};

module_slim_driver(wcd934x_slim_driver);
MODULE_DESCRIPTION("WCD934X slim driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org>");
