// SPDX-License-Identifier: GPL-2.0-only
/* Realme X2 Pro MAX98937, revision 0x43. No DSM/calibration/NV interface.
 * Register configuration derived from the GPLv2 Realme max98937 driver:
 * realme-kernel-opensource/realmeX2Pro-kernel-source, 9668fcdc6ec1.
 * The original table credits Quinn Manley, Maxim Integrated.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define MAX37_MODE 0x0023
#define MAX37_MASTER 0x0024
#define MAX37_BCLK 0x0025
#define MAX37_RATE1 0x0026
#define MAX37_RATE2 0x0027
#define MAX37_AMP 0x0039
#define MAX37_GLOBAL 0x0500
#define MAX37_REV 0x0600

struct max37 {
	struct regmap *map;
	struct gpio_desc *reset;
	struct regulator *vdd;
	struct mutex lock;
	bool powered, muted, failed;
	u32 channel;
	unsigned int starts, stops;
};

/* The preparation script fills this table from the pinned GPL source and
 * checks every numeric address against its MAX98937 enum. */
static const struct reg_sequence max37_common[] = {
	{0x0011, 0x03}, /* Clock_monitor_enable */
	{0x0053, 0x00}, /* Brownout_level_infinite_hold */
	{0x0055, 0x00}, /* Brownout_level_hold */
	{0x005e, 0x16}, /* Brownout__level_1_current_limit */
	{0x005f, 0x00}, /* Brownout__level_1_amp_1_control_1 */
	{0x0060, 0x0c}, /* Brownout__level_1_amp_1_control_2 */
	{0x0061, 0x0c}, /* Brownout__level_1_amp_1_control_3 */
	{0x0062, 0x12}, /* Brownout__level_2_current_limit */
	{0x0063, 0x00}, /* Brownout__level_2_amp_1_control_1 */
	{0x0064, 0x0c}, /* Brownout__level_2_amp_1_control_2 */
	{0x0065, 0x0c}, /* Brownout__level_2_amp_1_control_3 */
	{0x0066, 0x0e}, /* Brownout__level_3_current_limit */
	{0x0067, 0x03}, /* Brownout__level_3_amp_1_control_1 */
	{0x0068, 0x18}, /* Brownout__level_3_amp_1_control_2 */
	{0x0069, 0x18}, /* Brownout__level_3_amp_1_control_3 */
	{0x006a, 0x0a}, /* Brownout__level_4_current_limit */
	{0x006b, 0x00}, /* Brownout__level_4_amp_1_control_1 */
	{0x006c, 0x80}, /* Brownout__level_4_amp_1_control_2 */
	{0x006d, 0x00}, /* Brownout__level_4_amp_1_control_3 */
	{0x005a, 0x00}, /* Brownout_threshold_hysterysis */
	{0x005b, 0x00}, /* Brownout_AMP_limiter_attack_release */
	{0x005c, 0x1f}, /* Brownout_AMP_gain_attack_release */
	{0x005d, 0x00}, /* Brownout_AMP1_clip_mode */
	{0x0042, 0x07}, /* Meas_ADC_Config */
	{0x0014, 0x75}, /* Meas_ADC_Thermal_Warning_Threshhold */
	{0x0015, 0x8c}, /* Meas_ADC_Thermal_Shutdown_Threshhold */
	{0x0017, 0x55}, /* Pin_Config */
	{0x003e, 0xf7}, /* Measurement_DSP_Config */
	{0x0023, 0x40}, /* PCM_Mode_Config */
	{0x003a, 0x00}, /* Speaker_source_select */
	{0x001b, 0x00}, /* PCM_Tx_Enables_B */
	{0x0019, 0x00}, /* PCM_Rx_Enables_B */
	{0x001f, 0x03}, /* PCM_Tx_DOUT_Control_2 */
	{0x001d, 0xff}, /* PCM_Tx_HiZ_Control_B */
	{0x003d, 0x00}, /* Measurement_enables */
	{0x0035, 0x00}, /* PDM_Rx_Enable */
	{0x0036, 0x2c}, /* AMP_volume_control */
	{0x0037, 0x0f}, /* AMP_DSP_Config */
	{0x003b, 0x51}, /* Speaker_Gain */
	{0x003c, 0x85}, /* SSM_Configuration */
	{0x003f, 0x1c}, /* Boost_Control_0 */
	{0x0043, 0x00}, /* Meas_ADC_Base_Divide_MSByte */
	{0x0044, 0xff}, /* Meas_ADC_Base_Divide_LSByte */
	{0x0016, 0x08}, /* Meas_ADC_Thermal_Hysteresis */
	{0x0073, 0x0f}, /* Env_Tracker_Vout_Headroom */
	{0x0077, 0x01}, /* Env_Tracker_Control */
	{0x0052, 0x02}, /* Brownout_enables */
	{0x0020, 0x00}, /* PCM_STATUS_WORD_CONFIG_0 */
	{0x0021, 0x00}, /* PCM_STATUS_WORD_CONFIG_1 */
	{0x0022, 0x00}, /* PCM_STATUS_WORD_CONFIG_2 */
	{0x004e, 0x10}, /* Squelch */
	{0x0079, 0x45}, /* Boost_Bypass_1 */
	{0x007a, 0x2b}, /* Boost_Bypass_2 */
	{0x007b, 0x04}, /* Boost_Bypass_3 */
	{0x0039, 0x80}, /* AMP_enables */
};

static bool max37_readable(struct device *dev, unsigned int reg)
{
	unsigned int i;
	if ((reg >= 1 && reg <= 9) || reg == MAX37_REV ||
	    (reg >= MAX37_MODE && reg <= MAX37_RATE2) ||
	    reg == MAX37_GLOBAL || (reg >= 0x18 && reg <= 0x1f) ||
	    reg == 0x28 || reg == 0x29 || reg == 0x40 || reg == 0x41)
		return true;
	for (i = 0; i < ARRAY_SIZE(max37_common); i++)
		if (reg == max37_common[i].reg)
			return true;
	return false;
}

static bool max37_writeable(struct device *dev, unsigned int reg)
{
	return reg != MAX37_REV && reg > 9 && max37_readable(dev, reg);
}

static const struct regmap_config max37_map_config = {
	.reg_bits = 16, .val_bits = 8, .max_register = MAX37_REV,
	.cache_type = REGCACHE_NONE,
	.readable_reg = max37_readable, .writeable_reg = max37_writeable,
};

/* An I2C failure makes this device unavailable until a fresh probe. */
static int max37_fault(struct max37 *amp, int ret)
{
	if (ret) {
		regmap_write(amp->map, MAX37_GLOBAL, 0);
		gpiod_set_value_cansleep(amp->reset, 1);
		amp->failed = true;
		amp->powered = false;
	}
	return ret;
}

static int max37_reconcile(struct max37 *amp)
{
	if (amp->failed)
		return -EIO;
	return max37_fault(amp, regmap_write(amp->map, MAX37_GLOBAL,
					   amp->powered && !amp->muted));
}

static int max37_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct max37 *amp = snd_soc_component_get_drvdata(dai->component);
	int ret;
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC ||
	    (fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	mutex_lock(&amp->lock);
	ret = amp->failed ? -EIO : regmap_update_bits(amp->map, MAX37_MASTER, 3, 0);
	if (!ret)
		ret = regmap_update_bits(amp->map, MAX37_MODE, 0x3c, 0);
	ret = max37_fault(amp, ret);
	mutex_unlock(&amp->lock);
	return ret;
}

static int max37_params(struct snd_pcm_substream *substream,
		       struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct max37 *amp = snd_soc_component_get_drvdata(dai->component);
	unsigned int width = params_width(params);
	int ret;
	if (params_rate(params) != 48000 || params_channels(params) != 2 ||
	    (width != 16 && width != 32))
		return -EINVAL;
	mutex_lock(&amp->lock);
	ret = amp->failed ? -EIO : regmap_update_bits(amp->map, MAX37_MODE,
							0xc0, width == 16 ? 0x40 : 0xc0);
	if (!ret)
		ret = regmap_update_bits(amp->map, MAX37_BCLK, 0x0f, width == 16 ? 2 : 4);
	if (!ret)
		ret = regmap_update_bits(amp->map, MAX37_RATE1, 0x0f, 8);
	/* 48k speaker, 24k interleaved IV ADC, matching the OEM clock contract.
 * IV transmit/measurement stay disabled; no DSP speaker model is assumed. */
	if (!ret)
		ret = regmap_write(amp->map, MAX37_RATE2, 0x85);
	ret = max37_fault(amp, ret);
	mutex_unlock(&amp->lock);
	return ret;
}

static int max37_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct max37 *amp = snd_soc_component_get_drvdata(dai->component);
	int ret;
	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;
	mutex_lock(&amp->lock);
	amp->muted = mute;
	ret = max37_reconcile(amp);
	mutex_unlock(&amp->lock);
	return ret;
}

static int max37_event(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	struct max37 *amp = snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	int ret;
	mutex_lock(&amp->lock);
	if (event == SND_SOC_DAPM_POST_PMU) {
		ret = amp->failed ? -EIO : regmap_update_bits(amp->map, MAX37_AMP, 1, 1);
		if (!ret) {
			amp->powered = true;
			ret = max37_reconcile(amp);
		}
		if (!ret)
			amp->starts++;
	} else {
		amp->powered = false;
		ret = max37_reconcile(amp);
		if (!ret)
			ret = regmap_update_bits(amp->map, MAX37_AMP, 1, 0);
		amp->stops++;
	}
	ret = max37_fault(amp, ret);
	mutex_unlock(&amp->lock);
	return ret;
}

static const struct snd_soc_dapm_widget max37_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIFIN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA_E("Amp", SND_SOC_NOPM, 0, 0, NULL, 0, max37_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};
static const struct snd_soc_dapm_route max37_routes[] = {
	{ "Amp", NULL, "AIFIN" }, { "OUT", NULL, "Amp" },
};
static const struct snd_soc_component_driver max37_component = {
	.dapm_widgets = max37_widgets, .num_dapm_widgets = ARRAY_SIZE(max37_widgets),
	.dapm_routes = max37_routes, .num_dapm_routes = ARRAY_SIZE(max37_routes),
	.use_pmdown_time = 1, .endianness = 1,
};
static const struct snd_soc_dai_ops max37_ops = {
	.set_fmt = max37_fmt, .hw_params = max37_params, .mute_stream = max37_mute,
};
static struct snd_soc_dai_driver max37_dai = {
	.name = "max98937-aif1",
	.playback = { .stream_name = "Playback", .channels_min = 2, .channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE },
	.ops = &max37_ops,
};

static void max37_power_off(void *data)
{
	struct max37 *amp = data;
	regulator_disable(amp->vdd);
}
static void max37_reset(void *data)
{
	struct max37 *amp = data;
	mutex_lock(&amp->lock);
	regmap_write(amp->map, MAX37_GLOBAL, 0);
	gpiod_set_value_cansleep(amp->reset, 1);
	amp->powered = false;
	amp->muted = true;
	mutex_unlock(&amp->lock);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max37 *amp = dev_get_drvdata(dev);
	unsigned int id, global, enable, raw1, raw2, raw3;
	int ret;
	mutex_lock(&amp->lock);
	if (amp->failed) {
		ret = -EIO;
		goto out;
	}
	ret = regmap_read(amp->map, MAX37_REV, &id);
	if (!ret) ret = regmap_read(amp->map, MAX37_GLOBAL, &global);
	if (!ret) ret = regmap_read(amp->map, MAX37_AMP, &enable);
	if (!ret) ret = regmap_read(amp->map, 1, &raw1);
	if (!ret) ret = regmap_read(amp->map, 2, &raw2);
	if (!ret) ret = regmap_read(amp->map, 3, &raw3);
	if (!ret)
		ret = sysfs_emit(buf, "id=%02x channel=%u global=%02x amp=%02x raw=%02x,%02x,%02x powered=%u muted=%u starts=%u stops=%u failed=%u\n",
			id, amp->channel, global, enable, raw1, raw2, raw3,
			amp->powered, amp->muted, amp->starts, amp->stops, amp->failed);
out:
	mutex_unlock(&amp->lock);
	return ret;
}
static DEVICE_ATTR_RO(status);
static struct attribute *max37_attrs[] = { &dev_attr_status.attr, NULL };
ATTRIBUTE_GROUPS(max37);

static int max37_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max37 *amp;
	unsigned int id;
	int ret;
	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	amp = devm_kzalloc(dev, sizeof(*amp), GFP_KERNEL);
	if (!amp)
		return -ENOMEM;
	mutex_init(&amp->lock);
	amp->muted = true;
	i2c_set_clientdata(client, amp);
	ret = device_property_read_u32(dev, "realme,pcm-channel", &amp->channel);
	if (ret || amp->channel > 1)
		return -EINVAL;
	amp->map = devm_regmap_init_i2c(client, &max37_map_config);
	if (IS_ERR(amp->map))
		return PTR_ERR(amp->map);
	amp->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(amp->vdd))
		return dev_err_probe(dev, PTR_ERR(amp->vdd), "vdd supply\n");
	ret = regulator_enable(amp->vdd);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, max37_power_off, amp);
	if (ret)
		return ret;
	amp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(amp->reset))
		return dev_err_probe(dev, PTR_ERR(amp->reset), "reset line\n");
	ret = devm_add_action_or_reset(dev, max37_reset, amp);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(amp->reset, 0);
	msleep(20);
	ret = regmap_read(amp->map, MAX37_REV, &id);
	if (ret)
		return dev_err_probe(dev, ret, "revision read\n");
	if (id != 0x43)
		return dev_err_probe(dev, -ENODEV, "unexpected revision %02x\n", id);
	ret = regmap_write(amp->map, MAX37_GLOBAL, 0);
	if (!ret)
		ret = regmap_multi_reg_write(amp->map, max37_common, ARRAY_SIZE(max37_common));
	/* Match the board's left/right boost and monomix; keep DOUT high impedance. */
	if (!ret) ret = regmap_write(amp->map, 0x41, amp->channel ? 0x3a : 0x34);
	if (!ret) ret = regmap_write(amp->map, 0x40, amp->channel ? 9 : 1);
	if (!ret) ret = regmap_write(amp->map, 0x18, BIT(amp->channel));
	if (!ret) ret = regmap_write(amp->map, 0x1a, 0);
	if (!ret) ret = regmap_write(amp->map, 0x1c, 0xff);
	if (!ret) ret = regmap_write(amp->map, 0x1e, 0x40 | amp->channel);
	if (!ret) ret = regmap_write(amp->map, 0x28, 0x80 | amp->channel);
	if (!ret) ret = regmap_write(amp->map, 0x29, amp->channel);
	if (ret)
		return dev_err_probe(dev, ret, "configuration\n");
	ret = devm_snd_soc_register_component(dev, &max37_component, &max37_dai, 1);
	if (ret)
		return ret;
	dev_info(dev, "MAX98937 revision=%02x channel=%u; global disabled, fixed conservative gain\n", id, amp->channel);
	return 0;
}
static void max37_shutdown(struct i2c_client *client)
{
	max37_reset(i2c_get_clientdata(client));
}
static const struct of_device_id max37_of[] = {
	{ .compatible = "realme,samurai-max98937" }, { }
};
MODULE_DEVICE_TABLE(of, max37_of);
static struct i2c_driver max37_driver = {
	.driver = { .name = "max98937-samurai", .of_match_table = max37_of,
		    .dev_groups = max37_groups },
	.probe = max37_probe, .shutdown = max37_shutdown,
};
module_i2c_driver(max37_driver);
MODULE_DESCRIPTION("Realme X2 Pro MAX98937 revision 0x43");
MODULE_LICENSE("GPL");
