// SPDX-License-Identifier: GPL-2.0-only
/* Realme X2 Pro: native Q6ASM/ADM/AFE, WCD934x SLIM and MAX98937 I2S.
 * SLIM channel setup follows the upstream SDM845 machine driver. */
#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "qdsp6/q6afe.h"
#include "common.h"

struct samurai_audio_data {
	struct snd_soc_jack jack;
	struct snd_soc_component *jack_component;
	bool jack_created;
};

static struct snd_soc_jack_pin samurai_jack_pins[] = {
	{ .pin = "Headphone Jack", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Mic Jack", .mask = SND_JACK_MICROPHONE },
	{ .pin = "Handset Mic", .mask = SND_JACK_MICROPHONE, .invert = 1 },
	{ .pin = "Left Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
	{ .pin = "Right Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
};

static int samurai_setup_jack(struct snd_soc_pcm_runtime *rtd)
{
	static const unsigned int buttons[] = {
		SND_JACK_BTN_0, SND_JACK_BTN_1, SND_JACK_BTN_2, SND_JACK_BTN_3,
	};
	static const unsigned int keys[] = {
		KEY_PLAYPAUSE, KEY_VOICECOMMAND, KEY_VOLUMEUP, KEY_VOLUMEDOWN,
	};
	struct samurai_audio_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec;
	int i, ret;

	if (!data->jack_created) {
		ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
						 SND_JACK_HEADSET | SND_JACK_BTN_0 |
						 SND_JACK_BTN_1 | SND_JACK_BTN_2 | SND_JACK_BTN_3,
						 &data->jack, samurai_jack_pins,
						 ARRAY_SIZE(samurai_jack_pins));
		if (ret)
			return ret;
		for (i = 0; i < ARRAY_SIZE(buttons); i++) {
			ret = snd_jack_set_key(data->jack.jack, buttons[i], keys[i]);
			if (ret)
				return ret;
		}
		data->jack_created = true;
	}

	for_each_rtd_codec_dais(rtd, i, codec) {
		/* RX and TX share this WCD component, but keep both channel maps. */
		if (data->jack_component == codec->component)
			continue;
		if (data->jack_component)
			return -EINVAL;
		ret = snd_soc_dai_set_sysclk(codec, 0, 9600000,
					     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
		ret = snd_soc_component_set_jack(codec->component, &data->jack, NULL);
		if (ret)
			return ret;
		data->jack_component = codec->component;
		dev_info(rtd->card->dev, "WCD934x headset detection connected\n");
	}
	return 0;
}

static void samurai_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct samurai_audio_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec;
	int i, ret;

	if (rtd->dai_link->id != SLIMBUS_0_RX &&
	    rtd->dai_link->id != SLIMBUS_0_TX)
		return;

	for_each_rtd_codec_dais(rtd, i, codec) {
		if (data->jack_component != codec->component)
			continue;
		/* Link exit runs before the codec component is removed. */
		ret = snd_soc_component_set_jack(codec->component, NULL, NULL);
		if (ret)
			dev_err(rtd->card->dev, "WCD934x jack disconnect: %d\n", ret);
		data->jack_component = NULL;
	}
}

static int samurai_card_probe(struct snd_soc_card *card)
{
	struct samurai_audio_data *data = snd_soc_card_get_drvdata(card);

	/* Start the card lifetime before BE init, including failed bindings. */
	data->jack_created = false;
	data->jack_component = NULL;

	return 0;
}

static int samurai_card_remove(struct snd_soc_card *card)
{
	struct samurai_audio_data *data = snd_soc_card_get_drvdata(card);

	/* ASoC allocates a new ALSA card and jack on the next binding. */
	data->jack_created = false;
	data->jack_component = NULL;

	return 0;
}

static int samurai_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *ch = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	rate->min = rate->max = 48000;
	ch->min = ch->max = rtd->dai_link->id == SLIMBUS_0_TX ? 1 : 2;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int samurai_init(struct snd_soc_pcm_runtime *rtd)
{
	unsigned int rx[] = {144,145,146,147,148,149,150,151,152,153,154,155,156};
	unsigned int tx[] = {128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143};
	struct snd_soc_dai *codec;
	int i, ret;
	if (rtd->dai_link->id != SLIMBUS_0_RX && rtd->dai_link->id != SLIMBUS_0_TX)
		return 0;
	/* Set up every SLIM runtime: never gate capture on a card-wide flag. */
	for_each_rtd_codec_dais(rtd, i, codec) {
		ret = snd_soc_dai_set_channel_map(codec, ARRAY_SIZE(tx), tx,
						ARRAY_SIZE(rx), rx);
		if (ret)
			return ret;
	}
	return samurai_setup_jack(rtd);
}

static int samurai_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0), *codec;
	int i, ret;
	if (cpu->id != QUATERNARY_MI2S_RX)
		return 0;
	ret = snd_soc_dai_set_fmt(cpu, SND_SOC_DAIFMT_BP_FP | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
	if (ret)
		return ret;
	for_each_rtd_codec_dais(rtd, i, codec) {
		ret = snd_soc_dai_set_fmt(codec, SND_SOC_DAIFMT_BC_FC | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
		if (ret)
			return ret;
	}
	return snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				     48000 * 2 * 16, SNDRV_PCM_STREAM_PLAYBACK);
}

static void samurai_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;
	if (cpu->id != QUATERNARY_MI2S_RX)
		return;
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				    0, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		dev_err(rtd->dev, "quaternary clock shutdown: %d\n", ret);
}

static int samurai_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int rx[13], tx[16], nrx = 0, ntx = 0;
	int ret;
	if (cpu->id == QUATERNARY_MI2S_RX)
		return 0;
	ret = snd_soc_dai_get_channel_map(codec, &ntx, tx, &nrx, rx);
	if (ret)
		return ret;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!nrx || nrx > ARRAY_SIZE(rx))
			return -EINVAL;
		return snd_soc_dai_set_channel_map(cpu, 0, NULL, nrx, rx);
	}
	if (!ntx || ntx > ARRAY_SIZE(tx))
		return -EINVAL;
	return snd_soc_dai_set_channel_map(cpu, ntx, tx, 0, NULL);
}

static const struct snd_soc_ops samurai_ops = {
	.startup = samurai_startup, .shutdown = samurai_shutdown,
	.hw_params = samurai_params,
};

static int samurai_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct samurai_audio_data *data;
	struct snd_soc_dai_link *link;
	int i, ret;
	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	card->probe = samurai_card_probe;
	card->remove = samurai_card_remove;
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;
	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->init = samurai_init;
			link->exit = samurai_exit;
			link->ops = &samurai_ops;
			link->be_hw_params_fixup = samurai_fixup;
		}
	}
	return devm_snd_soc_register_card(&pdev->dev, card);
}
static const struct of_device_id samurai_audio_of[] = {
	{ .compatible = "realme,samurai-audio" }, { }
};
MODULE_DEVICE_TABLE(of, samurai_audio_of);
static struct platform_driver samurai_audio_driver = {
	.probe = samurai_probe,
	.driver = { .name = "samurai-audio", .of_match_table = samurai_audio_of },
};
module_platform_driver(samurai_audio_driver);
MODULE_DESCRIPTION("Realme X2 Pro QDSP6 audio machine");
MODULE_LICENSE("GPL");
