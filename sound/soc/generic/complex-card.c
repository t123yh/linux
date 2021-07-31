/*
 * ASoC complex sound card support
 *
 * Copyright (C) 2012 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>

#undef dev_dbg
#undef dev_info
#undef dev_warn
#undef dev_notice

#define dev_dbg dev_err
#define dev_info dev_err
#define dev_warn dev_err
#define dev_notice dev_err

struct asoc_complex_dai {
	const char *name;
	unsigned int sysclk;
	int slots;
	int slot_width;
	unsigned int tx_slot_mask;
	unsigned int rx_slot_mask;
};

struct asoc_complex_card_info {
	const char *name;
	const char *card;
	const char *codec;
	const char *platform;

	unsigned int daifmt;
	struct asoc_complex_dai cpu_dai;
	struct asoc_complex_dai codec_dai;
};

struct complex_card_data {
	struct snd_soc_card snd_card;
	struct complex_dai_props {
		struct asoc_complex_dai cpu_dai;
		struct asoc_complex_dai codec_dai;
	} *dai_props;
	// struct gpio_desc *mute_gpio;
	struct snd_soc_dai_link dai_link[];	/* dynamically allocated */
};

#define complex_priv_to_dev(priv) ((priv)->snd_card.dev)
#define complex_priv_to_link(priv, i) ((priv)->snd_card.dai_link + i)
#define complex_priv_to_props(priv, i) ((priv)->dai_props + i)

static int asoc_complex_card_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int asoc_complex_card_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void asoc_complex_card_shutdown(struct snd_pcm_substream *substream)
{
}

static int asoc_complex_card_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
         unsigned int mclk, lrck_freq;
        int ret = 0;

        lrck_freq = params_rate(params);
	switch (lrck_freq) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		mclk = 11289600;
		break;
	default:
		dev_err(rtd->dev, "Invalid LRCK freq: %u Hz\n",
			lrck_freq);
		return -EINVAL;
	}
	printk("Setting MCLK to %d\n", mclk);
       // mclk = lrck_freq * 64 * 2;
        ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
                                                SND_SOC_CLOCK_IN);
        if (ret && ret != -ENOTSUPP)
                return ret;

        ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
                                                SND_SOC_CLOCK_OUT);
        if (ret && ret != -ENOTSUPP)
                return ret;

        return 0;
}

static int asoc_complex_card_trigger(struct snd_pcm_substream *substream, int event)
{
/*
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct complex_card_data *priv = snd_soc_card_get_drvdata(rtd->card);

	switch (event) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(rtd->dev, "Card unmuting\n");
		gpiod_set_value_cansleep(priv->mute_gpio, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_dbg(rtd->dev, "Card muting\n");
		gpiod_set_value_cansleep(priv->mute_gpio, 0);
		break;
	default:
		return -EINVAL;
	}
	*/

	return 0;
}

static struct snd_soc_ops asoc_complex_card_ops = {
	.startup = asoc_complex_card_startup,
	.shutdown = asoc_complex_card_shutdown,
	.hw_params = asoc_complex_card_hw_params,
	.prepare = asoc_complex_card_prepare,
	.trigger = asoc_complex_card_trigger,
};

static int __asoc_complex_card_dai_init(struct snd_soc_dai *dai,
				       struct asoc_complex_dai *set)
{
	int ret;

	if (set->sysclk) {
		ret = snd_soc_dai_set_sysclk(dai, 0, set->sysclk, 0);
		if (ret && ret != -ENOTSUPP) {
			dev_err(dai->dev, "complex-card: set_sysclk error\n");
			goto err;
		}
	}

	if (set->slots) {
		ret = snd_soc_dai_set_tdm_slot(dai,
					       set->tx_slot_mask,
					       set->rx_slot_mask,
						set->slots,
						set->slot_width);
		if (ret && ret != -ENOTSUPP) {
			dev_err(dai->dev, "complex-card: set_tdm_slot error\n");
			goto err;
		}
	}

	ret = 0;

err:
	return ret;
}

static int asoc_complex_card_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct complex_card_data *priv =	snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec = rtd->codec_dai;
	struct snd_soc_dai *cpu = rtd->cpu_dai;
	struct complex_dai_props *dai_props;
	int num, ret;

	num = rtd - rtd->card->rtd;
	dai_props = &priv->dai_props[num];
	ret = __asoc_complex_card_dai_init(codec, &dai_props->codec_dai);
	if (ret < 0)
		return ret;

	ret = __asoc_complex_card_dai_init(cpu, &dai_props->cpu_dai);
	if (ret < 0)
		return ret;

	return 0;
}

static int
asoc_complex_card_sub_parse_of(struct device_node *np,
			      struct asoc_complex_dai *dai,
			      struct device_node **p_node,
			      const char **name,
			      int *args_count)
{
	struct of_phandle_args args;
	struct clk *clk;
	u32 val;
	int ret;

	/*
	 * Get node via "sound-dai = <&phandle port>"
	 * it will be used as xxx_of_node on soc_bind_dai_link()
	 */
	ret = of_parse_phandle_with_args(np, "sound-dai",
					 "#sound-dai-cells", 0, &args);
	if (ret)
		return ret;

	*p_node = args.np;

	if (args_count)
		*args_count = args.args_count;

	/* Get dai->name */
	ret = snd_soc_of_get_dai_name(np, name);
	if (ret < 0)
		return ret;

	/* Parse TDM slot */
	ret = snd_soc_of_parse_tdm_slot(np, &dai->tx_slot_mask,
					&dai->rx_slot_mask,
					&dai->slots, &dai->slot_width);
	if (ret)
		return ret;

	/*
	 * Parse dai->sysclk come from "clocks = <&xxx>"
	 * (if system has common clock)
	 *  or "system-clock-frequency = <xxx>"
	 *  or device's module clock.
	 */
	if (of_property_read_bool(np, "clocks")) {
		clk = of_clk_get(np, 0);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			return ret;
		}

		dai->sysclk = clk_get_rate(clk);
	} else if (!of_property_read_u32(np, "system-clock-frequency", &val)) {
		dai->sysclk = val;
	} else {
		clk = of_clk_get(args.np, 0);
		if (!IS_ERR(clk))
			dai->sysclk = clk_get_rate(clk);
	}

	return 0;
}

static int asoc_complex_card_dai_link_of(struct device_node *node,
					struct complex_card_data *priv,
					int idx,
					bool is_top_level_node)
{
	struct device *dev = complex_priv_to_dev(priv);
	struct snd_soc_dai_link *dai_link = complex_priv_to_link(priv, idx);
	struct complex_dai_props *dai_props = complex_priv_to_props(priv, idx);
	struct device_node *cpu = NULL;
	struct device_node *plat = NULL;
	struct device_node *codec = NULL;
	char *name;
	char prop[128];
	char *prefix = "";
	int ret, cpu_args;

	/* For single DAI link & old style of DT node */
	if (is_top_level_node)
		prefix = "complex-audio-card,";

	snprintf(prop, sizeof(prop), "%scpu", prefix);
	cpu = of_get_child_by_name(node, prop);

	if (!cpu) {
		ret = -EINVAL;
		dev_err(dev, "%s: Can't find %s DT node\n", __func__, prop);
		goto dai_link_of_err;
	}

	snprintf(prop, sizeof(prop), "%splat", prefix);
	plat = of_get_child_by_name(node, prop);

	snprintf(prop, sizeof(prop), "%scodec", prefix);
	codec = of_get_child_by_name(node, prop);

	if (!codec) {
		ret = -EINVAL;
		dev_err(dev, "%s: Can't find %s DT node\n", __func__, prop);
		goto dai_link_of_err;
	}

	dai_link->dai_fmt = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	ret = asoc_complex_card_sub_parse_of(cpu, &dai_props->cpu_dai,
					    &dai_link->cpu_of_node,
					    &dai_link->cpu_dai_name,
					    &cpu_args);
	if (ret < 0)
		goto dai_link_of_err;

	ret = asoc_complex_card_sub_parse_of(codec, &dai_props->codec_dai,
					    &dai_link->codec_of_node,
					    &dai_link->codec_dai_name, NULL);
	if (ret < 0)
		goto dai_link_of_err;

	if (!dai_link->cpu_dai_name || !dai_link->codec_dai_name) {
		ret = -EINVAL;
		goto dai_link_of_err;
	}

	if (plat) {
		struct of_phandle_args args;

		ret = of_parse_phandle_with_args(plat, "sound-dai",
						 "#sound-dai-cells", 0, &args);
		dai_link->platform_of_node = args.np;
	} else {
		/* Assumes platform == cpu */
		dai_link->platform_of_node = dai_link->cpu_of_node;
	}

	/* DAI link name is created from CPU/CODEC dai name */
	name = devm_kzalloc(dev,
			    strlen(dai_link->cpu_dai_name)   +
			    strlen(dai_link->codec_dai_name) + 2,
			    GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto dai_link_of_err;
	}

	sprintf(name, "%s-%s", dai_link->cpu_dai_name,
				dai_link->codec_dai_name);
	dai_link->name = dai_link->stream_name = name;
	dai_link->ops = &asoc_complex_card_ops;
	dai_link->init = asoc_complex_card_dai_init;

	dev_dbg(dev, "\tname : %s\n", dai_link->stream_name);
	dev_dbg(dev, "\tformat : %04x\n", dai_link->dai_fmt);
	dev_dbg(dev, "\tcpu : %s / %d\n",
		dai_link->cpu_dai_name,
		dai_props->cpu_dai.sysclk);
	dev_dbg(dev, "\tcodec : %s / %d\n",
		dai_link->codec_dai_name,
		dai_props->codec_dai.sysclk);

	/*
	 * In soc_bind_dai_link() will check cpu name after
	 * of_node matching if dai_link has cpu_dai_name.
	 * but, it will never match if name was created by
	 * fmt_single_name() remove cpu_dai_name if cpu_args
	 * was 0. See:
	 *	fmt_single_name()
	 *	fmt_multiple_name()
	 */
	if (!cpu_args)
		dai_link->cpu_dai_name = NULL;

dai_link_of_err:
	of_node_put(cpu);
	of_node_put(codec);

	return ret;
}

static int asoc_complex_card_parse_of(struct device_node *node,
				     struct complex_card_data *priv)
{
	struct device *dev = complex_priv_to_dev(priv);
	int ret;

	if (!node)
		return -EINVAL;

	/* Parse the card name from DT */
	snd_soc_of_parse_card_name(&priv->snd_card, "complex-audio-card,name");

	/* The off-codec widgets */
	if (of_property_read_bool(node, "complex-audio-card,widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(&priv->snd_card,
					"complex-audio-card,widgets");
		if (ret)
			return ret;
	}

	/* DAPM routes */
	if (of_property_read_bool(node, "complex-audio-card,routing")) {
		ret = snd_soc_of_parse_audio_routing(&priv->snd_card,
					"complex-audio-card,routing");
		if (ret)
			return ret;
	}

	dev_dbg(dev, "New complex-card: %s\n", priv->snd_card.name ?
		priv->snd_card.name : "");

/*
	printk("Get GPIO Line...\n");
	priv->mute_gpio = devm_gpiod_get(dev, "mute", GPIOD_OUT_LOW);
	if (IS_ERR(priv->mute_gpio)) {
		ret = PTR_ERR(priv->mute_gpio);
		dev_err(dev, "Failed to get mute gpio line: %d\n", ret);
		return ret;
	}
	*/

	/* Single/Muti DAI link(s) & New style of DT node */
	if (of_get_child_by_name(node, "complex-audio-card,dai-link")) {
		struct device_node *np = NULL;
		int i = 0;

		for_each_child_of_node(node, np) {
			dev_dbg(dev, "\tlink %d:\n", i);
			ret = asoc_complex_card_dai_link_of(np, priv,
							   i, false);
			if (ret < 0) {
				of_node_put(np);
				return ret;
			}
			i++;
		}
	} 

	if (!priv->snd_card.name)
		priv->snd_card.name = priv->snd_card.dai_link->name;

	return 0;
}

/* Decrease the reference count of the device nodes */
static int asoc_complex_card_unref(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *dai_link;
	int num_links;

	for (num_links = 0, dai_link = card->dai_link;
	     num_links < card->num_links;
	     num_links++, dai_link++) {
		of_node_put(dai_link->cpu_of_node);
		of_node_put(dai_link->codec_of_node);
	}
	return 0;
}

static int asoc_complex_card_probe(struct platform_device *pdev)
{
	struct complex_card_data *priv;
	struct snd_soc_dai_link *dai_link;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int num_links, ret;

	/* Get the number of DAI links */
	if (np && of_get_child_by_name(np, "complex-audio-card,dai-link"))
		num_links = of_get_child_count(np);
	else
		num_links = 1;

	/* Allocate the private data and the DAI link array */
	priv = devm_kzalloc(dev,
			sizeof(*priv) + sizeof(*dai_link) * num_links,
			GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Init snd_soc_card */
	priv->snd_card.owner = THIS_MODULE;
	priv->snd_card.dev = dev;
	dai_link = priv->dai_link;
	priv->snd_card.dai_link = dai_link;
	priv->snd_card.num_links = num_links;

	/* Get room for the other properties */
	priv->dai_props = devm_kzalloc(dev,
			sizeof(*priv->dai_props) * num_links,
			GFP_KERNEL);
	if (!priv->dai_props)
		return -ENOMEM;

	if (np && of_device_is_available(np)) {

		ret = asoc_complex_card_parse_of(np, priv);
		if (ret < 0) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "parse error %d\n", ret);
			goto err;
		}

	} else {
		struct asoc_complex_card_info *cinfo;

		cinfo = dev->platform_data;
		if (!cinfo) {
			dev_err(dev, "no info for asoc-complex-card\n");
			return -EINVAL;
		}

		if (!cinfo->name ||
		    !cinfo->codec_dai.name ||
		    !cinfo->codec ||
		    !cinfo->platform ||
		    !cinfo->cpu_dai.name) {
			dev_err(dev, "insufficient asoc_complex_card_info settings\n");
			return -EINVAL;
		}

		priv->snd_card.name	= (cinfo->card) ? cinfo->card : cinfo->name;
		dai_link->name		= cinfo->name;
		dai_link->stream_name	= cinfo->name;
		dai_link->platform_name	= cinfo->platform;
		dai_link->codec_name	= cinfo->codec;
		dai_link->cpu_dai_name	= cinfo->cpu_dai.name;
		dai_link->codec_dai_name = cinfo->codec_dai.name;
		dai_link->dai_fmt	= cinfo->daifmt;
		dai_link->init		= asoc_complex_card_dai_init;
		memcpy(&priv->dai_props->cpu_dai, &cinfo->cpu_dai,
					sizeof(priv->dai_props->cpu_dai));
		memcpy(&priv->dai_props->codec_dai, &cinfo->codec_dai,
					sizeof(priv->dai_props->codec_dai));

	}

	snd_soc_card_set_drvdata(&priv->snd_card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->snd_card);
	if (ret >= 0)
		return ret;

err:
	asoc_complex_card_unref(&priv->snd_card);
	return ret;
}

static int asoc_complex_card_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	return asoc_complex_card_unref(card);
}

static const struct of_device_id asoc_complex_of_match[] = {
	{ .compatible = "complex-audio-card", },
	{},
};
MODULE_DEVICE_TABLE(of, asoc_complex_of_match);

static struct platform_driver asoc_complex_card = {
	.driver = {
		.name = "asoc-complex-card",
		.pm = &snd_soc_pm_ops,
		.of_match_table = asoc_complex_of_match,
	},
	.probe = asoc_complex_card_probe,
	.remove = asoc_complex_card_remove,
};

module_platform_driver(asoc_complex_card);

MODULE_ALIAS("platform:asoc-complex-card");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASoC Complex Sound Card");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
