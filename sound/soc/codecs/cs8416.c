// SPDX-License-Identifier: GPL-2.0-only
//
// cs8416.c  --  CS8416 Digital Audio Interface Receiver ALSA Driver
//
// Copyright 2022 Yunhao Tian
//
// Authors: Yunhao Tian <t123yh.xyz@gmail.com>
//

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/soc.h>

#include "cs8416.h"

#define NUMBER_TO_ACTIVE 3

struct cs8416_priv {
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct gpio_desc *reset;
	struct gpio_desc *irq;
	struct delayed_work poll_work;
	uint32_t active_count;
	struct clk *omck;
	struct snd_kcontrol *snd_kctl_rate;
	struct snd_kcontrol *snd_kctl_chstat;
	struct snd_kcontrol *snd_kctl_chstat_mask;
};

static const unsigned int sample_rates[] = 
	{ 32000, 44100, 48000, 88200, 96000, 176400, 192000 };
static unsigned int cs8416_get_rate(struct snd_soc_component *component)
{
	u32 pulse, clk_rate, est_pulse;
	int i;
	struct cs8416_priv *priv_data = snd_soc_component_get_drvdata(component);

	pulse = snd_soc_component_read(component, CS8416_OMCK_RMCK_RATIO);
	if (pulse != 0) {
		clk_rate = clk_get_rate(priv_data->omck);
		for (i = 0; i < ARRAY_SIZE(sample_rates); i++) {
			est_pulse = DIV_ROUND_CLOSEST(clk_rate,
						      sample_rates[i] * 4);
			if (pulse >= est_pulse - 1 && pulse <= est_pulse + 1)
				return sample_rates[i];
		}
	}
	return 0;
}

static int cs8416_set_dai_fmt_master(unsigned int format)
{
	int dif;

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dif = CS8416_AUDIO_FORMAT_SODEL | CS8416_AUDIO_FORMAT_SOLRPOL;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		dif = CS8416_AUDIO_FORMAT_SOJUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dif = 0;
		break;
	default:
		return -ENOTSUPP;
	}

	dif |= CS8416_AUDIO_FORMAT_SOMS;

	return dif;
}

static int cs8416_set_dai_fmt_slave(unsigned int format)
{
	int dif;

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dif = CS8416_AUDIO_FORMAT_SODEL | CS8416_AUDIO_FORMAT_SOLRPOL;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dif = 0;
		break;
	default:
		return -ENOTSUPP;
	}

	return dif;
}

static int cs8416_set_dai_fmt(struct snd_soc_dai *dai,
			      unsigned int format)
{
	struct snd_soc_component *component = dai->component;
	int dif;
	int ret = 0;

	switch (format & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dif = cs8416_set_dai_fmt_master(format);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		dif = cs8416_set_dai_fmt_slave(format);
		break;
	default:
		ret = -ENOTSUPP;
		goto exit;
	}

	if (dif < 0) {
		ret = dif;
		goto exit;
	}

	ret = snd_soc_component_write(component, CS8416_AUDIO_FORMAT, dif);
	if (ret < 0)
		goto exit;

exit:
	return ret;
}

static int cs8416_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	unsigned int current_rate = cs8416_get_rate(dai->component);
	unsigned int param_rate = params_rate(params);
	if (current_rate != param_rate) {
		dev_err(dai->component->dev, "Sample rate mismatch: hardware currently running %uHz, but %uHz is requested\n",
			current_rate, param_rate);
		return -EINVAL;
	}
	return 0;
}


static int cs8416_iec958_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int cs8416_iec958_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	int i;

	for (i = 0; i < 10; i++) {
		ucontrol->value.iec958.status[i] =
		    snd_soc_component_read(component, CS8416_CHN_STAT + i);
	}
	
	return 0;
}

static int cs8416_iec958_mask_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	memset(ucontrol->value.iec958.status, 0x00,
	       sizeof(ucontrol->value.iec958.status));
	memset(ucontrol->value.iec958.status, 0xFF, 10);
	return 0;
}

static int cs8416_rate_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;
	return 0;
}

static int cs8416_rate_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);

	if (snd_soc_component_read(component, CS8416_RECV_ERR) != 0)
		ucontrol->value.integer.value[0] = 0;
	else
		ucontrol->value.integer.value[0] = cs8416_get_rate(component);
	return 0;
}


static const struct snd_soc_dai_ops cs8416_dai_ops = {
	.hw_params = cs8416_hw_params,
	.set_fmt   = cs8416_set_dai_fmt,
};

static struct snd_soc_dai_driver cs8416_dai = {
	.name = "cs8416-spdifrx",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE  |
			   SNDRV_PCM_FMTBIT_S24_3LE |
			   SNDRV_PCM_FMTBIT_S24_LE
	},
	.ops = &cs8416_dai_ops,
};

static int parse_gpio_sel(struct snd_soc_component *component, int idx, int reg, int off) {
	int val, sel = 0, ret;
	char name[17];
	snprintf(name, sizeof(name), "cirrus,gpio%d-sel", idx);
	if (!of_property_read_u32(component->dev->of_node, name,
				  &val)) {
		if ((val >= 0) && (val <= 0b1101))
			sel = val;
		else {
			dev_warn(component->dev, "Invalid gpio%d-sel value: %d\n", idx, val);
		}
	}
	ret = snd_soc_component_update_bits(component, reg, 0b1111 << off, sel << off);
	if (ret < 0) {
		dev_err(component->dev, "Failed to set gpio%d selection val\n", idx);
		return ret;
	}
	return 0;
}

static void cs8416_notify_rate(struct cs8416_priv* priv_data) {
	if (priv_data->component)
		snd_ctl_notify(priv_data->component->card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
						&priv_data->snd_kctl_rate->id);
}

static void cs8416_notify_chstat(struct cs8416_priv* priv_data) {
	if (priv_data->component)
		snd_ctl_notify(priv_data->component->card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
						&priv_data->snd_kctl_chstat->id);
}

static int cs8416_probe(struct snd_soc_component *component)
{
	struct cs8416_priv* priv_data = snd_soc_component_get_drvdata(component);
	struct snd_card *card = component->card->snd_card;
	int version, ret;
	struct snd_kcontrol_new control;

	priv_data->component = component;

	msleep(1);
	gpiod_set_value(priv_data->reset, 1);
	msleep(10);

	snd_soc_component_init_regmap(component, priv_data->regmap);

	version = snd_soc_component_read(component, CS8416_ID_VER);
	if ((version & 0xF0) != 0x20) {
		dev_err(component->dev, "Invalid chip ID: 0x%08X\n", version);
		ret = -EINVAL;
		goto err_after_rst;
	}
	dev_info(component->dev, "Chip version: 0x%X\n", version & 0xF);

	/* the access property of controls don't have to
	 * be volatile, as it will be notified by interrupt handler
	 */
	control = (struct snd_kcontrol_new){
		/* Sample Rate Control */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "Rate",
		/* access don't have to be volatile, as it will be notified by intr */
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = cs8416_rate_info,
		.get = cs8416_rate_get,
	};
	priv_data->snd_kctl_rate = snd_ctl_new1(&control, component);

	control = (struct snd_kcontrol_new){
		/* Channel Status Bits */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
		/* access don't have to be volatile, as it will be notified by intr */
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = cs8416_iec958_info,
		.get = cs8416_iec958_get,
		.private_value = 0
	};
	priv_data->snd_kctl_chstat = snd_ctl_new1(&control, component);

	control = (struct snd_kcontrol_new){
		/* Channel Status Mask */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, MASK),
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = cs8416_iec958_info,
		.get = cs8416_iec958_mask_get,
	};
	priv_data->snd_kctl_chstat_mask = snd_ctl_new1(&control, component);

	ret = snd_ctl_add(card, priv_data->snd_kctl_rate);
	if (ret < 0) {
		dev_err(component->dev, "Failed to add rate control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, priv_data->snd_kctl_chstat);
	if (ret < 0) {
		dev_err(component->dev,
			"Failed to add channel status control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, priv_data->snd_kctl_chstat_mask);
	if (ret < 0) {
		dev_err(component->dev,
			"Failed to add channel status mask control: %d\n", ret);
		return ret;
	}

	ret = parse_gpio_sel(component, 0, CS8416_CONTROL2, 0);
	if (ret < 0)
		goto err_after_rst;

	ret = parse_gpio_sel(component, 1, CS8416_CONTROL3, 4);
	if (ret < 0)
		goto err_after_rst;

	ret = parse_gpio_sel(component, 2, CS8416_CONTROL3, 0);
	if (ret < 0)
		goto err_after_rst;
	
	snd_soc_component_write(component, CS8416_AUDIO_FORMAT, CS8416_AUDIO_FORMAT_SOMS | CS8416_AUDIO_FORMAT_SOSF);

	// read to reset error status
	snd_soc_component_read(component, CS8416_INT);
	snd_soc_component_read(component, CS8416_RECV_ERR);

	snd_soc_component_write(component, CS8416_RECV_ERR_MSK, CS8416_RECV_ERR_CONF | CS8416_RECV_ERR_V);
	snd_soc_component_write(component, CS8416_INT_MSK, CS8416_INT_C_CHANGE | CS8416_INT_RERR);

	ret = clk_prepare_enable(priv_data->omck);
	if (ret) {
		dev_err(component->dev, "omck enable failed %d\n", ret);
		return ret;
	}

	// wake it up
	snd_soc_component_update_bits(component, CS8416_CONTROL4, CS8416_CONTROL4_RUN, CS8416_CONTROL4_RUN);

	// start polling
	schedule_delayed_work(&priv_data->poll_work, msecs_to_jiffies(5));

	return 0;
err_after_rst:
	gpiod_set_value(priv_data->reset, 0);
	return ret;
}

static void cs8416_remove(struct snd_soc_component *component)
{
	struct cs8416_priv *priv_data = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&priv_data->poll_work);
	gpiod_set_value(priv_data->reset, 0);
	clk_disable_unprepare(priv_data->omck);
}

static const struct snd_soc_component_driver soc_component_drv_cs8416 = {
	.probe			= cs8416_probe,
	.remove			= cs8416_remove,
	.non_legacy_dai_naming	= 1,
};

static void cs8416_poll_work(struct work_struct *t) {
	u32 err;
	struct cs8416_priv* priv_data = container_of(t, struct cs8416_priv, poll_work.work);
	regmap_read(priv_data->regmap, CS8416_RECV_ERR, &err);

	if (err == 0) {
		if (priv_data->active_count <= NUMBER_TO_ACTIVE) {
			priv_data->active_count++;
		}

		if (priv_data->active_count == NUMBER_TO_ACTIVE) {
			cs8416_notify_rate(priv_data);
		}
	} else {
		if (priv_data->active_count >= NUMBER_TO_ACTIVE) {
			// This shouldn't happen; errors should be detected in interrupt
			dev_warn(priv_data->component->dev, "Receive error detected within poll task, RERR = %02X\n", err);
			cs8416_notify_rate(priv_data);
		}
		priv_data->active_count = 0;
	}

	schedule_delayed_work(&priv_data->poll_work, msecs_to_jiffies(5));
}

static irqreturn_t cs8416_irq(int irq_no, void *handle) {
	u32 irq, err;
	struct cs8416_priv* priv_data = (struct cs8416_priv*)handle;

	regmap_read(priv_data->regmap, CS8416_INT, &irq);
	if (irq & CS8416_INT_RERR) {
		regmap_read(priv_data->regmap, CS8416_RECV_ERR, &err);
		if (priv_data->active_count >= NUMBER_TO_ACTIVE) {
			dev_dbg(priv_data->component->dev, "Receive error detected within interrupt, RERR = %02X\n", err);
			cs8416_notify_rate(priv_data);
		}
		priv_data->active_count = 0;
	}
	if (irq & CS8416_INT_C_CHANGE) {
		dev_dbg(priv_data->component->dev, "Channel status changed\n");
		cs8416_notify_chstat(priv_data);
	}

	return IRQ_HANDLED;
}

static bool cs8416_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x0 ... 0x26:
		return true;
	case 0x7F:
		return true;
	default:
		return false;
	}
}

static bool cs8416_writeable_register(struct device *dev, unsigned int reg)
{
	return reg <= CS8416_INT_MSK;
}

static bool cs8416_volatile_register(struct device* dev, unsigned int reg)
{
	switch (reg) {
	case 0xA ... 0x26:
		return true;
	default:
		return false;
	}
}

static bool cs8416_precious_register(struct device* dev, unsigned int reg)
{
	switch (reg) {
	case 0xC ... 0xD:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config cs8416_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,

	.max_register		= 0x7F,
	.readable_reg		= cs8416_readable_register,
	.writeable_reg		= cs8416_writeable_register,
	.volatile_reg	   = cs8416_volatile_register,
	.precious_reg	   = cs8416_precious_register,
	.cache_type			=  REGCACHE_RBTREE,
};


static int cs8416_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct cs8416_priv *priv_data;
	int ret = 0;

	priv_data = devm_kzalloc(&client->dev, sizeof(struct cs8416_priv), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;
	i2c_set_clientdata(client, priv_data);

	priv_data->reset = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv_data->reset)) {
		ret = PTR_ERR(priv_data->reset);
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get reset: %d\n", ret);
		return ret;
	}

	priv_data->irq = devm_gpiod_get(&client->dev, "irq", GPIOD_IN);
	if (IS_ERR(priv_data->irq)) {
		ret = PTR_ERR(priv_data->irq);
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get IRQ: %d\n", ret);
		return ret;
	}

	priv_data->regmap = devm_regmap_init_i2c(client, &cs8416_regmap);
	if (IS_ERR(priv_data->regmap)) {
		ret = PTR_ERR(priv_data->regmap);
		dev_err(&client->dev, "regmap_init() failed: %d\n", ret);
		return ret;
	}

	priv_data->omck = devm_clk_get(&client->dev, "omck");
	if (IS_ERR(priv_data->omck))
		return PTR_ERR(priv_data->omck);

	INIT_DELAYED_WORK(&priv_data->poll_work, cs8416_poll_work);

	ret = devm_snd_soc_register_component(&client->dev,
				&soc_component_drv_cs8416, &cs8416_dai, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register sound component: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(&client->dev, gpiod_to_irq(priv_data->irq),
					NULL, cs8416_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"cs8416-irq", priv_data);				
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request irq: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id cs8416_of_match[] = {
	{ .compatible = "cirrus,cs8416", },
	{},
};

MODULE_DEVICE_TABLE(of, cs8416_of_match);

static const struct i2c_device_id cs8416_i2c_id[] = {
	{"cs8416", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs8416_i2c_id);

static struct i2c_driver cs8416_i2c_driver = {
	.driver = {
		.name		= "cs8416",
		.of_match_table	= cs8416_of_match,
	},
	.id_table	= cs8416_i2c_id,
	.probe		= cs8416_i2c_probe,
};

module_i2c_driver(cs8416_i2c_driver);

MODULE_AUTHOR("Yunhao Tian <t123yh.xyz@gmail.com>");
MODULE_DESCRIPTION("CS8416 Digital Audio Interface Receiver ALSA Driver");
MODULE_LICENSE("GPL");