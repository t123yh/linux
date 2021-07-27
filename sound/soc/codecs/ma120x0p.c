// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ASoC Driver for Infineon Merus(TM) ma120x0p multi-level class-D amplifier
 *
 * Authors:	Ariel Muszkat <ariel.muszkat@gmail.com>
 * Jorgen Kragh Jakobsen <jorgen.kraghjakobsen@infineon.com>
 * Yunhao Tian <t123yh@outlook.com>
 *
 * Copyright (C) 2019 Infineon Technologies AG
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "ma120x0p.h"

#undef dev_dbg
#undef dev_info
#undef dev_warn
#undef dev_notice

#define dev_dbg dev_err
#define dev_info dev_err
#define dev_warn dev_err
#define dev_notice dev_err

#define SOC_ENUM_ERR(xname, xenum)\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_READ,\
	.info = snd_soc_info_enum_double,\
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double,\
	.private_value = (unsigned long)&(xenum) }


struct ma120x0p_priv {
	struct regmap *regmap;
	int default_pmp;
	unsigned int irq_number;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *error_gpio;
};

// Function prototype for the custom IRQ handler function
static irqreturn_t ma120x0p_irq_handler(int irq, void *data);

//Alsa Controls
static const char * const limenable_text[] = {"Bypassed", "Enabled"};
static const char * const limatack_text[] = {"Slow", "Normal", "Fast"};
static const char * const limrelease_text[] = {"Slow", "Normal", "Fast"};

static const char * const err_flycap_text[] = {"Ok", "Error"};
static const char * const err_overcurr_text[] = {"Ok", "Error"};
static const char * const err_pllerr_text[] = {"Ok", "Error"};
static const char * const err_pvddunder_text[] = {"Ok", "Error"};
static const char * const err_overtempw_text[] = {"Ok", "Error"};
static const char * const err_overtempe_text[] = {"Ok", "Error"};
static const char * const err_pinlowimp_text[] = {"Ok", "Error"};
static const char * const err_dcprot_text[] = {"Ok", "Error"};

static const char * const pwr_mode_prof_text[] = {"PMF0", "PMF1", "PMF2",
"PMF3", "PMF4"};

static const struct soc_enum lim_enable_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_limiterenable__a,
		ma_audio_proc_limiterenable__shift,
		ma_audio_proc_limiterenable__len + 1,
		limenable_text);
static const struct soc_enum limatack_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_attack__a,
		ma_audio_proc_attack__shift,
		ma_audio_proc_attack__len + 1,
		limatack_text);
static const struct soc_enum limrelease_ctrl =
	SOC_ENUM_SINGLE(ma_audio_proc_release__a,
		ma_audio_proc_release__shift,
		ma_audio_proc_release__len + 1,
		limrelease_text);
static const struct soc_enum err_flycap_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 0, 3, err_flycap_text);
static const struct soc_enum err_overcurr_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 1, 3, err_overcurr_text);
static const struct soc_enum err_pllerr_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 2, 3, err_pllerr_text);
static const struct soc_enum err_pvddunder_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 3, 3, err_pvddunder_text);
static const struct soc_enum err_overtempw_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 4, 3, err_overtempw_text);
static const struct soc_enum err_overtempe_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 5, 3, err_overtempe_text);
static const struct soc_enum err_pinlowimp_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 6, 3, err_pinlowimp_text);
static const struct soc_enum err_dcprot_ctrl =
	SOC_ENUM_SINGLE(ma_error__a, 7, 3, err_dcprot_text);
static const struct soc_enum pwr_mode_prof_ctrl =
	SOC_ENUM_SINGLE(ma_pmprofile__a, ma_pmprofile__shift, 5,
		pwr_mode_prof_text);

static const char * const pwr_mode_texts[] = {
		"Dynamic power mode",
		"Power mode 1",
		"Power mode 2",
		"Power mode 3",
	};

static const int pwr_mode_values[] = {
		0x10,
		0x50,
		0x60,
		0x70,
	};

static const SOC_VALUE_ENUM_SINGLE_DECL(pwr_mode_ctrl,
	ma_pm_man__a, 0, 0x70,
	pwr_mode_texts,
	pwr_mode_values);

static const DECLARE_TLV_DB_SCALE(ma120x0p_vol_tlv, -5000, 100,  0);
static const DECLARE_TLV_DB_SCALE(ma120x0p_lim_tlv, -5000, 100,  0);
static const DECLARE_TLV_DB_SCALE(ma120x0p_lr_tlv, -5000, 100,  0);

static const struct snd_kcontrol_new ma120x0p_snd_controls[] = {
	//Master Volume
	SOC_SINGLE_RANGE_TLV("A.Mstr Vol Volume",
		ma_vol_db_master__a, 0, 0x18, 0x4a, 1, ma120x0p_vol_tlv),

	//L-R Volume ch0
	SOC_SINGLE_RANGE_TLV("B.L Vol Volume",
		ma_vol_db_ch0__a, 0, 0x18, 0x4a, 1, ma120x0p_lr_tlv),
	SOC_SINGLE_RANGE_TLV("C.R Vol Volume",
		ma_vol_db_ch1__a, 0, 0x18, 0x4a, 1, ma120x0p_lr_tlv),

	//L-R Limiter Threshold ch0-ch1
	SOC_DOUBLE_R_RANGE_TLV("D.Lim thresh Volume",
		ma_thr_db_ch0__a, ma_thr_db_ch1__a, 0, 0x0e, 0x4a, 1,
		ma120x0p_lim_tlv),

	//Enum Switches/Selectors
	//SOC_ENUM("E.AudioProc Mute", audioproc_mute_ctrl),
	SOC_ENUM("F.Limiter Enable", lim_enable_ctrl),
	SOC_ENUM("G.Limiter Attck", limatack_ctrl),
	SOC_ENUM("H.Limiter Rls", limrelease_ctrl),

	//Enum Error Monitor (read-only)
	SOC_ENUM_ERR("I.Err flycap", err_flycap_ctrl),
	SOC_ENUM_ERR("J.Err overcurr", err_overcurr_ctrl),
	SOC_ENUM_ERR("K.Err pllerr", err_pllerr_ctrl),
	SOC_ENUM_ERR("L.Err pvddunder", err_pvddunder_ctrl),
	SOC_ENUM_ERR("M.Err overtempw", err_overtempw_ctrl),
	SOC_ENUM_ERR("N.Err overtempe", err_overtempe_ctrl),
	SOC_ENUM_ERR("O.Err pinlowimp", err_pinlowimp_ctrl),
	SOC_ENUM_ERR("P.Err dcprot", err_dcprot_ctrl),

	//Power modes profiles
	SOC_ENUM("Q.PM Prof", pwr_mode_prof_ctrl),

	// Power mode selection (Dynamic,1,2,3)
	SOC_ENUM("R.Power Mode", pwr_mode_ctrl),
};

//Machine Driver
static int ma120x0p_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	u16 blen;

	struct snd_soc_component *component = dai->component;

	// TODO: how to handle blen here?
	blen = 0;

	dev_dbg(dai->dev, "Setting ma12070p params\n");
	snd_soc_component_update_bits(component, ma_i2s_framesize__a,
		ma_i2s_framesize__mask, 0);

	return 0;
}

static int ma120x0p_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	u8 sck_pol, ws_pol, iface;
	int ret;

	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface = 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;  // We don't know the bit width!
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		sck_pol = 1;
		ws_pol = 0;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		sck_pol = 1;
		ws_pol = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		sck_pol = 0;
		ws_pol = 0;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		sck_pol = 0;
		ws_pol = 1;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(dai->dev, 
		"Setting ma12070p dai fmt: sck_pol = %d, ws_pol = %d, iface = %d\n",
		sck_pol, ws_pol, iface);
	ret = snd_soc_component_update_bits(component, ma_i2s_sck_pol__a,
		ma_i2s_sck_pol__mask, sck_pol);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component, ma_i2s_ws_pol__a,
		ma_i2s_ws_pol__mask, ws_pol);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component, ma_i2s_format__a,
		ma_i2s_format__mask, iface);
	if (ret < 0)
		return ret;

	return 0;
}

static int ma120x0p_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct ma120x0p_priv *ma120x0p;

	struct snd_soc_component *component = dai->component;

	ma120x0p = snd_soc_component_get_drvdata(component);

	(void)ma120x0p;

	return 0;
}

static const struct snd_soc_dai_ops ma120x0p_dai_ops = {
	.hw_params		=	ma120x0p_hw_params,
	.set_fmt = ma120x0p_set_fmt,
	.mute_stream	=	ma120x0p_mute_stream,
};

static struct snd_soc_dai_driver ma120x0p_dai = {
	// .name		= "ma120x0p-amp",
	.playback	=	{
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 44100,
		.rate_max = 96000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE
	},
	.ops        = &ma120x0p_dai_ops,
};

//Codec Driver
static int ma120x0p_clear_err(struct snd_soc_component *component)
{
	int ret = 0;

	struct ma120x0p_priv *ma120x0p;

	ma120x0p = snd_soc_component_get_drvdata(component);

	ret = snd_soc_component_update_bits(component,
		ma_eh_clear__a, ma_eh_clear__mask, 0x00);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component,
		ma_eh_clear__a, ma_eh_clear__mask, 0x04);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component,
		ma_eh_clear__a, ma_eh_clear__mask, 0x00);
	if (ret < 0)
		return ret;

	return 0;
}

static void ma120x0p_remove(struct snd_soc_component *component)
{
}

static int ma120x0p_probe(struct snd_soc_component *component)
{
	struct ma120x0p_priv *priv_data = snd_soc_component_get_drvdata(component);

	int ret = 0;

	// Reset error
	ma120x0p_clear_err(component);
	if (ret < 0)
		return ret;

	// set serial audio format I2S and enable audio processor
	ret = snd_soc_component_write(component, ma_i2s_format__a, 0x08);
	if (ret < 0)
		return ret;

	// set volume to 50%
	ret = snd_soc_component_write(component, ma_vol_db_master__a, 0x27);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component, ma_pmprofile__a, 
		ma_pmprofile__mask, priv_data->default_pmp);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct snd_soc_dapm_widget ma120x0p_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("OUT_A"),
	SND_SOC_DAPM_OUTPUT("OUT_B"),
};

static const struct snd_soc_dapm_route ma120x0p_dapm_routes[] = {
	{ "OUT_B",  NULL, "Playback" },
	{ "OUT_A",  NULL, "Playback" },
};

static const struct snd_soc_component_driver ma120x0p_component_driver = {
	.probe = ma120x0p_probe,
	.remove = ma120x0p_remove,
	.dapm_widgets		= ma120x0p_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ma120x0p_dapm_widgets),
	.dapm_routes		= ma120x0p_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ma120x0p_dapm_routes),
	.controls = ma120x0p_snd_controls,
	.num_controls = ARRAY_SIZE(ma120x0p_snd_controls),
};

//I2C Driver
static const struct reg_default ma120x0p_reg_defaults[] = {
	{	0x01,	0x3c	},
};

static bool ma120x0p_reg_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ma_error__a:
			return true;
	default:
			return false;
	}
}

static const struct of_device_id ma120x0p_of_match[] = {
	{ .compatible = "ma,ma120x0p", },
	{ }
};

MODULE_DEVICE_TABLE(of, ma120x0p_of_match);

static struct regmap_config ma120x0p_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 255,
	.volatile_reg = ma120x0p_reg_volatile,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = ma120x0p_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ma120x0p_reg_defaults),
};

static int ma120x0p_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	int ret, val;

	struct ma120x0p_priv *priv_data = devm_kzalloc(&i2c->dev, sizeof(*priv_data), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;
	i2c_set_clientdata(i2c, priv_data);

	priv_data->regmap = devm_regmap_init_i2c(i2c, &ma120x0p_regmap_config);
	if (IS_ERR(priv_data->regmap)) {
		ret = PTR_ERR(priv_data->regmap);
		return ret;
	}
	
	priv_data->default_pmp = 2;
	if (!of_property_read_u32(i2c->dev.of_node, "ma120x0p,default-pmp", &val)) {
		if ((val >= 0) && (val <= 4))
			priv_data->default_pmp = val;
	}

	//Enable ma120x0pp
	priv_data->enable_gpio = devm_gpiod_get_optional(&i2c->dev,
		"enable_gp", GPIOD_OUT_HIGH);
	if (IS_ERR(priv_data->enable_gpio)) {
		ret = PTR_ERR(priv_data->enable_gpio);
		dev_err(&i2c->dev,
		"Failed to get ma120x0p enable gpio line: %d\n", ret);
		return ret;
	}
	msleep(500);

	//Optional use of ma120x0pp error line as an interrupt trigger to
	//platform GPIO.
	//Get error input gpio ma120x0p
	priv_data->error_gpio = devm_gpiod_get_optional(&i2c->dev,
		 "error_gp", GPIOD_IN);
	if (IS_ERR(priv_data->error_gpio)) {
		ret = PTR_ERR(priv_data->error_gpio);
		dev_err(&i2c->dev,
			"Failed to get ma120x0p error gpio line: %d\n", ret);
		return ret;
	}

	if (priv_data->error_gpio != NULL) {
		priv_data->irq_number = gpiod_to_irq(priv_data->error_gpio);

		ret = devm_request_threaded_irq(&i2c->dev,
			 priv_data->irq_number, ma120x0p_irq_handler,
			 NULL, IRQF_TRIGGER_FALLING,
			 "ma120x0p", priv_data);
		if (ret != 0)
			dev_warn(&i2c->dev, "Failed to request IRQ: %d\n",
				ret);
	}

	dev_dbg(&i2c->dev, "ma12070p probed!\n");
	ret = devm_snd_soc_register_component(&i2c->dev,
		&ma120x0p_component_driver, &ma120x0p_dai, 1);

	return ret;
}

static irqreturn_t ma120x0p_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int ma120x0p_i2c_remove(struct i2c_client *i2c)
{
	struct ma120x0p_priv *priv_data = i2c_get_clientdata(i2c);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
	msleep(200);

	return 0;
}

static void ma120x0p_i2c_shutdown(struct i2c_client *i2c)
{
	struct ma120x0p_priv *priv_data = i2c_get_clientdata(i2c);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
	msleep(200);
}

static const struct i2c_device_id ma120x0p_i2c_id[] = {
	{ "ma120x0p", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ma120x0p_i2c_id);

static struct i2c_driver ma120x0p_i2c_driver = {
	.driver = {
		.name = "ma120x0p",
		.owner = THIS_MODULE,
		.of_match_table = ma120x0p_of_match,
	},
	.probe = ma120x0p_i2c_probe,
	.remove = ma120x0p_i2c_remove,
	.shutdown = ma120x0p_i2c_shutdown,
	.id_table = ma120x0p_i2c_id
};

module_i2c_driver(ma120x0p_i2c_driver);

MODULE_AUTHOR("Ariel Muszkat ariel.muszkat@gmail.com>");
MODULE_DESCRIPTION("ASoC driver for ma120x0p");
MODULE_LICENSE("GPL v2");