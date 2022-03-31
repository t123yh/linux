#include "smartcross.h"

#include <sound/pcm.h>
#include <linux/dev_printk.h>
#include <sound/pcm_params.h>


#define PCM179X_DAC_VOL_LEFT	0x10
#define PCM179X_DAC_VOL_RIGHT	0x11
#define PCM179X_FMT_CONTROL	0x12
#define PCM179X_MODE_CONTROL	0x13
#define PCM179X_SOFT_MUTE	PCM179X_FMT_CONTROL
#define PCM179X_IDR		0x17

#define PCM179X_FMT_MASK	0x70
#define PCM179X_FMT_SHIFT	4
#define PCM179X_MUTE_MASK	0x01
#define PCM179X_MUTE_SHIFT	0
#define PCM179X_ATLD_ENABLE	(1 << 7)

static bool pcm179x_accessible_reg(struct device *dev, unsigned int reg)
{
	return reg >= 0x10 && reg <= 0x17;
}

static bool pcm179x_writeable_reg(struct device *dev, unsigned int reg)
{
	bool accessible;

	accessible = pcm179x_accessible_reg(dev, reg);

	return accessible && reg != 0x16 && reg != 0x17;
}


static const struct reg_default pcm179x_reg_defaults[] = {
	{ 0x10, 0xff },
	{ 0x11, 0xff },
	{ 0x12, 0x50 },
	{ 0x13, 0x00 },
	{ 0x14, 0x00 },
	{ 0x15, 0x01 },
	{ 0x16, 0x00 },
	{ 0x17, 0x00 },
};

static int pcm1792a_apply_volume(struct snd_soc_dai* dai) {
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	
	uint8_t raw_vol = priv_data->volume;

	ENSURE_SUCCESS(regmap_clear_bits(priv_data->chips[dai->id], 
		PCM179X_FMT_CONTROL, PCM179X_ATLD_ENABLE));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], 
		PCM179X_DAC_VOL_LEFT, raw_vol));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], 
		PCM179X_DAC_VOL_RIGHT, raw_vol));
	ENSURE_SUCCESS(regmap_set_bits(priv_data->chips[dai->id], 
		PCM179X_FMT_CONTROL, PCM179X_ATLD_ENABLE));
	
	return 0;
}

static int pcm1792a_trigger(struct snd_pcm_substream *substream, int cmd, 
	struct snd_soc_dai * dai) {
	switch (cmd) {
	case SMARTCROSS_TRIGGER_VOLUME:
		pcm1792a_apply_volume(dai);
	}
	return 0;
}

static int pcm1792a_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);

	uint8_t flag = PCM179X_MUTE_MASK;

	ENSURE_SUCCESS(regmap_update_bits(
		priv_data->chips[dai->id], PCM179X_SOFT_MUTE, 
		flag, mute ? flag : 0));
	return 0;
}

static int pcm1792a_hw_params(struct snd_pcm_substream * stream,
	struct snd_pcm_hw_params * params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	int val = 0;

	priv_data->rate[dai->id] = params_rate(params);

	switch (priv_data->format[dai->id] & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		switch (params_width(params)) {
		case 24:
		case 32:
			val = 2;
			break;
		case 16:
			val = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_I2S:
		switch (params_width(params)) {
		case 24:
		case 32:
			val = 5;
			break;
		case 16:
			val = 4;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		dev_err(component->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	val = val << PCM179X_FMT_SHIFT | PCM179X_ATLD_ENABLE;

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM179X_FMT_CONTROL,
				 PCM179X_FMT_MASK | PCM179X_ATLD_ENABLE, val));
	return 0;
}

static int pcm1792a_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);

	priv_data->format[dai->id] = fmt;

	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(component->dev, "PCM179X needs to be a slave device\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		dev_err(component->dev, "PCM179X cannot use inverted i2s clocks\n");
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config pcm1792a_regmap_config = {
	.name = "pcm1792a",
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg		= pcm179x_writeable_reg,
	.readable_reg		= pcm179x_accessible_reg,
	.max_register = 23,
	.reg_defaults		= pcm179x_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(pcm179x_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

const int pcm1792a_addr_start = 0x4C;

static int pcm1792a_dai_probe(struct snd_soc_dai *dai) {
	struct i2c_client* client;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	dev_info(dai->component->dev, "Probing PCM1792a@%d", dai->id);

	client = devm_i2c_new_dummy_device(dai->component->dev, priv_data->adapter, 
		pcm1792a_addr_start + dai->id);
	if (IS_ERR(client)) {
		return PTR_ERR(client);
	}
	priv_data->chips[dai->id] = devm_regmap_init_i2c(
		client, &pcm1792a_regmap_config);

	ret = pcm1792a_apply_volume(dai);
	if (ret)
		return ret;

	return 0;
}

static const struct snd_soc_dai_ops smartcross_pcm1792a_dai_ops = {
	.hw_params = pcm1792a_hw_params,
	.set_fmt = pcm1792a_set_fmt,
	.mute_stream = pcm1792a_mute_stream,
	.trigger = pcm1792a_trigger,
};

int smartcross_pcm1792a_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx) {
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);
	int err;
	uint8_t addr = pcm1792a_addr_start + idx;

	/* Probe PCM1792A by checking ID register */
	union i2c_smbus_data data;
	err = i2c_smbus_xfer(priv_data->adapter, addr, 0, I2C_SMBUS_READ, PCM179X_IDR,
				I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0) {
		return -ENODEV;
	}

	if (data.byte != 0x5A) {
		dev_warn(&pdev->dev, "An unknown device (not pcm1792a) sits on i2c@%x\n", addr);
		return -ENODEV;
	}

	driver->id = idx;
	driver->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "PCM1792a@%d", idx);
	if (IS_ERR(driver->name)) {
		return PTR_ERR(driver->name);
	}
	driver->probe = pcm1792a_dai_probe;
	driver->ops = &smartcross_pcm1792a_dai_ops;
	driver->playback = (struct snd_soc_pcm_stream) {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = 
			SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32_LE |
			SNDRV_PCM_FMTBIT_S16_LE,
	};
	
	return 0;
}