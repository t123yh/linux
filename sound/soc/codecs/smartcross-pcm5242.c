#include "smartcross.h"
#include "pcm512x.h"

#include <sound/pcm.h>
#include <linux/dev_printk.h>
#include <sound/pcm_params.h>


static int pcm5242_apply_volume(struct snd_soc_dai* dai) {
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	
	int16_t raw_vol = 48 - ((int16_t)priv_data->volume - (int16_t)VOL_MAX);
	if (raw_vol > 255) {
		raw_vol = 255;
	}

	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], 
		PCM512x_DIGITAL_VOLUME_2, (uint8_t)raw_vol));

	return 0;
}

static int pcm5242_trigger(struct snd_pcm_substream *substream, int cmd, 
	struct snd_soc_dai * dai) {
	switch (cmd) {
	case SMARTCROSS_TRIGGER_VOLUME:
		pcm5242_apply_volume(dai);
	}
	return 0;
}

static int pcm5242_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);
	int ret;
	unsigned int mute_det;

	if (mute) {
		ret = regmap_update_bits(priv_data->chips[dai->id], PCM512x_MUTE,
					 PCM512x_RQML | PCM512x_RQMR,
					 PCM512x_RQML | PCM512x_RQMR);
		if (ret != 0) {
			dev_err(component->dev,
				"Failed to set digital mute: %d\n", ret);
			goto fail;
		}

		regmap_read_poll_timeout(priv_data->chips[dai->id],
					 PCM512x_ANALOG_MUTE_DET,
					 mute_det, (mute_det & 0x3) == 0,
					 200, 10000);
	} else {
		ret = regmap_update_bits(priv_data->chips[dai->id], PCM512x_MUTE,
					 PCM512x_RQML | PCM512x_RQMR, 0);
		if (ret != 0) {
			dev_err(component->dev,
				"Failed to update digital mute: %d\n", ret);
			goto fail;
		}

		regmap_read_poll_timeout(priv_data->chips[dai->id],
					 PCM512x_ANALOG_MUTE_DET,
					 mute_det,
					 (mute_det & 0x3) != 0,
					 200, 10000);
	}

	return 0;
fail:
	return ret;
}

static int pcm5242_hw_params(struct snd_pcm_substream * stream,
	struct snd_pcm_hw_params * params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	int afmt, alen;

	priv_data->rate[dai->id] = params_rate(params);

	switch (priv_data->format[dai->id] & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		afmt = PCM512x_AFMT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		afmt = PCM512x_AFMT_RTJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		afmt = PCM512x_AFMT_LTJ;
		break;
	default:
		dev_err(component->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_BCLK_LRCLK_CFG,
				 PCM512x_BCKP | PCM512x_BCKO | PCM512x_LRKO, 0));
	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_MASTER_MODE,
				 PCM512x_RLRK | PCM512x_RBCK, 0));
	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_I2S_1,
				 PCM512x_AFMT, afmt));
	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_I2S_2,
				 0xFF, 0));
				 
	switch (params_width(params)) {
	case 16:
		alen = PCM512x_ALEN_16;
		break;
	case 20:
		alen = PCM512x_ALEN_20;
		break;
	case 24:
		alen = PCM512x_ALEN_24;
		break;
	case 32:
		alen = PCM512x_ALEN_32;
		break;
	default:
		dev_err(component->dev, "Bad frame size: %d\n",
			params_width(params));
		return -EINVAL;
	}

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_I2S_1,
				 PCM512x_ALEN, alen));

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_ERROR_DETECT,
					 PCM512x_IDFS | PCM512x_IDBK
					 | PCM512x_IDSK | PCM512x_IDCH
					 | PCM512x_IDCM | PCM512x_DCAS
					 | PCM512x_IPLK,
					 PCM512x_IDFS | PCM512x_IDBK
					 | PCM512x_IDSK | PCM512x_IDCH
					 | PCM512x_IPLK));

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_PLL_EN,
					 PCM512x_PLLE, 0));

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_DAC_REF,
					 PCM512x_SDAC, PCM512x_SDAC_SCK));

	// Re-synchronize
	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_SYNCHRONIZE,
				 PCM512x_RQSY, PCM512x_RQSY_HALT));

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_SYNCHRONIZE,
				 PCM512x_RQSY, PCM512x_RQSY_RESUME));

	return 0;
}

static int pcm5242_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
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
		dev_err(component->dev, "PCM5242 needs to be a slave device\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		dev_err(component->dev, "PCM5242 cannot use inverted i2s clocks\n");
		return -EINVAL;
	}

	return 0;
}

const int pcm5242_addr_start = 0x4C;

static int pcm5242_dai_probe(struct snd_soc_dai *dai) {
	struct i2c_client* client;
	struct regmap_config config = pcm512x_regmap;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	dev_info(dai->component->dev, "Probing PCM5242@%d", dai->id);

	client = devm_i2c_new_dummy_device(dai->component->dev, priv_data->adapter, 
		pcm5242_addr_start + dai->id);
	if (IS_ERR(client)) {
		return PTR_ERR(client);
	}

	/* msb needs to be set to enable auto-increment of addresses */
	config.read_flag_mask = 0x80;
	config.write_flag_mask = 0x80;

	priv_data->chips[dai->id] = devm_regmap_init_i2c(client, &config);

	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], PCM512x_POWER,
				PCM512x_RQST | PCM512x_RQPD));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], PCM512x_RESET,
			   PCM512x_RSTM | PCM512x_RSTR));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], PCM512x_RESET, 0));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], PCM512x_POWER, 0));

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], PCM512x_GPIO_EN, PCM512x_G6OE, PCM512x_G6OE));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], PCM512x_GPIO_OUTPUT_6, 0b00011));

	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], 
		PCM512x_DIGITAL_VOLUME_1, 0b01));

	ret = pcm5242_apply_volume(dai);
	if (ret)
		return ret;

	return 0;
}

static const struct snd_soc_dai_ops smartcross_pcm5242_dai_ops = {
	.hw_params = pcm5242_hw_params,
	.set_fmt = pcm5242_set_fmt,
	.mute_stream = pcm5242_mute_stream,
	.trigger = pcm5242_trigger,
};

int smartcross_pcm5242_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx) {
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);
	int err;
	uint8_t addr = pcm5242_addr_start + idx;

	/* Probe PCM1792A by checking ID register */
	union i2c_smbus_data data;
	err = i2c_smbus_xfer(priv_data->adapter, addr, 0, I2C_SMBUS_READ, 0,
				I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0) {
		return -ENODEV;
	}

	driver->id = idx;
	driver->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "PCM5242@%d", idx);
	if (IS_ERR(driver->name)) {
		return PTR_ERR(driver->name);
	}
	driver->probe = pcm5242_dai_probe;
	driver->ops = &smartcross_pcm5242_dai_ops;
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