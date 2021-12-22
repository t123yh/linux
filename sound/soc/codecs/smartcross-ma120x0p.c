#include "smartcross.h"
#include "ma120x0p.h"

static int ma120x0p_apply_volume(struct snd_soc_dai* dai) {
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	
	int raw_vol = (VOL_MAX - (int)priv_data->volume) * 2;

	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], ma_vol_db_master__a, raw_vol >> 2));
	ENSURE_SUCCESS(regmap_write(priv_data->chips[dai->id], ma_vol_lsb_master__a, raw_vol & 0b11));

	return 0;
}

static int ma120x0p_trigger(struct snd_pcm_substream *substream, int cmd, 
	struct snd_soc_dai * dai) {
	switch (cmd) {
	case SMARTCROSS_TRIGGER_VOLUME:
		ma120x0p_apply_volume(dai);
	}
	return 0;
}

static int ma120x0p_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);

	uint8_t iface, pol;

	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(component->dev, "MA120x0p needs to be a slave device\n");
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
		// We don't know the bit width! No one uses right justfied, anyway
		dev_err(component->dev, "Sorry, this driver doesn't support right justified mode\n");
		return -EINVAL; 
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		pol = 0b01;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		pol = 0b11;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		pol = 0b00;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		pol = 0b10;
		break;
	default:
		return -EINVAL;
	}

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], ma_i2s_format__a,
		ma_i2s_format__mask, iface));
	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], ma_i2s_sck_pol__a,
		ma_i2s_sck_pol__mask | ma_i2s_ws_pol__mask, pol));

	return 0;
}

static int ma120x0p_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);

	ENSURE_SUCCESS(regmap_update_bits(
		priv_data->chips[dai->id], ma_system_mute__a, ma_system_mute__mask, 
		mute ? (1 << ma_system_mute__shift) : 0));
	return 0;
}

static bool ma120x0p_regmap_is_volatile(
	struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x7E: // clip and limiter
	case 0x60: // ch.0 freq and power mode
	case 0x61: // ch.0 monitor
	case 0x62: // ch.0 modulation index
	case 0x64: // ch.1 freq and power mode
	case 0x65: // ch.1 monitor
	case 0x66: // ch.1 modulation index
	case 0x6D: // Error accumulated register
	case 0x75: // MSEL monitor
	case 0x7C: // Error register
		return true;
	default:
		return false;

	}
}

static bool ma120x0p_regmap_is_writable(
	struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x1D: // PMprofile
	case 0x26: // DC protection
	case 0x2D: // Error handler clear
	case 0x35: // i2s format
	case 0x36: // i2s frame
	case 0x40: // master vol. db
	case 0x41: // master vol. frac
		return true;
	default:
		return false;

	}
}

static bool ma120x0p_regmap_is_readable(
	struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x1D: // PMprofile
	case 0x2D: // Error handler clear
	case 0x26: // DC protection
	case 0x35: // i2s format
	case 0x36: // i2s frame
	case 0x40: // master vol. db
	case 0x41: // master vol. frac
	case 0x7E: // clip and limiter
	case 0x60: // ch.0 freq and power mode
	case 0x61: // ch.0 monitor
	case 0x62: // ch.0 modulation index
	case 0x64: // ch.1 freq and power mode
	case 0x65: // ch.1 monitor
	case 0x66: // ch.1 modulation index
	case 0x6D: // Error accumulated register
	case 0x75: // MSEL monitor
	case 0x7C: // Error register
		return true;
	default:
		return false;

	}
}

static const struct regmap_config ma120x0p_regmap_config = {
	.name = "ma120x0p",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7E,
	.writeable_reg = ma120x0p_regmap_is_writable,
	.readable_reg = ma120x0p_regmap_is_readable,
	.volatile_reg = ma120x0p_regmap_is_volatile,
	.cache_type = REGCACHE_RBTREE,
};

static const struct snd_soc_dai_ops smartcross_ma120x0p_dai_ops = {
	.set_fmt = ma120x0p_set_fmt,
	.mute_stream = ma120x0p_mute_stream,
	.trigger = ma120x0p_trigger
};

const int ma120x0p_addr_start = 0x20;

static int ma120x0p_dai_probe(struct snd_soc_dai *dai) {
	struct i2c_client* client;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	dev_info(dai->component->dev, "Probing MA120x0P@%d", dai->id);

	client = devm_i2c_new_dummy_device(dai->component->dev, priv_data->adapter, 
		ma120x0p_addr_start + dai->id);
	if (IS_ERR(client)) {
		return PTR_ERR(client);
	}
	priv_data->chips[dai->id] = devm_regmap_init_i2c(
		client, &ma120x0p_regmap_config);

	ret = ma120x0p_apply_volume(dai);
	if (ret)
		return ret;

	ENSURE_SUCCESS(regmap_update_bits(priv_data->chips[dai->id], 
		ma_pmprofile__a, 
		ma_pmprofile__mask, 
		priv_data->default_pmp));

	ENSURE_SUCCESS(regmap_set_bits(priv_data->chips[dai->id], 
		ma_audio_proc_enable__a, 
		ma_audio_proc_enable__mask));


	return 0;
}

int smartcross_ma120x0p_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx) {
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);
	int err;
	uint8_t addr = ma120x0p_addr_start + idx;
	union i2c_smbus_data data;
	err = i2c_smbus_xfer(priv_data->adapter, addr, 0, I2C_SMBUS_READ, 0x03,
				I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0) {
		return -ENODEV;
	}
	/* Initial value of 0x03 should be 0x5A */
	if (data.byte != 0x5A) {
		dev_warn(&pdev->dev, "An unknown device sits on i2c@%x\n", addr);
		return -ENODEV;
	}

	driver->id = idx;
	driver->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "MA12070P@%d", idx);
	if (IS_ERR(driver->name)) {
		return PTR_ERR(driver->name);
	}
	driver->probe = ma120x0p_dai_probe;
	driver->ops = &smartcross_ma120x0p_dai_ops;
	driver->playback = (struct snd_soc_pcm_stream) {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = 
		SNDRV_PCM_FMTBIT_S24_LE |
		SNDRV_PCM_FMTBIT_S32_LE |
		SNDRV_PCM_FMTBIT_S16_LE 
	};

	return 0;
}