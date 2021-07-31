#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#undef dev_dbg
#undef dev_info
#undef dev_warn
#undef dev_notice

#define dev_dbg dev_err
#define dev_info dev_err
#define dev_warn dev_err
#define dev_notice dev_err

#define NUMBER_OF_CHIPS 4
#define VOL_MAX 672U

#define ENSURE_SUCCESS(_x_) { int __ret = _x_; if (__ret < 0) { printk("Error %d at line %d\n", __ret, __LINE__); return __ret;} }

struct ma120x0p_complex_priv {
    struct i2c_client* clients[NUMBER_OF_CHIPS];
    bool is_present[NUMBER_OF_CHIPS];

	struct gpio_desc *enable_gpio;
	struct gpio_desc *mute_gpio;

    int pmp;
	u8 pol, iface;
    u16 volume; // 0...VOL_MAX

    struct mutex op_mutex; 
    bool is_enabled;
};

static int ma120x0p_probe_existence(struct ma120x0p_complex_priv *priv_data) {
    s32 data;
    int i;

    memset(priv_data->is_present, 0, sizeof(priv_data->is_present));

    mutex_lock(&priv_data->op_mutex);
	gpiod_set_value_cansleep(priv_data->mute_gpio, 0);
    udelay(100);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 1);
    msleep(10);
    for (i = 0; i < NUMBER_OF_CHIPS; i++) {
        printk("check exist %d!!!\n", i);
        data = i2c_smbus_read_byte_data(priv_data->clients[i], 0x01);
        printk("got %d!!!\n", data);
        if (data == -ENXIO || data == -ENODEV) {
            printk("Not found!!!\n");
            continue;
        } else if (data < 0) {
            return data;
        } else if (data == 0x3C) {
            priv_data->is_present[i] = true;
        }
    }
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
    mutex_unlock(&priv_data->op_mutex);

    return 0;
}

static int ma120x0p_apply_volume(struct ma120x0p_complex_priv *priv_data) {
    int i;
    u16 raw_vol = min(VOL_MAX, VOL_MAX - priv_data->volume);
    if (priv_data->is_enabled) {
        for (i = 0; i < NUMBER_OF_CHIPS; i++) {
            if (priv_data->is_present[i]) {
                ENSURE_SUCCESS(i2c_smbus_write_byte_data(priv_data->clients[i], 0x40, raw_vol >> 2));
                ENSURE_SUCCESS(i2c_smbus_write_byte_data(priv_data->clients[i], 0x41, raw_vol & 0b11));
            }
        }
    }
    return 0;
}

static int ma120x0p_prepare(struct snd_pcm_substream* substream, struct snd_soc_dai *dai) {
    int i;
	struct snd_soc_component *component = dai->component;
	struct ma120x0p_complex_priv* priv_data = snd_soc_component_get_drvdata(component);

    printk("Prepare!!!\n");
    if (priv_data->is_enabled) {
        return 0;
    }

    ENSURE_SUCCESS(ma120x0p_probe_existence(priv_data));

    mutex_lock(&priv_data->op_mutex);

    priv_data->is_enabled = true;
	gpiod_set_value_cansleep(priv_data->enable_gpio, 1);
    msleep(5);

    for (i = 0; i < NUMBER_OF_CHIPS; i++) {
        if (priv_data->is_present[i]) {
            dev_dbg(component->dev, "Initializing ma120x0p@0x%02X\n", 0x20 + i);
            ENSURE_SUCCESS(i2c_smbus_write_byte_data(priv_data->clients[i], 0x1D, priv_data->pmp & 0b111));
            ENSURE_SUCCESS(i2c_smbus_write_byte_data(priv_data->clients[i], 0x35, priv_data->iface | 0b1000));
            ENSURE_SUCCESS(i2c_smbus_write_byte_data(priv_data->clients[i], 0x36, priv_data->pol));
        }
    }

    ENSURE_SUCCESS(ma120x0p_apply_volume(priv_data));

    msleep(20);

    mutex_unlock(&priv_data->op_mutex);

    return 0;
}

static void ma120x0p_shutdown(struct snd_pcm_substream* substream, struct snd_soc_dai *dai) {
	struct snd_soc_component *component = dai->component;
	struct ma120x0p_complex_priv* priv_data = snd_soc_component_get_drvdata(component);

    dev_dbg(component->dev, "MA120x0p shutting down\n");
    mutex_lock(&priv_data->op_mutex);
	gpiod_set_value_cansleep(priv_data->mute_gpio, 0);
    msleep(10);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
    priv_data->is_enabled = false;
    mutex_unlock(&priv_data->op_mutex);
}

static int ma120x0p_hw_params(struct snd_pcm_substream *substream,
							  struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int ma120x0p_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ma120x0p_complex_priv* priv_data = snd_soc_component_get_drvdata(component);

	/* check master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK)
	{
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK)
	{
	case SND_SOC_DAIFMT_I2S:
		priv_data->iface = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		priv_data->iface = 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL; // We don't know the bit width!
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK)
	{
	case SND_SOC_DAIFMT_NB_NF:
        priv_data->pol = 0b01;
		break;
	case SND_SOC_DAIFMT_NB_IF:
        priv_data->pol = 0b11;
		break;
	case SND_SOC_DAIFMT_IB_NF:
        priv_data->pol = 0b00;
		break;
	case SND_SOC_DAIFMT_IB_IF:
        priv_data->pol = 0b10;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ma120x0p_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct ma120x0p_complex_priv *priv_data = snd_soc_component_get_drvdata(dai->component);

    printk("Ma12707p unmuting \n");
	gpiod_set_value_cansleep(priv_data->mute_gpio, !mute);
	return 0;
}


static const struct snd_soc_dai_ops ma120x0p_dai_ops = {
	.hw_params = ma120x0p_hw_params,
    .prepare = ma120x0p_prepare,
    .shutdown = ma120x0p_shutdown,
	.set_fmt = ma120x0p_set_fmt,
	.mute_stream = ma120x0p_mute_stream,
};

static struct snd_soc_dai_driver ma120x0p_dai = {
	// .name		= "ma120x0p-amp", // have to be commented out because bug in 4.4
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE
    },
	.ops = &ma120x0p_dai_ops,
};

static const DECLARE_TLV_DB_SCALE(ma120x0p_vol_tlv, -14400, 25, 0);
static int ma_master_vol_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = VOL_MAX;
	return 0;
}

static int ma_master_vol_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct ma120x0p_complex_priv *priv_data = snd_soc_component_get_drvdata(codec);
    ucontrol->value.integer.value[0] = priv_data->volume;
	return 0;
}

static int ma_master_vol_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct ma120x0p_complex_priv *priv_data = snd_soc_component_get_drvdata(codec);
    u16 new_vol = ucontrol->value.integer.value[0];
    if (new_vol > VOL_MAX) {
        new_vol = VOL_MAX;
    }
    if (priv_data->volume != new_vol) {
        priv_data->volume = new_vol;

        mutex_lock(&priv_data->op_mutex);
        dev_dbg(codec->dev, "Setting ma120x0p volume to %d\n", new_vol);
        ENSURE_SUCCESS(ma120x0p_apply_volume(priv_data));
        mutex_unlock(&priv_data->op_mutex);
        return 1;
    } else {
        return 0;
    }
}

static const struct snd_kcontrol_new ma120x0p_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_TLV_READ),
		.name = "Master Playback Volume",
		.info = ma_master_vol_info,
		.get = ma_master_vol_get,
		.put = ma_master_vol_put,
		.tlv = { .p = ma120x0p_vol_tlv }
	},
};

static const struct snd_soc_component_driver ma120x0p_complex_codec_driver = {
	.controls = ma120x0p_snd_controls,
	.num_controls = ARRAY_SIZE(ma120x0p_snd_controls),
};

static int ma120x0p_complex_probe(struct platform_device *pdev)
{
    int ret, val, i;

	struct device_node *i2c_node;
    struct i2c_adapter *i2c_adapter;

    struct ma120x0p_complex_priv* priv_data = devm_kzalloc(&pdev->dev, sizeof(struct ma120x0p_complex_priv), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv_data);
    dev_info(&pdev->dev, "MA120x0p complex probing...\n");

    priv_data->volume = 400; // set to a low volume
    mutex_init(&priv_data->op_mutex);

	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (!i2c_node) {
		dev_err(&pdev->dev, "Failed to find i2c node in device tree\n");
		return -ENODEV;
	}

	i2c_adapter = of_find_i2c_adapter_by_node(i2c_node);
	if (!i2c_adapter) {
		dev_err(&pdev->dev, "Failed to get ma120x0p i2c adapter by node\n");
		return -EPROBE_DEFER;
	}

	priv_data->enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv_data->enable_gpio))
	{
		ret = PTR_ERR(priv_data->enable_gpio);
		dev_err(&pdev->dev,
				"Failed to get ma120x0p enable gpio line: %d\n", ret);
		return ret;
	}

	priv_data->mute_gpio = devm_gpiod_get(&pdev->dev, "mute", GPIOD_OUT_LOW);
	if (IS_ERR(priv_data->mute_gpio))
	{
		ret = PTR_ERR(priv_data->mute_gpio);
		dev_err(&pdev->dev,
				"Failed to get ma120x0p mute gpio line: %d\n", ret);
		return ret;
	}

	priv_data->pmp = 2;
	if (!of_property_read_u32(pdev->dev.of_node, "ma120x0p,default-pmp", &val))
	{
		if ((val >= 0) && (val <= 4))
			priv_data->pmp = val;
	}

    for (i = 0; i < NUMBER_OF_CHIPS; i++) {
        priv_data->clients[i] = i2c_new_dummy(i2c_adapter, 0x20 + i);
        if (IS_ERR(priv_data->clients[i])) {
            ret = PTR_ERR(priv_data->clients[i]);
            goto err_i2c;
        }
    }

	ret = devm_snd_soc_register_component(&pdev->dev,
										  &ma120x0p_complex_codec_driver, 
                                          &ma120x0p_dai, 1);
    return ret;

err_i2c:
    for (i = 0; i < NUMBER_OF_CHIPS; i++) {
        if (!IS_ERR(priv_data->clients[i])) {
            (void) i2c_unregister_device(priv_data->clients[i]);
        }
    }

    return ret;
}

static int ma120x0p_complex_remove(struct platform_device *pdev)
{
    int i;
	struct ma120x0p_complex_priv *priv_data = platform_get_drvdata(pdev);

    for (i = 0; i < NUMBER_OF_CHIPS; i++) {
        if (!IS_ERR(priv_data->clients[i])) {
            (void) i2c_unregister_device(priv_data->clients[i]);
        }
    }

    return 0;
}

static void ma120x0p_complex_shutdown(struct platform_device *pdev) {
	struct ma120x0p_complex_priv *priv_data = platform_get_drvdata(pdev);
    mutex_lock(&priv_data->op_mutex);
	gpiod_set_value_cansleep(priv_data->mute_gpio, 0);
    udelay(50);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
    priv_data->is_enabled = false;
    mutex_unlock(&priv_data->op_mutex);
}


static const struct of_device_id ma120x0p_complex_of_match[] = {
	{ .compatible = "ma120x0p-complex", },
	{ }
};

static struct platform_driver ma120x0p_complex_driver = {
	.probe   = ma120x0p_complex_probe,
	.remove  = ma120x0p_complex_remove,
    .shutdown = ma120x0p_complex_shutdown,
	.driver  = {
		.name = "ma120x0p-complex",
		.of_match_table = of_match_ptr(ma120x0p_complex_of_match),
	},
};

module_platform_driver(ma120x0p_complex_driver);

MODULE_AUTHOR("Yunhao Tian <t123yh@outlook.com>");
MODULE_DESCRIPTION("MA120x0p complex codec driver");
MODULE_LICENSE("GPL v2");
