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

#include "smartcross.h"

static const DECLARE_TLV_DB_SCALE(vol_tlv, -12750, 50, 0);

static int master_vol_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = VOL_MAX;
	return 0;
}

static int master_vol_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(codec);
	ucontrol->value.integer.value[0] = priv_data->volume;
	return 0;
}

static int master_vol_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dai *dai;
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(codec);
	u16 new_vol = ucontrol->value.integer.value[0];
	if (new_vol > VOL_MAX) {
		new_vol = VOL_MAX;
	}
	if (priv_data->volume != new_vol) {
		priv_data->volume = new_vol;

		for_each_component_dais(codec, dai) {
			if (dai->driver->ops && dai->driver->ops->trigger) {
				ret = dai->driver->ops->trigger(NULL, SMARTCROSS_TRIGGER_VOLUME, dai);
				if (ret < 0)
					return ret;
			}
		}
		return 1;
	} else {
		return 0;
	}
}

static void mute_function(struct work_struct* work) {
	struct delayed_work* dwork;
	struct smartcross_dac_priv *priv_data;

	dwork = container_of(work, struct delayed_work, work);
	priv_data = container_of(dwork, struct smartcross_dac_priv, mute_work);
	gpiod_set_value(priv_data->mute_gpio, priv_data->unmute && priv_data->playing);
}

static void apply_mute(struct smartcross_dac_priv* priv_data)
{
	bool unmute = priv_data->unmute && priv_data->playing;
	if (unmute) {
		/* wait for a short time before unmuting to avoid click-pop */
		schedule_delayed_work(&priv_data->mute_work, 3);
	} else {
		cancel_delayed_work_sync(&priv_data->mute_work);
		gpiod_set_value(priv_data->mute_gpio, 0);
	}
}

static int smartcross_trigger(struct snd_soc_component *component,
				struct snd_pcm_substream *cstream,
				int cmd)
{
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(component);
	bool playing;
	
	if (cstream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		playing = true;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		playing = false;
		break;

	default:
		return 0;
	}

	if (priv_data->playing != playing) {
		priv_data->playing = playing; 
		apply_mute(priv_data);
	}

	return 0;
}


static int master_mute_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int master_mute_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(codec);
	ucontrol->value.integer.value[0] = priv_data->unmute;
	return 0;
}

static int master_mute_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_kcontrol_chip(kcontrol);
	struct smartcross_dac_priv *priv_data =
		snd_soc_component_get_drvdata(codec);
	bool enable = ucontrol->value.integer.value[0];
	
	if (priv_data->unmute != enable) {
		priv_data->unmute = enable;
		apply_mute(priv_data);
		return 1;
	}

	return 0;
}

static const struct snd_kcontrol_new smartcross_snd_controls[] = {
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	  .access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
		     SNDRV_CTL_ELEM_ACCESS_TLV_READ),
	  .name = "Master Playback Volume",
	  .info = master_vol_info,
	  .get = master_vol_get,
	  .put = master_vol_put,
	  .tlv = { .p = vol_tlv } },
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	  .name = "Master Playback Switch",
	  .info = master_mute_info,
	  .get = master_mute_get,
	  .put = master_mute_put },
};

static const struct snd_soc_component_driver smartcross_dac_codec_driver = {
	.controls = smartcross_snd_controls,
	.num_controls = ARRAY_SIZE(smartcross_snd_controls),
	.trigger = smartcross_trigger,
	.non_legacy_dai_naming = true,
};

static int smartcross_dac_probe(struct platform_device *pdev)
{
	int ret, val, i, cnt;
	struct device_node *i2c_node;
	struct snd_soc_dai_driver* dai_drivers;
	struct smartcross_dac_priv *priv_data = devm_kzalloc(
		&pdev->dev, sizeof(struct smartcross_dac_priv), GFP_KERNEL);

	if (!priv_data)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv_data);

	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (!i2c_node) {
		dev_err(&pdev->dev, "Failed to find i2c node in device tree\n");
		ret = -ENODEV;
		goto fail_ret;
	}

	priv_data->adapter = of_find_i2c_adapter_by_node(i2c_node);
	of_node_put(i2c_node);
	if (!priv_data->adapter) {
		dev_err(&pdev->dev,
			"Failed to get i2c adapter by node\n");
		ret = -EPROBE_DEFER;
		goto fail_ret;
	}

	priv_data->enable_gpio =
		devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv_data->enable_gpio)) {
		ret = PTR_ERR(priv_data->enable_gpio);
		dev_err(&pdev->dev,
			"Failed to get enable gpio line: %d\n", ret);
		goto fail_i2c;
	}

	priv_data->mute_gpio =
		devm_gpiod_get(&pdev->dev, "mute", GPIOD_OUT_LOW);
	if (IS_ERR(priv_data->mute_gpio)) {
		ret = PTR_ERR(priv_data->mute_gpio);
		dev_err(&pdev->dev,
			"Failed to get mute gpio line: %d\n", ret);
		goto fail_i2c;
	}

	priv_data->default_pmp = 2;
	if (!of_property_read_u32(pdev->dev.of_node, "ma120x0p,default-pmp",
				  &val)) {
		if ((val >= 0) && (val <= 4))
			priv_data->default_pmp = val;
		else
			dev_warn(&pdev->dev, "Wrong default-pmp value: %d\n", val);
	}

	// set to a low volume (10%)
	priv_data->volume = 137; 

	priv_data->unmute = 1;
	priv_data->playing = 0;
	INIT_DELAYED_WORK(&priv_data->mute_work, mute_function);

	dai_drivers = devm_kzalloc(&pdev->dev, NUMBER_OF_CHIPS * sizeof(struct snd_soc_dai_driver), GFP_KERNEL);

	udelay(200);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 1);
	msleep(2);

	cnt = 0;
	for (i = 0; i < NUMBER_OF_CHIPS; i++) {
		if (smartcross_ma120x0p_initialize_driver(pdev, &dai_drivers[i], i) >= 0) {
			cnt++;
			continue;
		}

		if (smartcross_pcm1792a_initialize_driver(pdev, &dai_drivers[i], i) >= 0) {
			cnt++;
			continue;
		}

		ret = smartcross_null_initialize_driver(pdev, &dai_drivers[i], i);
		if (ret < 0)
			goto fail_i2c;
	}

	if (cnt == 0) {
		dev_err(&pdev->dev, "No SmartCross output modules are detected.\n");
		ret = -ENODEV;
		goto fail_i2c;
	} else {
		dev_info(&pdev->dev, "A total of %d SmartCross output modules are present\n", cnt);
	}

	ret = devm_snd_soc_register_component(
		&pdev->dev, &smartcross_dac_codec_driver, dai_drivers, NUMBER_OF_CHIPS);
	if (ret < 0)
		goto fail_i2c;

	return 0;

fail_i2c:
	i2c_put_adapter(priv_data->adapter);

fail_ret:
	return ret;
}

static void smartcross_dac_shutdown(struct platform_device *pdev)
{
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&priv_data->mute_work);
	gpiod_set_value_cansleep(priv_data->mute_gpio, 0);
	udelay(200);
	gpiod_set_value_cansleep(priv_data->enable_gpio, 0);
}

static int smartcross_dac_remove(struct platform_device *pdev)
{
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);
	smartcross_dac_shutdown(pdev);
	i2c_put_adapter(priv_data->adapter);
	return 0;
}

static const struct of_device_id smartcross_dac_of_match[] = {
	{
		.compatible = "smartcross,dac-complex",
	},
	{}
};

static struct platform_driver smartcross_dac_driver = {
	.probe   = smartcross_dac_probe,
	.remove  = smartcross_dac_remove,
	.shutdown = smartcross_dac_shutdown,
	.driver  = {
		.name = "smartcross-dac",
		.of_match_table = of_match_ptr(smartcross_dac_of_match),
	},
};

module_platform_driver(smartcross_dac_driver);

MODULE_AUTHOR("Yunhao Tian <t123yh@outlook.com>");
MODULE_DESCRIPTION("Driver for SmartCross DAC complex");
MODULE_LICENSE("GPL v2");
