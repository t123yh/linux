#include <sound/pcm.h>
#include <linux/dev_printk.h>
#include "smartcross.h"

int smartcross_null_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx) {
	struct smartcross_dac_priv *priv_data = platform_get_drvdata(pdev);

	driver->id = idx;
	driver->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "null@%d", idx);
	if (IS_ERR(driver->name)) {
		return PTR_ERR(driver->name);
	}
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