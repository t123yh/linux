#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio.h>

#define NUMBER_OF_CHIPS 4
/* A somewhat hacky solution to adjust volume */
#define SMARTCROSS_TRIGGER_VOLUME		1000
#define SMARTCROSS_TRIGGER_SUSPEND		1001
#define SMARTCROSS_TRIGGER_RESUME		1002
#define VOL_MAX 255U


#define ENSURE_SUCCESS(_x_)                                                    \
	{                                                                      \
		int __ret = _x_;                                               \
		if (__ret < 0) {                                               \
			printk("Error %d at line %d\n", __ret, __LINE__);      \
			return __ret;                                          \
		}                                                              \
	}

int smartcross_null_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx);
int smartcross_ma120x0p_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx);
int smartcross_cs4398_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx);
int smartcross_pcm1792a_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx);
int smartcross_pcm5242_initialize_driver(struct platform_device* pdev, struct snd_soc_dai_driver* driver, int idx);

struct smartcross_dac_priv {
	struct i2c_adapter* adapter;
	struct regmap *chips[NUMBER_OF_CHIPS];
	unsigned int format[NUMBER_OF_CHIPS];
	unsigned int rate[NUMBER_OF_CHIPS];

	struct gpio_desc *enable_gpio;
	struct gpio_desc *mute_gpio;

	struct delayed_work mute_work;
	bool unmute, playing;

	/* default power mode profile for MA120x0p */
	int default_pmp;
	/* 255 (0db) => max vol, 0 (-127.5db) => min vol, in 0.5db steps */
	u8 volume; 
};
