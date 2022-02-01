// SPDX-License-Identifier: GPL-2.0
//
// ALSA SoC Audio Layer - Rockchip SPDIF_RX Controller driver
//
// Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd
// Copyright (C) 2021 Yunhao Tian <t123yh.xyz@gmail.com>
//

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

#include "rockchip_spdifrx.h"

struct rk_spdifrx_dev {
	struct snd_soc_dai *dai;
	struct snd_pcm_substream *substream;
	struct snd_kcontrol *snd_kctl_sync;
	struct snd_kcontrol *snd_kctl_rate;
	struct snd_kcontrol *snd_kctl_chstat;
	struct snd_kcontrol *snd_kctl_chstat_mask;
	struct snd_kcontrol *snd_kctl_usr;
	/* Prevent race condition on stream state */
	spinlock_t irq_lock;

	struct device *dev;
	struct clk *mclk;
	struct clk *hclk;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct regmap *regmap;
	struct reset_control *reset;
	int irq;
};

static int rk_spdifrx_get_rate(const struct rk_spdifrx_dev *spdifrx);

static int rk_spdifrx_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	int current_rate = rk_spdifrx_get_rate(spdifrx);
	int requested_rate = params_rate(params);
	if (params_rate(params) != current_rate) {
		dev_err(spdifrx->dev, "SPDIF rate mismatch: requested %dHz, actual %dHz\n", requested_rate, current_rate);
		return -EINVAL;
	}
	return 0;
}

static void rk_spdifrx_reset(struct rk_spdifrx_dev *spdifrx)
{
	reset_control_assert(spdifrx->reset);
	udelay(1);
	reset_control_deassert(spdifrx->reset);
}

static int rk_spdifrx_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	spin_lock_irqsave(&spdifrx->irq_lock, flags);
	spdifrx->substream = substream;
	spin_unlock_irqrestore(&spdifrx->irq_lock, flags);

	return 0;
}

static void rk_spdifrx_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	spin_lock_irqsave(&spdifrx->irq_lock, flags);
	spdifrx->substream = NULL;
	spin_unlock_irqrestore(&spdifrx->irq_lock, flags);
}

static int rk_spdifrx_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	int ret = 0;
	const unsigned int irqs =
		SPDIFRX_INT_BMDE | SPDIFRX_INT_RXO | SPDIFRX_INT_PE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* Reset receiving clock */
		rk_spdifrx_reset(spdifrx);

		/* Clear interrupts before enabling them */
		regmap_write(spdifrx->regmap, SPDIFRX_INTCLR, irqs);
		regmap_update_bits(spdifrx->regmap, SPDIFRX_INTEN, irqs, irqs);

		regmap_update_bits(spdifrx->regmap, SPDIFRX_DMACR,
				   SPDIFRX_DMACR_RDE_MASK,
				   SPDIFRX_DMACR_RDE_ENABLE);

		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		regmap_update_bits(spdifrx->regmap, SPDIFRX_INTEN, irqs, 0);

		regmap_update_bits(spdifrx->regmap, SPDIFRX_DMACR,
				   SPDIFRX_DMACR_RDE_MASK,
				   SPDIFRX_DMACR_RDE_DISABLE);

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rk_spdifrx_iec958_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

#define CONTROL_PRIV_IS_USR BIT(0)

static int rk_spdifrx_iec958_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	int i;
	unsigned int val;
	unsigned int addr;

	for (i = 0; i < 12; i++) {
		addr = (kcontrol->private_value & CONTROL_PRIV_IS_USR) ?
			       SPDIFRX_USRDR(i) :
			       SPDIFRX_CHNSR(i);
		regmap_read(spdifrx->regmap, addr, &val);
		ucontrol->value.iec958.status[i * 2] = val & 0xFF;
		ucontrol->value.iec958.status[i * 2 + 1] = (val >> 8) & 0xFF;
	}

	return 0;
}

static int rk_spdifrx_iec958_mask_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	memset(ucontrol->value.iec958.status, 0xFF,
	       sizeof(ucontrol->value.iec958.status));

	return 0;
}

static int rk_spdifrx_is_sync(const struct rk_spdifrx_dev *spdifrx)
{
	u32 val;

	regmap_read(spdifrx->regmap, SPDIFRX_CDR, &val);
	return (val & SPDIFRX_CDR_CS_MASK) == SPDIFRX_CDR_CS_SYNC;
}

static const int sample_rates[] = 
	{ 32000, 44100, 48000, 88200, 96000, 176400, 192000 };
static int rk_spdifrx_get_rate(const struct rk_spdifrx_dev *spdifrx)
{
	u32 pulse, clk_rate, est_pulse, val;
	int i;

	regmap_read(spdifrx->regmap, SPDIFRX_CDRST, &val);
	pulse = val & 0xFF;
	if (pulse != 0) {
		clk_rate = clk_get_rate(spdifrx->mclk);
		for (i = 0; i < ARRAY_SIZE(sample_rates); i++) {
			est_pulse = DIV_ROUND_CLOSEST(clk_rate,
						      sample_rates[i] * 128);
			if (pulse >= est_pulse - 1 && pulse <= est_pulse + 1)
				return sample_rates[i];
		}
	}
	return 0;
}

static int rk_spdifrx_rate_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;
	return 0;
}

static int rk_spdifrx_rate_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);

	ucontrol->value.integer.value[0] = rk_spdifrx_get_rate(spdifrx);
	return 0;
}

static int rk_spdifrx_sync_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	return 0;
}

static int rk_spdifrx_sync_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);

	ucontrol->value.integer.value[0] = rk_spdifrx_is_sync(spdifrx);
	return 0;
}

static int rk_spdifrx_dai_probe(struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);
	struct snd_card *card = dai->component->card->snd_card;
	struct snd_kcontrol_new control;
	int ret;
	const unsigned int irqs = SPDIFRX_INT_ESYNC | SPDIFRX_INT_NSYNC |
				  SPDIFRX_INT_UBC | SPDIFRX_INT_CSC;

	rk_spdifrx_reset(spdifrx);

	ret = clk_prepare_enable(spdifrx->mclk);
	if (ret) {
		dev_err(spdifrx->dev, "mclk clock enable failed %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(spdifrx->hclk);
	if (ret) {
		dev_err(spdifrx->dev, "hclk clock enable failed %d\n", ret);
		return ret;
	}

	regmap_write(spdifrx->regmap, SPDIFRX_CLR, SPDIFRX_CLR_RXSC);
	regmap_update_bits(spdifrx->regmap, SPDIFRX_CDR,
			   SPDIFRX_CDR_AVGSEL_MASK | SPDIFRX_CDR_BYPASS_MASK,
			   SPDIFRX_CDR_AVGSEL_AVG | SPDIFRX_CDR_BYPASS_DIS);
	regmap_update_bits(spdifrx->regmap, SPDIFRX_DMACR,
			   SPDIFRX_DMACR_RDL_MASK, SPDIFRX_DMACR_RDL(8));
	dai->capture_dma_data = &spdifrx->capture_dma_data;

	/* the access property of controls don't have to
	 * be volatile, as it will be notified by interrupt handler
	 */
	control = (struct snd_kcontrol_new){
		/* Sample Rate Control */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "Rate",
		/* access don't have to be volatile, as it will be notified by intr */
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = rk_spdifrx_rate_info,
		.get = rk_spdifrx_rate_get,
	};
	spdifrx->snd_kctl_rate = snd_ctl_new1(&control, dai);

	control = (struct snd_kcontrol_new){
		/* Sync Control */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "Sync",
		/* access don't have to be volatile, as it will be notified by intr */
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = rk_spdifrx_sync_info,
		.get = rk_spdifrx_sync_get,
	};
	spdifrx->snd_kctl_sync = snd_ctl_new1(&control, dai);

	control = (struct snd_kcontrol_new){
		/* Channel Status Bits */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
		/* access don't have to be volatile, as it will be notified by intr */
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = rk_spdifrx_iec958_info,
		.get = rk_spdifrx_iec958_get,
		.private_value = 0
	};
	spdifrx->snd_kctl_chstat = snd_ctl_new1(&control, dai);

	control = (struct snd_kcontrol_new){
		/* User Bits */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "User Bit",
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = rk_spdifrx_iec958_info,
		.get = rk_spdifrx_iec958_get,
		.private_value = CONTROL_PRIV_IS_USR
	};
	spdifrx->snd_kctl_usr = snd_ctl_new1(&control, dai);

	control = (struct snd_kcontrol_new){
		/* Channel Status Mask */
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", CAPTURE, MASK),
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = rk_spdifrx_iec958_info,
		.get = rk_spdifrx_iec958_mask_get,
	};
	spdifrx->snd_kctl_chstat_mask = snd_ctl_new1(&control, dai);

	spdifrx->dai = dai;

	ret = snd_ctl_add(card, spdifrx->snd_kctl_rate);
	if (ret < 0) {
		dev_err(spdifrx->dev, "Failed to add rate control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, spdifrx->snd_kctl_sync);
	if (ret < 0) {
		dev_err(spdifrx->dev, "Failed to add sync control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, spdifrx->snd_kctl_chstat);
	if (ret < 0) {
		dev_err(spdifrx->dev,
			"Failed to add channel status control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, spdifrx->snd_kctl_chstat_mask);
	if (ret < 0) {
		dev_err(spdifrx->dev,
			"Failed to add channel status mask control: %d\n", ret);
		return ret;
	}

	ret = snd_ctl_add(card, spdifrx->snd_kctl_usr);
	if (ret < 0) {
		dev_err(spdifrx->dev, "Failed to add user bits control: %d\n",
			ret);
		return ret;
	}

	regmap_write(spdifrx->regmap, SPDIFRX_INTEN, irqs);

	regmap_update_bits(spdifrx->regmap, SPDIFRX_CFGR, SPDIFRX_EN_MASK,
			   SPDIFRX_EN);

	return 0;
}

static int rk_spdifrx_dai_remove(struct snd_soc_dai *dai)
{
	struct rk_spdifrx_dev *spdifrx = snd_soc_dai_get_drvdata(dai);

	/* disable spdif */
	regmap_update_bits(spdifrx->regmap, SPDIFRX_CFGR, SPDIFRX_EN_MASK,
			   SPDIFRX_DIS);

	clk_disable_unprepare(spdifrx->hclk);
	clk_disable_unprepare(spdifrx->mclk);

	return 0;
}

static const struct snd_soc_dai_ops rk_spdifrx_dai_ops = {
	.startup = rk_spdifrx_startup,
	.hw_params = rk_spdifrx_hw_params,
	.trigger = rk_spdifrx_trigger,
	.shutdown = rk_spdifrx_shutdown,
};

static struct snd_soc_dai_driver rk_spdifrx_dai = {
	.probe = rk_spdifrx_dai_probe,
	.remove	= rk_spdifrx_dai_remove,
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_32000 |
			  SNDRV_PCM_RATE_44100 |
			  SNDRV_PCM_RATE_48000 |
			  SNDRV_PCM_RATE_88200 |
			  SNDRV_PCM_RATE_96000 |
			  SNDRV_PCM_RATE_176400 |
			  SNDRV_PCM_RATE_192000),
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &rk_spdifrx_dai_ops,
};

static const struct snd_soc_component_driver rk_spdifrx_component = {
	.name = "rockchip-spdifrx",
};

static bool rk_spdifrx_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SPDIFRX_CFGR:
	case SPDIFRX_CLR:
	case SPDIFRX_CDR:
	case SPDIFRX_CDRST:
	case SPDIFRX_DMACR:
	case SPDIFRX_FIFOCTRL:
	case SPDIFRX_INTEN:
	case SPDIFRX_INTMASK:
	case SPDIFRX_INTSR:
	case SPDIFRX_INTCLR:
	case SPDIFRX_SMPDR:
	case SPDIFRX_BURSTINFO:
		return true;
	default:
		return false;
	}
}

static bool rk_spdifrx_rd_reg(struct device *dev, unsigned int reg)
{
	if (reg >= SPDIFRX_USRDR(0) && reg < SPDIFRX_CHNSR(12))
		return true;

	switch (reg) {
	case SPDIFRX_CFGR:
	case SPDIFRX_CLR:
	case SPDIFRX_CDR:
	case SPDIFRX_CDRST:
	case SPDIFRX_DMACR:
	case SPDIFRX_FIFOCTRL:
	case SPDIFRX_INTEN:
	case SPDIFRX_INTMASK:
	case SPDIFRX_INTSR:
	case SPDIFRX_INTCLR:
	case SPDIFRX_SMPDR:
	case SPDIFRX_BURSTINFO:
		return true;
	default:
		return false;
	}
}

static bool rk_spdifrx_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg >= SPDIFRX_USRDR(0) && reg < SPDIFRX_CHNSR(12))
		return true;

	switch (reg) {
	case SPDIFRX_CLR:
	case SPDIFRX_CDR:
	case SPDIFRX_CDRST:
	case SPDIFRX_FIFOCTRL:
	case SPDIFRX_INTSR:
	case SPDIFRX_INTCLR:
	case SPDIFRX_SMPDR:
	case SPDIFRX_BURSTINFO:
		return true;
	default:
		return false;
	}
}

static bool rk_spdifrx_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SPDIFRX_SMPDR:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rk_spdifrx_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SPDIFRX_BURSTINFO,
	.writeable_reg = rk_spdifrx_wr_reg,
	.readable_reg = rk_spdifrx_rd_reg,
	.volatile_reg = rk_spdifrx_volatile_reg,
	.precious_reg = rk_spdifrx_precious_reg,
	.cache_type = REGCACHE_FLAT,
};

static irqreturn_t rk_spdifrx_isr(int irq, void *dev_id)
{
	struct rk_spdifrx_dev *spdifrx = dev_id;
	struct snd_card *card;
	bool sync_changed = false, err = false, xrun_err = false;
	u32 intsr;

	regmap_read(spdifrx->regmap, SPDIFRX_INTSR, &intsr);

	if (!intsr)
		return IRQ_NONE;

	if (intsr & SPDIFRX_INT_NSYNC) {
		dev_dbg(spdifrx->dev, "Exiting sync status\n");
		sync_changed = true;
		err = true;
	}

	if (intsr & SPDIFRX_INT_ESYNC) {
		sync_changed = true;
	}

	if (intsr & SPDIFRX_INT_BMDE) {
		dev_warn(spdifrx->dev, "Biphase mark decoding error\n");
		err = true;
	}

	if (intsr & SPDIFRX_INT_PE) {
		dev_warn(spdifrx->dev, "Parity error\n");
		err = true;
	}

	if (intsr & SPDIFRX_INT_RXO) {
		dev_warn(spdifrx->dev, "FIFO overrun error\n");
		xrun_err = true;
	}

	/* clear all handled interrupts */
	regmap_write(spdifrx->regmap, SPDIFRX_INTCLR, intsr);

	spin_lock(&spdifrx->irq_lock);
	if (spdifrx->dai) {
		card = spdifrx->dai->component->card->snd_card;
		if (sync_changed) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &spdifrx->snd_kctl_sync->id);
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &spdifrx->snd_kctl_rate->id);
		}
		if (intsr & SPDIFRX_INT_UBC) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &spdifrx->snd_kctl_usr->id);
		}
		if (intsr & SPDIFRX_INT_CSC) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &spdifrx->snd_kctl_chstat->id);
		}
	}

	if (err && spdifrx->substream)
		snd_pcm_stop(spdifrx->substream, SNDRV_PCM_STATE_DISCONNECTED);

	if (xrun_err && spdifrx->substream)
		snd_pcm_stop_xrun(spdifrx->substream);

	spin_unlock(&spdifrx->irq_lock);

	return IRQ_HANDLED;
}

static int rk_spdifrx_probe(struct platform_device *pdev)
{
	struct rk_spdifrx_dev *spdifrx;
	struct resource *res;
	void __iomem *regs;
	int ret;

	spdifrx = devm_kzalloc(&pdev->dev, sizeof(*spdifrx), GFP_KERNEL);
	if (!spdifrx)
		return -ENOMEM;
	spin_lock_init(&spdifrx->irq_lock);

	spdifrx->reset = devm_reset_control_get(&pdev->dev, "spdifrx-m");
	if (IS_ERR(spdifrx->reset)) {
		ret = PTR_ERR(spdifrx->reset);
		if (ret != -ENOENT)
			return ret;
	}

	spdifrx->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(spdifrx->hclk))
		return PTR_ERR(spdifrx->hclk);

	spdifrx->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(spdifrx->mclk))
		return PTR_ERR(spdifrx->mclk);

	spdifrx->irq = platform_get_irq(pdev, 0);
	if (spdifrx->irq < 0)
		return spdifrx->irq;

	ret = devm_request_threaded_irq(&pdev->dev, spdifrx->irq, NULL,
					rk_spdifrx_isr,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					dev_name(&pdev->dev), spdifrx);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	spdifrx->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
						&rk_spdifrx_regmap_config);
	if (IS_ERR(spdifrx->regmap))
		return PTR_ERR(spdifrx->regmap);

	spdifrx->capture_dma_data.addr = res->start + SPDIFRX_SMPDR;
	spdifrx->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	spdifrx->capture_dma_data.maxburst = 4;

	spdifrx->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, spdifrx);

	ret = devm_snd_soc_register_component(&pdev->dev, &rk_spdifrx_component,
					      &rk_spdifrx_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		goto err;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		goto err;
	}

	return 0;

err:
	return ret;
}

static const struct of_device_id rk_spdifrx_match[] = {
	{
		.compatible = "rockchip,rk3308-spdifrx",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rk_spdifrx_match);

static struct platform_driver rk_spdifrx_driver = {
	.probe = rk_spdifrx_probe,
	.driver = {
		.name = "rockchip-spdifrx",
		.of_match_table = of_match_ptr(rk_spdifrx_match),
	},
};
module_platform_driver(rk_spdifrx_driver);

MODULE_ALIAS("platform:rockchip-spdifrx");
MODULE_DESCRIPTION("Rockchip SPDIF Receiver Interface");
MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_AUTHOR("Yunhao Tian <t123yh.xyz@gmail.com>");
MODULE_LICENSE("GPL v2");
