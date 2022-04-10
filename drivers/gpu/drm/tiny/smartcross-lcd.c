// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for SmartCross LCD
 *
 * Copyright 2022 Yunhao Tian <t123yh.xyz@gmail.com>
 *
 * Based on smartcross_lcd.c:
 * Copyright 2017 David Lechner <david@lechnology.com>
 * Copyright (C) 2019 Glider bvba
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>

struct smartcross_lcd_priv {
	struct mipi_dbi_dev dbidev; /* Must be first for .release() */
};

static void smartcross_lcd_pipe_enable(struct drm_simple_display_pipe *pipe,
				       struct drm_crtc_state *crtc_state,
				       struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int idx;
	u8 addr_mode;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	switch (dbidev->rotation) {
	default:
		addr_mode = 0;
		dbidev->left_offset = 34;
		dbidev->top_offset = 0;
		break;
	case 180:
		addr_mode = 0xC0;
		dbidev->left_offset = 34;
		dbidev->top_offset = 0;
		break;
	case 90:
		addr_mode = 0x70;
		dbidev->left_offset = 0;
		dbidev->top_offset = 34;
		break;
	case 270:
		addr_mode = 0xA0;
		dbidev->left_offset = 0;
		dbidev->top_offset = 34;
		break;
	}

	mipi_dbi_command(dbi, 0x01);
	msleep(120);
	mipi_dbi_command(dbi, 0x21);
	mipi_dbi_command(dbi, 0x3A);
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x05);
	mipi_dbi_command(dbi, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33);
	mipi_dbi_command(dbi, 0xB7, 0x35);
	mipi_dbi_command(dbi, 0xBB, 0x35);
	mipi_dbi_command(dbi, 0xC0, 0x2C);
	mipi_dbi_command(dbi, 0xC2, 0x01);
	mipi_dbi_command(dbi, 0xC3, 0x13);
	mipi_dbi_command(dbi, 0xC4, 0x20);
	mipi_dbi_command(dbi, 0xC6, 0x0F);
	mipi_dbi_command(dbi, 0xD0, 0xA4, 0xA1);
	mipi_dbi_command(dbi, 0xD6, 0xA1);
	mipi_dbi_command(dbi, 0xE0, 0xF0, 0x00, 0x04, 0x04, 0x04, 0x05, 0x29,
			 0x33, 0x3E, 0x38, 0x12, 0x12, 0x28, 0x30);
	mipi_dbi_command(dbi, 0xE1, 0xF0, 0x07, 0x0A, 0x0D, 0x0B, 0x07, 0x28,
			 0x33, 0x3E, 0x36, 0x14, 0x14, 0x29, 0x32);
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(5);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs smartcross_lcd_pipe_funcs = {
	.enable = smartcross_lcd_pipe_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
};

DEFINE_DRM_GEM_CMA_FOPS(smartcross_lcd_fops);

static const struct drm_driver smartcross_lcd_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &smartcross_lcd_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init = mipi_dbi_debugfs_init,
	.name = "smartcross_lcd",
	.desc = "SmartCross Front Panel LCD Module",
	.date = "20220402",
	.major = 1,
	.minor = 0,
};

static const struct of_device_id smartcross_lcd_of_match[] = {
	{ .compatible = "smartcross,spi-lcd" },
	{},
};
MODULE_DEVICE_TABLE(of, smartcross_lcd_of_match);

static const struct spi_device_id smartcross_lcd_id[] = {
	{ "smartcross,spi-lcd" },
	{},
};
MODULE_DEVICE_TABLE(spi, smartcross_lcd_id);

static int smartcross_lcd_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct smartcross_lcd_priv *priv;
	struct drm_display_mode mode = { DRM_SIMPLE_MODE(172, 320, 19, 36) };
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	priv = devm_drm_dev_alloc(dev, &smartcross_lcd_driver,
				  struct smartcross_lcd_priv, dbidev.drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	dbidev = &priv->dbidev;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc),
				     "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	spi->mode = SPI_MODE_3;
	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init(dbidev, &smartcross_lcd_pipe_funcs, &mode,
				rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int smartcross_lcd_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void smartcross_lcd_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver smartcross_lcd_spi_driver = {
	.driver = {
		.name = "smartcross_lcd",
		.of_match_table = smartcross_lcd_of_match,
	},
	.id_table = smartcross_lcd_id,
	.probe = smartcross_lcd_probe,
	.remove = smartcross_lcd_remove,
	.shutdown = smartcross_lcd_shutdown,
};
module_spi_driver(smartcross_lcd_spi_driver);

MODULE_DESCRIPTION("SmartCross LCD DRM driver");
MODULE_AUTHOR("Yunhao Tian <t123yh.xyz@gmail.com>");
MODULE_LICENSE("GPL");
