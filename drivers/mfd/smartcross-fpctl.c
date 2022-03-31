// SPDX-License-Identifier: GPL-2.0-only
/* Front Panel Controller Driver for the SmartCross Project
 * 
 * Author: Yunhao Tian <t123yh.xyz@gmail.com>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <media/rc-core.h>

#define DRIVER_NAME "smartcross-fpctl"

struct smartcross_fpctl_priv {
	struct regmap *regmap;
	struct gpio_desc *irq;
    struct device *dev;
	struct rc_dev *rc_dev;
};

#define FPCTL_REG_INT_ENABLE 0
#define FPCTL_REG_INT_ACK 1
#define FPCTL_REG_ID 32
#define FPCTL_REG_INT_STATUS 33
#define FPCTL_REG_IR_RX0 34
#define FPCTL_REG_IR_RX1 35
#define FPCTL_REG_IR_RX2 36
#define FPCTL_REG_IR_RX3 37

#define FPCTL_INT_IR_RECEIVED_SHIFT 0
#define FPCTL_INT_IR_RECEIVED (1 << FPCTL_INT_IR_RECEIVED_SHIFT)
#define FPCTL_INT_IR_REPEAT_SHIFT 1
#define FPCTL_INT_IR_REPEAT (1 << FPCTL_INT_IR_REPEAT_SHIFT)
 
static inline bool is_rev(uint8_t a, uint8_t b) {
	return a == (uint8_t)~b;
}

static irqreturn_t smartcross_fpctl_irq(int irq_no, void *handle) {
	u32 irq = 0;
	uint8_t ir_data[4];
	struct smartcross_fpctl_priv* priv_data = (struct smartcross_fpctl_priv*)handle;

	regmap_read(priv_data->regmap, FPCTL_REG_INT_STATUS, &irq);
    if (irq & FPCTL_INT_IR_RECEIVED) {
        dev_dbg(priv_data->dev, "IR Recv\n");
		regmap_bulk_read(priv_data->regmap, FPCTL_REG_IR_RX0, ir_data, sizeof(ir_data));
		if (is_rev(ir_data[0], ir_data[1]) && is_rev(ir_data[2], ir_data[3])) {
			// NEC Protocol
			rc_keydown(priv_data->rc_dev, RC_PROTO_NEC, RC_SCANCODE_NEC(ir_data[0], ir_data[2]), 0);
		} else if (is_rev(ir_data[2], ir_data[3])) {
			// NECX Protocol
			rc_keydown(priv_data->rc_dev, RC_PROTO_NECX, RC_SCANCODE_NECX(ir_data[0] << 8 | ir_data[1], ir_data[2]), 0);
		}
    }
    if (irq & FPCTL_INT_IR_REPEAT) {
        dev_dbg(priv_data->dev, "IR Repeat\n");
		rc_repeat(priv_data->rc_dev);
    }

    regmap_write(priv_data->regmap, FPCTL_REG_INT_ACK, irq);

	return IRQ_HANDLED;
}

static bool smartcross_fpctl_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... 1:
    case 32 ... 37:
		return true;
	default:
		return false;
	}
}

static bool smartcross_fpctl_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... 1:
		return true;
	default:
		return false;
	}
}

static bool smartcross_fpctl_volatile_register(struct device* dev, unsigned int reg)
{
	switch (reg) {
	case 33 ... 37:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config smartcross_fpctl_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,

	.max_register		= FPCTL_REG_IR_RX3,
	.readable_reg		= smartcross_fpctl_readable_register,
	.writeable_reg		= smartcross_fpctl_writeable_register,
	.volatile_reg       = smartcross_fpctl_volatile_register,
	.cache_type			=  REGCACHE_RBTREE,
};

static int smartcross_fpctl_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct smartcross_fpctl_priv *priv_data;
	int ret = 0, val;

	priv_data = devm_kzalloc(&client->dev, sizeof(struct smartcross_fpctl_priv), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;
	i2c_set_clientdata(client, priv_data);
    priv_data->dev = &client->dev;

	priv_data->irq = devm_gpiod_get(&client->dev, "irq", GPIOD_IN);
	if (IS_ERR(priv_data->irq)) {
		ret = PTR_ERR(priv_data->irq);
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get IRQ: %d\n", ret);
		return ret;
	}

	priv_data->regmap = devm_regmap_init_i2c(client, &smartcross_fpctl_regmap);
	if (IS_ERR(priv_data->regmap)) {
		ret = PTR_ERR(priv_data->regmap);
		dev_err(&client->dev, "regmap_init() failed: %d\n", ret);
		return ret;
	}

    ret = regmap_read(priv_data->regmap, FPCTL_REG_ID, &val);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read fpctl ID register: %d\n", ret);
        return ret;
    }

    if (val != 0xAB) {
        dev_err(&client->dev, "Invalid fpctl device ID: %02X\n", val);
        return -EINVAL;
    }

	priv_data->rc_dev = devm_rc_allocate_device(&client->dev, RC_DRIVER_SCANCODE);
	if (IS_ERR(priv_data->rc_dev)) {
		ret = PTR_ERR(priv_data->rc_dev);
		dev_err(&client->dev, "Failed to allocate rc_dev %d\n", ret);
		return ret;
	}

	priv_data->rc_dev->map_name = RC_MAP_EMPTY;
	priv_data->rc_dev->allowed_protocols = RC_PROTO_BIT_NECX | RC_PROTO_BIT_NEC;
	priv_data->rc_dev->input_phys = DRIVER_NAME "/input-rc";
	priv_data->rc_dev->driver_name = DRIVER_NAME;
	priv_data->rc_dev->device_name = dev_name(&client->dev);
	priv_data->rc_dev->input_id.bustype = BUS_I2C;
	priv_data->rc_dev->input_id.version = 1;

	ret = devm_rc_register_device(&client->dev, priv_data->rc_dev);
	if (ret < 0) {
		dev_err(&client->dev, "1Failed to register RC device: %d\n", ret);
        return -EINVAL;
	}

    regmap_write(priv_data->regmap, FPCTL_REG_INT_ACK, 0xFF);
    regmap_write(priv_data->regmap, FPCTL_REG_INT_ENABLE, FPCTL_INT_IR_RECEIVED | FPCTL_INT_IR_REPEAT);

	ret = devm_request_threaded_irq(&client->dev, gpiod_to_irq(priv_data->irq),
					NULL, smartcross_fpctl_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"smartcross-fpctl-irq", priv_data);				
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request irq: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "SmartCross front panel controller initialized\n");

	return 0;
}

static const struct of_device_id smartcross_fpctl_of_match[] = {
	{ .compatible = "smartcross,fpctl-v1", },
	{},
};

MODULE_DEVICE_TABLE(of, smartcross_fpctl_of_match);

static const struct i2c_device_id smartcross_fpctl_i2c_id[] = {
	{"smartcross-fpctl", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, smartcross_fpctl_i2c_id);

static struct i2c_driver smartcross_fpctl_i2c_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table	= smartcross_fpctl_of_match,
	},
	.id_table	= smartcross_fpctl_i2c_id,
	.probe		= smartcross_fpctl_i2c_probe,
};

module_i2c_driver(smartcross_fpctl_i2c_driver);

MODULE_AUTHOR("Yunhao Tian <t123yh.xyz@gmail.com>");
MODULE_DESCRIPTION("SmartCross Front Panel Controller Driver");
MODULE_LICENSE("GPL");