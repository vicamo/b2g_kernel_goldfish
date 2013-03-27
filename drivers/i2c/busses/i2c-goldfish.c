/*
 * Goldfish I2C bus driver
 *
 * Copyright (C) 2013 You-Sheng Yang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/goldfish.h>
#include <linux/i2c.h>

#ifdef CONFIG_ARM
#include <mach/hardware.h>
#endif

#include <asm/io.h>

#define GOLDFISH_I2C_BUFSIZE 4096

enum {
	GOLDFISH_I2C_REG_XFER = 0x00,
	GOLDFISH_I2C_REG_ADDR = 0x04,
	GOLDFISH_I2C_REG_LEN  = 0x08,
	GOLDFISH_I2C_REG_PTR  = 0x0C,
};

struct goldfish_i2c {
	struct i2c_adapter adap;
	void __iomem *base;
};

#define goldfish_out(adap, reg, data) \
	writel((data), ((struct goldfish_i2c*)(adap))->base + reg)
#define goldfish_in(adap, reg) \
	readl(((struct goldfish_i2c*)(adap))->base + reg)

static int goldfish_i2c_algo_master_xfer(struct i2c_adapter *adap,
					 struct i2c_msg msgs[],
					 int num)
{
	struct goldfish_i2c *priv;
	struct i2c_msg *msg;
	int i, ret;
	unsigned short flags;
	uint32_t addr;

	priv = (struct goldfish_i2c*)adap;
	adap = &priv->adap;

	if (adap->pre_xfer) {
		ret = adap->pre_xfer(adap);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		flags = pmsg->flags;

		if (!(flags & I2C_M_NOSTART)) {
			addr = msg->addr << 1;
			if (flags & I2C_M_RD)
				addr |= 1;
			if (flags & I2C_M_REV_DIR_ADDR)
				addr ^= 1;
			goldfish_out(adap, GOLDFISH_I2C_REG_ADDR, addr);
		}

		if ((flags & I2C_M_RD) && (flags & I2C_M_RECV_LEN)) {
			msg->len += goldfish_in(adap, GOLDFISH_I2C_REG_LEN);
			msg->len--;
		}

		goldfish_out(adap, GOLDFISH_I2C_REG_PTR, (uint32_t)msg->buf);
		goldfish_out(adap, GOLDFISH_I2C_REG_LEN, msg->len);
		goldfish_out(adap, GOLDFISH_I2C_REG_XFER, 0);
	}

	if (adap->post_xfer)
		adap->post_xfer(adap);
	return ret;
}

static u32 goldfish_i2c_algo_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C |
	       I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
	       I2C_FUNC_10BIT_ADDR |
	       I2C_FUNC_PROTOCOL_MANGLING;
}

static const struct i2c_algorithm goldfish_i2c_algo = {
	.master_xfer	= goldfish_i2c_algo_master_xfer,
	.functionality	= goldfish_i2c_algo_functionality,
};

static int __devinit goldfish_i2c_probe(struct platform_device *pdev)
{
	struct goldfish_i2c *priv;
	struct resource *r;
	struct i2c_adapter *adap;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -EINVAL;
		goto err_get_resources;
	}
#ifdef CONFIG_ARM
	priv->base = (void __iomem *)IO_ADDRESS(r->start - IO_START);
#else
	priv->base = ioremap(r->start, 0x1000);
#endif

	adap = &priv->adap;
	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-goldfish%d", pdev->id);
	adap->algo = &goldfish_i2c_algo;
	adap->algo_data = priv;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_add_numbered_adapter(adap);
	if (!ret)
		return 0;

err_get_resources:
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return ret;
}

static int __devexit goldfish_i2c_remove(struct platform_device *pdev)
{
	struct goldfish_i2c *priv;

	priv = platform_get_drvdata(pdev);

	i2c_del_adapter(&priv->adap);

	kfree(priv);
	return 0;
}

static struct platform_driver goldfish_i2c_driver = {
	.probe		= goldfish_i2c_probe,
	.remove		= __devexit_p(goldfish_i2c_remove),
	.driver		= {
		.name	= "goldfish_i2c",
		.owner	= THIS_MODULE,
	},
};

static int __init goldfish_i2c_init(void)
{
	int ret;

	ret = platform_driver_register(&goldfish_i2c_driver);
	if (ret)
		printk(KERN_ERR "goldfish_i2c: probe failed: %d\n", ret);

	return ret;
}
subsys_initcall(goldfish_i2c_init);

static void __exit goldfish_i2c_exit(void)
{
	platform_driver_unregister(&goldfish_i2c_driver);
}
module_exit(goldfish_i2c_exit);

MODULE_AUTHOR("You-Sheng Yang");
MODULE_DESCRIPTION("Goldfish I2C driver");
MODULE_LICENSE("GPL");
