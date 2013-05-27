/*
 * RFKILL driver for the Goldfish emulator.
 *
 * Copyright (c) 2013 Mozilla Foundation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/rfkill.h>
#include <linux/io.h>

#define PDEV_NAME		"goldfish_rfkill"

#ifdef CONFIG_BT
# define RFKILL_NAME_BLUETOOTH	"goldfish-bluetooth"
#endif

enum {
	OFFSET_INT_MASK  = 0x00, /* W, 32 bits.   */
	OFFSET_INT       = 0x04, /* R/W, 32 bits. */
	OFFSET_HW_BLOCK  = 0x08, /* R, 32 bits.   */
	OFFSET_BLOCK     = 0x0C  /* W, 32 bits.   */
};

enum {
	STATUS_ENABLE_INTERRUPT = 0x01,
};

struct goldfish_rfkill_priv;
struct rfkill_data {
	struct goldfish_rfkill_priv *priv;
	struct rfkill *rfkill;
	enum rfkill_type type;
};

struct goldfish_rfkill_priv {
	void __iomem *base;
	u32 irq;

	spinlock_t irq_lock;
	unsigned long status; /* Guarded by irq_lock. */
	struct tasklet_struct irq_tasklet;

	struct rfkill_data data[RFKILL_TYPE_MAX];

	u32 sw_block;
	u32 hw_block;
	u32 hw_mask;
};

#define goldfish_rfkill_read32(priv, ofs) \
		ioread32((priv)->base + (ofs))
#define goldfish_rfkill_write32(priv, v, ofs) \
		iowrite32((v), (priv)->base + (ofs))

#define goldfish_rfkill_read_irq(priv) \
		goldfish_rfkill_read32((priv), OFFSET_INT)
#define goldfish_rfkill_ack_irq(priv, v) \
		goldfish_rfkill_write32((priv), (v), OFFSET_INT)
#define goldfish_rfkill_disable_irq(priv) \
		goldfish_rfkill_write32((priv), 0x00000000, OFFSET_INT_MASK)
#define goldfish_rfkill_enable_irq(priv) \
		goldfish_rfkill_write32((priv), (priv)->hw_mask, OFFSET_INT_MASK)

#define goldfish_rfkill_read_hw_block(priv) \
		goldfish_rfkill_read32((priv), OFFSET_HW_BLOCK)
#define goldfish_rfkill_write_block(priv, v) \
		goldfish_rfkill_write32((priv), (v), OFFSET_BLOCK)

static irqreturn_t goldfish_rfkill_isr(int irq, void *data)
{
	struct goldfish_rfkill_priv *priv = data;
	u32 inta;

	spin_lock(&priv->irq_lock);

	/* Disable (but don't clear!) interrupts here to avoid back-to-back
	 *   ISRs and sporadic interrupts.
	 * If we have something to service, the tasklet will re-enable ints.
	 * If we *don't* have something, we'll re-enable before leaving here.
	 */
	goldfish_rfkill_disable_irq(priv);

	/* Discover which interrupts are active/pending */
	inta = goldfish_rfkill_read_irq(priv);

	/* Ignore interrupt if there's nothing to service. */
	if (!inta) {
		/* Re-enable interrupt again if necessary. */
		if (test_bit(STATUS_ENABLE_INTERRUPT, &priv->status))
			goldfish_rfkill_enable_irq(priv);

		spin_unlock(&priv->irq_lock);
		return IRQ_NONE;
	}

	tasklet_schedule(&priv->irq_tasklet);

	spin_unlock(&priv->irq_lock);
	return IRQ_HANDLED;
}

static void goldfish_rfkill_irq_tasklet(struct goldfish_rfkill_priv *priv)
{
	struct rfkill *rfkill;
	enum rfkill_state state;
	u32 inta, hw_block, diff, mask, ii;

	/* IRQ must have been disabled in goldfish_rfkill_isr(). */
	spin_lock(&priv->irq_lock);

	/* Discover which interrupts are active/pending */
	inta = goldfish_rfkill_read_irq(priv);
	goldfish_rfkill_ack_irq(priv, inta);

	/* Leave only those we're interested in. */
	hw_block = goldfish_rfkill_read_hw_block(priv);
	hw_block &= priv->hw_mask;
	if (priv->hw_block == hw_block)
		goto tasklet_done;

	diff = (~priv->hw_block & hw_block) | (priv->hw_block & ~hw_block);
	priv->hw_block = hw_block;

	ii = ARRAY_SIZE(priv->data);
	while (ii > 0) {
		--ii;
		mask = 0x01 << ii;

		if (!(diff & mask))
			continue;

		if (priv->hw_block & mask)
			state = RFKILL_STATE_HARD_BLOCKED;
		else if (priv->sw_block & mask)
			state = RFKILL_STATE_SOFT_BLOCKED;
		else
			state = RFKILL_STATE_UNBLOCKED;

		rfkill_force_state(priv->data[ii].rfkill, state);
	}

tasklet_done:
	/* Re-enable interrupt again if necessary. */
	if (test_bit(STATUS_ENABLE_INTERRUPT, &priv->status))
		goldfish_rfkill_enable_irq(priv);

	spin_unlock(&priv->irq_lock);
}

static int goldfish_rfkill_toggle_radio(void *d, enum rfkill_state state)
{
	struct rfkill_data *data = (struct rfkill_data*)d;
	struct goldfish_rfkill_priv *priv = data->priv;
	unsigned long irq_flags;
	int ret = 0;
	u32 mask;

	spin_lock_irqsave(&priv->irq_lock, irq_flags);

	mask = (0x01UL << data->type);
	if (state == RFKILL_STATE_UNBLOCKED) {
		if (priv->hw_block & mask) {
			ret = -EINVAL;
			goto toggle_radio_done;
		}
		priv->sw_block &= ~mask;
	} else {
		priv->sw_block |= mask;
	}

	goldfish_rfkill_write_block(priv, priv->hw_block | priv->sw_block);

toggle_radio_done:
	spin_unlock_irqrestore(&priv->irq_lock, irq_flags);
	return ret;
}

#if defined(CONFIG_BT)
static int goldfish_rfkill_init_rfkill(struct goldfish_rfkill_priv *priv,
		struct device *parent, enum rfkill_type type, const char *name)
{
	struct rfkill_data *data;
	struct rfkill *rfkill;
	u32 mask;
	int ret;

	rfkill = rfkill_allocate(parent, type);
	if (!rfkill)
		return -ENOMEM;

	mask = 0x01UL << type;
	data = &priv->data[type];

	rfkill->name = name;
	rfkill->state = priv->hw_block & mask ? RFKILL_STATE_HARD_BLOCKED
						: RFKILL_STATE_UNBLOCKED;
	rfkill->toggle_radio = goldfish_rfkill_toggle_radio;
	rfkill->data = data;

	ret = rfkill_register(rfkill);
	if (ret) {
		rfkill_free(rfkill);
		return ret;
	}

	data->priv = priv;
	data->rfkill = rfkill;
	data->type = type;
	priv->hw_mask |= mask;

	return 0;
}
#endif /* CONFIG_BT */

static int goldfish_rfkill_probe(struct platform_device *pdev)
{
	struct goldfish_rfkill_priv *priv;
	struct resource *r;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	spin_lock_init(&priv->irq_lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		return -ENXIO;
	}

	priv->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (priv->base == NULL) {
		dev_err(&pdev->dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		return priv->irq;
	}

	ret = devm_request_irq(&pdev->dev, priv->irq, goldfish_rfkill_isr,
				IRQF_SHARED, pdev->name, priv);
	if (ret)
		return ret;

	priv->hw_block = goldfish_rfkill_read_hw_block(priv);
	priv->sw_block = 0;

	tasklet_init(&priv->irq_tasklet, (void (*)(unsigned long))
			goldfish_rfkill_irq_tasklet, (unsigned long)priv);

	platform_set_drvdata(pdev, priv);


#ifdef CONFIG_BT
	goldfish_rfkill_init_rfkill(priv, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
					RFKILL_NAME_BLUETOOTH);
#endif
	if (priv->hw_mask) {
		/* All done! Enable interrupt. */
		set_bit(STATUS_ENABLE_INTERRUPT, &priv->status);
		goldfish_rfkill_enable_irq(priv);
	}

	return 0;
}

static int __devexit goldfish_rfkill_remove(struct platform_device *pdev)
{
	struct goldfish_rfkill_priv *priv = platform_get_drvdata(pdev);
	struct rfkill *rfkill;
	unsigned long irq_flags, ii;

	if (!priv)
		return 0;

	/* Disable interrupt forever. */
	spin_lock_irqsave(&priv->irq_lock, irq_flags);
	clear_bit(STATUS_ENABLE_INTERRUPT, &priv->status);
	goldfish_rfkill_disable_irq(priv);
	spin_unlock_irqrestore(&priv->irq_lock, irq_flags);

	synchronize_irq(priv->irq);
	tasklet_kill(&priv->irq_tasklet);

	ii = ARRAY_SIZE(priv->data);
	while (ii > 0) {
		--ii;

		rfkill = priv->data[ii].rfkill;
		if (!rfkill)
			continue;

		rfkill_unregister(rfkill);
		priv->data[ii].priv = NULL;
		priv->data[ii].rfkill = NULL;
	}

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver goldfish_rfkill_driver = {
	.probe = goldfish_rfkill_probe,
	.remove = __devexit_p(goldfish_rfkill_remove),

	.driver = {
		.name = PDEV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init goldfish_rfkill_init(void)
{
	return platform_driver_register(&goldfish_rfkill_driver);
}

static void __exit goldfish_rfkill_exit(void)
{
	platform_driver_unregister(&goldfish_rfkill_driver);
}

module_init(goldfish_rfkill_init);
module_exit(goldfish_rfkill_exit);

MODULE_AUTHOR("You-Sheng Yang vicamo@gmail.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RFKILL driver for the Goldfish emulator");
