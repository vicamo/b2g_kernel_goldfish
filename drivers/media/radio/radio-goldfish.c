/*
 * FM radio driver for the Goldfish emulator.
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
#include <linux/io.h>
#include <asm/atomic.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#define PDEV_NAME	"goldfish_fm"
#define DRIVER_NAME	"goldfish-fm"
#define CARD_NAME	"Goldfish FM Radio"

enum {
	OFFSET_VERSION                 = 0x00, /* R,   32 bits. */
	OFFSET_N_TUNERS                = 0x04, /* R,   32 bits. */
	OFFSET_STATE                   = 0x08, /* W,   32 bits. */

	OFFSET_TUNER_BASE              = 0x100,
	OFFSET_TUNER_CAPABILITY        = 0x00, /* R,   32 bits. */
	OFFSET_TUNER_RANGELOW          = 0x04, /* R/W, 32 bits. */
	OFFSET_TUNER_RANGEHIGH         = 0x08, /* R/W, 32 bits. */
	OFFSET_TUNER_RXSUBCHANS        = 0x0C, /* R,   32 bits. */
	OFFSET_TUNER_AUDMODE           = 0x10, /* R/W, 32 bits. */
	OFFSET_TUNER_SIGNAL            = 0x14, /* R,   32 bits. */
	OFFSET_TUNER_AFC               = 0x18, /* R/W, 32 bits. */
	OFFSET_TUNER_FREQUENCY         = 0X1C, /* R/W, 32 bits. */
	OFFSET_TUNER_HW_SEEK           = 0x20, /* W,   32 bits. */
};

struct goldfish_fm_priv {
	struct platform_device *pdev;
	void __iomem *base;
	u32 version;
	u32 n_tuners;

	struct video_device *videodev;
	atomic_t users;
};

#define goldfish_fm_read32(priv, ofs) \
		ioread32((priv)->base + (ofs))
#define goldfish_fm_write32(priv, v, ofs) \
		iowrite32((v), (priv)->base + (ofs))

#define tuner_read32(priv, n, ofs) \
		goldfish_fm_read32(priv, OFFSET_TUNER_BASE + (n << 8) + (ofs))
#define tuner_write32(priv, n, v, ofs) \
		goldfish_fm_write32(priv, (v), OFFSET_TUNER_BASE + (n << 8) + (ofs))

static int goldfish_fm_v4l2_fops_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);

	if (!atomic_dec_and_test(&priv->users)) {
		atomic_inc(&priv->users);
		return -EBUSY;
	}

	goldfish_fm_write32(priv, 0x01, OFFSET_STATE);

	return 0;
}

static int goldfish_fm_v4l2_fops_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);

	goldfish_fm_write32(priv, 0x00, OFFSET_STATE);

	atomic_inc(&priv->users);
	return 0;
}

static const struct v4l2_file_operations goldfish_fm_v4l2_fops = {
	.owner = THIS_MODULE,
	.ioctl = video_ioctl2,
	.open = goldfish_fm_v4l2_fops_open,
	.release = goldfish_fm_v4l2_fops_release,
};

static int goldfish_fm_vidioc_querycap(struct file *file, void *p,
					struct v4l2_capability *capability)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);

	strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strlcpy(capability->card, CARD_NAME, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		"pdev_bus:%d", priv->pdev->id);
	capability->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	capability->version = priv->version;

	return 0;
}

static int goldfish_fm_vidioc_g_tuner(struct file *file, void *p,
					struct v4l2_tuner *tuner)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);
	u32 index = tuner->index;

	if (index >= priv->n_tuners)
		return -EINVAL;

	snprintf(tuner->name, sizeof(tuner->name),
		"Goldfish FM Tuner - %d", index);
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = tuner_read32(priv, index, OFFSET_TUNER_CAPABILITY);
	tuner->rangelow = tuner_read32(priv, index, OFFSET_TUNER_RANGELOW);
	tuner->rangehigh = tuner_read32(priv, index, OFFSET_TUNER_RANGEHIGH);
	tuner->rxsubchans = tuner_read32(priv, index, OFFSET_TUNER_RXSUBCHANS);
	tuner->audmode = tuner_read32(priv, index, OFFSET_TUNER_AUDMODE);
	tuner->signal = tuner_read32(priv, index, OFFSET_TUNER_SIGNAL);
	tuner->afc = tuner_read32(priv, index, OFFSET_TUNER_AFC);

	return 0;
}

static int goldfish_fm_vidioc_s_tuner(struct file *file, void *p,
					struct v4l2_tuner *tuner)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);
	u32 index = tuner->index;

	if (index >= priv->n_tuners)
		return -EINVAL;

	tuner_write32(priv, index, tuner->rangelow, OFFSET_TUNER_RANGELOW);
	tuner_write32(priv, index, tuner->rangehigh, OFFSET_TUNER_RANGEHIGH);
	tuner_write32(priv, index, tuner->audmode, OFFSET_TUNER_AUDMODE);
	tuner_write32(priv, index, tuner->afc, OFFSET_TUNER_AFC);

	return 0;
}

static int goldfish_fm_vidioc_g_frequency(struct file *file, void *p,
					struct v4l2_frequency *freq)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);
	u32 tuner = freq->tuner;

	if (tuner >= priv->n_tuners)
		return -EINVAL;

	freq->type = V4L2_TUNER_RADIO;
	freq->frequency = tuner_read32(priv, tuner, OFFSET_TUNER_FREQUENCY);

	return 0;
}

static int goldfish_fm_vidioc_s_frequency(struct file *file, void *p,
					struct v4l2_frequency *freq)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);
	u32 tuner = freq->tuner;

	if (tuner >= priv->n_tuners)
		return -EINVAL;

	if (freq->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	tuner_write32(priv, tuner, freq->frequency, OFFSET_TUNER_FREQUENCY);

	return 0;
}

static int goldfish_fm_vidioc_s_hw_freq_seek(struct file *file, void *p,
					struct v4l2_hw_freq_seek *seek)
{
	struct video_device *vdev = video_devdata(file);
	struct goldfish_fm_priv *priv = video_get_drvdata(vdev);
	u32 tuner = seek->tuner;
	u32 dir_wrap = 0;

	if (tuner >= priv->n_tuners)
		return -EINVAL;

	if (seek->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	if (seek->seek_upward)
		dir_wrap |= 0x01;

	if (seek->wrap_around)
		dir_wrap |= 0x02;

	tuner_write32(priv, tuner, dir_wrap, OFFSET_TUNER_HW_SEEK);

	return 0;
}

static const struct v4l2_ioctl_ops goldfish_fm_v4l2_ioctl_ops = {
	.vidioc_querycap = goldfish_fm_vidioc_querycap,
	.vidioc_g_tuner = goldfish_fm_vidioc_g_tuner,
	.vidioc_s_tuner = goldfish_fm_vidioc_s_tuner,
	.vidioc_g_frequency = goldfish_fm_vidioc_g_frequency,
	.vidioc_s_frequency = goldfish_fm_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek = goldfish_fm_vidioc_s_hw_freq_seek,
};

static int goldfish_fm_probe(struct platform_device *pdev)
{
	struct goldfish_fm_priv *priv;
	struct resource *r;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->pdev = pdev;

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

	priv->videodev = video_device_alloc();
	if (priv->videodev == NULL) {
		dev_err(&pdev->dev, "allocate video_device failed\n");
		return -ENOMEM;
	}

	snprintf(priv->videodev->name, sizeof(priv->videodev->name), DRIVER_NAME);
	priv->videodev->fops = &goldfish_fm_v4l2_fops;
	priv->videodev->ioctl_ops = &goldfish_fm_v4l2_ioctl_ops;
	priv->videodev->release = video_device_release;

	atomic_set(&priv->users, 1);

	priv->version = goldfish_fm_read32(priv, OFFSET_VERSION);
	priv->n_tuners = goldfish_fm_read32(priv, OFFSET_N_TUNERS);

	video_set_drvdata(priv->videodev, priv);

	ret = video_register_device(priv->videodev, VFL_TYPE_RADIO, pdev->id);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME
			": Could not register video device\n");
		goto probe_err_release_video;
	}

	return 0;

probe_err_release_video:
	video_device_release(priv->videodev);
	return ret;
}

static int __devexit goldfish_fm_remove(struct platform_device *pdev)
{
	struct goldfish_fm_priv *priv = platform_get_drvdata(pdev);

	video_unregister_device(priv->videodev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver goldfish_fm_driver = {
	.probe = goldfish_fm_probe,
	.remove = __devexit_p(goldfish_fm_remove),

	.driver = {
		.name = PDEV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init goldfish_fm_init(void)
{
	return platform_driver_register(&goldfish_fm_driver);
}

static void __exit goldfish_fm_exit(void)
{
	platform_driver_unregister(&goldfish_fm_driver);
}

module_init(goldfish_fm_init);
module_exit(goldfish_fm_exit);

MODULE_AUTHOR("You-Sheng Yang vicamo@gmail.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FM radio driver for the Goldfish emulator");
