/*
 * NFC driver for the Goldfish emulator.
 *
 * Copyright (c) 2013 Mozilla Foundation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/poll.h>

#define CDEV_NAME "bcm2079x"
#define CDEV_COUNT 1
#define PDEV_NAME "goldfish_nfc"

#define BCM2079x_NCI_TAG 0x10
#define BCM2079x_HCI_TAG 0x01 /* for device writes */
#define BCM2079x_BT_TAG 0x04 /* for device reads */

#define NFC_MSG_BUFSIZ 385 /* tag + register size */

struct nfc_msg {
        struct list_head list;
        unsigned long len;
        u8 buf[];
};

static struct nfc_msg *
read_nfc_msg(const u8 __iomem *mem, gfp_t flags, unsigned char tag)
{
        struct nfc_msg *msg;
        size_t i;
        unsigned char len;

        msg = kmalloc(sizeof(*msg)+NFC_MSG_BUFSIZ, flags);
        if (!msg)
                return ERR_PTR(ENOMEM);

        INIT_LIST_HEAD(&msg->list);
        msg->len = 1;

        /* add tag for for bcm2079x driver */
        msg->buf[0] = tag;

        for (i = 0; i < 3; ++i, ++msg->len) {
                msg->buf[msg->len] = ioread8(mem+i);
        }

        len = 3 + msg->buf[3]; /* header + payload length */

        for (; i < len; ++i, ++msg->len) {
                BUG_ON(msg->len >= NFC_MSG_BUFSIZ);
                msg->buf[msg->len] = ioread8(mem+i);
        }

        return msg;
}

static void
free_nfc_msg(struct nfc_msg *msg)
{
        kfree(msg);
}

#define REG_SIZE          (NFC_MSG_BUFSIZ-1)
#define REG_OFFSET(_prev) ((_prev) + REG_SIZE)

enum {
        REG_STATUS = 0,
        REG_CTRL   = 1,
        REG_CMND   = 4,
        REG_RESP   = REG_OFFSET(REG_CMND),
        REG_NTFN   = REG_OFFSET(REG_RESP),
        REG_DATA   = REG_OFFSET(REG_NTFN),
        REG_RESERVED2 = REG_OFFSET(REG_DATA)
};

enum {
        STATUS_INTR = 0x01,
        STATUS_NCI_CMND = 0x02,
        STATUS_NCI_RESP = 0x04,
        STATUS_NCI_NTFN = 0x08,
        STATUS_NCI_DATA = 0x10,
        STATUS_HCI_CMND = 0x20,
        STATUS_HCI_RESP = 0x40
};

enum {
        CTRL_INTR_ACK = 0x00,
        CTRL_NCI_CMND_SNT = 0x01,
        CTRL_RESP_RCV = 0x02,
        CTRL_NTFN_RCV = 0x03,
        CTRL_DATA_RCV = 0x04,
        CTRL_HCI_CMND_SNT = 0x05
};

struct goldfish_nfc {
        dev_t dev;
        struct cdev cdev;
        struct class *class;
        struct device *device;

        u8 __iomem *base;

        int irq;

        spinlock_t lock;
        struct list_head msgs; /* protected by lock */
        u8 buf[REG_RESP-REG_CMND]; /* protected by sem */

        struct semaphore sem;
        wait_queue_head_t r_wq;
        wait_queue_head_t w_wq;
        atomic_t rflag;
        atomic_t wflag;
};

static struct goldfish_nfc _nfc[1];

static ssize_t goldfish_nfc_read(struct file *f, char __user *buf,
                                    size_t len, loff_t *off)
{
        struct goldfish_nfc *nfc;
        struct nfc_msg *msg;

        nfc = f->private_data;

        do {
                if (!atomic_read(&nfc->rflag)) {
                        if (f->f_flags&O_NONBLOCK)
                                return -EWOULDBLOCK;
                        if (wait_event_interruptible(nfc->r_wq, atomic_read(&nfc->rflag) != 0))
                                return -ERESTARTSYS;
                }

                spin_lock_bh(&nfc->lock);
                if (!list_empty(&nfc->msgs)) {
                        msg = list_entry(nfc->msgs.next, struct nfc_msg, list);
                        if (len < msg->len) {
                                spin_unlock_bh(&nfc->lock);
                                printk(KERN_ERR "NFC read buffer too small\n");
                                return -EMSGSIZE;
                        }
                        list_del(&msg->list);
                        atomic_dec(&nfc->rflag);
                } else {
                        msg = NULL;
                }
                spin_unlock_bh(&nfc->lock);
        } while (!msg); /* concurrent read calls might have fetched the msg */

        if (copy_to_user(buf, msg->buf, msg->len))
                return -EFAULT;

        len = msg->len;
        free_nfc_msg(msg);

        return len;
}

static ssize_t goldfish_nfc_write(struct file *f, const char __user *buf,
                                  size_t len, loff_t *off)
{
        struct goldfish_nfc *nfc;
        int wflag;
        ssize_t res;

        nfc = f->private_data;

        if (!len || (len > sizeof(nfc->buf)))
                return -ENOMEM; /* for compat with bcm2079x driver; should be -EMSGSIZE */

        do {
                if (!atomic_read(&nfc->wflag)) {
                        if (f->f_flags&O_NONBLOCK)
                                return -EWOULDBLOCK;
                        if (wait_event_interruptible(nfc->w_wq, atomic_read(&nfc->wflag) != 0))
                                return -ERESTARTSYS;
                }

                down(&nfc->sem);

                wflag = atomic_read(&nfc->wflag);

                if (!wflag)
                        up(&nfc->sem);
        } while (!wflag); /* concurrent call wrote first */

        if (copy_from_user(nfc->buf, buf, len)) {
                res = -EFAULT;
                goto out;
        }

        atomic_set(&nfc->wflag, 0); /* no writing until HW acknowledged msg */

        /* Strip tag when writing message data; the HW will see
         * the message type from the control command. Writes to
         * control reg may only happen after command reg.
         */
        memcpy_toio(nfc->base+REG_CMND, nfc->buf+1, len-1);
        barrier();

        if (nfc->buf[0] == BCM2079x_NCI_TAG)
                iowrite8(CTRL_NCI_CMND_SNT, nfc->base+REG_CTRL);
        else
                iowrite8(CTRL_HCI_CMND_SNT, nfc->base+REG_CTRL);

        res = len;
out:
        up(&nfc->sem);
        return res;
}

static unsigned int goldfish_nfc_poll(struct file *f,
                                      struct poll_table_struct *wait)
{
        struct goldfish_nfc *nfc;
        unsigned int mask;

        nfc = f->private_data;

        down(&nfc->sem);

        poll_wait(f, &nfc->r_wq, wait);
        poll_wait(f, &nfc->w_wq, wait);

        mask = 0;

        spin_lock_bh(&nfc->lock);

        if (atomic_read(&nfc->rflag))
                mask |= POLLIN|POLLRDNORM;
        if (atomic_read(&nfc->wflag))
                mask |= POLLOUT|POLLWRNORM;

        spin_unlock_bh(&nfc->lock);

        up(&nfc->sem);

        return mask;
}

static int goldfish_nfc_open(struct inode *inode, struct file *f)
{
        struct goldfish_nfc *nfc = container_of(inode->i_cdev,
                                                struct goldfish_nfc,
                                                cdev);
        f->private_data = nfc;

        return 0;
}

static int goldfish_nfc_release(struct inode *inode, struct file *f)
{
        f->private_data = NULL;

        return 0;
}

static struct file_operations fops = {
        .owner = THIS_MODULE,
        .read = goldfish_nfc_read,
        .write = goldfish_nfc_write,
        .poll = goldfish_nfc_poll,
        .open = goldfish_nfc_open,
        .release = goldfish_nfc_release
};

struct irq_info {
        unsigned int flags;
        unsigned long reg;
        unsigned char tag;
        unsigned char rcv;
};

static void
goldfish_nfc_irq_bh(unsigned long data)
{
        static const struct irq_info irq_info[] = {
                {STATUS_NCI_RESP, REG_RESP, BCM2079x_NCI_TAG, CTRL_RESP_RCV},
                {STATUS_HCI_RESP, REG_RESP, BCM2079x_BT_TAG,  CTRL_RESP_RCV},
                {STATUS_NCI_NTFN, REG_NTFN, BCM2079x_NCI_TAG, CTRL_NTFN_RCV},
                {STATUS_NCI_DATA, REG_DATA, BCM2079x_NCI_TAG, CTRL_DATA_RCV}
        };

        struct goldfish_nfc *nfc;
        int r_wakeup, w_wakeup;
        u8 status;
        const struct irq_info* info;
        struct nfc_msg* msg;

        nfc = _nfc + data;
        r_wakeup = 0;
        w_wakeup = 0;
        status = ioread8(nfc->base+REG_STATUS);

        if ( !(status&(STATUS_NCI_CMND|STATUS_HCI_CMND)) ) {
                atomic_set(&nfc->wflag, 1);
                w_wakeup = 1;
        }

#define ARRAY_END(_a) ((_a) + ARRAY_SIZE(_a))
        for (info = irq_info; info < ARRAY_END(irq_info); ++info) {
                if (!(status&info->flags))
                        continue;

                msg = read_nfc_msg(nfc->base+info->reg, GFP_ATOMIC,
                                   info->tag);
                if (IS_ERR(msg))
                        break;

                spin_lock(&nfc->lock);
                list_add_tail(&msg->list, &nfc->msgs);
                atomic_inc(&nfc->rflag);
                spin_unlock(&nfc->lock);
                iowrite8(info->rcv, nfc->base+REG_CTRL);
                r_wakeup = 1;
        }
#undef ARRAY_END

        /* wake up blocked processes */
        if (w_wakeup)
                wake_up(&nfc->w_wq);
        if (r_wakeup)
                wake_up(&nfc->r_wq);
}

DECLARE_TASKLET(goldfish_nfc_irq, goldfish_nfc_irq_bh, 0);

static irqreturn_t goldfish_nfc_irq_th(int irq, void *data)
{
        struct goldfish_nfc *nfc;
        u8 status;

        nfc = data;
        status = ioread8(nfc->base+REG_STATUS);

        if (!(status&STATUS_INTR))
                return IRQ_NONE;

        /* The HW being an emulated device, we completely control
         * what goes in and out the device. Cannot have more then
         * one pair of command and response messages pending, and
         * we directly control the data and notification messages
         * that the device generates. Hence interupt flooding
         * or missing one is not a concern here.
         */
        iowrite8(CTRL_INTR_ACK, nfc->base+REG_CTRL);

        tasklet_schedule(&goldfish_nfc_irq);

        return IRQ_HANDLED;
}

static int goldfish_nfc_probe(struct platform_device *pdev)
{
        struct goldfish_nfc *nfc;
        struct resource *r;
        int res;

        nfc = &_nfc[0];

        platform_set_drvdata(pdev, nfc);

        spin_lock_init(&nfc->lock);
        INIT_LIST_HEAD(&nfc->msgs);
        sema_init(&nfc->sem, 1);
        init_waitqueue_head(&nfc->r_wq);
        init_waitqueue_head(&nfc->w_wq);
        atomic_set(&nfc->rflag, 0);
        atomic_set(&nfc->wflag, 1);

        r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!r) {
                res = -ENXIO;
                dev_err(&pdev->dev, "platform_get_resource failed\n");
                goto err_platform_get_resource;
        }

        nfc->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
        if (!nfc->base) {
                res = -ENOMEM;
                dev_err(&pdev->dev, "devm_ioremap failed\n");
                goto err_devm_ioremap;
        }

        res = alloc_chrdev_region(&nfc->dev, 0, CDEV_COUNT, CDEV_NAME);
        if (res < 0) {
                dev_err(&pdev->dev,
                        "alloc_chrdev_region failed with %d", res);
                goto err_alloc_chrdev_region;
        }

        cdev_init(&nfc->cdev, &fops);
        nfc->cdev.owner = fops.owner;

        res = cdev_add(&nfc->cdev, nfc->dev, CDEV_COUNT);
        if (res < 0) {
                dev_err(&pdev->dev, "cdev_add failed with %d", res);
                goto err_cdev_add;
        }

        nfc->class = class_create(THIS_MODULE, CDEV_NAME);
        if (IS_ERR(nfc->class)) {
                res = PTR_ERR(nfc->class);
                dev_err(&pdev->dev, "class_create failed with %d", res);
                goto err_class_create;
        }

        nfc->device = device_create(nfc->class, NULL, nfc->dev,
                                    nfc, CDEV_NAME);
        if (IS_ERR(nfc->device)) {
                res = PTR_ERR(nfc->device);
                dev_err(&pdev->dev, "device_create failed with %d", res);
                goto err_device_create;
        }

        /* setup IRQ at the end to not get called during initialization */

        nfc->irq = platform_get_irq(pdev, 0);
        if (nfc->irq < 0) {
                res = nfc->irq;
                dev_err(&pdev->dev, "platform_get_irq failed with %d\n", res);
                goto err_platform_get_irq;
        }

        res = devm_request_irq(&pdev->dev, nfc->irq, goldfish_nfc_irq_th,
                               IRQF_SHARED, pdev->name, nfc);
        if (res < 0) {
                dev_err(&pdev->dev, "devm_request_irq failed with %d", res);
                goto err_devm_request_irq;
        }

        return 0;

err_devm_request_irq:
err_platform_get_irq:
        device_destroy(nfc->class, nfc->dev);
err_device_create:
        class_destroy(nfc->class);
err_class_create:
        cdev_del(&nfc->cdev);
err_cdev_add:
        unregister_chrdev_region(nfc->dev, CDEV_COUNT);
err_alloc_chrdev_region:
        devm_ioremap_release(&pdev->dev, nfc->base);
err_devm_ioremap:
err_platform_get_resource:
        platform_set_drvdata(pdev, NULL);
        return res;
}

static struct platform_driver goldfish_nfc_driver = {
        .probe = goldfish_nfc_probe,
        .driver = {
                .name = PDEV_NAME,
                .owner = THIS_MODULE,
        }
};

static int __init goldfish_nfc_init(void)
{
        return platform_driver_register(&goldfish_nfc_driver);
}

module_init(goldfish_nfc_init);

MODULE_AUTHOR("Thomas Zimmermann");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NFC driver for the Goldfish emulator");
