/*
 * Copyright (C) 2012 Dmytro Milinevskyy
 *
 * Kernel USB-FIFO device module.
 *
 * Author: Dmytro Milinevskyy <milinevskyy@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/circ_buf.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>

#define DEBUG 1

#ifdef DEBUG
static unsigned int debug_level = 1;
module_param(debug_level, uint, S_IRUGO|S_IWUSR);
#define DBG(level, kern_level, fmt, ...)                            \
    do {                                                            \
        if (level <= debug_level) {                                 \
            printk(kern_level "usbfifo[%s:%u]: " fmt,               \
                    __func__, __LINE__,                             \
                    ## __VA_ARGS__);                                \
        }                                                           \
    } while (0)
#else
#define DBG(...)
#endif

static unsigned int major = 0;
module_param(major, uint, 0);

static unsigned int cbuffer_size = PAGE_SIZE;
module_param(cbuffer_size, uint, 0);

struct usbfifo_ep {
    int in_pipe, out_pipe;
    size_t in_max_size, out_max_size;
};

struct usbfifo_cdev {
    dev_t dev;
    struct cdev *cdev;
};

struct usbfifo_cbuffer {
    struct circ_buf *cbuffer;
    struct mutex mutex;
};

struct usbfifo_urb {
    struct urb *urb;
    char *buffer;
    size_t buffer_max_size;
};

struct usbfifo {
    struct usbfifo_cdev cmd_cdev;
    struct usbfifo_cdev data_cdev;

    struct usb_device *udev;
    struct usb_interface *intf;
    struct usb_driver *driver;

    struct usbfifo_ep cmd_ep;
    struct usbfifo_ep data_ep;

    struct usbfifo_cbuffer cbuffer_data_in;
    struct usbfifo_cbuffer cbuffer_data_out;

    struct usbfifo_urb urb_cmd_in;
    struct usbfifo_urb urb_data_in;
    struct usbfifo_urb urb_data_out;

    struct work_struct urb_cmd_in_work;
    struct work_struct urb_data_in_work;
    struct work_struct urb_data_out_work;

    char *cmd_in_buffer;
    size_t cmd_in_buffer_data_size;
    struct mutex cmd_in_buffer_mutex;

    size_t urb_data_out_buffer_sent;
    atomic_t urb_data_out_in_progress; // FIXME: likely overkill to use atomic as cbuffer mutex is exploited
};

/* FIXME: USB disconnect may race with file operations */

struct usbfifo_devlist {
    struct list_head list;
    struct usbfifo *usbfifo;
};
static DEFINE_SPINLOCK(usbfifo_devlist_spinlock);
static LIST_HEAD(usbfifo_devlist);

#define CIRC_SIZE               cbuffer_size
#define CIRC_MASK               (CIRC_SIZE - 1)
#define circ_empty(circ)        ((circ)->head == (circ)->tail)
#define circ_free(circ)         CIRC_SPACE((circ)->head, (circ)->tail, CIRC_SIZE)
#define circ_cnt(circ)          CIRC_CNT((circ)->head, (circ)->tail, CIRC_SIZE)
#define circ_byte(circ, idx)    ((circ)->buf[(idx) & CIRC_MASK])

static int cmd_rx_submit(struct usbfifo *usbfifo);
static void cmd_rx_complete(struct urb *urb);

static void usbfifo_urb_cmd_in_work(struct work_struct *work)
{
    struct usbfifo *usbfifo = container_of(work, struct usbfifo, urb_cmd_in_work);
    struct urb *urb = usbfifo->urb_cmd_in.urb;

    if (urb->status == 0) {
        mutex_lock(&usbfifo->cmd_in_buffer_mutex);
        memcpy(usbfifo->cmd_in_buffer, usbfifo->urb_cmd_in.buffer, urb->actual_length);
        usbfifo->cmd_in_buffer_data_size = urb->actual_length;
        mutex_unlock(&usbfifo->cmd_in_buffer_mutex);
    }

    cmd_rx_submit(usbfifo);
}

static int cmd_rx_submit(struct usbfifo *usbfifo)
{
    int res;

    usb_fill_bulk_urb(usbfifo->urb_cmd_in.urb,
            usbfifo->udev, usbfifo->cmd_ep.in_pipe,
            usbfifo->urb_cmd_in.buffer, usbfifo->urb_cmd_in.buffer_max_size,
            cmd_rx_complete, usbfifo);

    res = usb_submit_urb(usbfifo->urb_cmd_in.urb, GFP_KERNEL);
    if (res < 0) {
        DBG(0, KERN_ERR, "unable to submit CMD IN URB\n");
        return res;
    }

    return 0;
}

static void cmd_rx_complete(struct urb *urb)
{
    struct usbfifo *usbfifo = (struct usbfifo *)urb->context;

    schedule_work(&usbfifo->urb_cmd_in_work);
}

static int data_rx_submit(struct usbfifo *usbfifo);
static void data_rx_complete(struct urb *urb);

static void usbfifo_urb_data_in_work(struct work_struct *work)
{
    struct usbfifo *usbfifo = container_of(work, struct usbfifo, urb_data_in_work);
    struct urb *urb = usbfifo->urb_data_in.urb;

    if (urb->status == 0) {
        struct circ_buf *cbuffer = usbfifo->cbuffer_data_in.cbuffer;
        struct mutex *mutex = &usbfifo->cbuffer_data_in.mutex;
        size_t free, count;
        char *buf = usbfifo->urb_data_in.buffer;
        int i;

        mutex_lock(mutex);

        /* FIXME: move this to deffered work */
        free = circ_free(cbuffer);
        if (unlikely(!free)) {
            DBG(0, KERN_WARNING, "no free space to store DATA IN URB buffer(%d bytes)\n", urb->actual_length);
            mutex_unlock(mutex);
            goto out;
        }
        count = min(urb->actual_length, free);

        if (unlikely(urb->actual_length > free)) {
            DBG(0, KERN_WARNING, "no enough space(%d bytes) to store DATA IN URB buffer(%d bytes)\n", free, urb->actual_length);
        }

        for (i=0;i<count;++i)
            circ_byte(cbuffer, cbuffer->head + i) = buf[i];

        cbuffer->head += count;

        DBG(2, KERN_DEBUG, "head: %u tail: %u\n", cbuffer->head, cbuffer->tail);

        mutex_unlock(mutex);
    }

  out:
    data_rx_submit(usbfifo);
}

static int data_rx_submit(struct usbfifo *usbfifo)
{
    int res;

    usb_fill_bulk_urb(usbfifo->urb_data_in.urb,
            usbfifo->udev, usbfifo->data_ep.in_pipe,
            usbfifo->urb_data_in.buffer, usbfifo->urb_data_in.buffer_max_size,
            data_rx_complete, usbfifo);

    res = usb_submit_urb(usbfifo->urb_data_in.urb, GFP_KERNEL);
    if (res < 0) {
        DBG(0, KERN_ERR, "unable to submit DATA IN URB\n");
        return res;
    }

    return 0;
}

static void data_rx_complete(struct urb *urb)
{
    struct usbfifo *usbfifo = (struct usbfifo *)urb->context;

    schedule_work(&usbfifo->urb_data_in_work);
}

static int data_tx_submit(struct usbfifo *usbfifo);
static void data_tx_complete(struct urb *urb);

static void usbfifo_urb_data_out_work(struct work_struct *work)
{
    struct usbfifo *usbfifo = container_of(work, struct usbfifo, urb_data_out_work);
    struct urb *urb = usbfifo->urb_data_out.urb;

    struct circ_buf *cbuffer = usbfifo->cbuffer_data_out.cbuffer;
    struct mutex *mutex = &usbfifo->cbuffer_data_out.mutex;
    size_t avail;
    char *buf = usbfifo->urb_data_out.buffer;
    int i;

    if (urb->status != 0 || urb->actual_length != usbfifo->urb_data_out_buffer_sent) {
        DBG(0, KERN_WARNING, "Prvious URB failed to complete successfully: %d (%d bytes requested, %d sent)\n",
                urb->status, usbfifo->urb_data_out_buffer_sent, urb->actual_length);
    }

    mutex_lock(mutex);

    if (circ_empty(cbuffer)) {
        atomic_set(&usbfifo->urb_data_out_in_progress, 0);
        mutex_unlock(mutex);
        return;
    }

    avail = circ_cnt(cbuffer);
    usbfifo->urb_data_out_buffer_sent = min(usbfifo->urb_data_out.buffer_max_size, avail);

    for (i=0;i<usbfifo->urb_data_out_buffer_sent;++i)
		buf[i] = circ_byte(cbuffer, cbuffer->tail + i);

    cbuffer->tail += usbfifo->urb_data_out_buffer_sent;

    mutex_unlock(mutex);

    DBG(2, KERN_DEBUG, "head: %u tail: %u\n", cbuffer->head, cbuffer->tail);

    data_tx_submit(usbfifo);
}

static int data_tx_submit(struct usbfifo *usbfifo)
{
    int res;

    usb_fill_bulk_urb(usbfifo->urb_data_out.urb,
            usbfifo->udev, usbfifo->data_ep.out_pipe,
            usbfifo->urb_data_out.buffer, usbfifo->urb_data_out_buffer_sent,
            data_tx_complete, usbfifo);

    res = usb_submit_urb(usbfifo->urb_data_out.urb, GFP_KERNEL);
    if (res < 0) {
        DBG(0, KERN_ERR, "unable to submit DATA OUT URB\n");
        return res;
    }

    return 0;
}

static void data_tx_complete(struct urb *urb)
{
    struct usbfifo *usbfifo = (struct usbfifo *)urb->context;

    schedule_work(&usbfifo->urb_data_out_work);
}

static struct usbfifo *usbfifo_find_by_dev(dev_t dev)
{
    struct usbfifo_devlist *devlist;
    struct usbfifo *usbfifo = NULL;

    spin_lock(&usbfifo_devlist_spinlock);
    list_for_each_entry(devlist, &usbfifo_devlist, list) {
        if (dev == devlist->usbfifo->cmd_cdev.dev
                || dev == devlist->usbfifo->data_cdev.dev) {
            usbfifo = devlist->usbfifo;
            break;
        }
    }
    spin_unlock(&usbfifo_devlist_spinlock);

    return usbfifo;
}

static int usbfifo_open(struct inode *inode, struct file *file)
{
    struct cdev *cdev = inode->i_cdev;
    struct usbfifo *usbfifo;

    DBG(2, KERN_DEBUG, "open\n");

    usbfifo = usbfifo_find_by_dev(cdev->dev);
    if (unlikely(!usbfifo)) {
        DBG(0, KERN_ERR, "unable to find device\n");
        return -ENODEV;
    }

    file->private_data = usbfifo;

    try_module_get(THIS_MODULE);

    return 0;
}

static int usbfifo_release(struct inode *inode, struct file *file)
{
    DBG(2, KERN_DEBUG, "release\n");

    module_put(THIS_MODULE);

	return 0;
}

static ssize_t usbfifo_data_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
    struct circ_buf *cbuffer = ((struct usbfifo *)file->private_data)->cbuffer_data_in.cbuffer;
    struct mutex *mutex = &((struct usbfifo *)file->private_data)->cbuffer_data_in.mutex;
    size_t avail;
    int i;
    char *kbuf;
    ssize_t res = 0;

    DBG(2, KERN_DEBUG, "read\n");

    count = min(count, CIRC_SIZE);
    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf) {
        DBG(0, KERN_ERR, "unable to allocate %zu bytes\n", count);
        return -ENOMEM;
    }

    mutex_lock(mutex);

    if (circ_empty(cbuffer)) {
        mutex_unlock(mutex);
        goto out_free_kbuf;
    }

    avail = circ_cnt(cbuffer);
    count = min(count, avail);

	for (i=0;i<count;++i)
		kbuf[i] = circ_byte(cbuffer, cbuffer->tail + i);

    cbuffer->tail += count;

    DBG(2, KERN_DEBUG, "head: %u tail: %u\n", cbuffer->head, cbuffer->tail);

    mutex_unlock(mutex);

	if (copy_to_user(buf, kbuf, count) != 0) {
        DBG(0, KERN_ERR, "unable to copy to user\n");
		res = -EIO;
        goto out_free_kbuf;
    }

    DBG(2, KERN_DEBUG, "read %zu bytes\n", count);

	res = count;

  out_free_kbuf:
    kfree(kbuf);

	return res;
}

static ssize_t usbfifo_data_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
    struct usbfifo *usbfifo = (struct usbfifo *)file->private_data;
    struct circ_buf *cbuffer = usbfifo->cbuffer_data_out.cbuffer;
    struct mutex *mutex = &usbfifo->cbuffer_data_out.mutex;
    size_t free, stored = 0;
    int i;
    char *kbuf = NULL;
    ssize_t res = 0;

    DBG(2, KERN_DEBUG, "write\n");

    mutex_lock(mutex); // grabbing mutex here to ensure that DATA OUT URB complete handler will grab remainig data

    /* try to submit URB first */
    if (atomic_cmpxchg(&usbfifo->urb_data_out_in_progress, 0, 1) == 0) {
        usbfifo->urb_data_out_buffer_sent = min(usbfifo->urb_data_out.buffer_max_size, count);

        if (copy_from_user(usbfifo->urb_data_out.buffer, buf, usbfifo->urb_data_out_buffer_sent) != 0) {
            DBG(0, KERN_ERR, "unable to copy from user\n");
            mutex_unlock(mutex);
            return -EIO;
        }

        res = data_tx_submit(usbfifo);
        if (res < 0) {
            mutex_unlock(mutex);
            return res;
        }

        count -= usbfifo->urb_data_out_buffer_sent;
        buf  += usbfifo->urb_data_out_buffer_sent;
        stored += usbfifo->urb_data_out_buffer_sent;

        DBG(2, KERN_DEBUG, "submitted %u bytes directly to DATA OUT URB\n", usbfifo->urb_data_out_buffer_sent);
    }

    /* everything else should be picked up in DATA OUT URB complete handler */
    if (count) {
        count = min(count, CIRC_SIZE);
        kbuf = kmalloc(count, GFP_KERNEL);
        if (!kbuf) {
            DBG(0, KERN_ERR, "unable to allocate %zu bytes\n", count);
            mutex_unlock(mutex);
            return -ENOMEM;
        }

        if (copy_from_user(kbuf, buf, count) != 0) {
            DBG(0, KERN_ERR, "unable to copy from user\n");
            mutex_unlock(mutex);
            res = -EIO;
            goto out_free_kbuf;
        }

        free = circ_free(cbuffer);
        if (!free) {
            DBG(2, KERN_DEBUG, "circ buffer full\n");
            mutex_unlock(mutex);
            res = stored;
            goto out_free_kbuf;
        }
        count = min(count, free);

        for (i=0;i<count;++i)
            circ_byte(cbuffer, cbuffer->head + i) = kbuf[i];

        cbuffer->head += count;
        stored += count;

        DBG(2, KERN_DEBUG, "head: %u tail: %u\n", cbuffer->head, cbuffer->tail);
    }

    mutex_unlock(mutex);

    DBG(2, KERN_DEBUG, "written %zu bytes\n", stored);

    res = stored;

  out_free_kbuf:
    kfree(kbuf);

	return res;
}

static ssize_t usbfifo_cmd_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
    ssize_t res = 0;
    struct usbfifo *usbfifo = (struct usbfifo *)file->private_data;

    DBG(2, KERN_DEBUG, "read\n");

    mutex_lock(&usbfifo->cmd_in_buffer_mutex);
    if (count < usbfifo->cmd_in_buffer_data_size) {
        DBG(2, KERN_DEBUG, "not enough space(%zu bytes) to store CMD IN buffer(%zu bytes)\n",
                count, usbfifo->cmd_in_buffer_data_size);
        res = -ENOSPC;
        goto out;
    }

	if (copy_to_user(buf, usbfifo->cmd_in_buffer, usbfifo->cmd_in_buffer_data_size) != 0) {
        DBG(0, KERN_ERR, "unable to copy to user\n");
		res = -EIO;
        goto out;
    }
    count = usbfifo->cmd_in_buffer_data_size;
    usbfifo->cmd_in_buffer_data_size = 0;
    mutex_unlock(&usbfifo->cmd_in_buffer_mutex);

    DBG(2, KERN_DEBUG, "read %zu bytes\n", count);

    return count;

  out:
    mutex_unlock(&usbfifo->cmd_in_buffer_mutex);
    return res;
}

static ssize_t usbfifo_cmd_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
    char *kbuf;
    ssize_t res = 0;
    int bulk_write = 0;
    struct usbfifo *usbfifo = (struct usbfifo *)file->private_data;

    DBG(2, KERN_DEBUG, "write\n");

    count = min(count, usbfifo->cmd_ep.out_max_size);
    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf) {
        DBG(0, KERN_ERR, "unable to allocate %zu bytes\n", count);
        return -ENOMEM;
    }

    if (copy_from_user(kbuf, buf, count) != 0) {
        DBG(0, KERN_ERR, "unable to copy from user\n");
    	res = -EIO;
        goto out_free_kbuf;
    }

    /* mutex_lock(&file->f_path.dentry->d_inode->i_mutex); */

    res = usb_bulk_msg(usbfifo->udev, usbfifo->cmd_ep.out_pipe,
            kbuf, count, &bulk_write, 5000);

    /* mutex_unlock(&file->f_path.dentry->d_inode->i_mutex); */

    if (res < 0) {
        DBG(0, KERN_ERR, "unable to complete USB bulk OUT transaction: %d\n", res);
        goto out_free_kbuf;
    }

    DBG(2, KERN_DEBUG, "written %zu bytes\n", bulk_write);

    res = bulk_write;

  out_free_kbuf:
    kfree(kbuf);

	return res;
}

static struct file_operations usbfifo_data_fops = {
	.open       = usbfifo_open,
	.release    = usbfifo_release,
	.read       = usbfifo_data_read,
	.write      = usbfifo_data_write,
	.owner = THIS_MODULE,
};

static struct file_operations usbfifo_cmd_fops = {
	.open       = usbfifo_open,
	.release    = usbfifo_release,
	.read       = usbfifo_cmd_read,
	.write      = usbfifo_cmd_write,
	.owner = THIS_MODULE,
};

static int usbfifo_init_cdev(struct usbfifo_cdev *ucdev, struct file_operations *fops, const char *name)
{
    dev_t dev;
    struct cdev *cdev;
    int res = 0;

    if (major > 0) {
        dev = MKDEV(major, 0);
        res = register_chrdev_region(dev, 1, name);
        if (res < 0) {
            DBG(0, KERN_ERR, "Unable to register a range of char device numbers for %u major for %s\n",
                    major, name);
            goto out;
        }
    } else {
        res = alloc_chrdev_region(&dev, 0, 1, name);
        if (res < 0) {
            DBG(0, KERN_ERR, "Unable to register a range of char device numbers for %s\n", name);
            goto out;
        }
    }

    DBG(0, KERN_INFO, "%s region registered: %u major, %u minor\n",
            name, MAJOR(dev), MINOR(dev));

    cdev = cdev_alloc();
    if (cdev == NULL) {
        res = -ENOMEM;
        DBG(0, KERN_ERR, "Unable to allocate cdev structure for %s\n", name);
        goto out_free_dev;
    }

    cdev_init(cdev, fops);

    res = cdev_add(cdev, dev, 1);
    if (res < 0) {
        DBG(0, KERN_ERR, "Unable to add %s char device to the system\n", name);
        goto out_free_cdev;
    }

    ucdev->dev = dev;
    ucdev->cdev = cdev;

    return 0;

  out_free_cdev:
    cdev_del(cdev);
  out_free_dev:
    unregister_chrdev_region(dev, 1);
  out:
    return res;
}

static void usbfifo_destroy_cdev(struct usbfifo_cdev *ucdev)
{
    cdev_del(ucdev->cdev);
    unregister_chrdev_region(ucdev->dev, 1);
}

static int usbfifo_init_cbuffer(struct usbfifo_cbuffer *cbuffer, int len)
{
    struct circ_buf *cb;

    len += sizeof(struct circ_buf);

    cb = kmalloc(len, GFP_KERNEL);
    if (!cb) {
        DBG(0, KERN_ERR, "Unable to allocate ring buffer %u bytes long\n", len);
        return -ENOMEM;
    }
    cb->head = cb->tail = 0;
    cb->buf = (char *)(cb+1);

    mutex_init(&cbuffer->mutex);
    cbuffer->cbuffer = cb;

    return 0;
}

static int usbfifo_add_to_devlist(struct usbfifo *usbfifo)
{
    struct usbfifo_devlist *devlist;

    devlist = kmalloc(sizeof(struct usbfifo_devlist), GFP_KERNEL);
    if (!devlist) {
        DBG(0, KERN_ERR, "Unable to allocate entry for devlist\n");
        return -ENOMEM;
    }
    devlist->usbfifo = usbfifo;

    spin_lock(&usbfifo_devlist_spinlock);
    list_add_tail(&devlist->list, &usbfifo_devlist);
    spin_unlock(&usbfifo_devlist_spinlock);

    return 0;
}

static inline int usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd)
{
	return __le16_to_cpu(epd->wMaxPacketSize);
}

int usbfifo_get_endpoints(struct usbfifo *usbfifo, struct usb_interface *intf)
{
	int	i;
	struct usb_host_interface *alt = NULL;
	struct usb_host_endpoint *cmd_in = NULL, *cmd_out = NULL;
    struct usb_host_endpoint *data_in = NULL, *data_out = NULL;

	for (i=0; i<intf->num_altsetting; i++) {
        struct usb_host_endpoint *e0, *e1;

        cmd_in = cmd_out = data_in = data_out = NULL;

		alt = intf->altsetting + i;

        /* Usbfifo has 4 endpoints,
           at least this driver only knows how to deal with this configuration */

        if (alt->desc.bNumEndpoints != 4)
            continue;

        e0 = alt->endpoint + 0;
        e1 = alt->endpoint + 1;
        if (e0->desc.bmAttributes != USB_ENDPOINT_XFER_BULK
                || e1->desc.bmAttributes != USB_ENDPOINT_XFER_BULK)
            continue;

        if (usb_endpoint_dir_in(&e0->desc) && usb_endpoint_dir_out(&e1->desc)) {
            cmd_in  = e0;
            cmd_out = e1;
        } else if (usb_endpoint_dir_out(&e0->desc) && usb_endpoint_dir_in(&e1->desc)) {
            cmd_out = e0;
            cmd_in  = e1;
        } else
            continue;

        e0 = alt->endpoint + 2;
        e1 = alt->endpoint + 3;
        if (usb_endpoint_dir_in(&e0->desc) && usb_endpoint_dir_out(&e1->desc)) {
            data_in  = e0;
            data_out = e1;
        } else if (usb_endpoint_dir_out(&e0->desc) && usb_endpoint_dir_in(&e1->desc)) {
            data_out = e0;
            data_in  = e1;
        } else
            continue;

        break;
	}

	if (!cmd_in || !cmd_out || !data_in || !data_out) {
        DBG(0, KERN_ERR, "Can not find suitable endpoints\n");
		return -EINVAL;
    }

	if (alt->desc.bAlternateSetting != 0) {
        int res;;
		res = usb_set_interface (usbfifo->udev, alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (res < 0) {
            DBG(0, KERN_ERR, "Can not set interface=%d alternate setting=%d\n",
                    alt->desc.bInterfaceNumber, alt->desc.bAlternateSetting);
			return res;
        }
	}

	usbfifo->cmd_ep.in_pipe = usb_rcvbulkpipe (usbfifo->udev,
			cmd_in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    usbfifo->cmd_ep.in_max_size = usb_endpoint_maxp(&cmd_in->desc);

	usbfifo->cmd_ep.out_pipe = usb_sndbulkpipe (usbfifo->udev,
			cmd_out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    usbfifo->cmd_ep.out_max_size = usb_endpoint_maxp(&cmd_out->desc);

    usbfifo->data_ep.in_pipe = usb_rcvbulkpipe (usbfifo->udev,
			data_in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    usbfifo->data_ep.in_max_size = usb_endpoint_maxp(&data_in->desc);

    usbfifo->data_ep.out_pipe = usb_sndbulkpipe (usbfifo->udev,
			data_out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    usbfifo->data_ep.out_max_size = usb_endpoint_maxp(&data_out->desc);

	return 0;
}

static int usbfifo_allocate_urb_and_buffer(struct usbfifo_urb *urb, size_t size)
{
    urb->buffer_max_size = size;
    urb->buffer = kmalloc(size, GFP_KERNEL);
    if (!urb->buffer) {
        DBG(0, KERN_ERR, "unable to allocate buffer for URB(%d bytes)\n", size);
        return -ENOMEM;
    }

    urb->urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb->urb) {
        DBG(0, KERN_ERR, "unable to allocate URB\n");
        goto out_free_buffer;
    }

    return 0;

  out_free_buffer:
    kfree(urb->buffer);

    return -ENOMEM;
}
static void usbfifo_free_urb_and_buffer(struct usbfifo_urb *urb)
{
    usb_free_urb(urb->urb);
    kfree(urb->buffer);
}

static int usbfifo_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct usbfifo *usbfifo = NULL;
    int res = 0;

    DBG(2, KERN_DEBUG, "probe\n");

    usbfifo = kmalloc(sizeof(struct usbfifo), GFP_KERNEL);
    if (!usbfifo) {
        DBG(0, KERN_ERR, "Unable to allocate memory for usbfifo\n");
        return -ENOMEM;
    }

    res = usbfifo_init_cdev(&usbfifo->cmd_cdev, &usbfifo_cmd_fops, "usbfifo_cmd");
    if (res < 0)
        goto out_free_usbfifo;
    res = usbfifo_init_cdev(&usbfifo->data_cdev, &usbfifo_data_fops, "usbfifo_data");
    if (res < 0)
        goto out_free_cmd_cdev;

    res = usbfifo_init_cbuffer(&usbfifo->cbuffer_data_out, CIRC_SIZE);
    if (res < 0)
        goto out_free_data_cdev;
    res = usbfifo_init_cbuffer(&usbfifo->cbuffer_data_in, CIRC_SIZE);
    if (res < 0)
        goto out_free_cbuffer_data_out;

    res = usbfifo_add_to_devlist(usbfifo);
    if (res < 0)
        goto out_free_cbuffer_data_in;

    usbfifo->udev = interface_to_usbdev(intf);
    usb_get_dev(usbfifo->udev);
    usbfifo->intf = intf;
    usbfifo->driver = to_usb_driver(intf->dev.driver);

    usb_set_intfdata (intf, usbfifo);

    res = usbfifo_get_endpoints(usbfifo, intf);
    if (res < 0)
        goto out_release_usb_dev;

    res = usbfifo_allocate_urb_and_buffer(&usbfifo->urb_cmd_in,
            usbfifo->cmd_ep.in_max_size); // FIXME: can submit more than max size of EP
    if (res < 0)
        goto out_release_usb_dev;

    res = usbfifo_allocate_urb_and_buffer(&usbfifo->urb_data_in,
            usbfifo->data_ep.in_max_size); // FIXME: can submit more than max size of EP
    if (res < 0)
        goto out_free_urb_cmd_in;

    res = usbfifo_allocate_urb_and_buffer(&usbfifo->urb_data_out,
            usbfifo->data_ep.out_max_size); // FIXME: can submit more than max size of EP
    if (res < 0)
        goto out_free_urb_data_in;
    atomic_set(&usbfifo->urb_data_out_in_progress, 0);

    usbfifo->cmd_in_buffer_data_size = usbfifo->urb_cmd_in.buffer_max_size;
    usbfifo->cmd_in_buffer = kmalloc(usbfifo->cmd_in_buffer_data_size, GFP_KERNEL);
    if (!usbfifo->cmd_in_buffer) {
        DBG(0, KERN_ERR, "unable to allocate buffer for CMD IN %d bytes\n", usbfifo->cmd_in_buffer_data_size);
        goto out_free_urb_data_out;
    }
    mutex_init(&usbfifo->cmd_in_buffer_mutex);

    INIT_WORK(&usbfifo->urb_cmd_in_work, usbfifo_urb_cmd_in_work);
    INIT_WORK(&usbfifo->urb_data_in_work, usbfifo_urb_data_in_work);
    INIT_WORK(&usbfifo->urb_data_out_work, usbfifo_urb_data_out_work);

    res = data_rx_submit(usbfifo);
    if (res < 0)
        goto out_free_cmd_buffer_in;

    res = cmd_rx_submit(usbfifo);
    if (res < 0)
        goto out_cancel_data_rx_urb;

    DBG(1, KERN_INFO, "USB device was successfully settled in the system\n");

    return 0;

  out_cancel_data_rx_urb:
    usb_kill_urb(usbfifo->urb_data_in.urb);
  out_free_cmd_buffer_in:
    kfree(usbfifo->cmd_in_buffer);
  out_free_urb_data_out:
    usbfifo_free_urb_and_buffer(&usbfifo->urb_data_out);
  out_free_urb_data_in:
    usbfifo_free_urb_and_buffer(&usbfifo->urb_data_in);
  out_free_urb_cmd_in:
    usbfifo_free_urb_and_buffer(&usbfifo->urb_cmd_in);
  out_release_usb_dev:
    usb_put_dev(usbfifo->udev);
    usb_set_intfdata (intf, NULL);
  out_free_cbuffer_data_in:
    kfree(usbfifo->cbuffer_data_in.cbuffer);
  out_free_cbuffer_data_out:
    kfree(usbfifo->cbuffer_data_out.cbuffer);
  out_free_data_cdev:
    usbfifo_destroy_cdev(&usbfifo->data_cdev);
  out_free_cmd_cdev:
    usbfifo_destroy_cdev(&usbfifo->cmd_cdev);
  out_free_usbfifo:
    kfree(usbfifo);
    return res;
}

void usbfifo_disconnect(struct usb_interface *intf)
{
	struct usbfifo *usbfifo = usb_get_intfdata(intf);

    DBG(2, KERN_DEBUG, "disconnect\n");

    if (usbfifo) {
        struct usbfifo_devlist *devlist;

        spin_lock(&usbfifo_devlist_spinlock);
        list_for_each_entry(devlist, &usbfifo_devlist, list) {
            if (usbfifo->cmd_cdev.dev == devlist->usbfifo->cmd_cdev.dev
                    || usbfifo->data_cdev.dev == devlist->usbfifo->data_cdev.dev) {
                list_del(&devlist->list);
                kfree(devlist); // FIXME: a bit dangerous while spinlock is acquired
                break;
            }
        }
        spin_unlock(&usbfifo_devlist_spinlock);

        usbfifo_destroy_cdev(&usbfifo->data_cdev);
        usbfifo_destroy_cdev(&usbfifo->cmd_cdev);

        usb_kill_urb(usbfifo->urb_data_out.urb);
        usb_kill_urb(usbfifo->urb_data_in.urb);
        usb_kill_urb(usbfifo->urb_cmd_in.urb);

        cancel_work_sync(&usbfifo->urb_cmd_in_work);
        cancel_work_sync(&usbfifo->urb_data_in_work);
        cancel_work_sync(&usbfifo->urb_data_out_work);

        flush_scheduled_work();

        usbfifo_free_urb_and_buffer(&usbfifo->urb_data_out);
        usbfifo_free_urb_and_buffer(&usbfifo->urb_data_in);
        usbfifo_free_urb_and_buffer(&usbfifo->urb_cmd_in);

        kfree(usbfifo->cmd_in_buffer);

        kfree(usbfifo->cbuffer_data_in.cbuffer);
        kfree(usbfifo->cbuffer_data_out.cbuffer);

        usb_put_dev(usbfifo->udev);

        kfree(usbfifo);

        usb_set_intfdata (intf, NULL);
    }
}

#define VENDOR_ID_MICROCHIP         0x04D8
#define PRODUCT_ID_USB_FIFO         0x0400

enum usbfifo_type {
	USB_FIFO_TYPE_SMART = 1,
};


static const struct usb_device_id usbfifo_products [] = {
  {
      USB_DEVICE(VENDOR_ID_MICROCHIP, PRODUCT_ID_USB_FIFO),
      .driver_info = (kernel_ulong_t) USB_FIFO_TYPE_SMART,
  }
, { }		// END
};
MODULE_DEVICE_TABLE(usb, usbfifo_products);

static struct usb_driver usbfifo_driver = {
	.name =		"usbfifo",
	.id_table =	usbfifo_products,
	.probe =	usbfifo_probe,
	.disconnect =	usbfifo_disconnect,
};

static int __init usbfifo_init(void)
{
    int res;

    DBG(0, KERN_INFO, "usbfifo init\n");
    DBG(1, KERN_DEBUG, "debug level %d\n", debug_level);

    res = usb_register(&usbfifo_driver);
    if (res)
        DBG(0, KERN_ERR, "Unable to register usb driver\n");
    else
        DBG(1, KERN_INFO, "usbfifo driver was successfully added to the system\n");

	return res;
}
module_init(usbfifo_init);

static void __exit usbfifo_exit(void)
{
    DBG(0, KERN_INFO, "usbfifo exit\n");

    usb_deregister(&usbfifo_driver);
}
module_exit(usbfifo_exit);

MODULE_AUTHOR("Dmytro Milinevskyy <milinevskyy@gmail.com>");
MODULE_DESCRIPTION("Kernel usbfifo module.");
MODULE_LICENSE("GPL");
