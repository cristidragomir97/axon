// SPDX-License-Identifier: GPL-2.0
/*
 * i2c-robocore.c — I2C adapter driver for RoboCore Link101</｜DSML｜parameter>
 *
 * Based on i2c-tiny-usb by Till Harbaum <till@harbaum.org>
 * Modified for composite device support by RoboCore
 *
 * Changes from i2c-tiny-usb:
 *   - Interface filtering: only binds to subclass=0x01 protocol=0x01
 *   - USB_DEVICE_AND_INTERFACE_INFO matching for auto-bind
 *   - DMA-safe kmalloc'd buffer for all USB control transfers
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

/* i2c-tiny-usb protocol commands */
#define CMD_ECHO       0x00
#define CMD_GET_FUNC   0x01
#define CMD_SET_DELAY  0x02
#define CMD_GET_STATUS 0x03

#define CMD_I2C_IO     0x04
#define CMD_I2C_IO_BEGIN (1 << 0)
#define CMD_I2C_IO_END   (1 << 1)

/* Status codes */
#define STATUS_IDLE    0x00
#define STATUS_ACK     0x01
#define STATUS_NAK     0x02

/* Interface markers for composite filtering */
#define ROBOCORE_I2C_SUBCLASS  0x01
#define ROBOCORE_I2C_PROTOCOL  0x01

/* USB timeout in ms */
#define USB_TIMEOUT    1000

/* Size of the DMA-safe buffer for control transfers.
 * Must be at least as large as the biggest I2C transfer we support.
 * i2c-tiny-usb uses EP0 control transfers, max 64 bytes per transfer. */
#define USB_BUF_SIZE   64

struct i2c_robocore {
	struct usb_device *usb_dev;
	struct usb_interface *interface;
	struct i2c_adapter adapter;

	/* DMA-safe buffer for USB control transfers.
	 * USB requires transfer buffers to be kmalloc'd, not on the stack.
	 * Protected by io_mutex since I2C transactions are multi-step. */
	u8 *usb_buf;
	struct mutex io_mutex;
};

static int usb_read(struct i2c_robocore *dev, int cmd,
		    int value, int index, void *data, int len)
{
	int ret;

	/* Read into DMA-safe buffer, then copy to caller */
	ret = usb_control_msg(dev->usb_dev,
			      usb_rcvctrlpipe(dev->usb_dev, 0),
			      cmd,
			      USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			      value, index, dev->usb_buf, len, USB_TIMEOUT);

	if (ret > 0 && data != dev->usb_buf)
		memcpy(data, dev->usb_buf, ret);

	return ret;
}

static int usb_write(struct i2c_robocore *dev, int cmd,
		     int value, int index, void *data, int len)
{
	/* Copy caller data into DMA-safe buffer, then send */
	if (len > 0 && data != dev->usb_buf)
		memcpy(dev->usb_buf, data, len);

	return usb_control_msg(dev->usb_dev,
			       usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd,
			       USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			       value, index, dev->usb_buf, len, USB_TIMEOUT);
}

static int usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct i2c_robocore *dev = i2c_get_adapdata(adapter);
	struct i2c_msg *pmsg;
	int i, ret;
	u8 status;

	mutex_lock(&dev->io_mutex);

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		/* Build flags for begin/end markers */
		int flags = 0;
		if (i == 0)
			flags |= CMD_I2C_IO_BEGIN;
		if (i == num - 1)
			flags |= CMD_I2C_IO_END;

		if (pmsg->flags & I2C_M_RD) {
			/* Read from I2C device */
			ret = usb_read(dev, CMD_I2C_IO + flags,
				       pmsg->flags, pmsg->addr,
				       dev->usb_buf, pmsg->len);
			if (ret != pmsg->len) {
				dev_err(&adapter->dev,
					"read failed: addr=0x%02x len=%d ret=%d\n",
					pmsg->addr, pmsg->len, ret);
				mutex_unlock(&dev->io_mutex);
				return -EIO;
			}
			memcpy(pmsg->buf, dev->usb_buf, pmsg->len);
		} else {
			/* Write to I2C device */
			memcpy(dev->usb_buf, pmsg->buf, pmsg->len);
			ret = usb_write(dev, CMD_I2C_IO + flags,
					pmsg->flags, pmsg->addr,
					dev->usb_buf, pmsg->len);
			if (ret != pmsg->len) {
				dev_err(&adapter->dev,
					"write failed: addr=0x%02x len=%d ret=%d\n",
					pmsg->addr, pmsg->len, ret);
				mutex_unlock(&dev->io_mutex);
				return -EIO;
			}
		}

		/* Check status */
		ret = usb_read(dev, CMD_GET_STATUS, 0, 0, dev->usb_buf, 1);
		if (ret != 1) {
			dev_err(&adapter->dev, "status read failed\n");
			mutex_unlock(&dev->io_mutex);
			return -EIO;
		}
		status = dev->usb_buf[0];

		if (status == STATUS_NAK) {
			dev_dbg(&adapter->dev, "NAK from addr 0x%02x\n",
				pmsg->addr);
			mutex_unlock(&dev->io_mutex);
			return -ENXIO;
		}

		if (status != STATUS_ACK) {
			dev_err(&adapter->dev,
				"unexpected status 0x%02x from addr 0x%02x\n",
				status, pmsg->addr);
			mutex_unlock(&dev->io_mutex);
			return -EIO;
		}
	}

	mutex_unlock(&dev->io_mutex);
	return num;
}

static u32 usb_func(struct i2c_adapter *adapter)
{
	struct i2c_robocore *dev = i2c_get_adapdata(adapter);
	__le32 func;
	int ret;

	mutex_lock(&dev->io_mutex);
	ret = usb_read(dev, CMD_GET_FUNC, 0, 0, dev->usb_buf, sizeof(func));
	if (ret != sizeof(func)) {
		dev_err(&adapter->dev, "get_func failed: %d\n", ret);
		mutex_unlock(&dev->io_mutex);
		return 0;
	}
	memcpy(&func, dev->usb_buf, sizeof(func));
	mutex_unlock(&dev->io_mutex);

	return le32_to_cpu(func);
}

static const struct i2c_algorithm usb_algorithm = {
	.master_xfer = usb_xfer,
	.functionality = usb_func,
};

static int i2c_robocore_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct i2c_robocore *dev;
	__le32 func;
	int ret;

	/*
	 * COMPOSITE DEVICE FILTER
	 *
	 * Only bind to interfaces with subclass=0x01 protocol=0x01.
	 * Link101 sets this only on its I2C interface, so we skip CAN
	 * and any other vendor-class interfaces.
	 */
	if (hostif->desc.bInterfaceSubClass != ROBOCORE_I2C_SUBCLASS ||
	    hostif->desc.bInterfaceProtocol != ROBOCORE_I2C_PROTOCOL) {
		dev_dbg(&interface->dev,
			"skipping interface %d (sub=0x%02x proto=0x%02x)\n",
			hostif->desc.bInterfaceNumber,
			hostif->desc.bInterfaceSubClass,
			hostif->desc.bInterfaceProtocol);
		return -ENODEV;
	}

	dev_info(&interface->dev,
		 "RoboCore I2C adapter on interface %d\n",
		 hostif->desc.bInterfaceNumber);

	dev = devm_kzalloc(&interface->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* Allocate DMA-safe buffer for USB control transfers */
	dev->usb_buf = kmalloc(USB_BUF_SIZE, GFP_KERNEL);
	if (!dev->usb_buf)
		return -ENOMEM;

	mutex_init(&dev->io_mutex);

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* Set up I2C adapter */
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &usb_algorithm;
	dev->adapter.dev.parent = &interface->dev;
	i2c_set_adapdata(&dev->adapter, dev);
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
         "RoboCore Link101 I2C at usb-%s-%s if%d",
		 dev->usb_dev->bus->bus_name,
		 dev->usb_dev->devpath,
		 hostif->desc.bInterfaceNumber);

	/* Verify the device responds to our protocol */
	msleep(100);  /* Wait for device to be ready */
	mutex_lock(&dev->io_mutex);
	ret = usb_read(dev, CMD_GET_FUNC, 0, 0, dev->usb_buf, sizeof(func));
	mutex_unlock(&dev->io_mutex);
	if (ret != sizeof(func)) {
		dev_err(&interface->dev,
			"failed to read I2C functionality: %d\n", ret);
		kfree(dev->usb_buf);
		usb_put_dev(dev->usb_dev);
		return -EIO;
	}
	memcpy(&func, dev->usb_buf, sizeof(func));
	dev_info(&interface->dev, "I2C functionality: 0x%08x\n",
		 le32_to_cpu(func));

	ret = i2c_add_adapter(&dev->adapter);
	if (ret) {
		dev_err(&interface->dev, "failed to add I2C adapter: %d\n", ret);
		kfree(dev->usb_buf);
		usb_put_dev(dev->usb_dev);
		return ret;
	}

	usb_set_intfdata(interface, dev);

	dev_info(&interface->dev,
		 "RoboCore I2C adapter registered as i2c-%d\n",
		 dev->adapter.nr);
	return 0;
}

static void i2c_robocore_disconnect(struct usb_interface *interface)
{
	struct i2c_robocore *dev = usb_get_intfdata(interface);

	i2c_del_adapter(&dev->adapter);
	kfree(dev->usb_buf);
	usb_put_dev(dev->usb_dev);

	dev_info(&interface->dev, "RoboCore I2C adapter disconnected\n");
}

static const struct usb_device_id i2c_robocore_table[] = {
	{
		USB_DEVICE_AND_INTERFACE_INFO(
			0x1209,
			0xAC01,
			USB_CLASS_VENDOR_SPEC,
			ROBOCORE_I2C_SUBCLASS,
			ROBOCORE_I2C_PROTOCOL
		),
	},
	{ }
};
MODULE_DEVICE_TABLE(usb, i2c_robocore_table);

static struct usb_driver i2c_robocore_driver = {
	.name       = "i2c-robocore",
	.probe      = i2c_robocore_probe,
	.disconnect = i2c_robocore_disconnect,
	.id_table   = i2c_robocore_table,
};
module_usb_driver(i2c_robocore_driver);

MODULE_AUTHOR("RoboCore <hello@robocore.dev>");
MODULE_DESCRIPTION("I2C adapter for RoboCore Link101 (composite-aware i2c-tiny-usb)");
MODULE_LICENSE("GPL");