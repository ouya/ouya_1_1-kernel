/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/mii.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>

#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

/* ethernet packet ethertype for IP packets */
#define NET_IP_ETHERTYPE		{0x08, 0x00}

#define	TX_TIMEOUT		10

#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  /* USB_NET_BUFSIZ */

/* maximum interface number supported */
#define MAX_INTFS	3

MODULE_LICENSE("GPL");

/* To support more rmnet interfaces, increase the default max_intfs or
 * pass kernel module parameter.
 * e.g. insmod raw_ip_net.ko max_intfs=5
 */
static int max_intfs = 3;	/* xmm6260 default number of interfaces */

static unsigned long usb_net_raw_ip_intf[MAX_INTFS] = { 3, 5, 7};
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

/* max_intfs should not be changed at runtime */
module_param(max_intfs, int, S_IRUGO);
MODULE_PARM_DESC(max_intfs, "usb net (raw-ip) - max. interfaces supported");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");


static void find_usb_pipe(struct usbnet *usbnet)
{
	struct usb_device *usbdev = usbnet->udev;
	struct usb_interface *intf = usbnet->intf;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			usbnet->in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
			pr_debug("usbnet : endpoint[%d] bulk in , pipe.bulk.in = %d\n"
					, n, usbnet->in);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			usbnet->out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
			pr_debug("usbnet : endpoint[%d] bulk in , pipe.bulk.out = %d\n"
					, n, usbnet->out);
		} else
			pr_info("endpoint[%d] skipped\n", n);
	}
}

static struct net_device_stats *baseband_usb_netdev_get_stats(
				struct net_device *dev)
{
	pr_debug("[%s]\n", __func__);
	return &dev->stats;
}

static struct net_device_ops usb_net_raw_ip_ops = {
	.ndo_open =		usbnet_open,
	.ndo_stop =		usbnet_stop,
	.ndo_start_xmit =	usbnet_start_xmit,
	.ndo_get_stats = baseband_usb_netdev_get_stats,
};

static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	int j;
	int status;

	for (j = 0; j < max_intfs; j++) {
		if (usb_net_raw_ip_intf[j] ==
				intf->cur_altsetting->desc.bInterfaceNumber) {
			pr_info("%s: raw_ip using interface %d\n", __func__
				, intf->cur_altsetting->desc.bInterfaceNumber);
			break;
		}
	}

	if (j == max_intfs)
		return -ENODEV;

	status = usbnet_probe(intf, id);
	if (status < 0) {
		dev_info(&intf->dev, "usbnet_probe failed %d\n", status);
		goto out;
	}
out:
	return status;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	usbnet_disconnect(intf);
}

#ifdef CONFIG_PM
static int baseband_usb_driver_suspend(struct usb_interface *intf,
	pm_message_t message)
{
	return usbnet_suspend(intf, message);
}

static int baseband_usb_driver_resume(struct usb_interface *intf)
{
	return usbnet_resume(intf);
}

static int baseband_usb_driver_reset_resume(struct usb_interface *intf)
{
	pr_debug("%s intf %p\n", __func__, intf);
	return baseband_usb_driver_resume(intf);
}
#endif /* CONFIG_PM */

static int rawipnet_bind(struct usbnet *usbnet, struct usb_interface *iface)
{
	struct net_device *netdev = usbnet->net;
	/* net device setting */
	netdev->netdev_ops = &usb_net_raw_ip_ops;
	netdev->watchdog_timeo = TX_TIMEOUT;
	random_ether_addr(netdev->dev_addr);

	/* usbnet setting */
	find_usb_pipe(usbnet);
	usbnet->rx_urb_size = USB_NET_BUFSIZ;
	return 0;
}

static struct sk_buff *rawipnet_usb_tx_fixup(struct usbnet *dev,
		struct sk_buff *skb, gfp_t flags)
{
	skb_pull(skb, 14);
	return skb;
}

static int rawipnet_usb_rx_fixup(struct usbnet *dev
		, struct sk_buff *skb)
{
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
				/* Destination MAC */
				0x00, 0x00,
				0x00, 0x00,
				0x00, 0x00,
				/* Source MAC */
				0x00, 0x00,
				0x00, 0x00,
				0x00, 0x00,
				/* EtherType */
				NET_IP_ETHERTYPE,
			};

	if ((((unsigned char *) skb->data)[0]
		& 0xf0) == 0x60) {
		/* ipv6 ether type */
		ethernet_header[12] = 0x86;
		ethernet_header[13] = 0xdd;
	}

	if (!pskb_expand_head(skb, 14, 0, GFP_ATOMIC)) {
		memcpy(ethernet_header + 0,
				dev->net->dev_addr, 6);
		memcpy(ethernet_header + 6,
			"0x01\0x02\0x03\0x04\0x05\0x06", 6);
		dst = skb_push(skb, 14);
		memcpy(dst, ethernet_header, 14);
	} else {
		pr_info("usbnet: usb_net_raw_ip_rx_urb_comp_work - "
			"pskb_expand_head() failed\n");
		return 0;
	}
	return 1;
}

static int rawipnet_usb_manage_power(struct usbnet *dev, int on)
{
	return 0;
}

static const struct driver_info baseband_usb_driver_info = {
	.description   = "raw ip net device",
	.flags = FLAG_RMNET,
	.bind          = rawipnet_bind,
	.tx_fixup      = rawipnet_usb_tx_fixup,
	.rx_fixup      = rawipnet_usb_rx_fixup,
	.manage_power	= rawipnet_usb_manage_power,

};

static struct usb_device_id baseband_usb_driver_id_table[] = {
	/* xmm modem vid, pid */
	{
		USB_DEVICE(0x1519, 0x0020),
		.driver_info = (unsigned long)&baseband_usb_driver_info,
	},
	{ },
};

static struct usb_driver baseband_usb_driver = {
		.name = "bb_raw_ip_net",
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table,
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
};

MODULE_DEVICE_TABLE(usb, baseband_usb_driver_id_table);

static int usb_net_raw_ip_init(void)
{
	int err;

	pr_debug("usb_net_raw_ip_init { max_intfs %d\n", max_intfs);

	err = usb_register(&baseband_usb_driver);
	if (err < 0)
		pr_err("cannot open usb driver - err %d\n", err);

	return err;
}

static void usb_net_raw_ip_exit(void)
{
	pr_debug("usb_net_raw_ip_exit {\n");
	usb_deregister(&baseband_usb_driver);
	pr_debug("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)

