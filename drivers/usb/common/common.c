// SPDX-License-Identifier: GPL-2.0+
/*
 * Provides code common for host and device side USB.
 *
 * (C) Copyright 2016
 *     Texas Instruments Incorporated, <www.ti.com>
 */

#include <dm.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <linux/printk.h>
#include <linux/usb/otg.h>
#include <linux/usb/ch9.h>
#include <linux/usb/phy.h>

DECLARE_GLOBAL_DATA_PTR;

static const char *const usb_dr_modes[] = {
	[USB_DR_MODE_UNKNOWN]		= "",
	[USB_DR_MODE_HOST]		= "host",
	[USB_DR_MODE_PERIPHERAL]	= "peripheral",
	[USB_DR_MODE_OTG]		= "otg",
};

/**
 * get_remote_node_from_graph - Resolve the remote node from a graph binding
 * @node: Starting ofnode (e.g., connector)
 * @port_id: Port unit address (e.g., 0 for port@0, or -1 for first port)
 * @endpoint_id: Endpoint unit address (e.g., 0 for endpoint@0, or -1 for first endpoint)
 * Return: ofnode of the remote node, or ofnode_null() on failure
 */
static ofnode get_remote_node_from_graph(ofnode node, int port_id, int endpoint_id)
{
	ofnode port_node, endpoint_node, remote_node;
	u32 phandle_value;
	char port_name[16];
	char endpoint_name[16];

	/* Validate the starting node */
	if (!ofnode_valid(node))
		return ofnode_null();

	/* Construct port name (e.g., "port" or "port@0") */
	if (port_id == -1)
		strcpy(port_name, "port");
	else
		snprintf(port_name, sizeof(port_name), "port@%d", port_id);

	/* Find the port node */
	port_node = ofnode_find_subnode(node, port_name);
	if (!ofnode_valid(port_node)) {
		log_debug("No '%s' node found\n", port_name);
		return ofnode_null();
	}

	/* Construct endpoint name (e.g., "endpoint" or "endpoint@0") */
	if (endpoint_id == -1)
		strcpy(endpoint_name, "endpoint");
	else
		snprintf(endpoint_name, sizeof(endpoint_name), "endpoint@%d", endpoint_id);

	/* Find the endpoint node */
	endpoint_node = ofnode_find_subnode(port_node, endpoint_name);
	if (!ofnode_valid(endpoint_node)) {
		log_debug("No '%s' node found under '%s'\n", endpoint_name, port_name);
		return ofnode_null();
	}

	/* Read the remote-endpoint phandle */
	phandle_value = ofnode_read_u32_default(endpoint_node, "remote-endpoint", 0);
	if (phandle_value == 0) {
		log_debug("No valid 'remote-endpoint' phandle in '%s'\n", endpoint_name);
		return ofnode_null();
	}

	/* Resolve the phandle to the remote node */
	remote_node = ofnode_get_by_phandle(phandle_value);
	if (!ofnode_valid(remote_node)) {
		log_debug("Failed to resolve phandle %u\n", phandle_value);
		return ofnode_null();
	}

	return remote_node;
}

static enum usb_dr_mode get_connector_drmode(ofnode node)
{
	struct gpio_desc id;
	enum usb_dr_mode dr_mode = USB_DR_MODE_OTG;
	ofnode conn;

	/* get remote endpoint */
	conn = get_remote_node_from_graph(node, -1, -1);
	/* get port endpoint */
	if (ofnode_valid(conn))
		conn = ofnode_get_parent(conn);
	/* get connector */
	if (ofnode_valid(conn))
		conn = ofnode_get_parent(conn);
	if (ofnode_valid(conn) &&
	    ofnode_device_is_compatible(conn, "gpio-usb-b-connector") &&
	    !gpio_request_by_name_nodev(conn, "id-gpios", 0, &id, GPIOD_IS_IN)) {
		if (dm_gpio_get_value(&id))
			dr_mode = USB_DR_MODE_PERIPHERAL;
		else
			dr_mode = USB_DR_MODE_HOST;
		gpio_free_list_nodev(&id, 1);
		pr_debug("%s got dr_mode from connector %s dr_mode=%s\n", __func__,
			 ofnode_get_name(node),
			 dr_mode == USB_DR_MODE_HOST ? "host" : "peripheral");
	}

	return dr_mode;
}

enum usb_dr_mode usb_get_dr_mode(ofnode node)
{
	const char *dr_mode;
	int i;

	dr_mode = ofnode_read_string(node, "dr_mode");
	if (!dr_mode) {
		pr_debug("usb dr_mode not found\n");
		return USB_DR_MODE_UNKNOWN;
	}

	for (i = 0; i < ARRAY_SIZE(usb_dr_modes); i++)
		if (!strcmp(dr_mode, usb_dr_modes[i]))
			return (i == USB_DR_MODE_OTG) ?  get_connector_drmode(node) : i;

	return USB_DR_MODE_UNKNOWN;
}

enum usb_dr_mode usb_get_role_switch_default_mode(ofnode node)
{
	const char *dr_mode;
	int i;

	dr_mode = ofnode_read_string(node, "role-switch-default-mode");
	if (!dr_mode)
		return USB_DR_MODE_UNKNOWN;

	for (i = 0; i < ARRAY_SIZE(usb_dr_modes); i++)
		if (!strcmp(dr_mode, usb_dr_modes[i]))
			return i;

	return USB_DR_MODE_UNKNOWN;
}

static const char *const speed_names[] = {
	[USB_SPEED_UNKNOWN] = "UNKNOWN",
	[USB_SPEED_LOW] = "low-speed",
	[USB_SPEED_FULL] = "full-speed",
	[USB_SPEED_HIGH] = "high-speed",
	[USB_SPEED_WIRELESS] = "wireless",
	[USB_SPEED_SUPER] = "super-speed",
	[USB_SPEED_SUPER_PLUS] = "super-speed-plus",
};

const char *usb_speed_string(enum usb_device_speed speed)
{
	if (speed < 0 || speed >= ARRAY_SIZE(speed_names))
		speed = USB_SPEED_UNKNOWN;
	return speed_names[speed];
}

enum usb_device_speed usb_get_maximum_speed(ofnode node)
{
	const char *max_speed;
	int i;

	max_speed = ofnode_read_string(node, "maximum-speed");
	if (!max_speed) {
		pr_debug("usb maximum-speed not found\n");
		return USB_SPEED_UNKNOWN;
	}

	for (i = 0; i < ARRAY_SIZE(speed_names); i++)
		if (!strcmp(max_speed, speed_names[i]))
			return i;

	return USB_SPEED_UNKNOWN;
}

#if CONFIG_IS_ENABLED(DM_USB)
static const char *const usbphy_modes[] = {
	[USBPHY_INTERFACE_MODE_UNKNOWN]	= "",
	[USBPHY_INTERFACE_MODE_UTMI]	= "utmi",
	[USBPHY_INTERFACE_MODE_UTMIW]	= "utmi_wide",
	[USBPHY_INTERFACE_MODE_ULPI]	= "ulpi",
	[USBPHY_INTERFACE_MODE_SERIAL]	= "serial",
	[USBPHY_INTERFACE_MODE_HSIC]	= "hsic",
};

enum usb_phy_interface usb_get_phy_mode(ofnode node)
{
	const char *phy_type;
	int i;

	phy_type = ofnode_get_property(node, "phy_type", NULL);
	if (!phy_type)
		return USBPHY_INTERFACE_MODE_UNKNOWN;

	for (i = 0; i < ARRAY_SIZE(usbphy_modes); i++)
		if (!strcmp(phy_type, usbphy_modes[i]))
			return i;

	return USBPHY_INTERFACE_MODE_UNKNOWN;
}
#endif
