#include "NordicNUSDriver.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <stdio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include "platform/config.h"

#define LOG_MODULE_NAME posey_nus_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

static struct bt_conn *current_conn = NULL;
static struct bt_conn *auth_conn = NULL;

static struct bt_data ad[2] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void connected(
	struct bt_conn * conn,
	uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(
	struct bt_conn * conn,
	uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %x)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(
	struct bt_conn *conn,
	const uint8_t *const data,
	uint16_t len)
{
	// int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received %d bytes from %s. Ignoring.", len, addr);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

int init_nus()
{
	int err = 0;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Error bt_enable: %d", err);
		return err;
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("bt_nus_init: Failed to initialize UART service (err: %d)", err);
		return err;
	}

	// Update the BLE name using the device config.
	size_t name_len = strlen(device_config.name);
	ad[1].data = device_config.name;
	ad[1].data_len = name_len;

	err = bt_le_adv_start(
		BT_LE_ADV_CONN,
		ad, ARRAY_SIZE(ad),
		sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	return 0;
}
