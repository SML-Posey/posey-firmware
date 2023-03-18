#include "NordicNUSDriver.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>

#ifdef CONFIG_ROLE_HUB
#include <bluetooth/scan.h>
#endif

#include <stdio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include "platform/config.h"

#define LOG_MODULE_NAME posey_nus
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

/***
****      Buffers
****/

#if defined(CONFIG_ROLE_HUB)
#define MaxSensors 3
#define MaxConnections (MaxSensors + 1)
#define PCConnection (MaxConnections - 1)

static char * names[MaxConnections] = {
    "Posey w8 Thistle",
    "Posey w8 Flox",
    "Posey r2 Tulip",
    NULL // PC connection.
};
static struct bt_conn * connections[MaxConnections] = {
    NULL,
    NULL,
    NULL,
    NULL
};
#else
#define MaxSensors 0
#define MaxConnections (MaxSensors + 1)
#define PCConnection (MaxConnections - 1)

static char * names[MaxConnections] = {
    NULL // PC connection.
};
static struct bt_conn * connections[MaxConnections] = {
    NULL
};
#endif

static struct bt_nus_client nus_clients[MaxConnections];

static const char * scan_name = NULL;
static struct bt_conn * scan_conn = NULL;

static struct bt_data ad[2] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static int slot_from_conn(struct bt_conn * const conn)
{
    if (conn == NULL)
        return -1;

    for (int i = 0; i < MaxConnections; ++i)
    {
        if (conn == connections[i])
            return i;
    }

    return -1;
}

static int slot_from_name(const char * const name)
{
    for (int i = 0; i < MaxSensors; i++)
        if (strcmp(name, names[i])) return i;
    return PCConnection;
}

/***
****      GATT NUS service discovery.
****/

static void discovery_complete(
    struct bt_gatt_dm *dm,
    void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("NUS service discovery completed.");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(
    struct bt_conn *conn,
	void *context)
{
	LOG_INF("NUS service not found.");
}

static void discovery_error(
    struct bt_conn *conn,
	int err,
	void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

#ifdef CONFIG_ROLE_HUB
/***
****      Scanning.
****/

static void scan_filter_match(
    struct bt_scan_device_info *device_info,
	struct bt_scan_filter_match *filter_match,
    bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

    scan_name = filter_match->name.name;
	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Name: %s (slot %d) Address: %s connectable: %d",
		scan_name, slot_from_name(scan_name), addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Scan connecting failed");
}

static void scan_connecting(
    struct bt_scan_device_info *device_info,
    struct bt_conn *conn)
{
    LOG_INF("scan_connecting");
	scan_conn = conn;
}

BT_SCAN_CB_INIT(scan_cb,
    scan_filter_match,
    NULL, // nomatch
	scan_connecting_error,
    scan_connecting);

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

    // Add scan filters for the intended devices.
    for (int i = 0; i < MaxSensors; ++i)
    {
        LOG_INF("Connecting filter %d for device %s...", i, names[i]);
        err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, names[i]);
        if (err) {
            LOG_ERR("Filter %d cannot be set for device %s (err %d)",
            i, names[i], err);
            return err;
        }
    }
	
	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
    if (err){
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }

	LOG_INF("Scan module initialized");
	return err;
}

#endif

/***
****      (Dis)Connection, security.
****/

static void gatt_discover(const int id)
{
	int err;

    if ((id < 0) || (id >= MaxConnections))
    {
        LOG_WRN("Invalid slot ID (%d), skipping gatt discovery.", id);
        return;
    }

    LOG_INF("Running NUS discovery for %s",
        id == PCConnection ? "PC" : names[id]);
	err = bt_gatt_dm_start(
        connections[id],
		BT_UUID_NUS_SERVICE,
		&discovery_cb,
        &nus_clients[id]);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error code: %d", err);
	}
}

static void exchange_func(
    struct bt_conn *conn,
    uint8_t err,
    struct bt_gatt_exchange_params *params)
{
	if (!err) LOG_INF("MTU exchange done");
	else LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
}

static void connected(
    struct bt_conn * conn,
    uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);
		bt_conn_unref(conn);
		return;
	}

    #ifdef CONFIG_ROLE_HUB
    // Stop scanning while we handle this connection.
	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}
    #endif

    const char * name = (conn == scan_conn) ? scan_name : "Unknown";
    // if (conn == scan_conn) bt_conn_unref(scan_conn);
    LOG_INF("Name: %s", name);

    // Determine which slot this should go in.
    int slot = 0;
    for (int ci = 0; ci < MaxConnections; ++ci)
    {
        // If names match or we hit the last connection...
        LOG_INF("Comparing %d %s to %s...", ci, name, names[ci]);
        if ((ci == PCConnection) || (strcmp(name, names[ci]) == 0))
        {
            LOG_INF("Connected to %s at %s",
                ci == PCConnection ? "PC" : names[ci],
                addr);
            if (connections[ci] != NULL)
            {
                LOG_WRN("  -> Previous connection in slot %d will be released.", ci);
                bt_conn_disconnect(connections[ci], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                bt_conn_unref(connections[ci]);
            }
            connections[ci] = conn; //bt_conn_ref(conn);
            slot = ci;
            break;
        }
    }

    int sensor_connections = 0;
    for (int ci = 0; ci < MaxSensors; ++ci)
    {
        if (connections[ci] != NULL)
            ++sensor_connections;
    }

    // Exchange MTU config.
	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	err = bt_conn_set_security(conn, BT_SECURITY_L1);
	if (err) {
		LOG_WRN("Failed to set security: %d", err);
	}
    gatt_discover(slot);
    
    #ifdef CONFIG_ROLE_HUB
    // Restart scanning if necessary.
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);
    if (sensor_connections < MaxSensors)
    {
        LOG_INF("Restarting scanning.");
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        if ((err) && (err != -EALREADY)) LOG_ERR("Scanning failed to start (err %d)", err);
    }
    #endif

    // bt_conn_unref(conn);
}

static void disconnected(
    struct bt_conn * conn,
    uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
	int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Determine which slot is being disconnected.
    bool found = false;
    int sensor_connections = 0;
    for (int ci = 0; ci < MaxConnections; ++ci)
    {
        // If handles match...
        if (conn == connections[ci])
        {
            LOG_INF("Disconnected from %s at %s (reason: %x)",
                ci == PCConnection ? "PC" : names[ci],
                addr,
                reason);
            connections[ci] = NULL;
            found = true;
        }

        if ((ci != PCConnection) && (connections[ci] != NULL))
            ++sensor_connections;
    }

    if (!found)
    {
        LOG_WRN("Disconnecting from unknown device at %s (reason: %x)",
            addr, reason);
    }
    bt_conn_unref(conn);

    #ifdef CONFIG_ROLE_HUB
    // Restart scanning if necessary.
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);
    if (sensor_connections < MaxSensors)
    {
        LOG_INF("Restarting scanning.");
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        if ((err) && (err != -EALREADY)) LOG_ERR("Scanning failed to start (err %d)", err);
    }
    #endif
}

static void security_changed(
    struct bt_conn *conn,
    bt_security_t level,
	enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

/***
****      NUS
****/

static uint8_t bt_nus_pc_received(
    struct bt_nus_client *nus,
	const uint8_t *data,
    uint16_t len)
{
    // int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(nus->conn), addr, ARRAY_SIZE(addr));

    LOG_INF("Received %d bytes from PC %s. Ignoring.", len, addr);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bt_nus_sensor_received(
    struct bt_nus_client *nus,
	const uint8_t *data,
    uint16_t len)
{
    // int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(nus->conn), addr, ARRAY_SIZE(addr));
    const uint8_t slot = slot_from_conn(nus->conn);
    const char * name = "Unknown";
    if (slot < MaxSensors)
        name = names[slot];

    LOG_INF("Received %d bytes from SENSOR %s (slot %d, addr %s).",
        len, name, slot, addr);
    process_data(nus->conn, slot, data, len);

    return BT_GATT_ITER_CONTINUE;
}

static int nus_client_init(void)
{
	int err;

    struct bt_nus_client_init_param pc_init = {
		.cb = {
			.received = bt_nus_pc_received
		}
	};
	struct bt_nus_client_init_param sensor_init = {
		.cb = {
			.received = bt_nus_sensor_received
		}
	};

    for (int i = 0; i < MaxConnections; ++i)
    {
        err = bt_nus_client_init(
            &nus_clients[i],
            i == PCConnection ? &pc_init : &sensor_init);
        if (err) {
            LOG_ERR("NUS Client initialization failed for %s (slot %d, err %d)",
                i == PCConnection ? "PC" : "SENSOR",
                i, err);
            return err;
        }
    }

	LOG_INF("NUS Clients module initialized");
	return err;
}

int init_nus()
{
    int err = 0;

    LOG_INF("Initializing Flash, BLE, and NUS...");

    if (!init_flash())
    {
        LOG_ERR("Cannot initialize BLE w/o flash!");
        return -1;
    }

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return err;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization info callbacks.\n");
		return err;
	}

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Error bt_enable: %d", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    #ifdef CONFIG_ROLE_HUB
    err = scan_init();
    if (err) {
        LOG_ERR("Error scan_init: %d", err);
        return err;
    }
    #endif

    err = nus_client_init();
    if (err) {
        LOG_ERR("nus_client_init: Failed to initialize BLE UART service (err: %d)", err);
        return err;
    }

    // Update the BLE name using the device config.
    size_t name_len = strlen(device_config.name);
    ad[1].data = device_config.name;
    ad[1].data_len = name_len;

    bt_set_name(device_config.name);

    LOG_INF("Starting BLE advertising with name %s", ad[1].data);
    err = bt_le_adv_start(
        BT_LE_ADV_CONN,
        ad, ARRAY_SIZE(ad),
        sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

    #ifdef CONFIG_ROLE_HUB
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Scanning successfully started");
    #endif

    return 0;
}
