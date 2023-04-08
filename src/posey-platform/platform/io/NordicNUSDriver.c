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
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

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
    "<PC>" // PC connection.
};
static struct bt_conn * connections[MaxConnections] = {
    NULL,
    NULL,
    NULL,
    NULL
};
static uint32_t throughput[MaxConnections] = {
    0,
    0,
    0,
    0
};
static uint32_t last_update[MaxConnections] = {
    0,
    0,
    0,
    0
};
#else
#define MaxSensors 0
#define MaxConnections (MaxSensors + 1)
#define PCConnection (MaxConnections - 1)

static char * names[MaxConnections] = {
    "<PC>" // PC connection.
};
static struct bt_conn * connections[MaxConnections] = {
    NULL
};
static uint32_t throughput[MaxConnections] = {
    0
};
static uint32_t last_update[MaxConnections] = {
    0
};
#endif

static int num_connected_sensors()
{
    int sensor_connections = 0;
    for (int ci = 0; ci < MaxSensors; ++ci)
    {
        if (connections[ci] != NULL)
            ++sensor_connections;
    }
    return sensor_connections;
}

#define INTERVAL_MIN 0x4 /* 320 units, 400 ms */
#define INTERVAL_MAX 0x4 /* 320 units, 400 ms */
#define CONN_LATENCY 0

#define MIN_CONN_INTERVAL 6
#define MAX_CONN_INTERVAL 3200
#define SUPERVISION_TIMEOUT 1000

static struct bt_gatt_exchange_params mtu_exchange_params;

static struct test_params
{
    struct bt_le_conn_param *conn_param;
    struct bt_conn_le_phy_param *phy;
    struct bt_conn_le_data_len_param *data_len;
} test_params = {
    .conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, CONN_LATENCY,
                                   SUPERVISION_TIMEOUT),
    .phy = BT_CONN_LE_PHY_PARAM_2M,
    .data_len = BT_LE_DATA_LEN_PARAM_MAX};

static volatile bool data_length_req;
static const char *phy2str(uint8_t phy)
{
    switch (phy)
    {
    case 0:
        return "No packets";
    case BT_GAP_LE_PHY_1M:
        return "LE 1M";
    case BT_GAP_LE_PHY_2M:
        return "LE 2M";
    case BT_GAP_LE_PHY_CODED:
        return "LE Coded";
    default:
        return "Unknown";
    }
}

static int print_connection_info(const struct bt_conn *conn)
{
    struct bt_conn_info info = {0};
    int err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d", err);
        return err;
    }

    // Display some of the connection settings ... more can be displayed ...
    LOG_INF("Data Length TX:%d, RX:%d",
        info.le.data_len->tx_max_len,
        info.le.data_len->rx_max_len);
    LOG_INF("Phy TX:%d, RX: %d",
        info.le.phy->tx_phy,
        info.le.phy->rx_phy);

    return 0;
}

static struct bt_nus_client nus_clients[MaxSensors];

static const char * scan_name = NULL;
static struct bt_conn * scan_conn = NULL;

static struct bt_data ad[2] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static uint8_t slot_from_conn(struct bt_conn * const conn)
{
    if (conn == NULL)
        return 0x64; // 100

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

struct bt_conn * get_pc_connection()
{
    return connections[PCConnection];
}

/***
****      GATT NUS service discovery.
****/

static void mtu_exchange_updated(
    struct bt_conn *conn,
    uint8_t err,
    struct bt_gatt_exchange_params *params)
{
    if (!err)
    {
        LOG_INF("MTU exchange done. MTU = %d", bt_gatt_get_mtu(conn));
    }
    else
    {
        LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
    }
}

static int connection_configuration_set(
    struct bt_conn *conn,
    const struct bt_le_conn_param *conn_param,
    const struct bt_conn_le_phy_param *phy,
    const struct bt_conn_le_data_len_param *data_len)
{
    int err = print_connection_info(conn);
    if (err) return err;

    // Define time for negotiation to occur ...
    #define LINK_UPDATE_TIMEOUT K_SECONDS(5)

    err = bt_conn_le_phy_update(conn, phy);
    if (err)
    {
        LOG_WRN("PHY update failed: %d", err);
        // return err;
    }

    LOG_INF("PHY update pending");

    struct bt_conn_info info = {0};
    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d", err);
        return err;
    }

    if (info.le.data_len->tx_max_len != data_len->tx_max_len)
    {
        data_length_req = true;

        err = bt_conn_le_data_len_update(conn, data_len);
        if (err)
        {
            LOG_WRN("LE data length update failed: %d", err);
            // return err;
        }

        LOG_INF("LE Data length update pending");
    }

    if (info.le.interval != conn_param->interval_max)
    {
        err = bt_conn_le_param_update(conn, conn_param);
        if (err)
        {
            LOG_WRN("Connection parameters update failed: %d", err);
            // return err;
        }

        LOG_INF("Connection parameters update pending");
    }

    // So if we want a different MTU than default, we need to exchange with the peer.
    // Important understandings found in:
    //  https://devzone.nordicsemi.com/f/nordic-q-a/81860/set-mtu-size-in-zephyr/341062#341062
    //
    // In our proj.conf, we already have CONFIG_BT_L2CAP_TX_MTU=247, but that only
    //  seems to be referenced within functions of bt_gatt_exchange_mtu()
    // Setup callback to capture MTU exchange status with Peer.
    mtu_exchange_params.func = mtu_exchange_updated;
    err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (err)
    {
        LOG_WRN("MTU exchange failed (err %d)", err);
        // return err;
    }

    LOG_INF("MTU exchange pending");

    // If we are here then all the connection negotiations completed succesfully
    return 0;
}

static int scan_start(const bool active);

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

    int sensor_connections = num_connected_sensors();
    int err = scan_start(sensor_connections < MaxSensors);
	if (err) LOG_ERR("Scanning failed to start (err %d)", err);

    // int err = connection_configuration_set(
    //     nus->conn,
    //     test_params.conn_param,
    //     test_params.phy,
    //     test_params.data_len);
    // if (err) {
    //     LOG_WRN("Error in connection_configuration_set()");
    // }
}

static void discovery_service_not_found(
    struct bt_conn *conn,
	void *context)
{
	LOG_INF("NUS service not found.");

    // Restart active scanning if necessary.
    int sensor_connections = num_connected_sensors();
    int err = scan_start(sensor_connections < MaxSensors);
	if (err) LOG_ERR("Scanning failed to start (err %d)", err);
}

static void discovery_error(
    struct bt_conn *conn,
	int err,
	void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);

    int sensor_connections = num_connected_sensors();
    err = scan_start(sensor_connections < MaxSensors);
	if (err) LOG_ERR("Scanning failed to start (err %d)", err);
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
    LOG_INF("NordicNUSDriver::scan_filter_match");
    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    // Matching a device by name?
    if (filter_match->name.match)
    {
        scan_name = filter_match->name.name;

        LOG_INF("Filters matched. Name: %s (slot %d) Address: %s connectable: %d",
            scan_name, slot_from_name(scan_name), addr, connectable);
    }

    // Why did we match?
    else
    {
        LOG_WRN("Not sure why we matched address %s", addr);
    }
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Scan connecting failed");

    // int sensor_connections = num_connected_sensors();
    // int err = scan_start(sensor_connections < MaxSensors);
	// if (err) LOG_ERR("Scanning failed to start (err %d)", err);
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

static int scan_start(const bool active)
{
    LOG_INF("NordicNUSDriver::scan_start %s",
        active ? "active" : "passive");

    static const uint16_t BT_SCAN_INTERVAL = BT_GAP_SCAN_FAST_INTERVAL;
    static const uint16_t BT_SCAN_WINDOW   = BT_GAP_SCAN_FAST_WINDOW;

    struct bt_le_scan_param scan_param = {
        .type       = active ? BT_LE_SCAN_TYPE_ACTIVE : BT_LE_SCAN_TYPE_PASSIVE,
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval   = BT_SCAN_INTERVAL,
        .window     = BT_SCAN_WINDOW,
    };
    
    int err = bt_le_scan_start(&scan_param, BLE_Zephyr_callback);
    if (err)
    {
        LOG_ERR("BLE scanning could not start. (err %d)", err);
    }

    return err;
}

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

    LOG_INF("NordicNUSDriver::scan_init");

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
        LOG_ERR("Device name filters cannot be turned on (err %d)", err);
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
    LOG_INF("NordicNUSDriver::gatt_discover");

    if ((id < 0) || (id >= MaxSensors))
    {
        LOG_WRN("Invalid slot ID (%d), skipping gatt discovery.", id);
        return;
    }

    LOG_INF("Running NUS discovery for %s", names[id]);
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

    LOG_INF("NordicNUSDriver::connected");

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

        // Need to dereference on of our connection slots?
        uint8_t slot = slot_from_conn(conn);
        if (slot < MaxConnections)
        {
            LOG_INF("  -> Releasing connection slot %d", slot);
            bt_conn_unref(connections[slot]);
            connections[slot] = NULL;
        }
		return;
	}

    const char * name = (conn == scan_conn) ? scan_name : "Unknown";
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
            if ((connections[ci] != NULL) && (connections[ci] != conn))
            {
                LOG_WRN("  -> Previous connection in slot %d will be released.", ci);
                bt_conn_disconnect(connections[ci], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                bt_conn_unref(connections[ci]);
            }
            connections[ci] = bt_conn_ref(conn);
            slot = ci;
            break;
        }
    }

    int sensor_connections = num_connected_sensors();

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

    #ifdef CONFIG_ROLE_HUB
    if (slot < MaxSensors) gatt_discover(slot);
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);
    #endif
}

static void disconnected(
    struct bt_conn * conn,
    uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
	int err;

    LOG_INF("NordicNUSDriver::disconnected");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Determine which slot is being disconnected.
    bool found = false;
    bool sensor_disconnected = false;
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
            bt_conn_unref(connections[ci]);
            connections[ci] = NULL;
            found = true;

            if (ci < MaxSensors) sensor_disconnected = true;
        }

        if ((ci != PCConnection) && (connections[ci] != NULL))
            ++sensor_connections;
    }

    if (!found)
    {
        LOG_WRN("Disconnecting from unknown device at %s (reason: %x)",
            addr, reason);
    }

    #ifdef CONFIG_ROLE_HUB
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);

    // Restart active scanning if necessary.
    err = scan_start(sensor_connections < MaxSensors);
    if (err) LOG_ERR("Scanning failed to start (err %d)", err);
    #endif
}

static void security_changed(
    struct bt_conn *conn,
    bt_security_t level,
	enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::security_changed");
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
    }
}

static bool le_param_req(
    struct bt_conn *conn,
    struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.");
	LOG_INF("  - Connection interval: [%d, %d]",
	       param->interval_min, param->interval_max);
	LOG_INF("  - Latency: %d", param->latency);
    LOG_INF("  - Timeout: %d", param->timeout);

	return true;
}

static void le_param_updated(
    struct bt_conn *conn,
    uint16_t interval,
	uint16_t latency,
    uint16_t timeout)
{
	LOG_INF("Connection parameters updated:");
    LOG_INF("  - Interval: %d; Latency: %d; Timeout: %d",
	       interval, latency, timeout);
}

static void le_phy_updated(
    struct bt_conn *conn,
    struct bt_conn_le_phy_info *param)
{
    LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s",
            phy2str(param->tx_phy), phy2str(param->rx_phy));
}

static void le_data_length_updated(
    struct bt_conn *conn,
    struct bt_conn_le_data_len_info *info)
{
    if (!data_length_req)
    {
        return;
    }

    LOG_INF("LE data len updated: TX (len: %d time: %d)"
            " RX (len: %d time: %d)",
            info->tx_max_len,
            info->tx_max_time, info->rx_max_len, info->rx_max_time);

    data_length_req = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_length_updated,
	.security_changed = security_changed
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::auth_cancel");
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::pairing_complete");
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::pairing_failed");
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

static uint8_t bt_nus_sensor_received(
    struct bt_nus_client *nus,
	const uint8_t *data,
    uint16_t len)
// static uint8_t bt_nus_sensor_received(
//     struct bt_conn *nus,
//     const uint8_t slot,
// 	const uint8_t *data,
//     uint16_t len)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(nus->conn), addr, ARRAY_SIZE(addr));
    const uint8_t slot = slot_from_conn(nus->conn);
    const char * name = "Unknown";
    if (slot < MaxSensors)
    {
        name = names[slot];
        uint32_t now = k_cyc_to_ms_floor32(sys_clock_tick_get_32());
        float dt = (now - last_update[slot])/1.0e3;
        throughput[slot] += len;
        if (last_update[slot] == 0)
        {
            last_update[slot] = now;
        }
        else if (dt > 5.0)
        {
            LOG_INF("Slot %d bandwidth: %d B in %.2fs - %.2f KBps",
                slot, throughput[slot], dt, throughput[slot] / 1024.0 / dt);
            last_update[slot] = now;
            throughput[slot] = 0;

            static int iter = 0;
            if (++iter % 12 == 0)
                print_connection_info(nus->conn);
        }
    }

    LOG_DBG("Received %d bytes from SENSOR %s (slot %d, addr %s).",
        len, name, slot, addr);

    #if defined(CONFIG_ROLE_HUB)
    process_data(nus->conn, slot, data, len);
    #endif

    return BT_GATT_ITER_CONTINUE;
}

// static void bt_nus_dispatch_received(
// 	struct bt_conn *conn,
// 	const uint8_t *data,
//     uint16_t len)
// {
//     LOG_INF("Dispatch called for %d bytes", len);
//     const uint8_t slot = slot_from_conn(conn);
//     if (slot < MaxSensors)
//         bt_nus_sensor_received(conn, slot, data, len);
//     else
//         bt_nus_pc_received(conn, data, len);
// }

static int nus_client_init(void)
{
	int err = 0;

    LOG_INF("NordicNUSDriver::nus_client_init");
	struct bt_nus_client_init_param sensor_init = {
		.cb = {
			.received = bt_nus_sensor_received
		}
	};

    // Only the sensors need NUS clients. The PC communication will not find
    // a NUS GATT service.
    for (int i = 0; i < MaxSensors; ++i)
    {
        err = bt_nus_client_init(&nus_clients[i], &sensor_init);
        if (err) {
            LOG_ERR("NUS Client initialization failed for sensor (slot %d, err %d)",
                i, err);
            return err;
        }
    }

    #if defined(CONFIG_ROLE_HUB)
    // Initialize a general callback for PC connections. This is only needed for
    // the hub units which can accept data input. The peripheral sensors can
    // just ignore it.
    struct bt_nus_cb nus_cb = {
        .received = bt_nus_pc_received
    };
	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("bt_nus_init: Failed to initialize UART service (err: %d)", err);
		return err;
	}
    #endif

	LOG_INF("NUS Clients module initialized");
	return err;
}

int init_nus()
{
    int err = 0;

    #if defined(CONFIG_ROLE_HUB)
    LOG_INF("Initializing Flash...");
    if (!init_flash())
    {
        LOG_ERR("Cannot initialize BLE w/o flash!");
        return -1;
    }
    #endif

    LOG_INF("Initializing BLE and NUS...");

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
	// err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    err = scan_start(true);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Scanning successfully started");
    #endif

    return 0;
}
