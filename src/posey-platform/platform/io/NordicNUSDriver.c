#include "NordicNUSDriver.h"

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <soc.h>
#include <zephyr/device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/mds.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>

// #include <memfault/metrics/metrics.h>
// #include <memfault/core/trace_event.h>

#ifdef CONFIG_ROLE_HUB
    #include <bluetooth/scan.h>
#endif

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "platform/config.h"

#define LOG_MODULE_NAME posey_nus
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/***
****      Buffers
****/

#if defined(CONFIG_ROLE_HUB)
    #define MaxSensors 3
    #define MaxConnections (MaxSensors + 1)
    #define PCConnection (MaxConnections - 1)

// ATW: Fix this.
static const char* names[MaxConnections] = {
    // "Posey r2 Tulip",           // Flox
    // "Posey w8 Thistle",      // Carnation
    "Posey r2 Indigo",  // Aster
    "Posey xx Unknown", "Posey xx Unknown",
    "<PC>"  // PC connection.
};
static struct bt_conn* connections[MaxConnections] = {NULL, NULL, NULL, NULL};
static uint32_t throughput[MaxConnections] = {0, 0, 0, 0};
static uint32_t last_update[MaxConnections] = {0, 0, 0, 0};
#else
    #define MaxSensors 0
    #define MaxConnections (MaxSensors + 1)
    #define PCConnection (MaxConnections - 1)

static char* names[MaxConnections] = {
    "<PC>"  // PC connection.
};
static struct bt_conn* connections[MaxConnections] = {NULL};
// static uint32_t throughput[MaxConnections] = {0};
// static uint32_t last_update[MaxConnections] = {0};
#endif

int num_connected_sensors() {
    int sensor_connections = 0;
    for (int ci = 0; ci < MaxSensors; ++ci) {
        if (connections[ci] != NULL)
            ++sensor_connections;
    }
    return sensor_connections;
}

#if defined(CONFIG_ROLE_HUB)
static struct bt_gatt_exchange_params mtu_exchange_params;
#endif

static volatile bool data_length_req;
static const char* phy2str(uint8_t phy) {
    switch (phy) {
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

#if defined(CONFIG_ROLE_HUB)
static int print_connection_info(const struct bt_conn* conn) {
    struct bt_conn_info info = {0};
    int err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info %d", err);
        return err;
    }

    // Display some of the connection settings ... more can be displayed ...
    LOG_INF(
        "Data Length TX:%d, RX:%d", info.le.data_len->tx_max_len,
        info.le.data_len->rx_max_len);
    LOG_INF("Phy TX:%d, RX: %d", info.le.phy->tx_phy, info.le.phy->rx_phy);

    return 0;
}

static struct bt_nus_client nus_clients[MaxSensors];
#endif

static char scan_name[30];
static struct bt_conn* scan_conn = NULL;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL)};

static struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)};

static uint8_t slot_from_conn(struct bt_conn* const conn) {
    if (conn == NULL)
        return 0x64;  // 100

    for (int i = 0; i < MaxConnections; ++i) {
        if (conn == connections[i])
            return i;
    }

    return -1;
}

static int slot_from_name(const char* const name) {
    for (int i = 0; i < MaxSensors; i++)
        if (strcmp(name, names[i]))
            return i;
    return PCConnection;
}

struct bt_conn* get_pc_connection() {
    return connections[PCConnection];
}

static bool mds_access_enable(struct bt_conn* conn) {
    if (slot_from_conn(conn) == PCConnection) {
        LOG_INF("Allowing access to MDS for PC connection.");
        return true;
    }

    LOG_INF("Denying access to MDS for sensor connection.");
    return false;
}

static const struct bt_mds_cb mds_cb = {
    .access_enable = mds_access_enable,
};

#if defined(CONFIG_ROLE_HUB)

/***
****      GATT NUS service discovery.
****/

// static void mtu_exchange_updated(
//     struct bt_conn *conn,
//     uint8_t err,
//     struct bt_gatt_exchange_params *params)
// {
//     if (!err)
//     {
//         LOG_INF("MTU exchange done. MTU = %d", bt_gatt_get_mtu(conn));
//     }
//     else
//     {
//         LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
//     }
// }

// static int connection_configuration_set(
//     struct bt_conn *conn,
//     const struct bt_le_conn_param *conn_param,
//     const struct bt_conn_le_phy_param *phy,
//     const struct bt_conn_le_data_len_param *data_len)
// {
//     int err = print_connection_info(conn);
//     if (err) return err;

//     err = bt_conn_le_phy_update(conn, phy);
//     if (err)
//     {
//         LOG_WRN("PHY update failed: %d", err);
//         // return err;
//     }

//     LOG_INF("PHY update pending");

//     struct bt_conn_info info = {0};
//     err = bt_conn_get_info(conn, &info);
//     if (err)
//     {
//         LOG_ERR("Failed to get connection info %d", err);
//         return err;
//     }

//     if (info.le.data_len->tx_max_len != data_len->tx_max_len)
//     {
//         data_length_req = true;

//         err = bt_conn_le_data_len_update(conn, data_len);
//         if (err)
//         {
//             LOG_WRN("LE data length update failed: %d", err);
//             // return err;
//         }

//         LOG_INF("LE Data length update pending");
//     }

//     if (info.le.interval != conn_param->interval_max)
//     {
//         err = bt_conn_le_param_update(conn, conn_param);
//         if (err)
//         {
//             LOG_WRN("Connection parameters update failed: %d", err);
//             // return err;
//         }

//         LOG_INF("Connection parameters update pending");
//     }

//     // So if we want a different MTU than default, we need to exchange with
//     the peer.
//     // Important understandings found in:
//     //
//     https://devzone.nordicsemi.com/f/nordic-q-a/81860/set-mtu-size-in-zephyr/341062#341062
//     //
//     // In our proj.conf, we already have CONFIG_BT_L2CAP_TX_MTU=247, but that
//     only
//     //  seems to be referenced within functions of bt_gatt_exchange_mtu()
//     // Setup callback to capture MTU exchange status with Peer.
//     mtu_exchange_params.func = mtu_exchange_updated;
//     err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
//     if (err)
//     {
//         LOG_WRN("MTU exchange failed (err %d)", err);
//         // return err;
//     }

//     LOG_INF("MTU exchange pending");

//     // If we are here then all the connection negotiations completed
//     succesfully return 0;
// }

static bool scanning_enabled = true;
static int scan_start();
static void scan_stop();

static bool sensors_enabled = true;

void disable_sensors() {
    LOG_WRN("Disabling sensors");
    sensors_enabled = false;

    // Restart scanning in passive mode.
    scan_start();

    // Disconnect any connected sensor devices.
    for (int i = 0; i < MaxSensors; ++i) {
        if (connections[i] != NULL) {
            LOG_WRN("Disconnecting sensor %d %s", i, names[i]);
            bt_conn_disconnect(
                connections[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
    }
}

void enable_sensors() {
    LOG_WRN("Enabling sensors");
    sensors_enabled = true;

    // Restart scanning in active mode.
    scan_start();
}

static void discovery_complete(struct bt_gatt_dm* dm, void* context) {
    struct bt_nus_client* nus = (struct bt_nus_client*)context;
    LOG_INF("NUS service discovery completed.");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);

    scan_start();
}

static void discovery_service_not_found(struct bt_conn* conn, void* context) {
    LOG_INF("NUS service not found.");
}

static void discovery_error(struct bt_conn* conn, int err, void* context) {
    LOG_WRN(
        "Error while discovering GATT service: (%d - %s)", err, strerror(err));
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found = discovery_error,
};

/***
****      Scanning.
****/

static void scan_filter_match(
    struct bt_scan_device_info* device_info,
    struct bt_scan_filter_match* filter_match,
    bool connectable) {
    char addr[BT_ADDR_LE_STR_LEN];
    LOG_INF("NordicNUSDriver::scan_filter_match");
    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    // Matching a device by name?
    if (filter_match->name.match) {
        memcpy(scan_name, filter_match->name.name, filter_match->name.len);
        scan_name[filter_match->name.len] = '\0';

        LOG_INF(
            "Filters matched. Name: %s (slot %d) Address: %s connectable: %d",
            scan_name, slot_from_name(scan_name), addr, connectable);
    }

    // Why did we match?
    else {
        LOG_WRN("Not sure why we matched address %s", addr);
    }
}

static void scan_connecting_error(struct bt_scan_device_info* device_info) {
    LOG_WRN("Scan connecting failed");
    scan_start();
}

static void scan_connecting(
    struct bt_scan_device_info* device_info, struct bt_conn* conn) {
    LOG_INF("scan_connecting");
    scan_conn = conn;
}

BT_SCAN_CB_INIT(
    scan_cb,
    scan_filter_match,
    NULL,  // nomatch
    scan_connecting_error,
    scan_connecting);

static int scan_start() {
    #if defined(CONFIG_ROLE_HUB)

    scan_stop();
    if (!scanning_enabled)
        return -1;

    int err;
    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    int sensors_connected = num_connected_sensors();
    // ATW: Passive after a single connection.
    if ((!sensors_enabled) || (sensors_connected >= 1))  // MaxSensors))
    {
        LOG_INF(
            "BLE scanning: Connected to %d sensors. Passive scanning.",
            sensors_connected);
    } else {
        LOG_INF(
            "BLE scanning: Connected to %d sensors. Active scanning.",
            sensors_connected);

        // Add scan filters for the intended devices.
        for (int i = 0; i < MaxSensors; ++i) {
            if (connections[i] != NULL)
                continue;
            err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, names[i]);
            if (err) {
                LOG_ERR(
                    "Filter %d cannot be set for device %s (err %d - %s)", i,
                    names[i], err, strerror(err));
                return err;
            }
        }

        err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
        if (err) {
            LOG_ERR(
                "Device name filters cannot be turned on (err %d, %s)", err,
                strerror(err));
            return err;
        }

        scan_param.type = BT_LE_SCAN_TYPE_ACTIVE;
    }

    err = bt_le_scan_start(&scan_param, BLE_Zephyr_callback);
    if (err) {
        LOG_ERR(
            "BLE scanning could not start. (err %d, %s)", err, strerror(err));
        return err;
    }

    #endif

    return 0;
}

static void scan_stop() {
    LOG_INF("NordicNUSDriver::scan_stop");

    bt_scan_filter_disable();
    bt_scan_filter_remove_all();

    int err = bt_le_scan_stop();
    if ((err != 0) && (err != -EALREADY)) {
        LOG_ERR("Stop LE scan failed (err %d, %s)", err, strerror(err));
    }
}

static int scan_init(void) {
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };

    LOG_INF("NordicNUSDriver::scan_init");

    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    LOG_INF("Scan module initialized");
    return 0;
}

/***
****      (Dis)Connection, security.
****/

static void gatt_discover(const int id) {
    int err;
    LOG_INF("NordicNUSDriver::gatt_discover");

    if ((id < 0) || (id >= MaxSensors)) {
        LOG_WRN("Invalid slot ID (%d), skipping gatt discovery.", id);
        return;
    }

    LOG_INF("Running NUS discovery for %s", names[id]);
    struct bt_uuid_128 nus_uuid = BT_UUID_INIT_128(BT_UUID_NUS_VAL);
    err = bt_gatt_dm_start(
        connections[id], (struct bt_uuid*)&nus_uuid, &discovery_cb,
        &nus_clients[id]);
    if (err) {
        LOG_ERR(
            "could not start the discovery procedure, error code: %d (%s)", err,
            strerror(err));
    }
}

static void exchange_func(
    struct bt_conn* conn, uint8_t err, struct bt_gatt_exchange_params* params) {
    if (!err)
        LOG_INF("MTU exchange done");
    else
        LOG_WRN(
            "MTU exchange failed (err %" PRIu8 " - %s)", err, strerror(err));
}

#endif

static void connected(struct bt_conn* conn, uint8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::connected");

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    if (conn_err) {
        LOG_INF(
            "Failed to connect to %s (%d - %s)", addr, conn_err,
            strerror(conn_err));

        // Need to dereference on of our connection slots?
        uint8_t slot = slot_from_conn(conn);
        if (slot < MaxConnections) {
            LOG_INF("  -> Releasing connection slot %d", slot);
            bt_conn_unref(connections[slot]);
            connections[slot] = NULL;
        }

#if defined(CONFIG_ROLE_HUB)
        scan_start();
#endif

        return;
    }

    const char* name = (conn == scan_conn) ? scan_name : "Unknown";
    LOG_INF("Name: %s", name);

    // Determine which slot this should go in.
    int slot = 0;
    for (int ci = 0; ci < MaxConnections; ++ci) {
        // If names match or we hit the last connection...
        LOG_DBG("Comparing %d %s to %s...", ci, name, names[ci]);
        if ((ci == PCConnection) || (strcmp(name, names[ci]) == 0)) {
            LOG_INF(
                "Connected to %s at %s", ci == PCConnection ? "PC" : names[ci],
                addr);
            if ((connections[ci] != NULL) && (connections[ci] != conn)) {
                LOG_WRN(
                    "  -> Previous connection in slot %d will be released.",
                    ci);
                bt_conn_disconnect(
                    connections[ci], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                bt_conn_unref(connections[ci]);
            }
            connections[ci] = bt_conn_ref(conn);
            slot = ci;
            break;
        }
    }

#ifdef CONFIG_ROLE_HUB
    int err;
    int sensor_connections = num_connected_sensors();

    // Exchange MTU config.
    static struct bt_gatt_exchange_params exchange_params;
    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);

    if (err) {
        LOG_WRN("MTU exchange failed (err %d - %s)", err, strerror(err));
    }

    err = bt_conn_set_security(conn, BT_SECURITY_L1);
    if (err) {
        LOG_WRN("Failed to set security: %d (%s)", err, strerror(err));
    }

    if (slot < MaxSensors) {
        scan_stop();
        gatt_discover(slot);
    }
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);
#endif
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::disconnected");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Determine which slot is being disconnected.
    bool found = false;
    bool sensor_disconnected = false;
    int sensor_connections = 0;
    for (int ci = 0; ci < MaxConnections; ++ci) {
        // If handles match...
        if (conn == connections[ci]) {
            // err can mean either of the following:
            // - BT_HCI_ERR_UNKNOWN_CONN_ID
            // Creating the connection started by bt_conn_le_create
            // canceled either by the user through bt_conn_disconnect
            // or by the timeout in the host through bt_conn_le_create_param
            // timeout parameter, which defaults to
            // @kconfig{CONFIG_BT_CREATE_CONN_TIMEOUT} seconds.
            // - BT_HCI_ERR_ADV_TIMEOUT
            // High duty cycle directed connectable advertiser started by
            // bt_le_adv_start failed to be connected within the timeout.
            const char* reason_str = "Unknown";
            switch (reason) {
                case BT_HCI_ERR_UNKNOWN_CONN_ID:
                    reason_str = "Unknown connection ID";
                    break;
                case BT_HCI_ERR_CONN_FAIL_TO_ESTAB:
                    reason_str = "Connection failed to establish";
                    break;
                case BT_HCI_ERR_ADV_TIMEOUT:
                    reason_str = "Advertising timeout";
                    break;
                case BT_HCI_ERR_CONN_TIMEOUT:
                    reason_str = "Connection timeout";
                    break;
            }
            LOG_WRN(
                "Disconnected from %s at %s (reason: %x - %s)",
                ci == PCConnection ? "PC" : names[ci], addr, reason,
                reason_str);
            bt_conn_unref(connections[ci]);
            connections[ci] = NULL;
            found = true;

            if (ci < MaxSensors)
                sensor_disconnected = true;
        }

        if ((ci != PCConnection) && (connections[ci] != NULL))
            ++sensor_connections;
    }

    if (!found) {
        LOG_WRN(
            "Disconnecting from unknown device at %s (reason: %x)", addr,
            reason);
    }

#ifdef CONFIG_ROLE_HUB
    int err;
    LOG_INF("%d of %d sensors connected.", sensor_connections, MaxSensors);

    // Restart active scanning if necessary.
    err = scan_start();
    if (err)
        LOG_ERR("Scanning failed to start (err %d - %s)", err, strerror(err));
#endif
}

static bool le_param_req(struct bt_conn* conn, struct bt_le_conn_param* param) {
    LOG_INF("Connection parameters update request received.");
    LOG_INF(
        "  - Connection interval: [%d, %d]", param->interval_min,
        param->interval_max);
    LOG_INF("  - Latency: %d", param->latency);
    LOG_INF("  - Timeout: %d", param->timeout);

    return true;
}

static void le_param_updated(
    struct bt_conn* conn,
    uint16_t interval,
    uint16_t latency,
    uint16_t timeout) {
    LOG_INF("Connection parameters updated:");
    LOG_INF(
        "  - Interval: %d; Latency: %d; Timeout: %d", interval, latency,
        timeout);
}

static void le_phy_updated(
    struct bt_conn* conn, struct bt_conn_le_phy_info* param) {
    LOG_INF(
        "LE PHY updated: TX PHY %s, RX PHY %s", phy2str(param->tx_phy),
        phy2str(param->rx_phy));
}

static void le_data_length_updated(
    struct bt_conn* conn, struct bt_conn_le_data_len_info* info) {
    if (!data_length_req) {
        return;
    }

    LOG_INF(
        "LE data len updated: TX (len: %d time: %d)"
        " RX (len: %d time: %d)",
        info->tx_max_len, info->tx_max_time, info->rx_max_len,
        info->rx_max_time);

    data_length_req = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_req = le_param_req,
    .le_param_updated = le_param_updated,
#if defined(CONFIG_ROLE_HUB)
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_length_updated
#endif
};

static void auth_cancel(struct bt_conn* conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::auth_cancel");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn* conn, bool bonded) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::pairing_complete");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn* conn, enum bt_security_err reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    LOG_INF("NordicNUSDriver::pairing_failed");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, .pairing_failed = pairing_failed};

/***
****      NUS
****/

#if defined(CONFIG_ROLE_HUB)
static uint8_t bt_nus_sensor_received(
    struct bt_nus_client* nus, const uint8_t* data, uint16_t len) {
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(nus->conn), addr, ARRAY_SIZE(addr));
    const uint8_t slot = slot_from_conn(nus->conn);
    const char* name = "Unknown";
    if (slot < MaxSensors) {
        name = names[slot];
        uint32_t now = k_cyc_to_ms_floor32(sys_clock_tick_get_32());
        float dt = (now - last_update[slot]) / 1.0e3;
        throughput[slot] += len;
        if (last_update[slot] == 0) {
            last_update[slot] = now;
        } else if (dt > 5.0) {
            LOG_INF(
                "Slot %d bandwidth: %d B in %.2fs - %.2f KBps", slot,
                throughput[slot], dt, throughput[slot] / 1024.0 / dt);
            last_update[slot] = now;
            throughput[slot] = 0;
        }
    }

    LOG_DBG(
        "Received %d bytes from SENSOR %s (slot %d, addr %s).", len, name, slot,
        addr);

    #if defined(CONFIG_ROLE_HUB)
    process_data(nus->conn, slot, data, len);
    #endif

    return BT_GATT_ITER_CONTINUE;
}
#endif

#if defined(CONFIG_ROLE_HUB)
static int nus_client_init(void) {
    int err = 0;

    LOG_INF("NordicNUSDriver::nus_client_init");
    struct bt_nus_client_init_param sensor_init = {
        .cb = {.received = bt_nus_sensor_received}};

    // Only the sensors need NUS clients. The PC communication will not find
    // a NUS GATT service.
    for (int i = 0; i < MaxSensors; ++i) {
        err = bt_nus_client_init(&nus_clients[i], &sensor_init);
        if (err) {
            LOG_ERR(
                "NUS Client initialization failed for sensor (slot %d, err %d "
                "- %s)",
                i, err, strerror(err));
            return err;
        }
    }

    // Initialize a general callback for PC connections. This is only needed for
    // the hub units which can accept data input. The peripheral sensors can
    // just ignore it.
    struct bt_nus_cb nus_cb = {.received = bt_nus_pc_received};
    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR(
            "bt_nus_init: Failed to initialize UART service (err: %d, %s)", err,
            strerror(err));
        return err;
    }

    LOG_INF("NUS Clients module initialized");
    return err;
}
#endif

int init_nus() {
    int err = 0;

#if defined(CONFIG_ROLE_HUB)
    LOG_INF("Initializing Flash...");
    if (!init_flash()) {
        LOG_ERR("Cannot initialize BLE w/o flash!");
        return -1;
    }
#endif

    // LOG_INF("Initializing MDS");
    // err = bt_mds_cb_register(&mds_cb);
    // if (err) {
    // 	LOG_ERR("Memfault Diagnostic service callback registration failed (err
    // %d - %s)\n", err, strerror(err)); 	return err;
    // }

    LOG_INF("Initializing BLE and NUS...");

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        LOG_ERR(
            "Failed to register authorization callbacks. %d - %s", err,
            strerror(err));
        return err;
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        LOG_ERR(
            "Failed to register authorization info callbacks. %d - %s", err,
            strerror(err));
        return err;
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Error bt_enable: %d - %s", err, strerror(err));
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

    err = nus_client_init();
    if (err) {
        LOG_ERR(
            "nus_client_init: Failed to initialize BLE UART service (err: %d - "
            "%s)",
            err, strerror(err));
        return err;
    }
#endif

    // Update the BLE name using the device config.
    size_t name_len = strlen(device_config.name);
    sd[0].data = (uint8_t*)device_config.name;
    sd[0].data_len = name_len;

    bt_set_name(device_config.name);

    LOG_INF("Starting BLE advertising with name %s", sd[0].data);
    err =
        bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

#ifdef CONFIG_ROLE_HUB
    // err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    err = scan_start();
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    LOG_INF("Scanning successfully started");
#endif

    return 0;
}

void close_connections() {
    for (int i = 0; i < MaxConnections; ++i) {
        if (connections[i]) {
            bt_conn_disconnect(
                connections[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            bt_conn_unref(connections[i]);
            connections[i] = NULL;
        }
    }
}

void disable_scanning() {
#if defined(CONFIG_ROLE_HUB)
    scanning_enabled = false;
    scan_stop();
#endif
}

void enable_scanning() {
#if defined(CONFIG_ROLE_HUB)
    scanning_enabled = true;
    scan_start();
#endif
}
