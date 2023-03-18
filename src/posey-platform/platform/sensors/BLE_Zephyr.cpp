#include "platform.hpp"

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "posey-platform/platform/sensors/BLE_Zephyr.hpp"

LOG_MODULE_REGISTER(BLE_Zephyr);

BLE_Zephyr * BLE_Zephyr::reference = nullptr;

static void BLE_Zephyr_callback(
	const bt_addr_le_t *addr,
	int8_t rssi,
	uint8_t type,
	struct net_buf_simple *ad)
{
    // ATW: TODO: Change to iBeacon detection.
    if (BLE_Zephyr::reference)
    {
        BLE_Zephyr::reference->add_detection(
            Clock::get_usec<uint32_t>(),
            addr->a.val,
            rssi);
    }
}

BLE_Zephyr::BLE_Zephyr()
{
}

bool BLE_Zephyr::setup()
{
    if (reference) return false;
    else
    {
        reference = this;

        // static const uint16_t BT_PERIOD_1000ms = 0x0650;
        // static const uint16_t BT_PERIOD_100ms  = 0x00a0;

        // static const uint16_t BT_SCAN_INTERVAL = BT_PERIOD_1000ms;
        // static const uint16_t BT_SCAN_WINDOW   = BT_PERIOD_100ms;

        static const uint16_t BT_SCAN_INTERVAL = BT_GAP_SCAN_FAST_INTERVAL;
        static const uint16_t BT_SCAN_WINDOW   = BT_GAP_SCAN_FAST_WINDOW;

        struct bt_le_scan_param scan_param = {
            .type       = BT_LE_SCAN_TYPE_PASSIVE,
            .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
            .interval   = BT_SCAN_INTERVAL,
            .window     = BT_SCAN_WINDOW,
        };

        int err = bt_le_scan_start(&scan_param, BLE_Zephyr_callback);
        if (err)
        {
            LOG_ERR("BLE scanning could not start. (err %d)", err);
            return false;
        }

        return true;
    }
}
