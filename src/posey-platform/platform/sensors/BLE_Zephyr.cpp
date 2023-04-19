#include "platform.hpp"

#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/logging/log.h>

#include "posey-platform/platform/sensors/BLE_Zephyr.hpp"
#include "posey-platform/platform/io/NordicNUSDriver.h"

LOG_MODULE_REGISTER(BLE_Zephyr);

BLE_Zephyr * BLE_Zephyr::reference = nullptr;

extern "C" void BLE_Zephyr_callback(
	const bt_addr_le_t *addr,
	int8_t rssi,
	uint8_t type,
	struct net_buf_simple *ad)
{
    // ATW: TODO: Change to iBeacon detection.
    if (BLE_Zephyr::reference)
    {
        uint8_t * data = ad->data;

        // Determine if it's an iBeacon or not.
        if (
            ad->len == 30 &&
            data[4] == 0xff &&
            data[5] == 0x4c &&
            data[6] == 0x00 &&
            data[7] == 0x02 &&
            data[8] == 0x15)
        {
            uint16_t major = sys_le16_to_cpu(*reinterpret_cast<const uint16_t *>(&data[25]));
            uint16_t minor = sys_le16_to_cpu(*reinterpret_cast<const uint16_t *>(&data[27]));
            int8_t pwr = *reinterpret_cast<const int8_t *>(&data[29]);

            BLE_Zephyr::reference->add_detection(
                Clock::get_msec<uint32_t>(),
                &data[9],
                major,
                minor,
                pwr,
                rssi);

            bt_uuid_128 uuid;
            char uuid_str[BT_UUID_STR_LEN];
            sys_mem_swap(&data[9], 16);
            bt_uuid_create(
                reinterpret_cast<bt_uuid *>(&uuid),
                &data[9],
                16);
            bt_uuid_to_str(
                reinterpret_cast<const bt_uuid *>(&uuid),
                uuid_str, BT_UUID_STR_LEN);
            uuid_str[BT_UUID_STR_LEN-1] = 0;

            LOG_DBG("iBeacon found: uuid: %s; major: %d (0x%x); minor: %d (0x%x); pwr: %d; rssi: %d",
                uuid_str, major, major, minor, minor, pwr, rssi);
            LOG_HEXDUMP_DBG(ad->data, ad->len, "Data:");
        }
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

        // ATW: The scanning was moved to Nordic NUS driver. This is
        // yucky code in dire need of a makeover.

        return true;
    }
}
