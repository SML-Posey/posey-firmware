#include "platform.hpp"

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/logging/log.h>

#include "platform/io/NordicNUSDriver.h"
#include "platform/config.h"

#define LOG_MODULE_NAME posey_platform
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

NordicNUSWriter writer;
NordicNUSReader reader;

#ifdef CONFIG_BNO08x
IMU_BNO08x imu;
#else
IMU_Stub imu;
#endif
BLE_Zephyr ble;

#define NVS_PARTITION			storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

DeviceConfig device_config;
static struct nvs_fs fs;

static bool read_device_config_param(
    const char * const name,
    const int id,
    char * const buffer)
{
    int rc = nvs_read(&fs, id, buffer, DeviceConfigBufferSize);
    if (rc > 0) LOG_INF("DeviceConfig[%s]: %s", name, buffer);
    else LOG_ERR("DeviceConfig[%s]: Not found!!", name);
    return rc > 0;
}

static bool read_device_config()
{
    int rc;
	struct flash_pages_info info;

    // Initialize flash.
	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return false;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return false;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return false;
	}

    bool okay = true;
    okay &= read_device_config_param("name", 1, device_config.name);
    okay &= read_device_config_param("role", 2, device_config.role);
    okay &= read_device_config_param("hw", 3, device_config.hw);
    okay &= read_device_config_param("sw", 4, device_config.sw);
    okay &= read_device_config_param("datetime", 5, device_config.dt);

    return okay;
}

bool init_platform()
{
    if (!read_device_config())
    {
        LOG_ERR("Unable to read device configuration!");
        return false;
    }
    if (init_nus() != 0)
    {
        LOG_ERR("Unable to initialize BLE and NUS!");
        return false;
    }

    return true;
}
