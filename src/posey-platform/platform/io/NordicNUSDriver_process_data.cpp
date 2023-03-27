#include "NordicNUSDriver.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>

#include "posey-platform/ZephyrPlatform.hpp"
#include "platform/config.h"

#define LOG_MODULE_NAME posey_nus_process
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#ifdef CONFIG_ROLE_HUB

#include "platform/sensors/BaseFlashBlock.hpp"

#define K 1024
#define M (K*K)

#define FLASH_SECTOR_SIZE (4*K)         // KBytes
#define FLASH_BLOCK_SIZE (64*K)         // KBytes
#define FLASH_SIZE (128*M/8)            // MBytes

const struct device * flash_dev = NULL;
static off_t flash_offset = 0;

static bool logging_enabled = false;

extern "C" void bt_nus_pc_received(
	struct bt_conn *conn,
	const uint8_t *data,
    uint16_t len)
{
    // int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    LOG_INF("NordicNUSDriver::bt_nus_pc_received");
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

    auto written = reader.write_from(data, len);
	LOG_INF("Received %d of %d bytes from PC %s.", written, len, addr);
}

static void read_handle_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = reinterpret_cast<struct bt_hci_cp_read_rssi *>(net_buf_add(buf, sizeof(*cp)));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		printk("Read RSSI err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = reinterpret_cast<struct bt_hci_rp_read_rssi *>(rsp->data);
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}

extern "C" int8_t read_conn_rssi(struct bt_conn * conn)
{
    static uint16_t conn_handle;
    int8_t rssi = 0;
    bt_hci_get_conn_handle(conn, &conn_handle);
    read_handle_rssi(conn_handle, &rssi);
    return rssi;
}

int write_flash(const uint8_t * const data, const uint16_t size)
{
    int rc = flash_write(flash_dev, flash_offset, data, size);
    if (rc != 0) LOG_ERR("FAILED to write %d bytes at offset %ld", size, flash_offset);
    else LOG_DBG("Wrote %d bytes at offset %ld", size, flash_offset);

    flash_offset += size;
    return rc;
}

extern "C" void process_data(
    struct bt_conn * conn,
    const uint8_t slot,
    const uint8_t * data,
    const uint16_t size)
{
    static BaseFlashBlock header;
    static int8_t rssi = 0;
    static int iter = 0;

    if (!logging_enabled) return;

    if ((++iter % 1000) == 0)
        LOG_INF("Recording in progress: %d", flash_log_size());

    if (conn != NULL)
    {
        rssi = read_conn_rssi(conn);
    }

    // Write block header.
    header.data.slot = slot;
    header.data.time = Clock::get_usec<uint32_t>();
    header.data.rssi = rssi;
    header.data.block_bytes = size;
    header.data.serialize(header.buffer);
    write_flash(header.buffer.get_buffer(), header.buffer.used());

    // Write block data.
    write_flash(data, size);
}

extern "C" bool erase_flash(const uint32_t erase_size)
{
    if (erase_size < 1024*1024)
    {
        LOG_INF("Starting flash erase ~ %.2f KB", erase_size/1024.0);
    } else {
        LOG_INF("Starting flash erase ~ %.2f MB", erase_size/1024.0/1024.0);
    }
    
    int rc = flash_erase(flash_dev, 0, static_cast<off_t>(erase_size));
	if (rc != 0) {
		LOG_ERR("Flash erase failed! err=%d\n", rc);
        return false;
	} else {
		LOG_INF("Flash erase succeeded!\n");
        return true;
	}

    flash_offset = 0;
}

extern "C" bool erase_used_flash()
{
    LOG_INF("Erasing used flash only.");
    uint32_t sectors = static_cast<uint32_t>((1.0*flash_offset)/FLASH_SECTOR_SIZE) + 1;
    uint32_t erase_size = sectors*FLASH_SECTOR_SIZE;
    return erase_flash(erase_size);
}

extern "C" bool erase_all_flash()
{
    LOG_INF("Performing full-chip flash erase!");
    return erase_flash(FLASH_SIZE);
}

extern "C" bool flash_is_logging()
{
    return logging_enabled;
}

extern "C" void start_flash_logging()
{
    logging_enabled = true;
}

extern "C" void stop_flash_logging()
{
    logging_enabled = false;

    config_update_data_end(flash_log_size());
}

extern "C" uint32_t flash_log_size()
{
    return flash_offset;
}

extern "C" bool init_flash()
{
    LOG_INF("Initializing SPI flash...");

    flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: flash device not ready.\n", flash_dev->name);
		return false;
	}

    flash_offset = device_config.data_end;
    LOG_INF("Existing flash started %s, %d bytes.",
        device_config.data_dt,
        device_config.data_end);
    LOG_INF("Starting flash from offset %ld", flash_offset);

    return true;
}

#endif