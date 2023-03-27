#pragma once

#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

int init_nus();

#ifdef CONFIG_ROLE_HUB
int8_t read_conn_rssi(struct bt_conn * conn);

bool init_flash();
bool erase_flash(const uint32_t size);
bool erase_used_flash();
bool erase_all_flash();

bool flash_is_logging();
void start_flash_logging();
void stop_flash_logging();

uint32_t flash_log_size();

void process_data(
    struct bt_conn * conn,
    const uint8_t slot,
    const uint8_t * data,
    const uint16_t size);

void BLE_Zephyr_callback(
	const bt_addr_le_t *addr,
	int8_t rssi,
	uint8_t type,
	struct net_buf_simple *ad);

void bt_nus_pc_received(
	struct bt_conn *conn,
	const uint8_t *data,
    uint16_t len);
#endif

#ifdef __cplusplus
}
#endif