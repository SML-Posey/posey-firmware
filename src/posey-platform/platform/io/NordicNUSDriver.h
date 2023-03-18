#pragma once

#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

int init_nus();

#ifdef CONFIG_ROLE_HUB
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
#endif

#ifdef __cplusplus
}
#endif