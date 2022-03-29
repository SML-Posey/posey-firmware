#define DT_DRV_COMPAT ceva_bno08x

#include <string.h>
#include <logging/log.h>

#include "bno08x.h"

// #if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(BNO08x, CONFIG_SENSOR_LOG_LEVEL);

// This pains me...
static const struct bno08x_config * hal_cfg = NULL;

static inline size_t min(const size_t a, const size_t b) { return a < b ? a : b; }

static int bno08x_i2c_hal_open(sh2_Hal_t *self)
{
    // Send a software reset.
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    if (i2c_write_dt(&hal_cfg->bus_cfg.i2c, softreset_pkt, 5) != 0)
    {
        return -1;
    }
    k_sleep(K_MSEC(200));

    if (i2c_write_dt(&hal_cfg->bus_cfg.i2c, softreset_pkt, 5) != 0)
    {
        return -1;
    }
    k_sleep(K_MSEC(200));

    return 0;
}

static void bno08x_i2c_hal_close(sh2_Hal_t *self) {}

static int bno08x_i2c_hal_read(
	sh2_Hal_t *self,
	uint8_t *pBuffer,
	unsigned len,
    uint32_t *t_us)
{
    const uint16_t i2c_buffer_max = 200;
    static uint8_t i2c_buffer[200];

    uint8_t header[4];
    if (i2c_read_dt(&hal_cfg->bus_cfg.i2c, header, 4) != 0)
        return 0;

    // Determine amount to read.
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    // Unset the "continue" bit.
    packet_size &= ~0x8000;

    // packet wouldn't fit in our buffer
    if (packet_size > len)
        return 0;

    // The number of non-header bytes to read
    uint16_t cargo_remaining = packet_size;
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;

    while (cargo_remaining > 0)
    {
        if (first_read)
            read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
        else
            read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);

        if (i2c_read_dt(&hal_cfg->bus_cfg.i2c, i2c_buffer, read_size) != 0)
            return 0;

        if (first_read)
        {
            // The first time we're saving the "original" header, so include it in the
            // cargo count
            cargo_read_amount = read_size;
            memcpy(pBuffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        }
        else
        {
            // this is not the first read, so copy from 4 bytes after the beginning of
            // the i2c buffer to skip the header included with every new i2c read and
            // don't include the header in the amount of cargo read
            cargo_read_amount = read_size - 4;
            memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
        }
        // advance our pointer by the amount of cargo read
        pBuffer += cargo_read_amount;
        // mark the cargo as received
        cargo_remaining -= cargo_read_amount;
    }

    return packet_size;
}

static int bno08x_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    static size_t i2c_buffer_max = 200;

    uint16_t write_size = min(i2c_buffer_max, len);
    if (i2c_write_dt(&hal_cfg->bus_cfg.i2c, pBuffer, write_size) != 0)
        return 0;

    return write_size;
}

static int bno08x_i2c_read_data(
    const struct device *dev,
    uint8_t reg_addr,
	uint8_t *value,
    uint8_t len)
{
	const struct bno08x_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static int bno08x_i2c_write_data(
    const struct device *dev,
    uint8_t reg_addr,
    uint8_t *value,
    uint8_t len)
{
	const struct bno08x_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static int bno08x_i2c_read_reg(
    const struct device *dev,
    uint8_t reg_addr,
    uint8_t *value)
{
	const struct bno08x_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->bus_cfg.i2c, reg_addr, value);
}

static int bno08x_i2c_update_reg(
    const struct device *dev,
    uint8_t reg_addr,
	uint8_t mask,
    uint8_t value)
{
	const struct bno08x_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->bus_cfg.i2c, reg_addr, mask, value);
}

static const struct bno08x_transfer_function bno08x_i2c_transfer_fn =
{
	.read_data = bno08x_i2c_read_data,
	.write_data = bno08x_i2c_write_data,
	.read_reg  = bno08x_i2c_read_reg,
	.update_reg = bno08x_i2c_update_reg,
};

int bno08x_i2c_init(const struct device *dev)
{
	struct bno08x_data *data = dev->data;
	const struct bno08x_config *cfg = dev->config;

	data->hw_tf = &bno08x_i2c_transfer_fn;

	if (!device_is_ready(cfg->bus_cfg.i2c.bus)) {
		return -ENODEV;
	}

    hal_cfg = dev->config;
	data->sh2_hal.open = bno08x_i2c_hal_open;
	data->sh2_hal.close = bno08x_i2c_hal_close;
	data->sh2_hal.read = bno08x_i2c_hal_read;
	data->sh2_hal.write = bno08x_i2c_hal_write;

	return 0;
}

// #endif
