#pragma once

#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>

#include "algorithm/RingBuffer.hpp"
#include "platform/io/BaseMessageReader.hpp"

class NordicNUSReader : public BaseMessageReader {
    public:
        NordicNUSReader(const bt_conn* const ble = nullptr) : ble(ble) {}

        uint16_t read_to(
            uint8_t* dst_buffer, const uint16_t max_size) override {
            return read_buffer.read_to(dst_buffer, max_size);
        }

        uint16_t write_from(
            const uint8_t* const src_buffer, const uint16_t max_size) {
            return read_buffer.write_from(src_buffer, max_size);
        }

    private:
        const bt_conn* ble;
        RingBuffer<uint8_t, 200> read_buffer;
};
