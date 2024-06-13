#pragma once

#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>

#include "platform/io/BaseMessageWriter.hpp"

class NordicNUSWriter : public BaseMessageWriter {
    public:
        NordicNUSWriter(bt_conn* const ble = nullptr) : ble(ble) {}

        uint16_t write(
            const uint8_t* src_buffer,
            const uint16_t src_size,
            const bool immediate = false) override {
            uint16_t remaining = src_size;
            while (remaining > 0) {
                remaining = add_to_nus_buffer(src_buffer, src_size, remaining);
                if (immediate || nus_buffer_full())
                    transmit_nus_buffer();
            }
            return src_size;
        }

    private:
        uint16_t add_to_nus_buffer(
            const uint8_t* src_buffer,
            const uint16_t src_size,
            const uint16_t src_remaining) {
            uint16_t free = nus_buffer_size - used;
            uint16_t to_write = free < src_remaining ? free : src_remaining;
            uint16_t src_remaining_after_write = src_remaining - to_write;

            uint16_t src_si = src_size - src_remaining;
            for (uint16_t di = 0; di < to_write; ++di)
                nus_buffer[used + di] = src_buffer[src_si + di];

            used += to_write;
            return src_remaining_after_write;
        }

        bool nus_buffer_full() const { return used == nus_buffer_size; }

        void transmit_nus_buffer() {
            bt_nus_send(ble, nus_buffer, used);
            used = 0;
        }

    private:
        bt_conn* ble = nullptr;

        static const uint16_t nus_buffer_size = 244;
        uint8_t nus_buffer[nus_buffer_size];
        uint16_t used = 0;
};
