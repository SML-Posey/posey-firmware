#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

class ZephyrClock {
    public:
        template <class T>
        static void set_usec(T& time) {
            time = k_cyc_to_us_floor64(sys_clock_tick_get());
        }

        template <class T>
        static void set_msec(T& time) {
            time = k_cyc_to_ms_floor64(sys_clock_tick_get());
        }

        template <class T>
        static T get_usec() {
            T usec;
            set_usec(usec);
            return usec;
        }

        template <class T>
        static T get_msec() {
            T msec;
            set_msec(msec);
            return msec;
        }

        template <class T>
        static void delay_usec(const T delay) {
            if (delay < USEC_PER_MSEC) {
                k_busy_wait(static_cast<uint32_t>(delay));
            } else {
                delay_msec(delay / USEC_PER_MSEC);
            }
        }

        template <class T>
        static void delay_msec(const T delay) {
            k_sleep(K_MSEC(delay));
        }
};
