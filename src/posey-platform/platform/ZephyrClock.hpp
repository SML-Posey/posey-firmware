#pragma once

#include <zephyr.h>
#include <sys/util.h>

class ZephyrClock
{
    public:
        template <class T>
        static void set_usec(T & time)
        {
            time = k_cyc_to_us_floor64(sys_clock_tick_get());
        }

        template <class T>
        static void set_msec(T & time)
        {
            time = k_cyc_to_ms_floor64(sys_clock_tick_get());
        }

        template <class T>
        static T get_usec()
        {
            T usec;
            set_usec(usec);
            return usec;
        }

        template <class T>
        static T get_msec()
        {
            T msec;
            set_msec(msec);
            return msec;
        }

        template <class T>
        static void delay_usec(const T delay)
        {
            uint32_t delay_us = static_cast<uint32_t>(delay);
            if (delay_us < USEC_PER_MSEC) {
                k_busy_wait(delay_us);
            } else {
                delay_msec(delay_us / USEC_PER_MSEC);
            }
        }

        template <class T>
        static void delay_msec(const T delay)
        {
            k_msleep(delay);
        }
};
