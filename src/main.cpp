#include "platform.hpp"

#include <zephyr/sys/reboot.h>
// #include "memfault/core/trace_event.h"

#define LOG_MODULE_NAME posey_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

void die(const char * const message)
{
    // MEMFAULT_TRACE_EVENT_WITH_LOG(
    //     system_death,
    //     "Message: %s", message);
    LOG_ERR("%s", message);
    LOG_ERR("Waiting 5s then rebooting.");
    Clock::delay_msec(5000);
    sys_reboot(SYS_REBOOT_COLD);
}

int main()
{
    #if defined(CONFIG_ROLE_HUB)
    TaskWaist task(imu, ble, reader, writer);
    RateTask rtmain(task, 50);
    #elif defined(CONFIG_ROLE_WATCH)
    TaskWatch task(imu, writer);
    RateTask rtmain(task, 50);
    #elif defined(CONFIG_ROLE_RING)
    TaskRing task();
    #endif

    if (!init_platform()) die("Platform init failed.");

    if (rtmain.setup())
    {
        LOG_INF("Setup complete. Entering loop.\n");
        while (true)
        {
            rtmain.loop();
        }
    }
    else die("rtmain.setup() failed.");
    return 0;
}
