#include "platform.hpp"

#define LOG_MODULE_NAME posey_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

void die(const char * const message)
{
    while (true)
    {
        LOG_ERR("%s", message);
        k_sleep(K_SECONDS(1));
    }
}

int main()
{
    #if defined(CONFIG_ROLE_HUB)
    TaskHub task(imu, ble, reader, writer);
    #elif defined(CONFIG_ROLE_WATCH)
    TaskWatch task(imu, writer);
    #elif defined(CONFIG_ROLE_RING)
    TaskRing task();
    #endif
    RateTask rtmain(task, 50);

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
