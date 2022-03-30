#include "platform.hpp"
#include "tasks/TaskMain.hpp"

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

void main()
{
    TaskMain taskmain(imu, reader, writer);
    RateTask rtmain(taskmain, 25);

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
}
