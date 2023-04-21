#include "NordicSAADACDriver.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/pm.h>

#define LOG_MODULE_NAME adc
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

void deep_sleep()
{
    k_timeout_t timeout = K_MINUTES(5);

	int sleep_m = (int)(timeout.ticks * 1.0 / CONFIG_SYS_CLOCK_TICKS_PER_SEC / 60.0);
	int sleep_s = (int)(timeout.ticks * 1.0 / CONFIG_SYS_CLOCK_TICKS_PER_SEC - sleep_m * 60);

	LOG_INF("Sleeping for %d minutes %d seconds...", sleep_m, sleep_s);
	static const struct pm_state_info suspend_state = {PM_STATE_SUSPEND_TO_IDLE, 0, 0};
	pm_state_force(0u, &suspend_state);
	k_sleep(timeout);

    LOG_INF("Waking up...");
}

#ifndef CONFIG_ADC
bool init_adc()
{
    LOG_INF("ADC not available.");
    return true;
}

float read_Vbatt()
{
    return 3.77f;
}

#else

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user),
    io_channels,
    DT_SPEC_AND_COMMA)
};

bool init_adc()
{
    LOG_INF("Initializing ADC...");

    int err = 0;

	// Configure channels individually prior to sampling.
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
        LOG_INF("Configuring ADC channel %d...", i);
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_ERR("ADC controller device not ready\n");
			return false;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return false;
		}
	}

    return true;
}

float read_Vbatt()
{
	int err;
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

    err = adc_sequence_init_dt(
        &adc_channels[0],
        &sequence);
    if (err) {
        LOG_ERR("ADC sequence init failed with error %d - %s",
            err, strerror(err));
        return 0.0;
    }
    
    err = adc_read(adc_channels[0].dev, &sequence);
    if (err < 0) {
        LOG_ERR("Could not read ADC (%d - %s)\n",
            err, strerror(err));
        return 0.0;
    }

    int32_t Vain_mv = buf;
    err = adc_raw_to_millivolts_dt(
        &adc_channels[0],
        &Vain_mv);
    if (err) {
        LOG_ERR("ADC raw to millivolts failed with error %d - %s",
            err, strerror(err));
        return 0.0;
    }

    float Vain = Vain_mv*1.0e-3;
    static const float R1 = 10.0e3;
    static const float R2 = 30.9e3;
    static const float alpha = 1.0 + R1/R2;
    float Vbatt = Vain*alpha;

    return Vbatt;
}

#endif