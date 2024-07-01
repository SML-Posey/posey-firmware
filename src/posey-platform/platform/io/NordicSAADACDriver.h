#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool init_adc();
float read_Vbatt(const bool verbose);
void deep_sleep();

#ifdef __cplusplus
}
#endif
