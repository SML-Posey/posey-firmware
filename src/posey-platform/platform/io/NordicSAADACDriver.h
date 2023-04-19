#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool init_adc();
float read_Vbatt();

#ifdef __cplusplus
}
#endif