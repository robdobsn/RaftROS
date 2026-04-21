#pragma once

#include <stdint.h>

bool RTPSRuntimeSchedule_isPeriodicDue(uint32_t nowMs,
                                       uint32_t lastRunMs,
                                       uint32_t intervalMs,
                                       bool triggerIfNeverRun = true);