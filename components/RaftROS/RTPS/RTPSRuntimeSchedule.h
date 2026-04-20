#pragma once

#include <stdint.h>

// Returns true when a periodic action is due.
// - nowMs: current monotonic time in milliseconds
// - lastRunMs: timestamp of last run
// - intervalMs: desired interval
// - triggerIfNeverRun: when true and lastRunMs == 0, returns true immediately
bool RTPSRuntimeSchedule_isPeriodicDue(uint32_t nowMs,
                                       uint32_t lastRunMs,
                                       uint32_t intervalMs,
                                       bool triggerIfNeverRun = true);
