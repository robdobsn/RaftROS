#include "runtime/schedule/RTPSRuntimeSchedule.h"

bool RTPSRuntimeSchedule_isPeriodicDue(uint32_t nowMs,
                                       uint32_t lastRunMs,
                                       uint32_t intervalMs,
                                       bool triggerIfNeverRun)
{
    if (intervalMs == 0)
        return false;
    if (triggerIfNeverRun && lastRunMs == 0)
        return true;

    return (nowMs - lastRunMs) >= intervalMs;
}