#include "cmd_util.h"

#include "FreeRTOS.h"
#include "task.h"

void flush_before_reset(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
}
