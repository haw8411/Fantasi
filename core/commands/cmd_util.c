#include "cmd_util.h"
#include "../cli.h"

#include "FreeRTOS.h"
#include "task.h"

void flush_before_reset(void)
{
    /* Protobuf command output is coalesced. Publish the final status before a
     * reset/mode switch can prevent handle_command() from returning normally. */
    cli_flush();
    vTaskDelay(pdMS_TO_TICKS(100));
}
