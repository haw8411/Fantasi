#include "app_api.h"
/* Runs forever (until Ctrl-C) so the kill/free path can be exercised. */
int app_main(const fantasi_api_t *api)
{
    for (int i = 0; ; i++) {
        api->printf("tick %d\n", i);
        for (volatile uint32_t d = 0; d < 2000000; d++) { }
    }
    return 0;
}
