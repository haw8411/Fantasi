#include "spi_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t s_mtx;
static spi_owner_t       s_owner = SPI_OWNER_NONE;

void spi_bus_init(void)
{
    if (!s_mtx) s_mtx = xSemaphoreCreateRecursiveMutex();
}

void spi_bus_lock(void)
{
    if (!s_mtx) spi_bus_init();                 /* boot-time safety net */
    if (s_mtx)  xSemaphoreTakeRecursive(s_mtx, portMAX_DELAY);
}

void spi_bus_unlock(void)
{
    if (s_mtx) xSemaphoreGiveRecursive(s_mtx);
}

bool spi_bus_claim(spi_owner_t who)
{
    if (s_owner == who) return false;
    s_owner = who;
    return true;
}
