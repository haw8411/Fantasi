#!/usr/bin/env python3
"""Unit test: the sleep-governance vote state machine (core/power.c).

Compiles the real TU on the host with -DPWR_HOST_TEST (FreeRTOS/HAL stubbed
inside the TU itself) and exercises the counted-vote rules:

  - no votes                -> DEEP allowed
  - any held vote           -> capped at LIGHT
  - votes are counted       -> two enters need two exits
  - exits never go negative -> a stray exit doesn't corrupt later enters
  - sleep disabled          -> NONE regardless of votes
  - stats accumulate and wake counters bucket correctly

Hardware-free - safe for CI.
"""
import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
POWER = os.path.join(REPO_ROOT, "core/power.c")

HARNESS = r"""
#include "hal_power.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL line %d: %s\n", __LINE__, #cond); exit(1); } } while (0)

int main(void)
{
    /* baseline: enabled, no votes */
    CHECK(pwr_sleep_enabled());
    CHECK(pwr_allowed_depth() == HAL_SLEEP_DEEP);

    /* one vote caps at LIGHT */
    pwr_inhibit_enter(PWR_CLIENT_USB_ACTIVE);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_LIGHT);
    CHECK(pwr_client_votes(PWR_CLIENT_USB_ACTIVE) == 1);

    /* counted: two enters, one exit -> still held */
    pwr_inhibit_enter(PWR_CLIENT_USB_ACTIVE);
    pwr_inhibit_exit(PWR_CLIENT_USB_ACTIVE);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_LIGHT);
    pwr_inhibit_exit(PWR_CLIENT_USB_ACTIVE);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_DEEP);

    /* never negative: stray exit is a no-op */
    pwr_inhibit_exit(PWR_CLIENT_FLASH);
    CHECK(pwr_client_votes(PWR_CLIENT_FLASH) == 0);
    pwr_inhibit_enter(PWR_CLIENT_FLASH);
    CHECK(pwr_client_votes(PWR_CLIENT_FLASH) == 1);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_LIGHT);
    pwr_inhibit_exit(PWR_CLIENT_FLASH);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_DEEP);

    /* independent clients overlap */
    pwr_inhibit_enter(PWR_CLIENT_BLE_LINK);
    pwr_inhibit_enter(PWR_CLIENT_SD_OP);
    pwr_inhibit_exit(PWR_CLIENT_BLE_LINK);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_LIGHT);   /* sd-op still held */
    pwr_inhibit_exit(PWR_CLIENT_SD_OP);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_DEEP);

    /* out-of-range client ids are ignored */
    pwr_inhibit_enter((pwr_client_t)PWR_CLIENT_COUNT);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_DEEP);

    /* disabled -> NONE even with votes */
    pwr_set_sleep_enabled(0);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_NONE);
    pwr_inhibit_enter(PWR_CLIENT_USB_ACTIVE);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_NONE);
    pwr_set_sleep_enabled(1);
    CHECK(pwr_allowed_depth() == HAL_SLEEP_LIGHT);
    pwr_inhibit_exit(PWR_CLIENT_USB_ACTIVE);

    /* stats + wake buckets */
    pwr_stats_t st;
    pwr_note_deep_sleep(100);
    pwr_note_deep_sleep(50);
    pwr_note_light_sleep();
    pwr_note_wake(PWR_WAKE_TIMER);
    pwr_note_wake(PWR_WAKE_TIMER);
    pwr_note_wake(PWR_WAKE_BUTTON);
    pwr_note_wake((pwr_wake_t)PWR_WAKE_COUNT);   /* out of range: dropped */
    pwr_get_stats(&st);
    CHECK(st.deep_entries == 2 && st.slept_ticks == 150);
    CHECK(st.light_entries == 1);
    CHECK(st.wake[PWR_WAKE_TIMER] == 2 && st.wake[PWR_WAKE_BUTTON] == 1);
    CHECK(st.wake[PWR_WAKE_OTHER] == 0);

    printf("all vote-machine checks passed\n");
    return 0;
}
"""


def main():
    cc = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory() as tmp:
        harness = os.path.join(tmp, "harness.c")
        exe = os.path.join(tmp, "test_power_votes")
        with open(harness, "w") as f:
            f.write(HARNESS)
        cmd = [cc, "-DPWR_HOST_TEST", "-I", os.path.join(REPO_ROOT, "hal"),
               "-Wall", "-Werror", "-o", exe, POWER, harness]
        r = subprocess.run(cmd)
        if r.returncode != 0:
            print("compile failed")
            return 1
        r = subprocess.run([exe])
        return r.returncode


if __name__ == "__main__":
    sys.exit(main())
