# Clock-rate probe (see test_clock_rate.py for the rationale). Prints the
# monotonic time.clock() value (seconds since boot). picolibc's clock() returns
# times() divided by CLOCKS_PER_SEC, so core/libc_glue.c's times() must scale
# FreeRTOS ticks into CLOCKS_PER_SEC units; a wrong scale makes this run fast or
# slow.
import time
print('CLK=' + str(time.clock()))
