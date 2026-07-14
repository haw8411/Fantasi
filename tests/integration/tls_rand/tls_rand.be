# TLS regression probe (see test_tls_rand.py for the rationale). picolibc keeps
# rand()'s state (_rand_next), errno and the localtime buffer in thread-local
# storage, reached through __aeabi_read_tp(). If the thread pointer is not backed
# by RAM, those accesses target low memory: the rand state write is dropped on
# Cortex-M (rand stops advancing) and hits the exception vectors on ARM7 (device
# hang). Emit several rand() values so the harness can check they advance and the
# device survives.
import math
var vals = str(math.rand())
for i : 1 .. 4
  vals = vals + ',' + str(math.rand())
end
print('TLS_RAND ' + vals)
print('TLS_DONE')
