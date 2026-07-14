# Berry launch integration test - run via `launch /ramfs/berry_test.be`.
# Exercises integer arithmetic, string ops, and the VFS-backed file port,
# emitting tagged tokens the harness asserts on. Reads/writes under /ramfs.

var t = 0
for i : 1 .. 10
  t = t + i
end
print('BERRY_SUM', t)               # 55
print('BERRY_STR', 'ab' * 3)        # ababab

var f = open('/ramfs/berry_launch.txt', 'w')
f.write('berry-vfs-')
f.write(str(t))
f.close()

var g = open('/ramfs/berry_launch.txt', 'r')
print('BERRY_FILE', g.readline())   # berry-vfs-55
g.close()

print('BERRY_DONE')
