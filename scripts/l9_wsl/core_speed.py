"""Time a fixed numpy / Python workload pinned to each CPU in turn (L9: which cores are the
performance cores of the 285HX as WSL numbers them)."""
import os
import time

import numpy as np

a = np.random.default_rng(0).standard_normal((120, 120))


def work():
    s = 0.0
    for _ in range(300):
        s += float(np.linalg.solve(a + 120 * np.eye(120), a[:, 0])[0])
    x = 0
    for i in range(400_000):
        x += i * i % 7
    return s + x


out = []
for cpu in range(os.cpu_count()):
    os.sched_setaffinity(0, {cpu})
    work()
    t = []
    for _ in range(3):
        t0 = time.perf_counter()
        work()
        t.append(time.perf_counter() - t0)
    out.append((cpu, min(t)))
    print(f"cpu {cpu:2d}: {1e3 * min(t):7.1f} ms", flush=True)
fast = sorted(out, key=lambda c: c[1])
print("fastest 8:", sorted(c for c, _ in fast[:8]))
