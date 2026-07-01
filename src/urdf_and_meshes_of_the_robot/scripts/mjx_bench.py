#!/usr/bin/env python3
"""Measure the BEST realistic MJX throughput: pyramidal cone (cheap contacts), a
lax.scan inner loop (no per-step dispatch), at several batch sizes. Reports both raw
physics-steps/s and the RL-relevant control-steps/s (=/10 substeps), vs ~850 CPU."""
import os, time
import numpy as np
import jax, jax.numpy as jp
import mujoco
from mujoco import mjx

MODEL = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "mujoco", "mybot.xml"))


def build(cone, iters):
    m = mujoco.MjModel.from_xml_path(MODEL)
    MESH = mujoco.mjtGeom.mjGEOM_MESH
    for g in range(m.ngeom):
        if m.geom_type[g] == MESH:
            m.geom_contype[g] = 0; m.geom_conaffinity[g] = 0
    m.opt.cone = cone                      # 0=pyramidal (cheap), 1=elliptic
    if iters:
        m.opt.iterations = iters
        m.opt.ls_iterations = max(4, iters // 2)
    return mjx.put_model(m)


def bench(mx, N, T=100):
    make = jax.vmap(lambda k: mjx.make_data(mx))
    batch = make(jax.random.split(jax.random.PRNGKey(0), N))

    @jax.jit
    def rollout(b):
        def body(d, _):
            return jax.vmap(mjx.step, (None, 0))(mx, d), None
        d, _ = jax.lax.scan(body, b, None, length=T)
        return d
    b = rollout(batch); b.qpos.block_until_ready()          # compile
    t0 = time.time()
    b = rollout(batch); b.qpos.block_until_ready()
    dt = time.time() - t0
    phys = N * T / dt
    return phys, phys / 10.0                                 # phys/s, ctrl/s


for cone, name in [(1, "elliptic(current)"), (0, "pyramidal")]:
    for N in (2048, 8192):
        mx = build(cone, iters=None)
        phys, ctrl = bench(mx, N)
        print(f"cone={name:18s} N={N:5d}  {phys:11,.0f} phys/s  {ctrl:9,.0f} ctrl/s  "
              f"({ctrl/850:5.1f}x CPU)")
# cheaper solver on the good cone
mx = build(0, iters=10)
phys, ctrl = bench(mx, 8192)
print(f"cone=pyramidal+iters10 N= 8192  {phys:11,.0f} phys/s  {ctrl:9,.0f} ctrl/s  "
      f"({ctrl/850:5.1f}x CPU)")
