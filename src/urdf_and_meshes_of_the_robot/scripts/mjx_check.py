#!/usr/bin/env python3
"""Feasibility probe for MJX (MuJoCo-on-GPU): what's installed, GPU visible, model loads?"""
import importlib, os, sys
print("python", sys.version.split()[0])
for m in ["jax", "jaxlib", "flax", "optax", "mujoco"]:
    try:
        mod = importlib.import_module(m)
        print(f"  {m:8s} {getattr(mod, '__version__', '?')}")
    except Exception as e:
        print(f"  {m:8s} MISSING ({type(e).__name__})")

try:
    import jax
    print("jax devices:", jax.devices())
    print("jax default backend:", jax.default_backend())
except Exception as e:
    print("jax devices ERR:", repr(e)[:200])

MODEL = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "mujoco", "mybot.xml"))
try:
    import mujoco
    from mujoco import mjx
    m = mujoco.MjModel.from_xml_path(MODEL)
    print(f"MjModel loaded: nq={m.nq} nv={m.nv} nu={m.nu}  cone={m.opt.cone} integrator={m.opt.integrator}")
    mx = mjx.put_model(m)
    print("mjx.put_model OK -> model is MJX-compatible")
    d = mjx.make_data(mx)
    import jax
    d = mjx.step(mx, d)          # one GPU step to confirm it actually runs
    print("mjx.step OK -> physics runs on", jax.default_backend())
except Exception as e:
    print("MJX MODEL/STEP issue:", type(e).__name__, str(e)[:400])
