#!/usr/bin/env python3
"""One-off converter for .pkl checkpoints saved before mjx_ppo.py's _save() switched to
plain-list serialization: re-pickles params/rms as nested Python lists (numpy-version-
agnostic) instead of raw numpy arrays, so older-numpy envs (e.g. depth_bench) can load
checkpoints trained in mybot_mjx (numpy 2.x, which introduced numpy._core). Must be run
with the SAME numpy that originally saved the file (mybot_mjx), since that's the only
interpreter that can still unpickle the old array format.
"""
import sys
import pickle
import numpy as np


def _to_list(x):
    if isinstance(x, dict):
        return {k: _to_list(v) for k, v in x.items()}
    return np.asarray(x).tolist()


def convert(path):
    with open(path, "rb") as f:
        ck = pickle.load(f)
    ck["params"] = _to_list(ck["params"])
    ck["rms"] = (np.asarray(ck["rms"][0]).tolist(), np.asarray(ck["rms"][1]).tolist())
    with open(path, "wb") as f:
        pickle.dump(ck, f)
    print(f"converted {path} -> portable list format (env={ck.get('env')})")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        convert(p)
