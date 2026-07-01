#!/usr/bin/env python3
"""
From-scratch PPO in JAX for the MJX (MuJoCo-on-GPU) env — the GPU twin of ppo_balance.py.
NO reinforcement-learning library: flax provides the MLP layers and optax the Adam step
(the JAX equivalents of torch.nn / torch.optim); GAE, the clipped surrogate, obs
normalization, and the whole training loop are written here by hand.

Thousands of MJX envs step in parallel on the GPU; a lax.scan collects each rollout, so
the whole rollout+GAE runs in one XLA graph. Trains balance (mjx_env) to validate the
GPU stack, then goal-nav once mjx_env grows a nav mode.

  MUJOCO_GL=osmesa XLA_PYTHON_CLIENT_MEM_FRACTION=0.6 \
    python mjx_ppo.py --total-timesteps 3000000 --run-dir <run>

Results layout matches ppo_balance.py / ppo_parallel.py (the "standard", see CLAUDE.md),
minus the TensorBoard event files (this trainer runs in the isolated mybot_mjx conda env,
which has no torch -> PngPlotWriter gives the same live PNG graphs without it):
    <run_dir>/model_episode_<N>.pkl, model_best.pkl, model_final.pkl, model_latest.pkl
    <run_dir>/videos/                       <- populate via mjx_render.py / mujoco_view.py
    <run_dir>/metrics/episodes_<sess>.csv   <- one row per completed episode
    <run_dir>/metrics/summary_<sess>.json
    <run_dir>/metrics/graphs/               <- live PNG per scalar + _overview.png
"""
import os, time, argparse, pickle
from typing import NamedTuple
import numpy as np
import jax
import jax.numpy as jp
import flax.linen as nn
import optax

import mjx_env as E

try:
    from tb_plot_writer import PngPlotWriter
    from metrics_logger import MetricsLogger
    _HAVE_METRICS = True
except Exception:
    _HAVE_METRICS = False


class ActorCritic(nn.Module):
    act_dim: int
    hidden: tuple = (64, 64)

    @nn.compact
    def __call__(self, x):
        a = x
        for h in self.hidden:
            a = nn.tanh(nn.Dense(h)(a))
        mean = nn.Dense(self.act_dim,
                        kernel_init=nn.initializers.orthogonal(0.01))(a)
        c = x
        for h in self.hidden:
            c = nn.tanh(nn.Dense(h)(c))
        value = nn.Dense(1)(c)[..., 0]
        log_std = self.param("log_std", lambda k: jp.full((self.act_dim,), -0.5))
        return mean, value, log_std


class Transition(NamedTuple):
    obs: jp.ndarray
    action: jp.ndarray
    logp: jp.ndarray
    value: jp.ndarray
    reward: jp.ndarray
    done: jp.ndarray


LOG2PI = float(np.log(2.0 * np.pi))


def gaussian_logp(action, mean, log_std):
    std = jp.exp(log_std)
    return jp.sum(-0.5 * ((action - mean) / std) ** 2 - log_std - 0.5 * LOG2PI, axis=-1)


def gaussian_entropy(log_std):
    return jp.sum(log_std + 0.5 * (LOG2PI + 1.0))


def normalize(obs, rms):
    return jp.clip((obs - rms[0]) / jp.sqrt(rms[1] + 1e-8), -10.0, 10.0)


def update_rms(rms, batch):
    # exact parallel (count-based) mean/var update — accurate from the FIRST batch, so
    # obs normalization is right immediately instead of drifting in over ~100 iters.
    mean, var, count = rms
    bmean, bvar, bcount = batch.mean(0), batch.var(0), batch.shape[0]
    delta = bmean - mean
    tot = count + bcount
    new_mean = mean + delta * (bcount / tot)
    m2 = var * count + bvar * bcount + (delta ** 2) * (count * bcount / tot)
    return (new_mean, m2 / tot, tot)


def make_train(args):
    net = ActorCritic(E.ACT_DIM)
    optim = optax.chain(optax.clip_by_global_norm(0.5), optax.adam(args.lr))
    vstep = jax.vmap(E.step)
    N, T = args.n_envs, args.rollout

    def apply(params, obs):
        return net.apply(params, obs)

    def rollout(params, rms, carry):
        def body(c, _):
            state, key, cret, clen = c
            obs = state.obs
            mean, value, log_std = apply(params, normalize(obs, rms))
            key, sub = jax.random.split(key)
            action = mean + jp.exp(log_std) * jax.random.normal(sub, mean.shape)
            logp = gaussian_logp(action, mean, log_std)
            nstate = vstep(state, action)
            cret = cret + nstate.reward
            clen = clen + 1.0
            d = nstate.done > 0.5
            ep = (jp.where(d, cret, 0.0), jp.where(d, clen, 0.0), jp.where(d, 1.0, 0.0))
            cret = jp.where(d, 0.0, cret)
            clen = jp.where(d, 0.0, clen)
            tr = Transition(obs, action, logp, value, nstate.reward, nstate.done)
            return (nstate, key, cret, clen), (tr, ep)
        carry, (traj, ep) = jax.lax.scan(body, carry, None, length=T)
        state = carry[0]
        _, last_value, _ = apply(params, normalize(state.obs, rms))
        return carry, traj, last_value, ep

    def gae(traj, last_value):
        def f(c, t):
            adv, nv = c
            delta = t.reward + args.gamma * nv * (1.0 - t.done) - t.value
            adv = delta + args.gamma * args.lam * (1.0 - t.done) * adv
            return (adv, t.value), adv
        _, advs = jax.lax.scan(f, (jp.zeros(N), last_value), traj, reverse=True)
        return advs, advs + traj.value

    def loss_fn(params, mb):
        obs, act, old_logp, adv, ret, old_val = mb
        mean, value, log_std = apply(params, obs)
        logp = gaussian_logp(act, mean, log_std)
        ratio = jp.exp(logp - old_logp)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        pg = jp.maximum(-adv * ratio,
                        -adv * jp.clip(ratio, 1 - args.clip, 1 + args.clip)).mean()
        vclip = old_val + jp.clip(value - old_val, -args.clip, args.clip)
        vloss = 0.5 * jp.maximum((value - ret) ** 2, (vclip - ret) ** 2).mean()
        ent = gaussian_entropy(log_std)
        approx_kl = jp.mean(old_logp - logp)
        loss = pg + args.vf * vloss - args.ent * ent
        return loss, (pg, vloss, ent, approx_kl)

    @jax.jit
    def mb_step(params, opt_state, mb):
        (loss, aux), grads = jax.value_and_grad(loss_fn, has_aux=True)(params, mb)
        updates, opt_state = optim.update(grads, opt_state, params)
        return optax.apply_updates(params, updates), opt_state, aux

    rollout_jit = jax.jit(rollout)
    gae_jit = jax.jit(gae)
    return net, optim, rollout_jit, gae_jit, mb_step


def _episode_rows(traj, ep_ret, ep_len, ep_cnt):
    """Extract per-completed-episode metrics from this iteration's rollout, for CSV/TB
    logging in the same schema as ppo_balance.py / ppo_parallel.py. ep_ret/ep_len/ep_cnt
    come out of the scan with shape (T, N) -- one slot per (rollout step, env); nonzero
    where THAT env's episode ended on THAT step (an env can complete more than one
    episode within a single T-step rollout window early in training, when ep_len < T).
    tilt/effort are a WINDOWED proxy (mean/max over this iteration's T-step rollout for
    that env), not the full episode -- MJX's scanned rollout only keeps a short in-flight
    window per iteration, so a true full-episode trace isn't available without threading
    extra state through the compiled scan body."""
    ep_cnt_np = np.asarray(ep_cnt)                            # (T, N)
    tt, nn = np.nonzero(ep_cnt_np > 0.5)
    if tt.size == 0:
        return []
    ret_np, len_np = np.asarray(ep_ret), np.asarray(ep_len)   # (T, N)
    obs = np.asarray(traj.obs)                                # (T, N, OBS_DIM)
    roll, pitch = obs[..., -E.FRAME_DIM], obs[..., -E.FRAME_DIM + 1]
    tilt_deg = np.degrees(np.hypot(roll, pitch))               # (T, N)
    act = np.abs(np.asarray(traj.action))                      # (T, N, ACT_DIM)
    mean_tilt_env, max_tilt_env = tilt_deg.mean(axis=0), tilt_deg.max(axis=0)   # (N,)
    mean_eff_env = act.mean(axis=(0, 2))                                        # (N,)
    ctrl_dt = E.SIM_DT * E.N_SUB
    rows = []
    for t, i in zip(tt, nn):
        steps = int(len_np[t, i])
        rows.append(dict(
            score=float(ret_np[t, i]), steps=steps, duration=float(steps * ctrl_dt),
            fell=bool(steps < E.MAX_STEPS),
            mean_tilt_deg=float(mean_tilt_env[i]), max_tilt_deg=float(max_tilt_env[i]),
            mean_effort=float(mean_eff_env[i])))
    return rows


def train(args):
    global E
    if args.env == "nav":
        import mjx_nav_env as _nav
        E = _nav
    net, optim, rollout_jit, gae_jit, mb_step = make_train(args)
    key = jax.random.PRNGKey(args.seed)
    key, ki, kr = jax.random.split(key, 3)
    params = net.init(ki, jp.zeros((1, E.OBS_DIM)))
    opt_state = optim.init(params)
    rms = (jp.zeros(E.OBS_DIM), jp.ones(E.OBS_DIM), jp.float32(1e-4))

    N, T = args.n_envs, args.rollout
    state = jax.jit(jax.vmap(E.reset))(jax.random.split(kr, N))
    carry = (state, key, jp.zeros(N), jp.zeros(N))
    steps_per_iter = N * T
    n_iters = args.total_timesteps // steps_per_iter
    B = steps_per_iter
    mb_size = B // args.minibatches

    # ---- run-folder output layout (mirrors ppo_balance.py, see CLAUDE.md) ----
    session_id = args.session_id or time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_dir
    metrics_dir = os.path.join(run_dir, "metrics")
    graphs_dir = os.path.join(metrics_dir, "graphs")
    videos_dir = os.path.join(run_dir, "videos")
    for d in (run_dir, metrics_dir, graphs_dir, videos_dir):
        os.makedirs(d, exist_ok=True)

    writer = PngPlotWriter(graphs_dir) if _HAVE_METRICS and args.tensorboard else None
    mlog = MetricsLogger(metrics_dir, session_id) if _HAVE_METRICS else None
    if mlog:
        mlog.set_start_time(time.time())

    logf = open(os.path.join(run_dir, "train.log"), "w")

    def log(s):
        print(s); logf.write(s + "\n"); logf.flush()

    log(f"[mjx-ppo] envs={N} rollout={T} steps/iter={B} iters={n_iters} "
        f"obs={E.OBS_DIM} act={E.ACT_DIM} backend={jax.default_backend()}")
    best = -1e9
    t0 = time.time()
    gstep, total_episodes, _last_ckpt_ep = 0, 0, 0
    for it in range(n_iters):
        carry, traj, last_value, (ep_ret, ep_len, ep_cnt) = rollout_jit(params, rms, carry)
        advs, returns = gae_jit(traj, last_value)
        # normalize with the SAME rms the rollout used (update rms AFTER, for next iter)
        b_obs = normalize(traj.obs.reshape(B, E.OBS_DIM), rms)
        b = (b_obs, traj.action.reshape(B, E.ACT_DIM), traj.logp.reshape(B),
             advs.reshape(B), returns.reshape(B), traj.value.reshape(B))
        kl = 0.0
        for epoch in range(args.epochs):
            key, ks = jax.random.split(key)
            perm = jax.random.permutation(ks, B)
            for i in range(args.minibatches):
                idx = perm[i * mb_size:(i + 1) * mb_size]
                mb = tuple(x[idx] for x in b)
                params, opt_state, aux = mb_step(params, opt_state, mb)
            kl = float(aux[3])
            if args.target_kl and kl > args.target_kl:
                break
        rms = update_rms(rms, traj.obs.reshape(-1, E.OBS_DIM))   # for next iteration
        gstep += B
        cnt = float(ep_cnt.sum())
        mret = float(ep_ret.sum() / max(cnt, 1.0))
        mlen = float(ep_len.sum() / max(cnt, 1.0))
        sps = gstep / (time.time() - t0)

        # per-completed-episode CSV row + TB scalars (same schema as ppo_balance.py)
        for row in _episode_rows(traj, ep_ret, ep_len, ep_cnt):
            total_episodes += 1
            if mlog:
                mlog.log_episode(total_episodes, row["score"], row["steps"], row["duration"],
                                 row["fell"], row["mean_tilt_deg"], row["max_tilt_deg"],
                                 row["mean_effort"], gstep, time.time())
            if writer:
                writer.add_scalar("episode/score", row["score"], total_episodes)
                writer.add_scalar("episode/reward", row["score"], total_episodes)
                writer.add_scalar("episode/steps", row["steps"], total_episodes)
                writer.add_scalar("episode/duration", row["duration"], total_episodes)
                writer.add_scalar("episode/mean_tilt_deg", row["mean_tilt_deg"], total_episodes)
        if writer:
            writer.add_scalar("charts/ep_rew_mean", mret, gstep)
            writer.add_scalar("charts/ep_len_mean", mlen, gstep)
            writer.add_scalar("loss/policy", float(aux[0]), gstep)
            writer.add_scalar("loss/value", float(aux[1]), gstep)
            writer.add_scalar("loss/entropy", float(aux[2]), gstep)
            writer.add_scalar("loss/kl", kl, gstep)
            writer.add_scalar("charts/sps", sps, gstep)

        if it % 5 == 0 or it == n_iters - 1:
            log(f"it {it:4d}/{n_iters}  step {gstep:9d}  ep_rew {mret:8.2f}  "
                f"ep_len {mlen:7.1f}  kl {kl:.3f}  {sps:,.0f} steps/s")
        if mret > best and cnt > 0:
            best = mret
            _save(params, rms, args, os.path.join(run_dir, "model_best.pkl"))
        if it % 25 == 0:
            _save(params, rms, args, os.path.join(run_dir, "model_latest.pkl"))
            if mlog:
                mlog.write_summary(time.time(), total_episodes)
        # decoupled numbered checkpoint every --checkpoint-every EPISODES (not iterations)
        if total_episodes >= _last_ckpt_ep + args.checkpoint_every:
            _last_ckpt_ep = (total_episodes // args.checkpoint_every) * args.checkpoint_every
            _save(params, rms, args, os.path.join(run_dir, f"model_episode_{total_episodes}.pkl"))
    _save(params, rms, args, os.path.join(run_dir, "model_final.pkl"))
    if mlog:
        mlog.write_summary(time.time(), total_episodes)
    if writer:
        writer.close()
    log(f"[mjx-ppo] done. best ep_rew={best:.2f}  total {time.time()-t0:.0f}s")


def _save(params, rms, args, path):
    # dump plain Python lists (not numpy arrays) so the checkpoint loads regardless of
    # the reading env's numpy version -- e.g. viewing a GPU (mybot_mjx, numpy 2.x) run
    # from the CPU viewer's (depth_bench) older numpy, which has no numpy._core module.
    flat = jax.tree_util.tree_map(lambda x: np.asarray(x).tolist(), params)
    with open(path, "wb") as f:
        pickle.dump({"params": flat,
                     "rms": (np.asarray(rms[0]).tolist(), np.asarray(rms[1]).tolist()),
                     "obs_dim": E.OBS_DIM, "act_dim": E.ACT_DIM,
                     "hidden": [64, 64], "env": f"mjx_{args.env}"}, f)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--total-timesteps", type=int, default=3_000_000)
    p.add_argument("--n-envs", type=int, default=2048)
    p.add_argument("--rollout", type=int, default=20)
    p.add_argument("--epochs", type=int, default=4)
    p.add_argument("--minibatches", type=int, default=8)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--lam", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--ent", type=float, default=0.005)
    p.add_argument("--vf", type=float, default=0.5)
    p.add_argument("--target-kl", type=float, default=0.02)
    p.add_argument("--env", choices=["balance", "nav"], default="balance")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--run-dir", default="../../../training_results/mjx_balance1")
    p.add_argument("--session-id", default="", help="metrics session id (empty -> timestamp)")
    p.add_argument("--checkpoint-every", type=int, default=200,
                   help="checkpoint cadence in EPISODES (model_episode_<N>.pkl)")
    p.add_argument("--tensorboard", action="store_true", default=True,
                   help="write metrics/graphs/ live PNGs (PngPlotWriter, no torch needed)")
    train(p.parse_args())
