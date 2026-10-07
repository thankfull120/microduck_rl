#!/usr/bin/env python3
"""Train a high-level MicroDuck object-approach policy with PPO.

This is intentionally separate from the 61D->14D locomotion policy.  It learns
only [forward velocity, yaw rate] from target-relative observations, so the
existing walking checkpoint remains frozen and reusable.

CPU example:
    uv run python scripts/train_object_approach_highlevel.py \
        --num-envs 256 --rollout-steps 64 --updates 250

The resulting checkpoint is a high-level controller candidate.  It is NOT a
replacement for model_61447.pt and must not be loaded into the 14-D joint-policy
slot.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import torch
from torch import nn
from torch.distributions import Normal

from mjlab_microduck.object_approach.highlevel_env import (
    ObjectApproachCfg,
    ObjectApproachVecEnv,
)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int) -> None:
        super().__init__()
        hidden = 128
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, action_dim),
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1),
        )
        self.log_std = nn.Parameter(torch.full((action_dim,), -0.5))

    def distribution(self, obs: torch.Tensor) -> Normal:
        mean = self.actor(obs)
        std = self.log_std.exp().expand_as(mean)
        return Normal(mean, std)

    def value(self, obs: torch.Tensor) -> torch.Tensor:
        return self.critic(obs).squeeze(-1)

    def act(
        self, obs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist = self.distribution(obs)
        raw = dist.rsample()
        action = torch.tanh(raw)
        # Tanh correction for the squashed Gaussian.
        log_prob = dist.log_prob(raw).sum(-1)
        log_prob -= torch.log(1.0 - action.square() + 1e-6).sum(-1)
        return action, log_prob, self.value(obs)

    def evaluate_actions(
        self, obs: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        # Invert tanh safely.
        a = action.clamp(-0.999999, 0.999999)
        raw = 0.5 * (torch.log1p(a) - torch.log1p(-a))
        dist = self.distribution(obs)
        log_prob = dist.log_prob(raw).sum(-1)
        log_prob -= torch.log(1.0 - a.square() + 1e-6).sum(-1)
        entropy = dist.entropy().sum(-1)
        return log_prob, entropy, self.value(obs)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--num-envs", type=int, default=256)
    p.add_argument("--rollout-steps", type=int, default=64)
    p.add_argument("--updates", type=int, default=250)
    p.add_argument("--epochs", type=int, default=4)
    p.add_argument("--minibatches", type=int, default=8)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--entropy-coef", type=float, default=0.002)
    p.add_argument("--value-coef", type=float, default=0.5)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="cpu")
    p.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/object_approach_highlevel"),
    )
    return p.parse_args()


@torch.no_grad()
def evaluate(
    model: ActorCritic,
    *,
    device: torch.device,
    seed: int,
    episodes: int = 512,
) -> dict[str, float]:
    env = ObjectApproachVecEnv(episodes, device=device, seed=seed + 10_000)
    obs = env.observe()
    finished = torch.zeros(episodes, dtype=torch.bool, device=device)
    success = torch.zeros_like(finished)
    final_distance = torch.zeros(episodes, device=device)

    for _ in range(env.cfg.max_steps):
        action = torch.tanh(model.actor(obs))
        obs, _, done, info = env.step(action)
        newly = done & ~finished
        success[newly] = info["success"][newly]
        final_distance[newly] = info["distance"][newly]
        finished |= done
        if bool(finished.all()):
            break

    unfinished = ~finished
    if unfinished.any():
        final_distance[unfinished] = env.target_polar()[0][unfinished]

    return {
        "episodes": float(episodes),
        "success_rate": float(success.float().mean().cpu()),
        "mean_terminal_distance_m": float(final_distance.mean().cpu()),
    }


def main() -> None:
    args = parse_args()
    torch.manual_seed(args.seed)

    device = torch.device(args.device)
    cfg = ObjectApproachCfg()
    env = ObjectApproachVecEnv(
        args.num_envs, device=device, seed=args.seed, cfg=cfg
    )
    model = ActorCritic(env.obs_dim, env.action_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    n = args.num_envs
    t = args.rollout_steps
    batch_size = n * t
    if batch_size % args.minibatches != 0:
        raise SystemExit(
            f"num_envs*rollout_steps ({batch_size}) must be divisible by minibatches ({args.minibatches})"
        )
    mb = batch_size // args.minibatches

    obs = env.observe()
    start = time.time()
    total_steps = 0

    for update in range(1, args.updates + 1):
        obs_buf = torch.empty(t, n, env.obs_dim, device=device)
        act_buf = torch.empty(t, n, env.action_dim, device=device)
        logp_buf = torch.empty(t, n, device=device)
        rew_buf = torch.empty(t, n, device=device)
        done_buf = torch.empty(t, n, device=device)
        val_buf = torch.empty(t, n, device=device)

        success_count = 0
        done_count = 0

        for step in range(t):
            with torch.no_grad():
                action, logp, value = model.act(obs)
            obs_buf[step] = obs
            act_buf[step] = action
            logp_buf[step] = logp
            val_buf[step] = value

            next_obs, reward, done, info = env.step(action)
            rew_buf[step] = reward
            done_buf[step] = done.float()

            success_count += int(info["success"].sum().cpu())
            done_count += int(done.sum().cpu())
            obs = next_obs

        with torch.no_grad():
            next_value = model.value(obs)

        adv = torch.zeros_like(rew_buf)
        last_gae = torch.zeros(n, device=device)
        for step in reversed(range(t)):
            next_nonterminal = 1.0 - done_buf[step]
            next_val = next_value if step == t - 1 else val_buf[step + 1]
            delta = (
                rew_buf[step]
                + args.gamma * next_val * next_nonterminal
                - val_buf[step]
            )
            last_gae = (
                delta
                + args.gamma
                * args.gae_lambda
                * next_nonterminal
                * last_gae
            )
            adv[step] = last_gae

        returns = adv + val_buf

        flat_obs = obs_buf.reshape(batch_size, env.obs_dim)
        flat_act = act_buf.reshape(batch_size, env.action_dim)
        flat_logp = logp_buf.reshape(batch_size)
        flat_adv = adv.reshape(batch_size)
        flat_ret = returns.reshape(batch_size)

        flat_adv = (flat_adv - flat_adv.mean()) / (flat_adv.std() + 1e-8)

        indices = torch.arange(batch_size, device=device)
        for _ in range(args.epochs):
            perm = indices[torch.randperm(batch_size, device=device)]
            for start_i in range(0, batch_size, mb):
                idx = perm[start_i : start_i + mb]
                new_logp, entropy, value = model.evaluate_actions(
                    flat_obs[idx], flat_act[idx]
                )
                ratio = (new_logp - flat_logp[idx]).exp()
                s1 = ratio * flat_adv[idx]
                s2 = (
                    ratio.clamp(1.0 - args.clip, 1.0 + args.clip)
                    * flat_adv[idx]
                )
                policy_loss = -torch.minimum(s1, s2).mean()
                value_loss = 0.5 * (value - flat_ret[idx]).square().mean()
                entropy_loss = -entropy.mean()

                loss = (
                    policy_loss
                    + args.value_coef * value_loss
                    + args.entropy_coef * entropy_loss
                )

                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), args.max_grad_norm)
                optimizer.step()

        total_steps += batch_size

        if update == 1 or update % 10 == 0 or update == args.updates:
            rollout_success = (
                success_count / done_count if done_count else 0.0
            )
            elapsed = max(time.time() - start, 1e-6)
            print(
                f"update={update:4d}/{args.updates} "
                f"steps={total_steps:9d} "
                f"reward={rew_buf.mean().item():+.4f} "
                f"rollout_success={rollout_success:.3f} "
                f"sps={total_steps/elapsed:,.0f}"
            )

    metrics = evaluate(
        model, device=device, seed=args.seed, episodes=512
    )

    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint = args.output / "object_approach_highlevel.pt"
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "obs_dim": env.obs_dim,
            "action_dim": env.action_dim,
            "env_cfg": asdict(cfg),
            "training_args": vars(args) | {"output": str(args.output)},
            "metrics": metrics,
            "contract": {
                "obs": [
                    "distance_norm",
                    "sin_bearing",
                    "cos_bearing",
                    "prev_forward_norm",
                    "prev_yaw_rate_norm",
                ],
                "action": [
                    "forward_velocity_norm",
                    "yaw_rate_norm",
                ],
                "low_level_policy": "external/frozen; do not replace 61D->14D locomotion",
            },
        },
        checkpoint,
    )
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8"
    )
    (args.output / "contract.json").write_text(
        json.dumps(
            {
                "obs_dim": env.obs_dim,
                "action_dim": env.action_dim,
                "observation": [
                    "distance_norm",
                    "sin_bearing",
                    "cos_bearing",
                    "prev_forward_norm",
                    "prev_yaw_rate_norm",
                ],
                "action": [
                    "forward_velocity_norm",
                    "yaw_rate_norm",
                ],
                "max_forward_speed_m_s": cfg.max_forward_speed,
                "max_yaw_rate_rad_s": cfg.max_yaw_rate,
                "stop_distance_m": cfg.stop_distance,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"saved: {checkpoint}")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
