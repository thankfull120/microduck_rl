"""High-level object-approach simulator for MicroDuck.

This module deliberately does NOT learn joint torques or 14-D joint actions.
It learns only a 2-D high-level command:
    [forward_velocity, yaw_rate]

That command is intended to be handed to the existing MicroDuck walking policy.
The observation is deliberately perception-shaped so that simulation target state
can later be replaced by camera/detector + distance estimation without changing
the policy interface.

Phase-1 assumptions:
- flat floor
- one stationary target
- no obstacles
- target starts within a wide forward sector
- no grasping / GroundPick in the episode
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import torch


@dataclass(frozen=True)
class ObjectApproachCfg:
    dt: float = 0.10
    max_steps: int = 180

    min_target_distance: float = 0.60
    max_target_distance: float = 2.50
    max_target_bearing_deg: float = 70.0

    max_forward_speed: float = 0.30
    max_yaw_rate: float = 1.50

    stop_distance: float = 0.28
    success_distance_tolerance: float = 0.05
    success_yaw_tolerance_deg: float = 15.0
    success_speed_tolerance: float = 0.06

    # Reward weights.
    progress_weight: float = 8.0
    alignment_weight: float = 0.08
    action_weight: float = -0.01
    yaw_action_weight: float = -0.004
    near_speed_weight: float = -0.15
    success_bonus: float = 12.0


def wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))


class ObjectApproachVecEnv:
    """A tiny vectorized 2-D simulator for high-level target approach.

    State is planar robot pose + a stationary target.  The dynamics are a
    unicycle approximation.  This is intentionally a high-level training
    environment: low-level balance and gait remain the responsibility of the
    already-trained 14-D MicroDuck locomotion policy.

    Observation (5-D):
      0: normalized distance to target, clipped to [0, 1]
      1: sin(target bearing)
      2: cos(target bearing)
      3: previous forward command / max_forward_speed
      4: previous yaw-rate command / max_yaw_rate

    Action (2-D), each clipped to [-1, 1]:
      0: normalized forward command.  Negative values are clipped to 0 for MVP.
      1: normalized yaw-rate command.
    """

    obs_dim = 5
    action_dim = 2

    def __init__(
        self,
        num_envs: int,
        *,
        device: str | torch.device = "cpu",
        seed: int = 0,
        cfg: ObjectApproachCfg | None = None,
    ) -> None:
        if num_envs < 1:
            raise ValueError("num_envs must be >= 1")
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.cfg = cfg or ObjectApproachCfg()

        self.generator = torch.Generator(device=self.device)
        self.generator.manual_seed(seed)

        self.x = torch.zeros(self.num_envs, device=self.device)
        self.y = torch.zeros(self.num_envs, device=self.device)
        self.yaw = torch.zeros(self.num_envs, device=self.device)
        self.target_x = torch.zeros(self.num_envs, device=self.device)
        self.target_y = torch.zeros(self.num_envs, device=self.device)
        self.prev_v = torch.zeros(self.num_envs, device=self.device)
        self.prev_w = torch.zeros(self.num_envs, device=self.device)
        self.steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        self.reset()

    def _rand(self, n: int) -> torch.Tensor:
        return torch.rand(n, generator=self.generator, device=self.device)

    def _sample_targets(self, ids: torch.Tensor) -> None:
        n = int(ids.numel())
        if n == 0:
            return
        c = self.cfg
        dist = c.min_target_distance + (
            c.max_target_distance - c.min_target_distance
        ) * self._rand(n)
        max_bearing = math.radians(c.max_target_bearing_deg)
        bearing = (2.0 * self._rand(n) - 1.0) * max_bearing

        # Robot always resets at the origin facing +x.
        self.target_x[ids] = dist * torch.cos(bearing)
        self.target_y[ids] = dist * torch.sin(bearing)

    def reset(self, ids: torch.Tensor | None = None) -> torch.Tensor:
        if ids is None:
            ids = torch.arange(self.num_envs, device=self.device)
        ids = ids.to(device=self.device, dtype=torch.long)
        self.x[ids] = 0.0
        self.y[ids] = 0.0
        self.yaw[ids] = 0.0
        self.prev_v[ids] = 0.0
        self.prev_w[ids] = 0.0
        self.steps[ids] = 0
        self._sample_targets(ids)
        return self.observe()

    def target_polar(self) -> tuple[torch.Tensor, torch.Tensor]:
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = torch.sqrt(dx.square() + dy.square()).clamp_min(1e-6)
        world_angle = torch.atan2(dy, dx)
        bearing = wrap_to_pi(world_angle - self.yaw)
        return distance, bearing

    def observe(self) -> torch.Tensor:
        distance, bearing = self.target_polar()
        c = self.cfg
        return torch.stack(
            (
                (distance / c.max_target_distance).clamp(0.0, 1.0),
                torch.sin(bearing),
                torch.cos(bearing),
                self.prev_v / c.max_forward_speed,
                self.prev_w / c.max_yaw_rate,
            ),
            dim=-1,
        )

    @torch.no_grad()
    def step(
        self, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict[str, torch.Tensor]]:
        if action.shape != (self.num_envs, self.action_dim):
            raise ValueError(
                f"action must have shape {(self.num_envs, self.action_dim)}, got {tuple(action.shape)}"
            )

        c = self.cfg
        action = action.to(self.device).clamp(-1.0, 1.0)

        old_distance, _ = self.target_polar()

        # Phase-1 approach does not need reverse walking.  Keeping v >= 0 makes
        # the learned command compatible with the intended slow forward approach.
        v = action[:, 0].clamp_min(0.0) * c.max_forward_speed
        w = action[:, 1] * c.max_yaw_rate

        # Semi-implicit planar unicycle step.
        self.yaw = wrap_to_pi(self.yaw + w * c.dt)
        self.x = self.x + v * torch.cos(self.yaw) * c.dt
        self.y = self.y + v * torch.sin(self.yaw) * c.dt
        self.steps += 1

        distance, bearing = self.target_polar()
        progress = old_distance - distance

        reward = c.progress_weight * progress
        reward = reward + c.alignment_weight * torch.cos(bearing)
        reward = reward + c.action_weight * action[:, 0].square()
        reward = reward + c.yaw_action_weight * action[:, 1].square()

        # Close to the object, arriving slowly matters more than rushing in.
        near = distance < (c.stop_distance + 0.15)
        reward = reward + c.near_speed_weight * near.float() * (
            v / c.max_forward_speed
        ).square()

        success = (
            (distance - c.stop_distance).abs() <= c.success_distance_tolerance
        ) & (bearing.abs() <= math.radians(c.success_yaw_tolerance_deg)) & (
            v <= c.success_speed_tolerance
        )
        reward = reward + c.success_bonus * success.float()

        timeout = self.steps >= c.max_steps
        done = success | timeout

        self.prev_v = v
        self.prev_w = w

        info = {
            "success": success,
            "timeout": timeout,
            "distance": distance,
            "bearing": bearing,
            "forward_speed": v,
            "yaw_rate": w,
        }

        obs = self.observe()

        # Auto-reset completed environments, but return the terminal flags and
        # metrics from before reset.
        reset_ids = torch.nonzero(done, as_tuple=False).flatten()
        if reset_ids.numel() > 0:
            self.reset(reset_ids)
            obs = self.observe()

        return obs, reward, done, info
