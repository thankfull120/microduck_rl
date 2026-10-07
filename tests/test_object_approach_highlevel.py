from __future__ import annotations

import math

import torch

from mjlab_microduck.object_approach.highlevel_env import (
    ObjectApproachVecEnv,
)


def test_contract_shapes() -> None:
    env = ObjectApproachVecEnv(8, seed=1)
    obs = env.observe()
    assert obs.shape == (8, 5)

    action = torch.zeros(8, 2)
    next_obs, reward, done, info = env.step(action)
    assert next_obs.shape == (8, 5)
    assert reward.shape == (8,)
    assert done.shape == (8,)
    assert info["distance"].shape == (8,)


def test_forward_command_reduces_distance_when_aligned() -> None:
    env = ObjectApproachVecEnv(1, seed=2)
    env.target_x[:] = 1.0
    env.target_y[:] = 0.0
    env.x[:] = 0.0
    env.y[:] = 0.0
    env.yaw[:] = 0.0

    before = env.target_polar()[0].item()
    env.step(torch.tensor([[1.0, 0.0]]))
    after = env.target_polar()[0].item()
    assert after < before


def test_turn_command_changes_target_bearing() -> None:
    env = ObjectApproachVecEnv(1, seed=3)
    env.target_x[:] = 1.0
    env.target_y[:] = 1.0
    env.x[:] = 0.0
    env.y[:] = 0.0
    env.yaw[:] = 0.0

    before = env.target_polar()[1].abs().item()
    env.step(torch.tensor([[0.0, 1.0]]))
    after = env.target_polar()[1].abs().item()
    assert after < before


def test_success_requires_near_slow_and_aligned() -> None:
    env = ObjectApproachVecEnv(1, seed=4)
    c = env.cfg
    env.target_x[:] = c.stop_distance
    env.target_y[:] = 0.0
    env.x[:] = 0.0
    env.y[:] = 0.0
    env.yaw[:] = 0.0

    _, _, done, info = env.step(torch.tensor([[0.0, 0.0]]))
    assert bool(done.item())
    assert bool(info["success"].item())


def test_no_reverse_command_in_phase1() -> None:
    env = ObjectApproachVecEnv(1, seed=5)
    x0 = env.x.item()
    env.step(torch.tensor([[-1.0, 0.0]]))
    assert math.isclose(env.x.item(), x0, abs_tol=1e-8)
