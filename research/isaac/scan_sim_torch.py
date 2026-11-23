#!/usr/bin/env python3
"""Batched GPU twin of scan_sim.py, for training only.

WHY A SECOND IMPLEMENTATION EXISTS AT ALL
    Two implementations of the same maths is exactly the situation obs_core.py
    was written to avoid, so this one needs a justification.  It is that
    training casts scans for thousands of robots at once on the GPU, and numpy
    cannot: a 4096-env step is 4096 x 898 rays, and every tensor has to stay in
    VRAM or the eGPU's PCIe 4.0 x4 link becomes the bottleneck.

    The mitigation is that this file owns no constants and no policy.  Every
    number comes from scan_sim.ScanCfg / obs_core, and
    tools/validate_isaac_obs.py suite 13 compares the two element by element.
    If they ever disagree, that is a test failure rather than a silent drift
    into a policy that behaves differently in Isaac and in Gazebo.

    Nothing here imports isaaclab.  The Isaac Lab manager terms wrap these
    functions; keeping the maths importable with only torch is what lets the
    offline harness check it without a GPU-bound simulator.

THE MEMORY CONSTRAINT THAT SHAPES THE CODE
    The obvious vectorisation builds an (E, B, K) tensor -- every env, every
    beam, every obstacle -- and takes a minimum over the last axis.  At
    E=4096, B=898, K=40 that is 147 M floats, 588 MB in float32, for one
    intermediate in one step.  On a 24 GB card shared with the policy, the
    rollout buffer and PhysX, that is not affordable.

    So obstacles are consumed in chunks with a running minimum of shape (E, B)
    -- 15 MB -- trading a short Python loop for two orders of magnitude less
    memory.  The loop runs over chunks, not over obstacles, so it stays short.
"""
import math

import numpy as np
import torch

from obs_core import N_POOLED, TRIM_BEAMS, N_BEAMS_RAW, BEAMS_PER_BUCKET, RANGE_CLIP
from scan_sim import ScanCfg

EPS = 1e-12

# Peak memory is governed by the (E, B, chunk) pair tensors, and several are
# live at once -- denom, t, u, the hit mask. A FIXED chunk is the trap: 64 is
# unnoticeable at 8 envs and allocates 940 MB per tensor at 4096 x 898, which
# measured as a 1257 MB peak before this was made adaptive. Budget the element
# count instead and let the chunk fall out of the batch size.
MAX_PAIR_ELEMS = 16_000_000
MIN_CHUNK = 1
MAX_CHUNK = 256


def pair_chunk(n_envs, n_beams):
    """Largest chunk whose (E, B, chunk) tensors stay inside the budget."""
    per = max(1, int(n_envs) * int(n_beams))
    return int(min(MAX_CHUNK, max(MIN_CHUNK, MAX_PAIR_ELEMS // per)))


def _as_tensor(x, device, dtype):
    if isinstance(x, torch.Tensor):
        return x.to(device=device, dtype=dtype)
    return torch.as_tensor(np.asarray(x), device=device, dtype=dtype)


def beam_dirs(poses, cfg, device, dtype):
    """(E, B, 2) unit directions in world frame, for poses (E, 3)."""
    alpha = torch.linspace(cfg.angle_min, cfg.angle_max, cfg.n_beams,
                           device=device, dtype=dtype)
    world = poses[:, 2:3] + alpha[None, :]
    return torch.stack([torch.cos(world), torch.sin(world)], dim=-1), alpha


def ray_discs_batch(origins, dirs, centres, radii, range_min, range_max,
                    chunk=None):
    """(E, B) nearest disc distance; +inf where nothing is hit.

    origins (E,2); dirs (E,B,2) unit; centres (K,2) or (E,K,2); radii (K,) or (E,K).
    """
    E, B = dirs.shape[0], dirs.shape[1]
    out = torch.full((E, B), float("inf"), device=dirs.device, dtype=dirs.dtype)
    if centres is None or centres.shape[-2] == 0:
        return out
    if centres.dim() == 2:
        centres = centres.unsqueeze(0).expand(E, -1, -1)
    if radii.dim() == 1:
        radii = radii.unsqueeze(0).expand(E, -1)

    K = centres.shape[1]
    chunk = chunk or pair_chunk(E, B)
    inf = float("inf")
    for lo in range(0, K, chunk):
        hi = min(lo + chunk, K)
        c = centres[:, lo:hi, :]                               # (E,k,2)
        r = radii[:, lo:hi]                                    # (E,k)
        f = origins[:, None, :] - c                            # (E,k,2)
        # b = dot(D, f) for every (beam, disc) pair
        b = torch.einsum("ebd,ekd->ebk", dirs, f)              # (E,B,k)
        cc = (f * f).sum(-1) - r * r                           # (E,k)
        disc = b * b - cc[:, None, :]
        sq = torch.sqrt(torch.clamp(disc, min=0.0))
        t_near = -b - sq
        t = torch.where(t_near >= range_min, t_near, -b + sq)
        del t_near, sq, b
        hit = (disc >= 0) & (t >= range_min) & (t <= range_max)
        del disc
        t = torch.where(hit, t, torch.full_like(t, inf))
        del hit
        out = torch.minimum(out, t.amin(dim=2))
        del t
    return out


def ray_segments_batch(origins, dirs, p0, p1, range_min, range_max,
                       chunk=None):
    """(E, B) nearest segment distance; +inf where nothing is hit.

    p0, p1 are (S,2) shared across envs, or (E,S,2) per env.
    """
    E, B = dirs.shape[0], dirs.shape[1]
    out = torch.full((E, B), float("inf"), device=dirs.device, dtype=dirs.dtype)
    if p0 is None or p0.shape[-2] == 0:
        return out
    if p0.dim() == 2:
        p0 = p0.unsqueeze(0).expand(E, -1, -1)
        p1 = p1.unsqueeze(0).expand(E, -1, -1)

    S = p0.shape[1]
    chunk = chunk or pair_chunk(E, B)
    inf = float("inf")
    for lo in range(0, S, chunk):
        hi = min(lo + chunk, S)
        a = p0[:, lo:hi, :]
        e = p1[:, lo:hi, :] - a                                # (E,s,2)
        g = a - origins[:, None, :]                            # (E,s,2)
        # denom = D x E, t = (g x E) / denom, u = (g x D) / denom
        denom = (dirs[:, :, None, 0] * e[:, None, :, 1]
                 - dirs[:, :, None, 1] * e[:, None, :, 0])     # (E,B,s)
        safe = torch.where(denom.abs() < EPS,
                           torch.full_like(denom, float("nan")), denom)
        del denom
        t = (g[:, None, :, 0] * e[:, None, :, 1]
             - g[:, None, :, 1] * e[:, None, :, 0]) / safe
        u = (g[:, None, :, 0] * dirs[:, :, None, 1]
             - g[:, None, :, 1] * dirs[:, :, None, 0]) / safe
        del safe
        hit = (torch.isfinite(t) & (t >= range_min) & (t <= range_max)
               & (u >= 0.0) & (u <= 1.0))
        del u
        t = torch.where(hit, t, torch.full_like(t, inf))
        del hit
        out = torch.minimum(out, t.amin(dim=2))
        del t
    return out


def synth_scan_batch(poses, cfg=None, discs=None, disc_radii=None,
                     lane_polylines=None, lane_p0=None, lane_p1=None,
                     noise=None, generator=None, device=None,
                     dtype=torch.float32, chunk=None):
    """Batched /scan_fused. Mirrors scan_sim.synth_scan beam for beam.

    poses           (E,3) world x, y, yaw
    lane_polylines  convenience: a list of (N,2) arrays shared by every env
    lane_p0/lane_p1 the per-env form, (E,S,2), used by the training env

    The camera cone restriction on lane returns is applied here exactly as in
    the numpy reference: paint is only in /scan_fused where the depth camera
    could have seen it.
    """
    cfg = cfg or ScanCfg()
    poses = _as_tensor(poses, device or "cpu", dtype)
    if poses.dim() == 1:
        poses = poses.unsqueeze(0)
    device = poses.device

    dirs, alpha = beam_dirs(poses, cfg, device, dtype)
    origins = poses[:, :2]
    ranges = torch.full(dirs.shape[:2], float("inf"), device=device, dtype=dtype)

    if discs is not None and len(discs):
        c = _as_tensor(discs, device, dtype)
        r = _as_tensor(disc_radii, device, dtype)
        ranges = torch.minimum(ranges, ray_discs_batch(
            origins, dirs, c, r, cfg.range_min, cfg.range_max, chunk))

    if lane_polylines:
        segs0, segs1 = [], []
        for poly in lane_polylines:
            pts = np.asarray(poly, dtype=np.float64).reshape(-1, 2)
            segs0.append(pts[:-1])
            segs1.append(pts[1:])
        lane_p0 = _as_tensor(np.vstack(segs0), device, dtype)
        lane_p1 = _as_tensor(np.vstack(segs1), device, dtype)
    elif lane_p0 is not None:
        lane_p0 = _as_tensor(lane_p0, device, dtype)
        lane_p1 = _as_tensor(lane_p1, device, dtype)

    if lane_p0 is not None and lane_p0.shape[-2] > 0:
        yaw = poses[:, 2]
        cam = origins + cfg.camera_offset_x * torch.stack(
            [torch.cos(yaw), torch.sin(yaw)], dim=-1)
        lane = ray_segments_batch(cam, dirs, lane_p0, lane_p1,
                                  cfg.lane_range_min, cfg.lane_range_max, chunk)
        inside = (alpha.abs() <= cfg.lane_half_fov)[None, :]
        lane = torch.where(inside, lane, torch.full_like(lane, float("inf")))
        ranges = torch.minimum(ranges, lane)

    if noise is not None:
        ranges = apply_noise_batch(ranges, cfg, noise, generator)
    return ranges


def apply_noise_batch(ranges, cfg, noise, generator=None):
    """Per-env noise, dropout, spurious returns and bias. Mirrors scan_sim."""
    device, dtype = ranges.device, ranges.dtype
    E, B = ranges.shape
    out = ranges.clone()
    finite = torch.isfinite(out)

    def rand(*shape):
        return torch.rand(*shape, device=device, dtype=dtype, generator=generator)

    def randn(*shape):
        return torch.randn(*shape, device=device, dtype=dtype, generator=generator)

    if noise.range_bias_std > 0.0:
        bias = randn(E, 1) * noise.range_bias_std
        out = torch.where(finite, out + bias, out)
    if noise.range_std > 0.0:
        out = torch.where(finite, out + randn(E, B) * noise.range_std, out)
    if noise.dropout_p > 0.0:
        out = torch.where(rand(E, B) < noise.dropout_p,
                          torch.full_like(out, float("inf")), out)
    if noise.spurious_p > 0.0:
        fake = cfg.range_min + rand(E, B) * (cfg.lane_range_max - cfg.range_min)
        out = torch.where(rand(E, B) < noise.spurious_p, fake, out)

    return torch.where(torch.isfinite(out),
                       torch.clamp(out, min=cfg.range_min), out)


def pool_scan_batch(ranges, range_clip=RANGE_CLIP):
    """(E, 898) -> (E, N_POOLED) by per-bucket minimum.

    Identical rule to obs_core.pool_scan: replace non-finite with the clip,
    clamp, drop the last TRIM_BEAMS beams, then reshape and take the minimum.
    The shapes line up exactly because 898 - 2 = 128 x 7.
    """
    if ranges.shape[-1] != N_BEAMS_RAW:
        raise ValueError("expected %d beams, got %d" % (N_BEAMS_RAW, ranges.shape[-1]))
    r = torch.where(torch.isfinite(ranges), ranges,
                    torch.full_like(ranges, range_clip))
    r = torch.clamp(r, min=0.0, max=range_clip)
    r = r[..., :N_BEAMS_RAW - TRIM_BEAMS]
    return r.reshape(*r.shape[:-1], N_POOLED, BEAMS_PER_BUCKET).amin(dim=-1)
