"""world_spawns.py -- read config/world_spawns.txt, the one copy of where the
robot starts in each world.

WHY THIS MODULE EXISTS.  The spawn poses used to live in a case block in
demo.sh, and vision/capture_labelled.py defaulted --spawn to "0,0,0".  Job 109
captured 672 frames of igvc_oval_slow_test without passing --spawn, so every
recorded pose was off by the real spawn (11.86, -2.31, 1.263 rad).  autolabel
would have projected the arena's paint into the wrong part of each image and
the recall number would have come out near zero -- reading as a broken model
rather than a broken offset.  A wrong number that looks like a finding is worse
than no number, so the fact now has one home and two readers.

An unlisted world spawns at the origin: that is correct for the hand-built
worlds, and config/world_spawns.txt explains why it is NOT correct for the
generated arenas.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_REPO = os.path.dirname(HERE)
REL = os.path.join("esda_simulation_2025", "config", "world_spawns.txt")


def table_path(repo=None):
    return os.path.join(repo or DEFAULT_REPO, REL)


def table(repo=None):
    """{world: (x, y, yaw)} for every listed world. Missing file -> {}."""
    path = table_path(repo)
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for raw in fh:
            line = raw.split("#", 1)[0].split()
            if len(line) < 4:
                continue
            try:
                out[line[0]] = (float(line[1]), float(line[2]), float(line[3]))
            except ValueError:
                continue
    return out


def spawn_of(world, repo=None):
    """(x, y, yaw) for one world, or None if it is not listed."""
    return table(repo).get(world)


def odom_to_world(ox, oy, oyaw, spawn):
    """Compose a raw /odom pose with the spawn offset.

    This is the same rotate-then-translate that capture_labelled.py applies as
    it writes, kept here so a capture recorded with the wrong spawn can be
    repaired from the raw odom it stored, instead of being thrown away.
    """
    import math
    sx, sy, syaw = spawn
    c, s = math.cos(syaw), math.sin(syaw)
    return (sx + c * ox - s * oy, sy + s * ox + c * oy, oyaw + syaw)
