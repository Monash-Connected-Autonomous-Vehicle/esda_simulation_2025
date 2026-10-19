#!/usr/bin/env python3
"""test_spawn_table.py -- the spawn table has one copy and two readers; assert
they agree, and that no generated arena is missing from it.

Job 109 captured 672 frames with every pose off by (11.86, -2.31, 1.263 rad)
because demo.sh knew the spawn and capture_labelled.py did not.  Two copies of
one fact, and the silent one won.  The fact now lives in
esda_simulation_2025/config/world_spawns.txt; this test fails if a reader
drifts from it, if a generated arena is added without an entry, or if an entry
names a world that does not exist.

    python3 vision/test_spawn_table.py
"""
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_spawns

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(REPO, "esda_simulation_2025", "worlds")
fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {name:<52} {'ok' if ok else 'FAIL  got %r want %r' % (got, want)}")
    if not ok:
        fails.append(name)


print("=" * 72)
print("spawn table -- one copy, two readers")
print("=" * 72)

table = world_spawns.table(REPO)
check("table is readable and non-empty", bool(table), True)
print(f"  {len(table)} worlds listed: {', '.join(sorted(table))}")

print("\n1. every listed world has a world file")
for w in sorted(table):
    check(f"{w}.sdf exists", os.path.exists(os.path.join(WORLDS, w + ".sdf")), True)

print("\n2. every generated arena is listed")
# The generated arenas are the ones make_worlds.py writes, and they are exactly
# the ones that do NOT start at the origin -- the origin sits in the infield.
# A new arena added without a spawn entry would spawn on blank asphalt and look
# like a blind lane detector, which already cost this project a night.
for f in sorted(os.listdir(WORLDS)):
    if not f.endswith(".sdf"):
        continue
    w = f[:-4]
    if re.fullmatch(r"igvc_[a-z0-9]+(_(slow|crawl))?_test", w):
        check(f"{w} is in the table", w in table, True)

print("\n3. the bash reader agrees with the python reader")
for w in sorted(table):
    out = subprocess.run(
        ["bash", "-c",
         f'set +u; ESDA_SRC="{REPO}" source "{REPO}/esda_lib.sh" >/dev/null 2>&1; '
         f'world_spawn "{w}"'],
        capture_output=True, text=True).stdout.split()
    got = tuple(float(v) for v in out) if len(out) == 3 else None
    check(f"world_spawn {w}", got, table[w])

print("\n4. demo.sh no longer carries its own copy")
demo = open(os.path.join(REPO, "demo.sh")).read()
# Any bare "SX=<number>" assignment other than the 0.0 default is a second copy.
strays = [m for m in re.findall(r"SX=(-?\d+\.?\d*)", demo) if float(m) != 0.0]
check("no hardcoded SX= values left in demo.sh", strays, [])
check("demo.sh calls world_spawn", "world_spawn" in demo, True)

print("\n5. composing a zero odom with the spawn returns the spawn")
for w in sorted(table):
    got = tuple(round(v, 9) for v in world_spawns.odom_to_world(0, 0, 0, table[w]))
    check(f"odom_to_world(0,0,0) == spawn for {w}", got, table[w])

print("\n6. driving forward from the spawn moves along the spawn heading")
import math
for w in sorted(table):
    sx, sy, syaw = table[w]
    x, y, yaw = world_spawns.odom_to_world(1.0, 0.0, 0.0, table[w])
    check(f"1 m forward in {w}",
          (round(x - sx, 6), round(y - sy, 6)),
          (round(math.cos(syaw), 6), round(math.sin(syaw), 6)))

print()
print("=" * 72)
if fails:
    print(f"{len(fails)} CHECK(S) FAILED: " + ", ".join(fails))
    print("=" * 72)
    raise SystemExit(1)
print("ALL CHECKS PASSED -- world_spawns.txt, esda_lib.sh and demo.sh agree")
print("=" * 72)
