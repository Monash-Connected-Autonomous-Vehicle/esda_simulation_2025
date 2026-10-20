#!/usr/bin/env python3
"""Which edge of the lane am I looking at?

THE BUG THIS EXISTS FOR

band_gap() turns one visible lane line into a drivable interval like this:

    elif left.size:
        hi = float(left.min()); lo = hi - lane_w     # lane is 3 m to the right

It never checks the assumption. The robot settles at a fixed half-lane
offset from whatever line it can see, and it cannot tell the lane's left
edge from its right edge. Cross the right-hand line once and it contentedly
tracks 1.5 m to the RIGHT of it -- parallel to the course, permanently
outside, and steering further out. Three saved frames show exactly that:
one line pinned to the left edge of a 90 deg view with empty ground to the
right, then a cone and two slivers of paint all on the left, then nothing
at all, 27 m from the start, parked.

The trap is absorbing, which is why it looks like confident driving right
up until there is nothing left to see.

THE TEST

Only one branch is actually ambiguous. With the line on the left at y_L:

    A: it is the lane's LEFT edge  -> lane [y_L - w, y_L], centre y_L - w/2
    B: it is the lane's RIGHT edge -> lane [y_L, y_L + w], centre y_L + w/2

A steers the robot AWAY from the line only when y_L < w/2. Further out than
that, A already aims back toward the lane and needs no help. So the check
applies to that narrow case alone, which is precisely the trap.

Inside it, each hypothesis predicts where the OTHER line should be, and the
camera's field of view says whether that prediction is falsifiable: at a
forward distance x the detector can see +-x*tan(fov/2) across. If A puts the
second line comfortably inside that window and nothing was detected there,
A is contradicted. If B puts it outside the window, B is untestable and
therefore survives. Prefer the hypothesis that was not refuted.

This cannot fire on a robot that is genuinely inside the lane but has lost
the near line to the camera's near limit: in that case the remaining line is
FAR (y_L near w), the gate closes, and A stands.
"""


def one_sided_lane(y_line, on_left, lane_w, view_half, margin=0.3):
    """Lane interval (lo, hi) in base_link y, from a single visible line.

    y_line     signed lateral position of the nearest point of that line
    on_left    True if it lies to the robot's left (y > 0)
    lane_w     assumed lane width
    view_half  half-width of ground the camera can see at this range;
               0.0 disables the check and restores the old behaviour
    margin     how far inside the view an implied line must fall before its
               absence counts as evidence

    Returns ((lo, hi), flipped).
    """
    if on_left:
        a = (y_line - lane_w, y_line)          # A: line is the LEFT edge
        b = (y_line, y_line + lane_w)          # B: line is the RIGHT edge
        near = y_line
        other_a, other_b = a[0], b[1]
    else:
        a = (y_line, y_line + lane_w)          # A: line is the RIGHT edge
        b = (y_line - lane_w, y_line)          # B: line is the LEFT edge
        near = -y_line
        other_a, other_b = a[1], b[0]

    if view_half > 0.0 and 0.0 < near < lane_w / 2.0:
        visible = view_half - margin
        if abs(other_a) < visible and not abs(other_b) < visible:
            return b, True
    return a, False


def view_half_at(x, fov_deg):
    """Half-width of ground the camera covers at forward distance x."""
    import math
    if fov_deg <= 0.0 or x <= 0.0:
        return 0.0
    return float(x) * math.tan(math.radians(fov_deg) / 2.0)
