"""Check a straight-line move in software before the arm makes it.

Uses Annin's own kinematics module from the AR4 HMI 6.7 (ARrobots.robot_kinematics,
the same solver as the firmware) with the DH parameters and joint limits from the
HMI's ARconfig.json. A straight-line move is simulated in 2 mm steps. It fails if
any step has no inverse-kinematics solution (out of reach) or exceeds a joint
limit; those are real limits and the move is refused. If a step needs a joint jump
of more than JUMP_MAX_DEG (the line passes near the wrist singularity, J5 = 0,
where J4/J6 would swing round mid-move), the caller makes a joint move to the
same end pose instead (see joint_target).
Tested: the solver reproduces every taught joint set and pose in calib.json exactly.
"""
import json
import math
import os
import sys

import numpy as np

HMI_DIR = os.environ.get(
    "AR4_HMI_DIR", os.path.expanduser("~/AR4/AR4 HMI Interface 6.7 source"))
JUMP_MAX_DEG = 6.0
STEP_MM = 2.0

sys.path.insert(0, HMI_DIR)
import ARrobots.robot_kinematics as rk  # noqa: E402

with open(os.path.join(HMI_DIR, "ARconfig.json")) as f:
    _cfg = json.load(f)


def _g(k):
    return float(str(_cfg[k]).strip())


_J = range(1, 7)
POS_LIM = [_g(f"J{i}PosLim") for i in _J]
NEG_LIM = [_g(f"J{i}NegLim") for i in _J]
# Tighter wrist limits for our end effector: the pen is taped to the J6 flange and
# breaks if the wrist rolls far. Range covers every taught pose (J4 -1..26,
# J6 -70..1) with margin. Only used for our checks; the firmware keeps its own.
TOOL_LIMITS = {4: (-45.0, 45.0), 6: (-90.0, 45.0)}   # joint: (min, max) deg
for _j, (_lo, _hi) in TOOL_LIMITS.items():
    POS_LIM[_j - 1] = min(POS_LIM[_j - 1], _hi)
    NEG_LIM[_j - 1] = min(NEG_LIM[_j - 1], -_lo)
rk.robot_set()
rk.set_dh_parameters_explicit(*[math.radians(_g(f"J{i}ΘDHpar")) for i in _J],
                              *[math.radians(_g(f"J{i}αDHpar")) for i in _J],
                              *[_g(f"J{i}aDHpar") for i in _J], *[_g(f"J{i}dDHpar") for i in _J])
rk.set_joint_limits([_g(f"J{i}PosLim") for i in _J], [_g(f"J{i}NegLim") for i in _J])
rk.set_robot_tool_frame(0, 0, 0, 0, 0, 0)


def fk(joints):
    """Joint angles (deg) -> (x, y, z, rz, ry, rx) in mm / deg."""
    p = rk.forward_kinematics(list(joints))
    return (*p[:3], *np.degrees(p[3:6]))


def ik(xyz, rot_deg, seed):
    """Pose -> joint angles (deg) nearest the seed, or None if unreachable."""
    js = rk.inverse_kinematics([*xyz, *np.radians(rot_deg)], list(seed))
    if not js or len(js) < 6:
        return None
    js = list(js[:6])
    if math.dist(fk(js)[:3], xyz) > 0.5:     # solver returns zeros when it fails
        return None
    return js


def within_limits(js):
    return all(-lo <= a <= hi for a, hi, lo in zip(js, POS_LIM, NEG_LIM))


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def mirror_wrist(js):
    """The other wrist solution for the same pose: J4 + 180, J5 -> -J5, J6 + 180."""
    return [js[0], js[1], js[2], _wrap(js[3] + 180), -js[4], _wrap(js[5] + 180)]


def joint_target(start_joints, end_xyz, rot_deg):
    """Joint angles for the end pose closest to the current ones (of the solver's
    solution and its mirror-wrist twin), or None. Near the wrist singularity the
    solver keeps J5 positive and so turns J4/J6 half a revolution; the twin lets
    J5 pass smoothly through zero instead."""
    js = ik(end_xyz, rot_deg, start_joints)
    if js is None:
        return None
    best = None
    for cand in (js, mirror_wrist(js)):
        if not within_limits(cand) or math.dist(fk(cand)[:3], end_xyz) > 0.5:
            continue
        d = max(abs(a - b) for a, b in zip(cand, start_joints))
        if best is None or d < best[0]:
            best = (d, cand)
    return best[1] if best else None


def check_linear(start_joints, end_xyz, rot_deg):
    """Simulate a straight-line move from the current joints.

    Returns (ok, kind, reason): kind is "ok", "reach", "limit" or "flip"."""
    start_xyz = fk(start_joints)[:3]
    n = max(1, int(math.ceil(math.dist(start_xyz, end_xyz) / STEP_MM)))
    prev = list(start_joints)
    for i in range(1, n + 1):
        t = i / n
        p = [a + (b - a) * t for a, b in zip(start_xyz, end_xyz)]
        js = ik(p, rot_deg, prev)
        where = f"at X {p[0]:.0f} Y {p[1]:.0f} Z {p[2]:.0f}"
        if js is None:
            return False, "reach", f"out of reach {where}"
        for j, (a, hi, lo) in enumerate(zip(js, POS_LIM, NEG_LIM), 1):
            if not -lo <= a <= hi:
                return False, "limit", f"J{j} limit ({a:.0f} deg) {where}"
        jump = max(abs(a - b) for a, b in zip(js, prev))
        if jump > JUMP_MAX_DEG:
            return False, "flip", f"joint jump {jump:.0f} deg {where}"
        prev = js
    return True, "ok", "ok"


def travel_height(points_xy, board_z, rot_deg, seed, heights=(100, 80, 60, 40)):
    """Highest clearance above the board (mm) at which every point is reachable,
    or None. Far corners of the board cannot be reached 100 mm up, so travel
    there runs lower."""
    for h in heights:
        ok = True
        for x, y in points_xy:
            js = ik((x, y, board_z(x, y) + h), rot_deg, seed)
            if js is None or not within_limits(js):
                ok = False
                break
        if ok:
            return h
    return None
