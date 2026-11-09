"""Test the motion limits before running camera-generated moves.

Run:  python check_limits.py            (robot connected, HMI closed)

1. Software checks, no motion: points off the board, beyond reach and below the
   board are each shown to be rejected by the same checks the demo uses.
2. Firmware check, no motion: a joint move to an unreachable pose (900 mm out)
   must be refused by the controller ("ER"/"EL" reply).
3. Envelope test, with motion: the pen visits travel height and hover height
   above each taught marker (the corners of the workspace), then parks.
   Commanded and reported positions are printed and logged.
Hand on the E-stop. Each motion stage asks for confirmation first.
"""
import argparse
import math

import config
import kin
from ar4link import AR4, AR4Error
from vision import BoardMapper, load_calib
from vision_demo import Logger


def software_check(mapper, x, y, z):
    margin = mapper.inside_margin(x, y)
    reach = math.hypot(x, y)
    if margin < config.BOARD_MARGIN_MM:
        return f"REJECTED off board (edge distance {margin:.0f} mm)"
    if not config.REACH_MIN_MM <= reach <= config.REACH_MAX_MM:
        return f"REJECTED unreachable (reach {reach:.0f} mm)"
    if z < mapper.board_z(x, y) + config.MIN_Z_MM:
        return "REJECTED below board"
    return "accepted"


def ask(q):
    return input(q + " [y/N] ").strip().lower() == "y"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    args = ap.parse_args()
    calib = load_calib()
    mapper = BoardMapper(calib)
    m = calib["markers"]
    cx = sum(q["x"] for q in m.values()) / 4
    cy = sum(q["y"] for q in m.values()) / 4
    bz = mapper.board_z(cx, cy)

    print("1. Software limit checks (no motion)")
    cases = [("board centre, hover", cx, cy, bz + config.HOVER_Z_MM),
             ("150 mm beyond the board", cx + 150 * math.copysign(1, cx), cy, bz + 15),
             ("700 mm from the base", 700, 0, bz + 15),
             ("on the base axis", 50, 0, bz + 15),
             ("20 mm below the board", cx, cy, bz - 20)]
    for name, x, y, z in cases:
        print(f"   {name:28s} X {x:7.1f} Y {y:7.1f} Z {z:6.1f}  -> {software_check(mapper, x, y, z)}")

    arm = AR4(args.port)
    log = Logger()
    try:
        p = arm.pose()
        if all(abs(a) < 1e-6 for a in p.joints):
            raise SystemExit("All joint angles read 0 - calibrate in the HMI first.")
        rot = mapper.tool_rot

        print("\n2. Firmware refusal check (should not move)")
        if ask("   Send a joint move to X 900 Y 0 Z 200 (unreachable)?"):
            try:
                arm.move_joint(900, 0, bz + 200, *rot, speed_pct=config.SPEED_PCT)
                print("   !! The controller ACCEPTED it - do not rely on firmware refusal")
            except AR4Error as e:
                print(f"   Refused as expected: {e}")
                log.log(event="limit_test", result="refused", note="unreachable 900,0")

        print("\n3. Envelope test: travel and hover above each marker, then park")
        if not ask("   Workspace clear, hand on the E-stop. Start?"):
            return
        p = arm.pose()
        seed = calib["park"]["joints"]
        for i in config.MARKER_IDS:
            q = m[str(i)]
            z_hover = mapper.board_z(q["x"], q["y"]) + config.HOVER_Z_MM
            js = kin.ik((q["x"], q["y"], z_hover), rot, seed)
            if js is None or not kin.within_limits(js):
                why = "out of reach" if js is None else "joint limit"
                print(f"   marker {i}: hover REFUSED before moving ({why})")
                log.log(event="limit_test", result="refused", target_x=q["x"],
                        target_y=q["y"], note=f"marker {i}: {why}")
                continue
            p = arm.pose()
            h = kin.travel_height([(p.x, p.y), (q["x"], q["y"])], mapper.board_z, rot, seed)
            if h is None:
                raise AR4Error(f"No reachable travel height to marker {i}")
            if p.z < mapper.board_z(p.x, p.y) + h:
                arm.move_linear(p.x, p.y, mapper.board_z(p.x, p.y) + h, *rot,
                                speed_pct=config.SPEED_PCT)
            z_safe = mapper.board_z(q["x"], q["y"]) + h
            arm.move_linear(q["x"], q["y"], z_safe, *rot, speed_pct=config.SPEED_PCT)
            r = arm.move_linear(q["x"], q["y"], z_hover, *rot, speed_pct=config.SPEED_PCT)
            d = math.dist((q["x"], q["y"], z_hover), r.xyz)
            print(f"   marker {i}: travel {h} mm, hover commanded {q['x']:.1f} {q['y']:.1f} "
                  f"{z_hover:.1f}  reported {r.x:.1f} {r.y:.1f} {r.z:.1f}  (diff {d:.2f} mm)  "
                  f"J {' '.join(f'{a:.0f}' for a in r.joints)}")
            log.log(event="limit_test", result="ok", target_x=q["x"], target_y=q["y"],
                    cmd_x=q["x"], cmd_y=q["y"], cmd_z=z_hover, rep_x=r.x, rep_y=r.y, rep_z=r.z,
                    note=f"marker {i}")
            arm.move_linear(q["x"], q["y"], z_safe, *rot, speed_pct=config.SPEED_PCT)
        k = calib["park"]
        z = max(k["z"], arm.pose().z)
        arm.move_linear(k["x"], k["y"], z, *rot, speed_pct=config.SPEED_PCT)
        arm.move_linear(k["x"], k["y"], k["z"], *rot, speed_pct=config.SPEED_PCT)
        print("   Envelope test done. Parked.")
    except AR4Error as e:
        print(f"\nSTOPPED: {e}")
    finally:
        try:
            arm.sync_app_config()
        except AR4Error:
            pass
        arm.close()


if __name__ == "__main__":
    main()
