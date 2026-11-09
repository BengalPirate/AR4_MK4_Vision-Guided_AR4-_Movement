"""Live camera-guided positioning: detect a coloured target and move the pen above it.

Run:  python vision_demo.py           (robot connected, HMI closed)
      python vision_demo.py --dry     (camera and safety checks only, no robot)

Keys (in the camera window):
  G       go: lift to travel height, move above the locked target, descend to hover
  T       touch the target with the pen (from hover), then back to hover
  I/K     jog +X / -X by 1 mm   (fine correction for error measurement)
  J/L     jog +Y / -Y by 1 mm
  E       record error: the jog correction from the commanded point is logged
  R       retreat straight up to travel height
  P       park (retreat, then go to the taught PARK pose); releases the board lock
  SPACE   software stop: no further moves start (the current move finishes;
          firmware 6.7.1 cannot be interrupted over USB - use the red E-stop)
  M       show / hide the colour mask
  click   re-pick the target colour at the clicked pixel
  Q       quit

A move is only allowed when: all four markers are visible, exactly one target is
found, it has been steady for 15 frames, it lies inside the board (minus a
margin) and within the arm's reach. Every move and error measurement is logged
to results/moves.csv.
"""
import argparse
import csv
import fcntl
import math
import os
import queue
import threading
import time

import cv2
import numpy as np

import config
import kin
from override import Override
from vision import BoardMapper, Stability, TargetDetector, load_calib, open_camera

GREEN, RED, AMBER, WHITE = (0, 190, 0), (0, 0, 255), (0, 170, 255), (255, 255, 255)


class BoardLock:
    """Interlock for a shared board: only the holder may move over it.

    A non-blocking exclusive lock on a file every arm controller on this machine
    uses (set AR4_BOARD_LOCK to a path on a shared filesystem for several machines)."""

    def __init__(self, path=config.LOCK_FILE):
        self.path, self.f = path, None

    def acquire(self):
        if self.f:
            return True
        f = open(self.path, "a+")
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            f.close()
            return False
        self.f = f
        return True

    def release(self):
        if self.f:
            fcntl.flock(self.f, fcntl.LOCK_UN)
            self.f.close()
            self.f = None


class Logger:
    FIELDS = ["time", "event", "result", "px_u", "px_v", "target_x", "target_y", "board_z",
              "cmd_x", "cmd_y", "cmd_z", "rep_x", "rep_y", "rep_z", "corr_x", "corr_y",
              "error_mm", "spread_mm", "note"]

    def __init__(self, path=config.LOG_FILE):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        self.f = open(path, "a", newline="")
        self.w = csv.DictWriter(self.f, self.FIELDS)
        if new:
            self.w.writeheader()

    def log(self, **kw):
        kw["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.w.writerow({k: (f"{v:.2f}" if isinstance(v, float) else v)
                         for k, v in kw.items()})
        self.f.flush()


class DryArm:
    """Stands in for the robot in --dry mode: accepts and reports every move."""

    class P:
        def __init__(self, x, y, z, rot):
            self.x, self.y, self.z = x, y, z
            self.rz, self.ry, self.rx = rot
            self.xyz, self.rot = (x, y, z), rot

    def __init__(self, park):
        self.p = self.P(park["x"], park["y"], park["z"], (park["rz"], park["ry"], park["rx"]))

    def pose(self):
        return self.p

    def move_linear(self, x, y, z, rz, ry, rx, speed_pct=10):
        time.sleep(0.3)
        self.p = self.P(x, y, z, (rz, ry, rx))
        return self.p

    move_joint = move_linear

    def move_joints(self, angles, speed_pct=10):
        return self.p

    def sync_app_config(self):
        pass

    def close(self):
        pass


class Controller:
    """Runs robot motion on a worker thread so the camera view stays live."""

    def __init__(self, arm, mapper, logger, lock):
        self.arm, self.mapper, self.log, self.lock = arm, mapper, logger, lock
        self.rot = mapper.tool_rot
        self.park = mapper.calib["park"]
        self.jobs = queue.Queue()
        self.ovr = Override()
        self.force = False
        self.busy = False
        self.stop = False
        self.msg = "Idle"
        self.at_target = None     # dict for the target the pen is currently over
        self.pose = None
        threading.Thread(target=self._run, daemon=True).start()

    def submit(self, name, fn, *args):
        if self.busy:
            self.msg = "Busy - wait for the current move"
            return
        self.stop = False
        self.busy = True
        self.force = self.ovr.armed(name)
        self.jobs.put((name, fn, args))

    def refused(self, name, why):
        left = self.ovr.refused(name)
        self.msg = (f"REFUSED: {why}" + (f"  (repeat {left} more time(s) to override)"
                                          if left else "  (next press overrides)"))

    def _run(self):
        from ar4link import AR4Error, EStopError
        while True:
            name, fn, args = self.jobs.get()
            try:
                self.msg = f"{'OVERRIDE: ' if self.force else ''}{name} ..."
                fn(*args)
                self.ovr.reset()
            except EStopError as e:
                self.msg = f"E-STOP: {e} Then press R to retreat."
                self.log.log(event=name, result="estop", note=str(e))
            except AR4Error as e:
                if str(e).startswith("Refused"):
                    self.refused(name, str(e))
                else:
                    self.msg = f"FAULT: {e}"
                self.log.log(event=name, result="refused", note=str(e))
                try:   # 6.7.1 refuses unreachable targets without moving; back off upward
                    self._retreat()
                except AR4Error:
                    pass
            except StopRequested:
                self.msg = "STOPPED by operator (SPACE). R = retreat, P = park."
                self.log.log(event=name, result="stopped")
            except ValueError as e:   # our own checks (heights, travel, corrections)
                self.refused(name, str(e))
                self.log.log(event=name, result="refused", note=str(e))
            except Exception as e:   # keep the worker alive
                self.msg = f"ERROR: {e}"
            finally:
                try:
                    self.pose = self.arm.pose()
                except AR4Error:
                    pass
                self.busy = False

    # Each segment checks the software stop before it starts.
    def _ml(self, x, y, z):
        if self.stop:
            raise StopRequested
        bz = self.mapper.board_z(x, y)
        if z < bz + config.MIN_Z_MM and not self.force:
            raise ValueError(f"Refused: Z {z:.1f} is below the board limit")
        return self.arm.move_linear(x, y, z, *self.rot, speed_pct=config.SPEED_PCT,
                                    force=self.force)

    def _travel(self, *xys):
        """Travel clearance usable above all the given points."""
        h = kin.travel_height(xys, self.mapper.board_z, self.rot, self.park["joints"])
        if h is None and self.force:
            return config.SAFE_Z_MM
        if h is None:
            raise ValueError("Refused: no reachable travel height for this move")
        return h

    def _retreat(self, h=None):
        p = self.arm.pose()
        h = h or self._travel((p.x, p.y))
        safe = self.mapper.board_z(p.x, p.y) + h
        if p.z < safe - 0.5:
            self._ml(p.x, p.y, safe)

    def go(self, target):
        if not self.lock.acquire():
            raise_msg = "Refused: another arm holds the board lock"
            self.msg = raise_msg
            self.log.log(event="go", result="refused", note=raise_msg)
            return
        tx, ty = target["xy"]                       # camera estimate
        x, y = self.mapper.correct(tx, ty)          # commanded (after refinement)
        bz = self.mapper.board_z(x, y)
        p = self.arm.pose()
        if math.hypot(p.x - x, p.y - y) > 1 or abs(p.z - (bz + config.HOVER_Z_MM)) > 1:
            # Straight-line moves only, all at the taught pen angle: a joint move to a
            # Cartesian pose can pick a flipped-wrist solution.
            h = self._travel((p.x, p.y), (x, y))
            self._retreat(h)
            self._ml(x, y, max(self.arm.pose().z, bz + h))
            self._ml(x, y, bz + h)
        rep = self._ml(x, y, bz + config.HOVER_Z_MM)
        self.at_target = dict(target, cmd=(x, y, bz + config.HOVER_Z_MM))
        self.log.log(event="hover", result="ok", px_u=target["px"][0], px_v=target["px"][1],
                     target_x=tx, target_y=ty, board_z=bz, cmd_x=x, cmd_y=y,
                     cmd_z=bz + config.HOVER_Z_MM, rep_x=rep.x, rep_y=rep.y, rep_z=rep.z,
                     spread_mm=target["spread"], note=self.mapper.correction_note)
        self.msg = (f"Hovering {config.HOVER_Z_MM:.0f} mm above target at X {x:.1f} Y {y:.1f}. "
                    "T = touch, I/J/K/L = correct, E = record error, P = park")

    def touch(self):
        if not self.at_target:
            self.msg = "Go to a target first (G)"
            return
        p = self.arm.pose()
        bz = self.mapper.board_z(p.x, p.y)
        z = bz + max(config.TOUCH_Z_MM, config.MIN_Z_MM)
        rep = self._ml(p.x, p.y, z)
        self.log.log(event="touch", result="ok", cmd_x=p.x, cmd_y=p.y, cmd_z=z,
                     rep_x=rep.x, rep_y=rep.y, rep_z=rep.z)
        time.sleep(0.5)
        self._ml(p.x, p.y, bz + config.HOVER_Z_MM)
        self.msg = "Touched and back at hover. Check the dot against the target centre."

    def jog(self, dx, dy):
        if not self.at_target:
            self.msg = "Jog corrections only work over a target (G first)"
            return
        p = self.arm.pose()
        if not self.force and math.hypot(p.x + dx - self.at_target["cmd"][0],
                                         p.y + dy - self.at_target["cmd"][1]) > 40:
            raise ValueError("Refused: corrections are limited to 40 mm")
        self._ml(p.x + dx, p.y + dy, p.z)
        c = self.at_target["cmd"]
        self.msg = f"Correction so far: dX {p.x + dx - c[0]:+.1f}  dY {p.y + dy - c[1]:+.1f} mm"

    def record_error(self):
        if not self.at_target:
            self.msg = "Go to a target first (G)"
            return
        p = self.arm.pose()
        c = self.at_target["cmd"]
        dx, dy = p.x - c[0], p.y - c[1]
        err = math.hypot(dx, dy)
        tx, ty = self.at_target["xy"]
        self.log.log(event="error", result="measured", target_x=tx, target_y=ty,
                     cmd_x=c[0], cmd_y=c[1], rep_x=p.x, rep_y=p.y, rep_z=p.z,
                     corr_x=dx, corr_y=dy, error_mm=err, note=self.mapper.correction_note)
        self.msg = f"Recorded positioning error {err:.1f} mm (dX {dx:+.1f}, dY {dy:+.1f})"

    def retreat(self):
        self._retreat()
        self.msg = "At travel height."

    def go_park(self):
        self._retreat()
        if self.stop:
            raise StopRequested
        k = self.park
        self._ml(k["x"], k["y"], max(k["z"], self.arm.pose().z))
        self._ml(k["x"], k["y"], k["z"])
        self.at_target = None
        self.lock.release()
        self.msg = "Parked. Board lock released."
        self.log.log(event="park", result="ok")


class StopRequested(Exception):
    pass


def check_target(cands, rejected, mapper, stab):
    """Return (ok, target_or_None, reason)."""
    # Objects well outside the board (e.g. on the robot's table) are not candidates
    cands = [c for c in cands if mapper.inside_margin(*c["xy"]) > -config.IGNORE_OUTSIDE_MM]
    if len(cands) == 0:
        stab.reset()
        why = f" ({rejected[0]['why']})" if rejected else ""
        return False, None, "No target" + why
    if len(cands) > 1:
        stab.reset()
        return False, None, f"Ambiguous: {len(cands)} targets of this colour"
    t = cands[0]
    stab.add(t["xy"])
    stable, mean, spread = stab.state()
    t = dict(t, xy=mean, spread=spread)
    x, y = mean
    margin = mapper.inside_margin(x, y)
    reach = math.hypot(x, y)
    if margin < config.BOARD_MARGIN_MM:
        return False, t, f"Off board (edge distance {margin:.0f} mm)"
    if not config.REACH_MIN_MM <= reach <= config.REACH_MAX_MM:
        return False, t, f"Unreachable (reach {reach:.0f} mm)"
    x, y = mapper.correct(x, y)
    bz = mapper.board_z(x, y)
    seed = mapper.calib["park"]["joints"]
    travel = kin.travel_height([(x, y)], mapper.board_z, mapper.tool_rot, seed)
    if travel is None:
        return False, t, "Unreachable (no travel height clears the board here)"
    for dz in (travel, config.HOVER_Z_MM, config.TOUCH_Z_MM):
        js = kin.ik((x, y, bz + dz), mapper.tool_rot, seed)
        if js is None:
            return False, t, f"Unreachable ({dz:.0f} mm above the board is out of reach)"
        if not kin.within_limits(js):
            return False, t, "Unreachable (joint limit)"
        seed = js
    if not stable:
        return False, t, f"Settling ({len(stab.hist)}/{stab.n} frames, spread {spread:.1f} mm)"
    return True, t, "READY"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry", action="store_true", help="no robot; camera and checks only")
    ap.add_argument("--device", help="video device (default: find the C930e)")
    ap.add_argument("--port", help="serial port (default: auto-detect the Teensy)")
    args = ap.parse_args()

    calib = load_calib()
    mapper = BoardMapper(calib)
    if args.dry:
        arm = DryArm(calib["park"])
    else:
        from ar4link import AR4
        arm = AR4(args.port)
        if all(abs(a) < 1e-6 for a in arm.pose().joints):
            raise SystemExit("All joint angles read 0 - calibrate in the HMI first.")
    ctl = Controller(arm, mapper, Logger(), BoardLock())
    cap, device = open_camera(args.device)
    det, stab = TargetDetector(), Stability()

    win = "Team 2 vision demo" + (" [DRY RUN]" if args.dry else "")
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    frame_box = {}

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and "f" in frame_box:
            hsv = det.pick(frame_box["f"], x, y)
            stab.reset()
            ctl.msg = f"Target colour set to HSV {tuple(int(v) for v in hsv)}"

    cv2.setMouseCallback(win, on_mouse)
    show_mask = False
    keys = {ord("i"): (config.JOG_STEP_MM, 0), ord("k"): (-config.JOG_STEP_MM, 0),
            ord("j"): (0, config.JOG_STEP_MM), ord("l"): (0, -config.JOG_STEP_MM)}
    ok_target = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame grab failed")
                break
            frame_box["f"] = frame.copy()
            board_ok = mapper.update(frame)
            ready, reason, t = False, "", None
            if not board_ok:
                stab.reset()
                missing = [m for m in config.MARKER_IDS if m not in mapper.visible]
                reason = f"Board markers not visible: {missing}"
                if ctl.busy:
                    reason = "Arm moving (target locked before the move)"
            else:
                cands, rejected = det.detect(frame, mapper)
                ready, t, reason = check_target(cands, rejected, mapper, stab)
                for r in rejected:
                    cv2.drawContours(frame, [r["contour"]], -1, (120, 120, 120), 1)
                if t:
                    cv2.drawContours(frame, [t["contour"]], -1, GREEN if ready else AMBER, 2)
                    cx, cy = mapper.to_pixel([t["xy"]])[0].astype(int)
                    cv2.drawMarker(frame, (int(cx), int(cy)), RED, cv2.MARKER_CROSS, 24, 2)
                    cv2.putText(frame, f"X {t['xy'][0]:.1f}  Y {t['xy'][1]:.1f} mm",
                                (int(cx) + 15, int(cy) - 15), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, WHITE, 2)
            ok_target = t if ready else None
            mapper.draw(frame)
            if show_mask and det.mask is not None:
                frame = cv2.addWeighted(frame, 0.6, cv2.cvtColor(det.mask, cv2.COLOR_GRAY2BGR),
                                        0.4, 0)
            h, w = frame.shape[:2]
            cv2.rectangle(frame, (0, 0), (w, 72), (0, 0, 0), -1)
            cv2.putText(frame, reason, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                        GREEN if ready else RED, 2)
            cv2.putText(frame, ctl.msg[:130], (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        AMBER if ctl.busy else WHITE, 1)
            cv2.putText(frame, "G go  T touch  IJKL jog  E error  R retreat  P park  "
                        "SPACE stop  M mask  click=colour  Q quit", (12, h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
            cv2.imshow(win, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                ctl.stop = True
                ctl.msg = "SOFTWARE STOP: no new moves will start. Use the E-stop to halt now."
            elif key == ord("m"):
                show_mask = not show_mask
            elif key == ord("g"):
                if ok_target is not None:
                    ctl.submit("Moving to target", ctl.go, ok_target)
                elif t is not None and not ctl.busy and ctl.ovr.armed("Moving to target"):
                    # Operator override: go to the detected (but refused) target
                    ctl.log.log(event="go", result="override", note=reason)
                    ctl.submit("Moving to target", ctl.go, dict(t, spread=t.get("spread") or 0.0))
                else:
                    ctl.refused("Moving to target", reason)
                    ctl.log.log(event="go", result="rejected", note=reason)
            elif key == ord("t"):
                ctl.submit("Touching", ctl.touch)
            elif key in keys:
                ctl.submit("Jogging", ctl.jog, *keys[key])
            elif key == ord("e"):
                ctl.submit("Recording error", ctl.record_error)
            elif key == ord("r"):
                ctl.submit("Retreating", ctl.retreat)
            elif key == ord("p"):
                ctl.submit("Parking", ctl.go_park)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if not args.dry:
            while ctl.busy:
                time.sleep(0.2)
            try:
                arm.sync_app_config()
            except Exception:
                pass
            arm.close()


if __name__ == "__main__":
    main()
