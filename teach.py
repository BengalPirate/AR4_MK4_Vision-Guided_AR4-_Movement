"""Teach the camera-to-robot calibration by jogging the pen tip onto each ArUco marker.

Run:  python teach.py [--port PORT]
Close the AR4 HMI first - only one program can use the serial port.

For each board marker (IDs 4, 5, 6, 7): jog the pen tip until it just touches
the CENTRE of the marker, then press "Save marker N". Finally jog to a pose that
is clear of the board and the camera's view and press "Save PARK".
The points go to calib.json. vision_demo.py maps camera pixels to robot X/Y with
a homography through the four marker centres, and the board height with a plane
fitted through their Z values (the fit residual is shown as a check).

Axes are the robot base frame: +X away from the base, +Y to the robot's left,
+Z up. The pen orientation is held fixed while jogging.
"""
import argparse
import json
import os
import time
import tkinter as tk
from tkinter import messagebox

import numpy as np

from ar4link import AR4, AR4Error
from override import Override

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "calib.json")
MARKER_IDS = (4, 5, 6, 7)
STEPS_MM = (0.5, 1, 2, 5, 10, 25)
# Rz, Ry, Rx of the pen. Tilted 10 deg from straight down on purpose: at exactly
# Ry 180 the firmware's angle readback wraps between +180 and -180, and its
# straight-line moves then sweep the wrist a full turn. +170 keeps the far corners
# reachable; the strip nearest the robot is then in the wrist singularity and is
# refused. (-170 was tried: it moves the flange ~50 mm outward, so the far
# corners go out of reach.)
PEN_DOWN_ROT = (0.0, 170.0, 0.0)
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 45.0, 0.0]   # the HMI's Home / end of Auto Calibrate
WRIST_MIN_DEG = 25   # |J5| below this is near the wrist singularity: J4/J6 spin on small moves


def plane_fit(points):
    """Least-squares plane z = a*x + b*y + c. Returns (a, b, c), residuals (mm)."""
    pts = np.array(points, float)
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    coef, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    return coef, pts[:, 2] - A @ coef


class TeachApp:
    def __init__(self, root, port=None):
        self.root = root
        self.arm = AR4(port)
        root.title("Team 2 - Teach board calibration")
        self.data = {"markers": {}, "park": None}
        self.ovr = Override()
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE) as f:
                old = json.load(f)
            self.data["markers"] = old.get("markers", {})
            self.data["park"] = old.get("park")

        self.pose_var = tk.StringVar()
        self.status_var = tk.StringVar(value=f"Connected: {os.path.basename(self.arm.port)}")
        self.saved_var = tk.StringVar()
        self.step = tk.DoubleVar(value=5)
        self.speed = tk.IntVar(value=10)

        tk.Label(root, textvariable=self.pose_var, font=("monospace", 11),
                 justify="left").grid(row=0, column=0, padx=8, pady=6, sticky="w")

        steps = tk.Frame(root)
        steps.grid(row=1, column=0, sticky="w", padx=8)
        tk.Label(steps, text="Step (mm):").pack(side="left")
        for s in STEPS_MM:
            tk.Radiobutton(steps, text=str(s), variable=self.step, value=s).pack(side="left")
        tk.Label(steps, text="   Speed (%):").pack(side="left")
        tk.Spinbox(steps, from_=2, to=25, width=4, textvariable=self.speed).pack(side="left")
        self.wrist_guard = tk.BooleanVar(value=False)
        tk.Checkbutton(steps, text="Wrist guard", variable=self.wrist_guard).pack(side="left")

        self.buttons = []
        jog = tk.Frame(root)
        jog.grid(row=2, column=0, pady=8)
        for col, (axis, neg, pos) in enumerate((("X", "X- (toward base)", "X+ (away)"),
                                                ("Y", "Y- (robot's right)", "Y+ (robot's left)"),
                                                ("Z", "Z- (down)", "Z+ (up)"))):
            for row, (label, sign) in enumerate(((pos, 1), (neg, -1))):
                b = tk.Button(jog, text=label, width=18,
                              command=lambda a=axis, s=sign: self.jog(a, s))
                b.grid(row=row, column=col, padx=4, pady=2)
                self.buttons.append(b)

        saves = tk.Frame(root)
        saves.grid(row=3, column=0, pady=4)
        for i, m in enumerate(MARKER_IDS):
            b = tk.Button(saves, text=f"Save marker {m}", bg="#2e7d32", fg="white", width=14,
                          command=lambda m=m: self.save_marker(m))
            b.grid(row=0, column=i, padx=3, pady=2)
            self.buttons.append(b)
            g = tk.Button(saves, text=f"Go 20 mm above {m}", width=14,
                          command=lambda m=m: self.goto_marker(m))
            g.grid(row=1, column=i, padx=3, pady=2)
            self.buttons.append(g)
        b = tk.Button(saves, text="Save PARK (clear of board)", bg="#1565c0", fg="white",
                      command=self.save_park)
        b.grid(row=2, column=0, columnspan=2, pady=4, sticky="ew")
        self.buttons.append(b)
        b = tk.Button(saves, text="Lift 30 mm", command=lambda: self.jog("Z", 1, 30))
        b.grid(row=2, column=2, columnspan=2, pady=4, sticky="ew")
        self.buttons.append(b)
        b = tk.Button(saves, text="Set pen angle (J5 only - after Go Home)",
                      command=self.pen_down)
        b.grid(row=3, column=0, columnspan=3, pady=4, sticky="ew")
        self.buttons.append(b)
        b = tk.Button(saves, text="Go Home (joint move)", command=self.go_home)
        b.grid(row=3, column=3, pady=4, sticky="ew")
        self.buttons.append(b)

        tk.Label(root, textvariable=self.saved_var, font=("monospace", 10),
                 justify="left").grid(row=4, column=0, padx=8, pady=4, sticky="w")
        tk.Label(root, textvariable=self.status_var, fg="#555", wraplength=640,
                 justify="left").grid(row=5, column=0, padx=8, pady=6, sticky="w")
        root.protocol("WM_DELETE_WINDOW", self.quit)

        p = self.refresh()
        if p is not None and all(abs(a) < 1e-6 for a in p.joints):
            messagebox.showwarning(
                "Arm not calibrated?",
                "All joint angles read exactly 0 - the controller has restarted since the "
                "last calibration. Auto Calibrate in the HMI first, close the HMI, then "
                "restart this tool.")
        self.show_saved()

    def refresh(self):
        try:
            p = self.arm.pose()
        except AR4Error as e:
            self.pose_var.set("Position unavailable - board not responding.\n"
                              "Check the E-stop; if it persists, replug the Teensy USB\n"
                              "and recalibrate.")
            self.status_var.set(f"ERROR: {e}")
            return None
        self.pose_var.set(f"X {p.x:8.2f}   Y {p.y:8.2f}   Z {p.z:8.2f}  mm\n"
                          f"Rz {p.rz:7.2f}   Ry {p.ry:7.2f}   Rx {p.rx:7.2f}  deg\n"
                          + "  ".join(f"J{i + 1} {a:.1f}" for i, a in enumerate(p.joints)))
        return p

    def show_saved(self):
        lines = []
        for m in MARKER_IDS:
            q = self.data["markers"].get(str(m))
            lines.append(f"marker {m}: " + (f"X {q['x']:7.1f}  Y {q['y']:7.1f}  Z {q['z']:6.1f}"
                                            if q else "-"))
        park = self.data["park"]
        lines.append("park:     " + (f"X {park['x']:7.1f}  Y {park['y']:7.1f}  Z {park['z']:6.1f}"
                                     if park else "-"))
        ms = self.data["markers"]
        if len(ms) >= 4:
            _, res = plane_fit([(q["x"], q["y"], q["z"]) for q in ms.values()])
            lines.append(f"board plane fit residual: max {np.abs(res).max():.2f} mm")
        self.saved_var.set("\n".join(lines))

    def busy(self, on):
        for b in self.buttons:
            b.config(state="disabled" if on else "normal")
        self.root.config(cursor="watch" if on else "")
        self.root.update()

    def jog(self, axis, sign, dist=None):
        key = (axis, sign)
        force = self.ovr.armed(key)
        self.busy(True)
        try:
            p = self.arm.pose()
            if not force and max(abs(a - b) for a, b in zip(p.rot, PEN_DOWN_ROT)) > 3:
                self.ovr.refused(key)
                self.busy(False)
                if messagebox.askyesno(
                        "Set the pen angle first",
                        f"The pen is at Ry {p.ry:.0f} but jogging holds it at Ry "
                        f"{PEN_DOWN_ROT[1]:.0f}. A straight-line jog would have to turn it "
                        "on the way, so it is refused.\n\nSet the pen angle now (J5 only)?"):
                    self.pen_down(confirm=False)
                return
            if not force and self.wrist_guard.get() and abs(p.joints[4]) < WRIST_MIN_DEG \
                    and not (axis == "Z" and sign > 0):
                msg = (f"J5 is {p.joints[4]:.0f} deg, near the wrist singularity - "
                       "a straight-line jog here spins J4/J6. Use 'Point pen straight down' "
                       "(or Go Home first).")
                self.status_var.set("REFUSED: " + msg)
                messagebox.showwarning("Jog refused", msg)
                return
            x, y, z = p.xyz
            d = sign * (dist or self.step.get())
            if axis == "X":
                x += d
            elif axis == "Y":
                y += d
            else:
                z += d
            self.status_var.set(f"{'OVERRIDE: ' if force else ''}Moving {axis} {d:+g} mm ...")
            self.root.update()
            # Command the fixed pen angle, never the read-back one: with the pen
            # straight down the firmware's Euler readback is degenerate (gimbal lock)
            # and can report e.g. Ry -112 for a vertical pen.
            self.arm.move_linear(x, y, z, *PEN_DOWN_ROT, speed_pct=int(self.speed.get()),
                                 force=force)
            self.ovr.reset()
            self.status_var.set(f"Moved {axis} {d:+g} mm" + (" (override)" if force else ""))
        except AR4Error as e:
            msg = str(e)
            if msg.startswith("Refused") and not force:
                left = self.ovr.refused(key)
                msg += (f"\n\nPress the same button {left} more time(s) to override "
                        "our software checks (the firmware's own limits still apply).")
            self.status_var.set(f"ERROR: {e}")
            messagebox.showerror("Jog failed", msg)
        finally:
            self.busy(False)
            self.refresh()

    def pen_down(self, confirm=True):
        """Set the pen angle with a pure J5 joint move.

        With J1, J4 and J6 at 0 the pen pitch is Ry = 90 + J2 + J3 + J5, so J5
        alone sets Ry exactly, without inverse kinematics or a wrist flip."""
        p = self.refresh()
        if p is None:
            return
        j = list(p.joints)
        if max(abs(j[0]), abs(j[3]), abs(j[5])) > 2:
            messagebox.showerror("Go Home first",
                                 "J1, J4 and J6 must be near 0. Press Go Home first.")
            return
        j[4] = PEN_DOWN_ROT[1] % 360 - 90 - j[1] - j[2]
        if confirm and not messagebox.askokcancel(
                "Move arm?", f"Turn only J5 from {p.joints[4]:.1f} to {j[4]:.1f} deg so the pen "
                f"points down at Ry {PEN_DOWN_ROT[1]:.0f}?\n\nHand on the E-stop."):
            return
        self.busy(True)
        try:
            self.status_var.set("Setting the pen angle ...")
            self.root.update()
            self.arm.move_joints(j, int(self.speed.get()))
            self.status_var.set("Pen angle set. Jog over a marker.")
        except AR4Error as e:
            self.status_var.set(f"ERROR: {e}")
        finally:
            self.busy(False)
            self.refresh()

    def go_home(self):
        if not messagebox.askokcancel(
                "Move arm?", "Joint move to Home (J1-J6 = 0, 0, 0, 0, 45, 0)?\n\n"
                "This unwinds the wrist. The pen rises away from the board. "
                "Hand on the E-stop."):
            return
        self.busy(True)
        try:
            self.status_var.set("Moving Home ...")
            self.root.update()
            self.arm.move_joints(HOME_JOINTS, int(self.speed.get()))
            self.status_var.set("Home. Now press 'Point pen straight down'.")
        except AR4Error as e:
            self.status_var.set(f"ERROR: {e}")
        finally:
            self.busy(False)
            self.refresh()

    def goto_marker(self, m):
        q = self.data["markers"].get(str(m))
        if not q:
            messagebox.showerror("Not taught", f"Marker {m} has not been saved yet.")
            return
        if not messagebox.askokcancel(
                "Move arm?", f"Lift, then move the pen to 20 mm above marker {m}?\n\n"
                "Hand on the E-stop."):
            return
        self.busy(True)
        try:
            p = self.arm.pose()
            safe_z = max(p.z, q["z"] + 60)
            self.status_var.set(f"Moving above marker {m} ...")
            self.root.update()
            self.arm.move_linear(p.x, p.y, safe_z, *PEN_DOWN_ROT, speed_pct=int(self.speed.get()))
            self.arm.move_linear(q["x"], q["y"], safe_z, q["rz"], q["ry"], q["rx"],
                                 speed_pct=int(self.speed.get()))
            self.arm.move_linear(q["x"], q["y"], q["z"] + 20, q["rz"], q["ry"], q["rx"],
                                 speed_pct=int(self.speed.get()))
            self.status_var.set(f"20 mm above marker {m}. The pen tip should be over its centre.")
        except AR4Error as e:
            self.status_var.set(f"ERROR: {e}")
        finally:
            self.busy(False)
            self.refresh()

    def _save(self):
        out = dict(self.data)
        out["saved"] = time.strftime("%Y-%m-%d %H:%M:%S")
        out["port"] = self.arm.port
        ms = self.data["markers"]
        if len(ms) >= 4:
            coef, res = plane_fit([(q["x"], q["y"], q["z"]) for q in ms.values()])
            out["board_plane"] = {"a": coef[0], "b": coef[1], "c": coef[2],
                                  "residual_mm": {k: float(r) for k, r in zip(ms, res)}}
        with open(CALIB_FILE, "w") as f:
            json.dump(out, f, indent=2)
        self.arm.sync_app_config()
        self.show_saved()

    @staticmethod
    def _record(p):
        """Pose to store: the commanded pen angle replaces the unreliable readback."""
        d = p.as_dict()
        d["reported_rot"] = [d["rz"], d["ry"], d["rx"]]
        d["rz"], d["ry"], d["rx"] = PEN_DOWN_ROT
        return d

    def save_marker(self, m):
        p = self.refresh()
        if p is None:
            return
        if not messagebox.askyesno("Save?", f"Pen tip touching the CENTRE of marker {m}?\n\n{p}"):
            return
        self.data["markers"][str(m)] = self._record(p)
        self._save()
        self.status_var.set(f"Saved marker {m}. Lift 30 mm before moving to the next marker.")

    def save_park(self):
        p = self.refresh()
        if p is None:
            return
        ms = self.data["markers"]
        if ms and p.z < max(q["z"] for q in ms.values()) + 80:
            messagebox.showerror("Too low", "PARK must be at least 80 mm above the board.")
            return
        self.data["park"] = self._record(p)
        self._save()
        self.status_var.set("Saved PARK.")

    def quit(self):
        try:
            self.arm.sync_app_config()
        except AR4Error:
            pass
        finally:
            self.arm.close()
            self.root.destroy()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect the Teensy)")
    args = ap.parse_args()
    root = tk.Tk()
    TeachApp(root, args.port)
    root.mainloop()
