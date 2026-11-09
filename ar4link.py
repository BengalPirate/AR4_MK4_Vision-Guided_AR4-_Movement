"""Minimal serial link to an AR4 Teensy running Annin firmware 6.7.x.

Every read has a timeout, so a missing reply raises an error instead of
freezing the program (unlike the stock HMI).
"""
import glob
import json
import os
import subprocess
import time

import serial

try:   # path simulation with Annin's kinematics (needs the AR4 HMI 6.7 folder)
    import kin
except ImportError as e:
    kin = None
    print(f"WARNING: kin.py unavailable ({e}) - straight-line moves are not pre-checked")

BAUD = 9600
MAX_ROT_CHANGE_DEG = 5.0   # max orientation change allowed in one straight-line move
TEENSY_VID, TEENSY_SERIAL_PID = 0x16C0, 0x0483
APP_CONFIG = os.environ.get(
    "AR4_APP_CONFIG",
    os.path.expanduser("~/AR4/AR4 HMI Interface 6.7 source/ARconfig.json"))


def find_port():
    """Find the Teensy's serial port, preferring the stable /dev/serial/by-id path
    (ttyACM numbers change on reconnect)."""
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Teensyduino_USB_Serial_*-if00"))
    if not ports:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()
                 if p.vid == TEENSY_VID and p.pid == TEENSY_SERIAL_PID]
    if not ports:
        raise RuntimeError("No Teensy found - is it plugged in (and passed through, "
                           "if using a VM)?")
    if len(ports) > 1:
        raise RuntimeError(f"More than one Teensy connected, pass --port: {ports}")
    return ports[0]


def stock_app_running():
    """True if the stock AR4 HMI (AR4.py) is running."""
    out = subprocess.run(["ps", "-axo", "args="], capture_output=True, text=True).stdout
    for line in out.splitlines():
        argv = line.split()
        if len(argv) > 1 and "python" in os.path.basename(argv[0]) \
                and os.path.basename(argv[1]) == "AR4.py":
            return True
    return False


class Pose:
    """Joint angles (deg) and tool pose (mm / deg) as reported by RP."""

    def __init__(self, reply):
        keys = "ABCDEFGHIJKL"
        idx = [reply.index(k) for k in keys] + [reply.index("M")]
        vals = [float(reply[idx[i] + 1:idx[i + 1]]) for i in range(12)]
        self.joints = vals[:6]
        self.x, self.y, self.z, self.rz, self.ry, self.rx = vals[6:]
        # Alarm flag between "O" and "P" ("EB" = E-stop pressed during the move)
        self.flag = reply[reply.rindex("O") + 1:reply.rindex("P")].strip() \
            if "O" in reply and "P" in reply else ""

    @property
    def xyz(self):
        return (self.x, self.y, self.z)

    @property
    def rot(self):
        return (self.rz, self.ry, self.rx)

    def as_dict(self):
        return {"joints": self.joints, "x": self.x, "y": self.y, "z": self.z,
                "rz": self.rz, "ry": self.ry, "rx": self.rx}

    def __str__(self):
        j = " ".join(f"J{i + 1}={a:.2f}" for i, a in enumerate(self.joints))
        return (f"X={self.x:.2f} Y={self.y:.2f} Z={self.z:.2f} "
                f"Rz={self.rz:.2f} Ry={self.ry:.2f} Rx={self.rx:.2f} | {j}")


class AR4Error(RuntimeError):
    pass


class EStopError(AR4Error):
    pass


class AR4:
    def __init__(self, port=None):
        if stock_app_running():
            raise RuntimeError("The stock AR4 app is open. Close it first - two programs "
                               "on one port corrupt the arm's position.")
        self.port = port or find_port()
        self.ser = serial.Serial(self.port, BAUD, timeout=0.1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _cmd(self, cmd, timeout=3.0):
        """Send one command and return its reply line. Never blocks past timeout."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        buf = b""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            buf += self.ser.read(self.ser.in_waiting or 1)
            if buf.endswith(b"\n"):
                break
        reply = buf.decode(errors="replace").strip()
        if not reply:
            raise AR4Error(f"No reply to {cmd[:2]} within {timeout:.0f}s "
                           "(arm still moving, E-stop pressed, or board hung)")
        return reply

    def hello(self):
        return json.loads(self._cmd("HO"))

    def pose(self):
        reply = self._cmd("RP")
        if not reply.startswith("A"):
            # 6.7.1 answers the first request after a refused move with a stale error
            time.sleep(0.3)
            reply = self._cmd("RP")
        if not reply.startswith("A"):
            raise AR4Error(f"Position request returned an alarm: {reply}")
        return Pose(reply)

    def limit_switches(self):
        return self._cmd("TL")

    def _move(self, kind, x, y, z, rz, ry, rx, speed_pct, timeout):
        # Percent speed only: firmware 6.7.1's mm/s ("Sm") timing math divides by
        # (1.2 * steps - 36), so short moves of ~30 motor steps get near-infinite
        # step delays and the board stops responding.
        if not 1 <= speed_pct <= 25:
            raise ValueError("speed_pct must be 1-25 for safe testing")
        wrist = "F" if self.pose().joints[4] > 0 else "N"
        cmd = (f"{kind}X{x:.3f}Y{y:.3f}Z{z:.3f}Rz{rz:.3f}Ry{ry:.3f}Rx{rx:.3f}"
               f"J70.00J80.00J90.00Sp{speed_pct:.0f}Ac10Dc10Rm50")
        if kind == "ML":
            cmd += "Rnd0"
        cmd += f"W{wrist}Lm111111"
        if kind == "ML":
            cmd += "Q0"
        reply = self._cmd(cmd, timeout)
        if not reply.startswith("A"):
            raise AR4Error(f"Move refused or faulted ({reply}) - target may be "
                           "out of reach or outside joint limits")
        return self._checked(Pose(reply))

    @staticmethod
    def _checked(p):
        if p.flag:
            raise EStopError(f"Move interrupted, alarm flag {p.flag!r} "
                             "(E-stop pressed?). Release the E-stop before moving again.")
        return p

    def move_linear(self, x, y, z, rz, ry, rx, speed_pct=10, force=False):
        """Straight-line move of the tool tip at speed_pct percent of max.

        The firmware interpolates the three orientation angles linearly from the
        reported pose to the target. Near Ry = +-180 the reported angles wrap, so a
        tiny physical change can read as ~360 deg and the wrist sweeps all the way
        round. Such moves are refused: the reported and commanded angles must agree
        within MAX_ROT_CHANGE_DEG. Re-orient with a joint move (move_joints) instead.
        force=True (operator override) skips these software checks; the firmware's
        own limits still apply.
        """
        p = self.pose()
        if force:
            dist = sum((a - b) ** 2 for a, b in zip((x, y, z), p.xyz)) ** 0.5
            return self._move("ML", x, y, z, rz, ry, rx, speed_pct,
                              timeout=30 + 4 * dist / speed_pct)
        diff = max(abs(a - b) for a, b in zip((rz, ry, rx), p.rot))
        if diff > MAX_ROT_CHANGE_DEG:
            raise AR4Error(
                f"Refused: straight-line move would turn the pen {diff:.0f} deg "
                f"(reported Rz/Ry/Rx {p.rz:.1f}/{p.ry:.1f}/{p.rx:.1f}, commanded "
                f"{rz:.1f}/{ry:.1f}/{rx:.1f}). Set the pen angle with a joint move first.")
        if kin is not None:
            ok, kind, why = kin.check_linear(p.joints, (x, y, z), (rz, ry, rx))
            if kind == "flip":
                # The straight line passes near the wrist singularity, where the
                # firmware would swing J4/J6 round mid-move. Go to the same end
                # pose with a smooth joint move instead.
                js = kin.joint_target(p.joints, (x, y, z), (rz, ry, rx))
                if js is None:
                    raise AR4Error(f"Refused before moving: no joint solution ({why})")
                return self.move_joints(js, speed_pct)
            if not ok:
                raise AR4Error(f"Refused before moving: {why}")
        dist = sum((a - b) ** 2 for a, b in zip((x, y, z), p.xyz)) ** 0.5
        return self._move("ML", x, y, z, rz, ry, rx, speed_pct,
                          timeout=10 + 2 * dist / speed_pct)

    def move_joint(self, x, y, z, rz, ry, rx, speed_pct=10):
        """Joint-interpolated move to a pose (path is not a straight line)."""
        dist = sum((a - b) ** 2 for a, b in zip((x, y, z), self.pose().xyz)) ** 0.5
        return self._move("MJ", x, y, z, rz, ry, rx, speed_pct,
                          timeout=20 + 3 * dist / speed_pct)

    def move_joints(self, angles, speed_pct=10):
        """Move to absolute joint angles J1..J6 (deg)."""
        if not 1 <= speed_pct <= 25:
            raise ValueError("speed_pct must be 1-25 for safe testing")
        wrist = "F" if angles[4] > 0 else "N"
        cmd = ("RJ" + "".join(f"{L}{a:.3f}" for L, a in zip("ABCDEF", angles))
               + f"J70.00J80.00J90.00Sp{speed_pct:.0f}Ac10Dc10Rm80W{wrist}Lm111111")
        travel = max(abs(a - b) for a, b in zip(angles, self.pose().joints))
        reply = self._cmd(cmd, timeout=20 + 3 * travel / speed_pct)
        if not reply.startswith("A"):
            raise AR4Error(f"Joint move refused or faulted ({reply})")
        return self._checked(Pose(reply))

    def sync_app_config(self):
        """Store the arm's current joint angles in the HMI config.

        The stock app sends its saved angles to the board on startup, so this
        keeps it from overwriting the calibrated position.
        """
        p = self.pose()
        if not os.path.exists(APP_CONFIG):
            return p
        with open(APP_CONFIG) as f:
            cfg = json.load(f)
        for i, a in enumerate(p.joints, start=1):
            cfg[f"J{i}AngCur"] = f"{a:.3f}"
        cfg["comPort"] = os.path.realpath(self.port)
        with open(APP_CONFIG, "w") as f:
            json.dump(cfg, f, indent=4, ensure_ascii=False)
        return p
