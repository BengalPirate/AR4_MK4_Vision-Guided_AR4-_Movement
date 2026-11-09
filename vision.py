"""Camera, ArUco board mapping and coloured-target detection.

Mapping: the four board markers are found in every frame. A homography maps
their pixel centres onto the robot X/Y positions taught in calib.json (pen tip
on each marker centre), so any pixel on the board plane maps straight to robot
X/Y. Board height comes from a plane fitted through the taught Z values.
Recomputing the homography every frame means a slightly bumped camera does not
invalidate the calibration; a moved board or robot does.
"""
import glob
import json
import os
from collections import deque

import cv2
import numpy as np

import config


def find_camera(name=config.CAMERA_NAME):
    """Return the /dev/videoN capture node of the named camera.

    Prefers the direct USB passthrough device (no ':' in its V4L2 name) over the
    Parallels-virtualised copy, which does not deliver frames."""
    matches = []
    for d in sorted(glob.glob("/sys/class/video4linux/video*")):
        with open(os.path.join(d, "name")) as f:
            dev_name = f.read().strip()
        with open(os.path.join(d, "index")) as f:
            index = int(f.read())
        if name in dev_name and index == 0:
            matches.append((":" in dev_name, "/dev/" + os.path.basename(d)))
    if not matches:
        raise RuntimeError(f"No camera named {name!r} found")
    return sorted(matches)[0][1]


def open_camera(device=None, size=config.FRAME_SIZE):
    device = device or find_camera()
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {device}")
    return cap, device


def load_calib(path=config.CALIB_FILE):
    if not os.path.exists(path):
        raise RuntimeError("calib.json not found - run teach.py first")
    with open(path) as f:
        c = json.load(f)
    missing = [m for m in config.MARKER_IDS if str(m) not in c.get("markers", {})]
    if missing:
        raise RuntimeError(f"calib.json has no taught position for markers {missing}")
    if not c.get("park"):
        raise RuntimeError("calib.json has no PARK pose - save one in teach.py")
    return c


class BoardMapper:
    """Pixel -> robot X/Y through the live marker homography."""

    def __init__(self, calib):
        self.calib = calib
        m = calib["markers"]
        self.robot_xy = {i: (m[str(i)]["x"], m[str(i)]["y"]) for i in config.MARKER_IDS}
        pl = calib["board_plane"]
        self.plane = (pl["a"], pl["b"], pl["c"])
        q = m[str(config.MARKER_IDS[0])]
        self.tool_rot = (q["rz"], q["ry"], q["rx"])
        # Optional refinement fitted by refine.py from measured trials:
        # commanded = A @ camera_xy + b
        corr = calib.get("correction")
        self.corr_A = np.array(corr["A"]) if corr else np.eye(2)
        self.corr_b = np.array(corr["b"]) if corr else np.zeros(2)
        self.correction_note = f"refined ({corr['model']}, {corr['n']} trials)" if corr \
            else "unrefined"
        # Workspace polygon: the taught marker centres in angular order
        pts = np.array(list(self.robot_xy.values()), np.float32)
        c = pts.mean(axis=0)
        order = np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
        self.polygon = pts[order].reshape(-1, 1, 2)
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters())
        self.H = None
        self.H_inv = None
        self.corners, self.ids = None, None
        self.visible = set()

    def update(self, frame):
        """Detect the markers; return True if all four are visible."""
        self.corners, self.ids, _ = self.detector.detectMarkers(frame)
        self.H = self.H_inv = None
        self.visible = set()
        if self.ids is None:
            return False
        px, rob = [], []
        for c, i in zip(self.corners, self.ids.flatten()):
            i = int(i)
            if i in self.robot_xy and i not in self.visible:
                self.visible.add(i)
                px.append(c.reshape(4, 2).mean(axis=0))
                rob.append(self.robot_xy[i])
        if len(px) < 4:
            return False
        self.H = cv2.getPerspectiveTransform(np.float32(px), np.float32(rob))
        self.H_inv = np.linalg.inv(self.H)
        return True

    def correct(self, x, y):
        """Camera estimate of a board point -> commanded robot X/Y (refinement applied)."""
        return tuple(self.corr_A @ np.array([x, y]) + self.corr_b)

    def to_robot(self, pts):
        """Pixels (N x 2) -> robot X/Y (N x 2)."""
        p = np.asarray(pts, np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.H).reshape(-1, 2)

    def to_pixel(self, pts):
        p = np.asarray(pts, np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.H_inv).reshape(-1, 2)

    def board_z(self, x, y):
        a, b, c = self.plane
        return a * x + b * y + c

    def inside_margin(self, x, y):
        """Signed distance (mm) from the board edge; positive is inside."""
        return cv2.pointPolygonTest(self.polygon, (float(x), float(y)), True)

    def draw(self, frame):
        if self.ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, self.corners, self.ids)
        if self.H is not None:
            poly = self.to_pixel(self.polygon.reshape(-1, 2)).astype(np.int32)
            cv2.polylines(frame, [poly], True, (255, 200, 0), 1)


class TargetDetector:
    """Finds flat coloured targets and rejects ambiguous ones."""

    def __init__(self, lo=config.TARGET_HSV_LO, hi=config.TARGET_HSV_HI):
        self.lo, self.hi = np.array(lo), np.array(hi)
        self.mask = None

    def pick(self, frame, x, y, tol=(10, 60, 60)):
        """Set the colour range from a clicked pixel (median of a 9x9 patch)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        patch = hsv[max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3)
        h, s, v = np.median(patch, axis=0).astype(int)
        self.lo = np.array([max(0, h - tol[0]), max(40, s - tol[1]), max(40, v - tol[2])])
        self.hi = np.array([min(179, h + tol[0]), 255, 255])
        return (h, s, v)

    def detect(self, frame, mapper):
        """Return (candidates, rejected) where each candidate is a dict with
        pixel centre 'px', robot 'xy', 'area_mm2' and 'contour'."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        if self.lo[0] > self.hi[0]:   # hue range wraps past 179
            mask = cv2.inRange(hsv, self.lo, (179, 255, 255)) | \
                cv2.inRange(hsv, (0, self.lo[1], self.lo[2]), self.hi)
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        self.mask = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cands, rejected = [], []
        for c in contours:
            if cv2.contourArea(c) < 40:
                continue
            rob = mapper.to_robot(c.reshape(-1, 2))
            area = float(cv2.contourArea(rob.astype(np.float32)))
            hull = cv2.convexHull(c)
            solidity = cv2.contourArea(c) / max(cv2.contourArea(hull), 1)
            m = cv2.moments(c)
            px = (m["m10"] / m["m00"], m["m01"] / m["m00"])
            # Centre in robot space: centroid of the projected outline, which is
            # the true centre of the square (pixel centroids are biased by perspective)
            mr = cv2.moments(rob.astype(np.float32))
            xy = (mr["m10"] / mr["m00"], mr["m01"] / mr["m00"]) if mr["m00"] else \
                tuple(mapper.to_robot([px])[0])
            d = {"px": px, "xy": xy, "area_mm2": area, "contour": c, "solidity": solidity}
            lo_a, hi_a = config.TARGET_AREA_MM2
            if not lo_a <= area <= hi_a:
                d["why"] = f"size {area:.0f} mm2"
                rejected.append(d)
            elif solidity < config.TARGET_MIN_SOLIDITY:
                d["why"] = f"shape (solidity {solidity:.2f})"
                rejected.append(d)
            else:
                cands.append(d)
        return cands, rejected


class Stability:
    """Accepts a position only after it has been steady for several frames."""

    def __init__(self, n=config.STABLE_FRAMES, spread=config.STABLE_SPREAD_MM):
        self.n, self.spread = n, spread
        self.hist = deque(maxlen=n)

    def reset(self):
        self.hist.clear()

    def add(self, xy):
        self.hist.append(xy)

    def state(self):
        """Return (stable, mean_xy, spread_mm)."""
        if not self.hist:
            return False, None, None
        pts = np.array(self.hist)
        mean = pts.mean(axis=0)
        spread = float(np.linalg.norm(pts - mean, axis=1).max() * 2)
        return (len(pts) == self.n and spread <= self.spread), tuple(mean), spread
