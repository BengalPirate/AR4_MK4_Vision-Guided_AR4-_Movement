"""Live camera view with ArUco marker overlay, for aiming the webcam.

Run:  python camera_check.py [--device /dev/video4]
Shows each detected marker ID and a banner saying whether all four board
markers (4, 5, 6, 7) are visible. Press S to save a snapshot, Q or Esc to quit.
"""
import argparse
import glob
import os
import time

import cv2

BOARD_IDS = (4, 5, 6, 7)
CAMERA_NAME = "Logitech Webcam C930e"


def find_camera(name=CAMERA_NAME):
    """Return the /dev/videoN capture node of the named camera.

    Prefers the direct USB device (no ':' in its V4L2 name) over the
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


def open_camera(device, width=1280, height=720):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {device}")
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="video device (default: find the C930e)")
    args = ap.parse_args()
    device = args.device or find_camera()
    cap = open_camera(device)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    win = f"Camera check ({device}) - S snapshot, Q quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed")
            break
        corners, ids, _ = detector.detectMarkers(frame)
        seen = set()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            seen = {int(i) for i in ids.flatten()}
        missing = [i for i in BOARD_IDS if i not in seen]
        text = "ALL 4 MARKERS VISIBLE" if not missing else f"MISSING: {missing}"
        color = (0, 180, 0) if not missing else (0, 0, 255)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 50), (0, 0, 0), -1)
        cv2.putText(frame, text, (15, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2)
        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            path = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
            cv2.imwrite(path, frame)
            print("Saved", path)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
