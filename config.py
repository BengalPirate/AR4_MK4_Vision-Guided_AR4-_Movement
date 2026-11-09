"""Settings for the vision-guided move demo. Distances in mm, speeds in percent."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(HERE, "calib.json")
LOG_FILE = os.path.join(HERE, "results", "moves.csv")
LOCK_FILE = os.environ.get("AR4_BOARD_LOCK", "/tmp/ar4_board.lock")

# Camera
CAMERA_NAME = "Logitech Webcam C930e"
FRAME_SIZE = (1280, 720)
MARKER_IDS = (4, 5, 6, 7)          # ArUco DICT_4X4_50 markers taped to the board

# Target: a flat coloured paper square. HSV range for OpenCV (H 0-179).
# Default is the magenta/pink card; click the target in the demo window to re-pick.
TARGET_HSV_LO = (140, 80, 70)
TARGET_HSV_HI = (172, 255, 255)
TARGET_AREA_MM2 = (150, 6000)      # accepted blob area on the board (~12x12 to 77x77 mm)
TARGET_MIN_SOLIDITY = 0.85         # blob area / convex hull area (rejects ragged blobs)

# Stability: a target is accepted only after STABLE_FRAMES consecutive detections
# whose positions all lie within STABLE_SPREAD_MM of each other.
STABLE_FRAMES = 15
STABLE_SPREAD_MM = 3.0

# Workspace limits (checked before every move)
BOARD_MARGIN_MM = 15               # target must lie inside the marker quadrilateral, shrunk by this
IGNORE_OUTSIDE_MM = 50             # objects further outside the board are ignored entirely
REACH_MIN_MM = 150                 # horizontal distance from the base axis
REACH_MAX_MM = 620                 # coarse check; kin.py does the exact one

# Heights above the board plane (pen tip)
SAFE_Z_MM = 100                    # travel height
HOVER_Z_MM = 15                    # hover height above the target
TOUCH_Z_MM = 0.0                   # touch height (0 = taught touch height); never below -2
MIN_Z_MM = -2.0

SPEED_PCT = 10                    # all demo moves; ar4link caps at 25
JOG_STEP_MM = 1.0                  # fine correction step for error measurement
HOLD_SECONDS = 2.0                 # pause at hover in "auto" runs
