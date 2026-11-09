"""Build the Checkpoint 1 team report PDF from the logged results.

Run from the repository root:  python docs/make_report.py
Writes Team2_Checkpoint1_Report.pdf. Needs reportlab.
"""
import csv
import json
import math
import os
import statistics

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_URL = "https://github.com/BengalPirate/AR4_MK4_Vision-Guided_AR4-_Movement"
VIDEO_URL = "https://www.youtube.com/watch?v=LK-3bo5Rrmg"
OUT = os.path.join(ROOT, "Team2_Checkpoint1_Report.pdf")

ss = getSampleStyleSheet()
H1, H2, BODY = ss["Title"], ss["Heading2"], ss["BodyText"]
SMALL = ParagraphStyle("small", parent=BODY, fontSize=8.5, leading=10.5)
CELL = ParagraphStyle("cell", parent=BODY, fontSize=8.5, leading=10.5)


def P(t, st=BODY):
    return Paragraph(t, st)


def table(rows, widths, header=True):
    data = [[P(str(c), CELL) for c in r] for r in rows]
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    style = [("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
             ("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]
    if header:
        style.append(("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eef6")))
    t.setStyle(TableStyle(style))
    return t


def img(name, width):
    from reportlab.lib.utils import ImageReader
    path = os.path.join(ROOT, "docs", name)
    w, h = ImageReader(path).getSize()
    return Image(path, width=width, height=width * h / w)


def load():
    with open(os.path.join(ROOT, "results", "moves.csv")) as f:
        rows = list(csv.DictReader(f))
    errs = []
    for r in rows:
        if r["event"] == "error" and not (errs and all(
                r[k] == errs[-1][k] for k in ("target_x", "target_y", "rep_x", "rep_y"))):
            errs.append(r)
    with open(os.path.join(ROOT, "calib.json")) as f:
        calib = json.load(f)
    return rows, errs, calib


def main():
    rows, errs, calib = load()
    f = lambda r, k: float(r[k])
    hovers = [r for r in rows if r["event"] == "hover" and r["result"] == "ok"]
    track = [math.dist((f(r, "cmd_x"), f(r, "cmd_y"), f(r, "cmd_z")),
                       (f(r, "rep_x"), f(r, "rep_y"), f(r, "rep_z"))) for r in hovers]
    # one error per distinct location (first visit), and the repeat group
    locs, groups = [], {}
    for r in errs:
        key = (round(f(r, "target_x")), round(f(r, "target_y")))
        g = next((k for k in groups if math.dist(k, key) < 5), None)
        if g is None:
            groups[key] = [r]
            locs.append(r)
        else:
            groups[g].append(r)
    loc_e = [f(r, "error_mm") for r in locs]
    rep_key, rep = max(groups.items(), key=lambda kv: len(kv[1]))
    rep_e = [f(r, "error_mm") for r in rep]
    rep_cmd = [(f(r, "cmd_x"), f(r, "cmd_y")) for r in rep]
    cmd_spread = max(math.dist(a, b) for a in rep_cmd for b in rep_cmd)
    res = calib["board_plane"]["residual_mm"]
    m = calib["markers"]

    doc = SimpleDocTemplate(OUT, pagesize=letter, leftMargin=0.8 * inch,
                            rightMargin=0.8 * inch, topMargin=0.7 * inch,
                            bottomMargin=0.7 * inch,
                            title="Team 2 - Vision-Guided AR4 Chess, Checkpoint 1")
    W = doc.width
    s = []
    s += [P("Vision-Guided AR4 Chess – Checkpoint 1", H1),
          P("<b>Team 2:</b> Brandon Newton, Roshan Padhan, Andrew Overton<br/>"
            f"<b>Repository:</b> <link href='{REPO_URL}'>{REPO_URL}</link><br/>"
            f"<b>Video:</b> <link href='{VIDEO_URL}'>{VIDEO_URL}</link> "
            "(camera-guided moves to different targets)<br/>"
            "<b>Robot:</b> Annin AR4 MK4, Teensy 4.1, Annin firmware 6.7.1 &nbsp; "
            "<b>Camera:</b> Logitech C930e (1280×720) &nbsp; <b>Date:</b> November 9, 2026"),
          Spacer(1, 8),
          P("<b>Summary.</b> A fixed webcam sees the board with four ArUco markers. The program "
            "finds a flat pink target in the live image, maps its pixel position to robot "
            "coordinates through a homography taught by touching the markers with the pen, "
            "checks the pose is safe and reachable, and moves the physical AR4 so the pen "
            "pointer hovers 15 mm above the target (and optionally touches it). Moving the "
            "target moves the commanded position. Uncertain detections and unreachable "
            f"targets are refused. Across {len(locs)} target locations the median "
            f"positioning error was {statistics.median(loc_e):.1f} mm."),
          ]

    s += [P("1. System", H2),
          P("Data flows top to bottom. All motion runs at 10 % speed. The firmware is "
            "unmodified; all checks are in our Python code."),
          img("system_diagram.png", W * 0.62)]

    s += [PageBreak(), P("2. Calibration and camera-to-robot mapping", H2),
          P("<b>Arm calibration.</b> Annin AR4 HMI 6.7 Auto Calibrate (limit switches), "
            "then the HMI is closed."),
          P("<b>Board calibration (teach.py).</b> The pen is held at a fixed angle "
            "(Rz 0°, Ry 170°, Rx 0°) and jogged until its tip just touches the centre of each "
            "marker. The robot X/Y/Z is saved for markers 4–7, plus a PARK pose above the "
            "board. Because the pen angle never changes, the flange-to-tip offset is a "
            "constant already contained in the taught points: the pen length is never "
            "measured, and no tape-measured marker layout is needed."),
          P("<b>Mapping (vision.py).</b> In every frame the four marker centres are detected "
            "(OpenCV ArUco, DICT_4X4_50) and a homography H maps their pixels onto the taught "
            "robot X/Y. The board is a plane, so H maps any board pixel to robot X/Y. "
            "Recomputing H each frame tolerates small camera bumps. Board height is a "
            "least-squares plane z = a·x + b·y + c through the four touch heights; hover and "
            "travel heights are measured from it. The target centre is the centroid of its "
            "outline after projection into robot coordinates (no perspective bias)."),
          table([["Marker", "Robot X (mm)", "Robot Y (mm)", "Touch Z (mm)", "Plane residual (mm)"]] +
                [[k, f"{m[k]['x']:.1f}", f"{m[k]['y']:.1f}", f"{m[k]['z']:.1f}",
                  f"{res[k]:+.2f}"] for k in sorted(m)],
                [W * 0.14, W * 0.2, W * 0.2, W * 0.2, W * 0.26]),
          Spacer(1, 6),
          P("<b>Validation of the mapping.</b> Each taught marker, seen by the camera and "
            "mapped through H, returns its taught robot position to within 0.1 mm (H is "
            "exact at the four markers). The live error at other points is measured below. "
            "A second teaching session was also tried; against the same measured points "
            "it gave 15–18 mm errors (one marker about 23 mm off), so the first was kept."),
          img("board_map.png", W * 0.78)]

    s += [PageBreak(), P("3. Test results", H2),
          P("<b>Method.</b> For each trial the target was placed on the board, the program "
            "moved the pen above it (G), and the operator nudged the pen in 1 mm steps until "
            "the tip was exactly over the target centre, then recorded it (E). The nudge is "
            "the positioning error. Every move is logged in results/moves.csv."),
          table([["Measure", "Result"],
                 ["Board plane fit (4 markers)",
                  f"max residual {max(abs(v) for v in res.values()):.2f} mm"],
                 [f"Commanded vs. controller-reported position ({len(track)} hovers)",
                  f"max {max(track):.2f} mm"],
                 ["Detection spread when a move starts",
                  f"max {max(f(r, 'spread_mm') for r in hovers):.1f} mm"],
                 [f"Positioning error, {len(locs)} target locations",
                  f"median {statistics.median(loc_e):.1f} mm, mean {statistics.mean(loc_e):.1f} mm, "
                  f"max {max(loc_e):.1f} mm"],
                 [f"Repeatability at ({rep_key[0]}, {rep_key[1]}) mm, {len(rep)} repeats",
                  f"commanded spread {cmd_spread:.2f} mm; errors "
                  + ", ".join(f"{e:.1f}" for e in rep_e) + " mm"]],
                [W * 0.5, W * 0.5]),
          Spacer(1, 6),
          table([["#", "Target X", "Target Y", "Correction dX", "dY", "Error (mm)"]] +
                [[i, f"{f(r, 'target_x'):.1f}", f"{f(r, 'target_y'):.1f}",
                  f"{f(r, 'corr_x'):+.1f}", f"{f(r, 'corr_y'):+.1f}", f"{f(r, 'error_mm'):.1f}"]
                 for i, r in enumerate(errs, 1)],
                [W * 0.08] + [W * 0.184] * 5),
          Spacer(1, 6),
          img("errors.png", W * 0.95),
          P("<b>Interpretation.</b> The arm reaches what it is commanded to within 1 mm, and "
            "the detection is steady to 0.5 mm, so the error comes from the calibration "
            "(how exactly the pen tip was centred on each marker while teaching) and the "
            "taped pen. Two repeats at one location needed about 16 mm in +Y although the "
            "commanded and reported positions were identical to 0.1 mm; the other four "
            "repeats were within 2.8 mm. The cause is not confirmed (pen shifting in its "
            "tape, J1 backlash, or an operator nudging error) and is listed under remaining work.",
            SMALL)]

    refusals = [r for r in rows if r["result"] in ("rejected", "refused", "stopped", "estop",
                                                     "override")]
    s += [PageBreak(), P("4. Safety checks and failure handling", H2),
          table([["Layer", "Behaviour"],
                 ["Speed", "All moves at 10 %; the serial library refuses more than 25 %."],
                 ["Uncertain detection", "Refused unless all 4 markers are visible, exactly one "
                  "target is on the board, its size (150–6000 mm²) and shape (solidity ≥ 0.85) "
                  "are plausible, and 15 frames agree within 3 mm."],
                 ["Workspace", "Target at least 15 mm inside the marker rectangle; nothing "
                  "below the board plane; travel at 100 mm (or the highest reachable of 80/60/40 mm)."],
                 ["Reach and joint limits", "Every straight-line move is simulated in 2 mm steps "
                  "with Annin's kinematics before it is sent; out-of-reach or out-of-limit moves are "
                  "refused. J4 ±45° and J6 −90…+45° protect the taped pen."],
                 ["Wrist singularity", "Where a straight line would make the firmware swing J4/J6 "
                  "half a turn, the same pose is reached by a joint move using the wrist solution "
                  "nearest the current one."],
                 ["Pen-angle guard", "Moves that would also turn the pen by more than 5° are refused "
                  "(avoids a full wrist sweep caused by Euler-angle wrap near Ry 180°)."],
                 ["Firmware", "Unreachable / out-of-limit poses refused by the controller (ER / EL) "
                  "without motion."],
                 ["Software stop", "SPACE cancels the rest of the sequence before the next move."],
                 ["E-stop", "Hardware interrupt stops the move; the program detects the EB flag. "
                  "Recovery: release, R (retreat up), P (park)."],
                 ["Interlock", "Exclusive board lock: a second arm controller is refused until "
                  "the first parks."],
                 ["Operator override", "Third repeat of a refused action within 20 s is sent "
                  "anyway; logged. Firmware limits still apply."]],
                [W * 0.24, W * 0.76]),
          Spacer(1, 6),
          P("<b>Limit test before camera moves (check_limits.py).</b> Off-board, 700 mm "
            "out-of-reach, base-axis and below-board points were rejected in software; an "
            "unreachable pose sent to the firmware was refused with ER and no motion; the pen "
            "then hovered over markers 4, 5 and 6 with commanded vs. reported differences of "
            "0.37, 0.13 and 0.09 mm, and parked."),
          P("<b>Logged refusals and stops during the session.</b> The 'wrist singularity' "
            "refusals before 12:30 come from an earlier version that refused near J5 = 0; it "
            "was replaced by the joint-move fallback and the pen-safe J4/J6 limits. At 13:07 "
            "the operator override was used to reach a target in the joint-limit band."),
          table([["Time", "Event", "Result", "Reason"]] +
                [[r["time"][11:], r["event"], r["result"], r["note"]] for r in refusals],
                [W * 0.13, W * 0.2, W * 0.13, W * 0.54])]

    s += [PageBreak(), P("5. Remaining work (checkpoint 2 and final)", H2),
          P("• Replace the colour target with chess-square detection: an 8×8 grid laid over "
            "the board through the same homography, and visual piece identification.<br/>"
            "• Fit the servo gripper and teach grasp heights per piece; re-teach the board "
            "with the gripper's tool point.<br/>"
            "• Improve calibration accuracy: teach more points (marker corners), or run the "
            "camera-loop refinement (refine.py) from measured trials, then validate.<br/>"
            "• Find the cause of the two 16 mm repeat outliers: mark touch dots on paper, "
            "check J1 backlash and secure the pen rigidly.<br/>"
            "• Chess program (move generation) and game flow for the November 19 demonstration."),
          P("6. Team member contributions", H2),
          table([["Member", "Part of the system they explain and operate"],
                 ["Brandon Newton", "Calibration and teaching (jogging the pen onto the markers, "
                  "board-plane fit, PARK); demo operation (live runs, error and repeatability trials)."],
                 ["Roshan Padhan", "Camera: mounting and aiming so all four markers are visible, "
                  "ArUco detection, camera-to-board mapping."],
                 ["Andrew Overton", "Safety testing: limit test, workspace and joint-limit checks, "
                  "rejection of uncertain or unreachable targets, software stop, E-stop and recovery."]],
                [W * 0.22, W * 0.78]),
          P("7. External code and AI assistance", H2),
          P("• Annin Robotics firmware 6.7.1 and the AR4 HMI 6.7 kinematics module, unmodified "
            "(Annin Open Source Non-Commercial License).<br/>"
            "• Board and marker approach adapted from another team's Milestone 4 work "
            "(github.com/MosesJames46/Embedded-Systems-ARM4-MK4); our implementation is "
            "separate.<br/>"
            "• ar4link.py and the jog tool extend our writing project "
            "(github.com/BengalPirate/AR4_MK4_Team-Writing).<br/>"
            "• The code and documentation were written with Claude (Anthropic) in Claude Code, "
            "directed by the team. Testing was done on the physical robot: the limit test, "
            "live camera-guided trials, and each failure case in section 4, all recorded in "
            "results/moves.csv. Issues found in testing (wrist sweep at Ry 180°, wrist "
            "singularity near marker 7, pen-safe wrist limits, reach with the opposite pen "
            "tilt) were fixed and re-tested on the arm.")]

    doc.build(s)
    print("Wrote", OUT)


if __name__ == "__main__":
    main()
