"""Regenerate the README/report figures in docs/.

Run from the repository root:  python docs/make_figures.py
  docs/system_diagram.png   data flow from camera to robot
  docs/board_map.png        taught markers, reachable board area, test results
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import kin                                  # noqa: E402
from vision import BoardMapper, load_calib  # noqa: E402

DOCS = os.path.join(ROOT, "docs")


def system_diagram():
    boxes = [
        ("Logitech C930e webcam\n1280x720, fixed mount", "#dbe9f6"),
        ("vision.py\nArUco markers 4-7 + taught positions (calib.json) -> homography\n"
         "pixel -> robot X/Y (every frame)\n"
         "HSV target, size/shape tests, 15-frame stability", "#dbe9f6"),
        ("vision_demo.py\nboard / reach / joint-limit checks\nrise -> travel -> descend -> "
         "hover/touch -> park\nboard interlock, operator override", "#e3f1df"),
        ("ar4link.py + kin.py\npen-angle guard, path simulation with Annin kinematics\n"
         "line move, or joint move near the wrist singularity", "#e3f1df"),
        ("Teensy 4.1, Annin firmware 6.7.1\nIK, joint limits, E-stop interrupt (pin 39)",
         "#f6e3d6"),
        ("AR4 MK4 arm\npen pointer taped to J6", "#f6e3d6"),
    ]
    fig, ax = plt.subplots(figsize=(7.2, 8.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, len(boxes) * 1.6)
    ax.axis("off")
    for i, (text, color) in enumerate(boxes):
        y = (len(boxes) - 1 - i) * 1.6 + 0.2
        ax.add_patch(FancyBboxPatch((0.6, y), 8.8, 1.15, boxstyle="round,pad=0.05",
                                    fc=color, ec="#444", lw=1))
        ax.text(5, y + 0.575, text, ha="center", va="center", fontsize=9)
        if i:
            ax.annotate("", xy=(5, y + 1.2), xytext=(5, y + 1.6),
                        arrowprops=dict(arrowstyle="->", lw=1.4, color="#444"))
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "system_diagram.png"), dpi=150)
    plt.close(fig)


def board_map():
    calib = load_calib()
    mp = BoardMapper(calib)
    rot, park = mp.tool_rot, calib["park"]["joints"]
    ok_pts, bad_pts = [], []
    for x in range(260, 580, 8):
        for y in range(-330, 60, 8):
            if mp.inside_margin(x, y) < 0:
                continue
            t = kin.travel_height([(x, y)], mp.board_z, rot, park)
            s = t and kin.ik((x, y, mp.board_z(x, y) + t), rot, park)
            e = s and kin.within_limits(s) and \
                kin.joint_target(s, (x, y, mp.board_z(x, y)), rot)
            (ok_pts if e else bad_pts).append((x, y))
    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    if ok_pts:
        ax.scatter([p[1] for p in ok_pts], [p[0] for p in ok_pts], s=6, c="#b9dcb0",
                   label="reachable (pen-safe wrist limits)")
    if bad_pts:
        ax.scatter([p[1] for p in bad_pts], [p[0] for p in bad_pts], s=6, c="#f0a8a0",
                   label="refused: J4/J6 limit (pen)")
    poly = mp.polygon.reshape(-1, 2)
    ax.plot(list(poly[:, 1]) + [poly[0, 1]], list(poly[:, 0]) + [poly[0, 0]], "k-", lw=1)
    for k, q in calib["markers"].items():
        ax.plot(q["y"], q["x"], "ks", ms=9)
        ax.annotate(f"ID {k}", (q["y"], q["x"]), xytext=(6, 6), textcoords="offset points")
    log = os.path.join(ROOT, "results", "moves.csv")
    if os.path.exists(log):
        with open(log) as f:
            errs = [r for r in csv.DictReader(f) if r["event"] == "error"]
        if errs:
            sc = ax.scatter([float(r["target_y"]) for r in errs],
                            [float(r["target_x"]) for r in errs],
                            c=[float(r["error_mm"]) for r in errs], cmap="viridis",
                            s=60, edgecolors="k", zorder=3, label="test targets")
            fig.colorbar(sc, ax=ax, label="positioning error (mm)")
    ax.set_xlim(80, -360)
    ax.set_xlabel("Robot Y (mm)  <- robot's left | robot's right ->")
    ax.set_ylabel("Robot X (mm, away from base)")
    ax.set_aspect("equal")
    ax.set_title("Board in robot coordinates (pen tip)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "board_map.png"), dpi=150)
    plt.close(fig)
    return len(ok_pts), len(bad_pts)


if __name__ == "__main__":
    system_diagram()
    print("board map: reachable/refused points", board_map())
