"""Summarise results/moves.csv into accuracy and repeatability numbers and a plot.

Run:  python analyze.py
Writes results/summary.md and results/errors.png.

- Positioning error: each "error" row is the jog correction (mm) the operator
  needed to put the pen exactly over the target centre after a camera-guided move.
- Repeatability: error rows are grouped by target location (within 15 mm); the
  spread of the corrected positions within a group shows how repeatably the
  camera + arm reach the same place.
- Controller tracking: commanded vs reported position for every hover.
"""
import csv
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")


def rows(path=os.path.join(RES, "moves.csv")):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    return float(r[k]) if r.get(k) not in (None, "") else math.nan


def main():
    data = rows()
    errs = []
    for r in data:   # drop E pressed twice for the same hover (identical row)
        if r["event"] == "error" and not (errs and all(
                r[k] == errs[-1][k] for k in ("target_x", "target_y", "rep_x", "rep_y"))):
            errs.append(r)
    hovers = [r for r in data if r["event"] == "hover" and r["result"] == "ok"]
    refused = [r for r in data if r["result"] in ("rejected", "refused")]
    out = ["# Test results", ""]

    if errs:
        e = np.array([fnum(r, "error_mm") for r in errs])
        dx = np.array([fnum(r, "corr_x") for r in errs])
        dy = np.array([fnum(r, "corr_y") for r in errs])
        out += ["## Positioning error (camera target -> pen tip)", "",
                f"- Trials: {len(e)}",
                f"- Mean {e.mean():.1f} mm, RMS {math.sqrt((e ** 2).mean()):.1f} mm, "
                f"max {e.max():.1f} mm",
                f"- Mean offset (bias): dX {dx.mean():+.1f} mm, dY {dy.mean():+.1f} mm", "",
                "| # | Time | Target X | Target Y | Correction dX | dY | Error (mm) |",
                "|---|---|---|---|---|---|---|"]
        for i, r in enumerate(errs, 1):
            out.append(f"| {i} | {r['time']} | {fnum(r, 'target_x'):.1f} | "
                       f"{fnum(r, 'target_y'):.1f} | {fnum(r, 'corr_x'):+.1f} | "
                       f"{fnum(r, 'corr_y'):+.1f} | {fnum(r, 'error_mm'):.1f} |")
        # Repeatability groups
        groups = []
        for r in errs:
            t = (fnum(r, "target_x"), fnum(r, "target_y"))
            for g in groups:
                if math.dist(g[0], t) < 15:
                    g[1].append(r)
                    break
            else:
                groups.append([t, [r]])
        out += ["", "## Repeatability (same target location, repeated trials)", "",
                "| Location (X, Y) | Trials | Commanded spread (mm) | "
                "Corrected-position spread (mm) |", "|---|---|---|---|"]
        for t, g in groups:
            if len(g) < 2:
                continue
            cmd = np.array([[fnum(r, "target_x"), fnum(r, "target_y")] for r in g])
            fin = np.array([[fnum(r, "rep_x"), fnum(r, "rep_y")] for r in g])
            sc = np.linalg.norm(cmd - cmd.mean(0), axis=1).max() * 2
            sf = np.linalg.norm(fin - fin.mean(0), axis=1).max() * 2
            out.append(f"| {t[0]:.0f}, {t[1]:.0f} | {len(g)} | {sc:.1f} | {sf:.1f} |")

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 4.2))
        tx = [fnum(r, "target_x") for r in errs]
        ty = [fnum(r, "target_y") for r in errs]
        ax[0].quiver(ty, tx, dy, dx, e, angles="xy", scale_units="xy", scale=0.5,
                     cmap="viridis")
        ax[0].scatter(ty, tx, c="k", s=10)
        pad = 45
        ax[0].set_xlim(max(ty) + pad, min(ty) - pad)   # robot's left (+Y) on the left
        ax[0].set_ylim(min(tx) - pad, max(tx) + pad)
        ax[0].set_xlabel("Robot Y (mm)")
        ax[0].set_ylabel("Robot X (mm)")
        ax[0].set_title("Correction vectors (drawn x2)")
        ax[0].set_aspect("equal")
        ax[1].bar(range(1, len(e) + 1), e, color="#3b6ea5")
        ax[1].axhline(e.mean(), color="k", ls="--", label=f"mean {e.mean():.1f} mm")
        ax[1].set_xlabel("Trial")
        ax[1].set_ylabel("Positioning error (mm)")
        ax[1].legend()
        fig.tight_layout()
        fig.savefig(os.path.join(RES, "errors.png"), dpi=130)

    if hovers:
        d = np.array([math.dist((fnum(r, "cmd_x"), fnum(r, "cmd_y"), fnum(r, "cmd_z")),
                                (fnum(r, "rep_x"), fnum(r, "rep_y"), fnum(r, "rep_z")))
                      for r in hovers])
        out += ["", "## Controller tracking (commanded vs reported)", "",
                f"- Hover moves: {len(d)}, max difference {d.max():.2f} mm",
                f"- Detection spread at the moment of each move: max "
                f"{max(fnum(r, 'spread_mm') for r in hovers):.1f} mm"]
    if refused:
        out += ["", "## Rejected / refused commands", "", "| Time | Event | Reason |",
                "|---|---|---|"]
        out += [f"| {r['time']} | {r['event']} | {r['note']} |" for r in refused]

    with open(os.path.join(RES, "summary.md"), "w") as f:
        f.write("\n".join(out) + "\n")
    print("\n".join(out))


if __name__ == "__main__":
    main()
