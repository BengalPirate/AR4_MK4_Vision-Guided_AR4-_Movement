"""Fit a correction to the camera-to-robot mapping from measured trials.

Run:  python refine.py [--model translation|affine]
Reads the "error" rows of results/moves.csv: for each, the camera estimate of the
target (target_x/y) and where the operator put the pen tip over the target centre
(rep_x/y). Fits  tip = A @ camera + b  (affine: 6 parameters, needs >= 4 trials;
translation: A = I, needs >= 1) and stores it in calib.json under "correction".
vision_demo.py then commands the corrected position. Validate with NEW trials:
the "error" rows logged after refinement are the residual error.
"""
import argparse
import csv
import json
import os

import numpy as np

import config

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", choices=["translation", "affine"], default=None,
                    help="default: affine with >= 5 trials, else translation")
    args = ap.parse_args()
    with open(config.LOG_FILE) as f:
        rows = [r for r in csv.DictReader(f)
                if r["event"] == "error" and not r["note"].startswith("refined")]
    if not rows:
        raise SystemExit("No unrefined error measurements in results/moves.csv")
    cam = np.array([[float(r["target_x"]), float(r["target_y"])] for r in rows])
    tip = np.array([[float(r["rep_x"]), float(r["rep_y"])] for r in rows])
    model = args.model or ("affine" if len(rows) >= 5 else "translation")
    if model == "affine":
        if len(rows) < 4:
            raise SystemExit("Affine needs at least 4 trials")
        X = np.c_[cam, np.ones(len(cam))]
        sol, *_ = np.linalg.lstsq(X, tip, rcond=None)
        A, b = sol[:2].T, sol[2]
    else:
        A, b = np.eye(2), (tip - cam).mean(axis=0)
    pred = cam @ A.T + b
    before = np.linalg.norm(tip - cam, axis=1)
    after = np.linalg.norm(tip - pred, axis=1)
    print(f"{len(rows)} trials, model {model}")
    print(f"  error before: mean {before.mean():.1f} mm, max {before.max():.1f} mm")
    print(f"  fit residual: mean {after.mean():.1f} mm, max {after.max():.1f} mm "
          "(in-sample; validate with new trials)")
    print(f"  A = {np.round(A, 4).tolist()}  b = {np.round(b, 2).tolist()} mm")
    with open(config.CALIB_FILE) as f:
        calib = json.load(f)
    calib["correction"] = {"model": model, "n": len(rows), "A": A.tolist(), "b": b.tolist(),
                           "mean_error_before_mm": float(before.mean()),
                           "mean_fit_residual_mm": float(after.mean())}
    with open(config.CALIB_FILE, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"Saved to {config.CALIB_FILE}")


if __name__ == "__main__":
    main()
