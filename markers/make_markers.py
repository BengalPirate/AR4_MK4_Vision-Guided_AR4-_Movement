"""Regenerate the board's ArUco markers (DICT_4X4_50, IDs 4-7).

Run from the repository root:  python markers/make_markers.py
  markers/aruco_<id>.png   one marker per file, white border included
  markers/aruco_4-7.pdf    all four on one A4/Letter-safe page, 60 mm each
"""
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
IDS = (4, 5, 6, 7)
SIDE_MM = 60

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
imgs = {}
for i in IDS:
    img = cv2.aruco.generateImageMarker(d, i, 600)
    img = cv2.copyMakeBorder(img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(os.path.join(HERE, f"aruco_{i}.png"), img)
    imgs[i] = img

fig = plt.figure(figsize=(8.27, 11.0))            # fits both A4 and Letter
s = SIDE_MM / 25.4 * 800 / 600                    # inches incl. border
for n, i in enumerate(IDS):
    x = 1.0 + (n % 2) * 3.6
    y = 6.2 - (n // 2) * 4.4
    ax = fig.add_axes([x / 8.27, y / 11.0, s / 8.27, s / 11.0])
    ax.imshow(imgs[i], cmap="gray", vmin=0, vmax=255)
    ax.axis("off")
    fig.text((x + s / 2) / 8.27, (y - 0.25) / 11.0, f"ID {i}", ha="center")
fig.text(0.5, 0.95, "DICT_4X4_50 markers 4-7, print at 100% (black square = 60 mm)",
         ha="center")
fig.savefig(os.path.join(HERE, "aruco_4-7.pdf"))
