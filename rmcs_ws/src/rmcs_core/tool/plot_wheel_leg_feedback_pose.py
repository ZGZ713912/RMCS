#!/usr/bin/env python3
"""Compare measured PhysX hip/knee/wheel axes with a user supplied pose image."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--measurement", type=Path, required=True)
parser.add_argument("--reference-image", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
report = json.loads(args.measurement.read_text())
cases = report["cases"]
assert len(cases) == 2
font = FontProperties(fname="/usr/share/fonts/wenquanyi/wqy-zenhei/wqy-zenhei.ttc")
plt.rcParams.update({"font.family": font.get_name(), "axes.unicode_minus": False,
                     "font.size": 12, "axes.spines.top": False, "axes.spines.right": False})

fig, axes = plt.subplots(1, 3, figsize=(15, 7), gridspec_kw={"width_ratios": [1.15, 1, 1]})
fig.patch.set_facecolor("#f7f9fc")
axes[0].imshow(Image.open(args.reference_image))
axes[0].axis("off")
axes[0].set_title("用户提供的姿态图（透视）", fontsize=15, pad=16)
colors = ["#156a9b", "#d96a14"]
labels = ["标定零位：现有 offset", "本次 CAN 反馈重建"]
for ax, side, column in zip(axes[1:], ("L", "R"), (0, 1)):
    ax.set_facecolor("#f7f9fc")
    for case, color, label in zip(cases, colors, labels):
        points = case["joint_axes_relative_to_base_m"][side]
        h, k, w = [np.array(points[key]) for key in ("hip", "knee", "wheel")]
        chain = np.array([h, k, w]) - h
        ax.plot(-chain[:, 0], chain[:, 2], "o-", color=color, linewidth=4,
                markersize=7, alpha=.85, label=label)
        ax.scatter(-chain[2, 0], chain[2, 2], s=650, facecolors="none", edgecolors=color, linewidths=2)
    ax.text(-.055, -.026, "H 髋轴", fontsize=12)
    ax.text(-.09, .235, "K 被动膝轴", fontsize=12)
    ax.text(.17, .405, "W 轮轴", fontsize=12)
    ax.set_title(("左腿" if side == "L" else "右腿") + "：轴心侧视图", fontsize=15, pad=16)
    ax.set_xlim(-.10, .34)
    ax.set_ylim(-.045, .43)
    ax.set_aspect("equal")
    ax.set_xlabel("-X / m（车体坐标）")
    ax.set_ylabel("Z / m（相对髋轴）")
    ax.grid(alpha=.18)
    angles = [c["inner_from_axes_deg"][column] for c in cases]
    difference = cases[1]["difference_rad"][column]
    ax.text(.015, .075, f"模型内角\n标定 {angles[0]:.2f}°\n本次 {angles[1]:.2f}°\n\n本次 d = {difference:.5f} rad",
            fontsize=12, bbox={"boxstyle": "round,pad=.6", "facecolor": "white", "edgecolor": "#dce3ed"})

fig.suptitle("V5 闭链姿态核对：标定零位与本次原始反馈", fontsize=21, y=.97)
axes[1].legend(loc="upper center", bbox_to_anchor=(1.13, -.19), ncol=2, frameon=False)
fig.text(.50, .035, "轴心来自 Isaac / PhysX 闭链测量；模型内角不是对截图或实车的直接测量。",
         ha="center", fontsize=12, color="#48576b")
fig.subplots_adjust(left=.025, right=.98, top=.87, bottom=.22, wspace=.30)
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output, dpi=160, facecolor=fig.get_facecolor())
