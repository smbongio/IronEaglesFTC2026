import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Arc, Rectangle, FancyArrowPatch

N = 1.0          # shooting distance
CONE_R = 2.7     # how far the cone is drawn (it really has no end)
CENTER = -90.0   # center line points down the page
HALF = 45.0

def pt(angle_deg, r):
    a = math.radians(angle_deg)
    return (r * math.cos(a), r * math.sin(a))

fig, ax = plt.subplots(figsize=(10, 9))

# Cone (shooting cone), faded to show it continues
left_edge, right_edge = CENTER - HALF, CENTER + HALF
ax.add_patch(Wedge((0, 0), CONE_R, left_edge, right_edge, facecolor="#cfe3ff", edgecolor="none", alpha=0.7))
ax.add_patch(Wedge((0, 0), CONE_R + 0.35, left_edge, right_edge, width=0.35, facecolor="#cfe3ff",
                   edgecolor="none", alpha=0.3))
for e in (left_edge, right_edge):
    x, y = pt(e, CONE_R)
    ax.plot([0, x], [0, y], color="#2b6cb0", lw=2)
    x2, y2 = pt(e, CONE_R + 0.35)
    ax.plot([x, x2], [y, y2], color="#2b6cb0", lw=2, ls=(0, (2, 3)))

# Center line (direction the tags face)
cx, cy = pt(CENTER, CONE_R + 0.35)
ax.plot([0, cx], [0, cy], color="#555", lw=1.2, ls="--")
ax.text(0.06, -CONE_R - 0.2, "center line\n(direction the tags face,\nflattened onto the field)",
        fontsize=9, color="#444", va="top")

# 45 degree angle markers
ax.add_patch(Arc((0, 0), 3.0, 3.0, theta1=left_edge, theta2=CENTER, color="#2b6cb0", lw=1.2))
ax.add_patch(Arc((0, 0), 3.0, 3.0, theta1=CENTER, theta2=right_edge, color="#2b6cb0", lw=1.2))
lx, ly = pt((left_edge + CENTER) / 2, 1.68)
rx, ry = pt((right_edge + CENTER) / 2, 1.68)
ax.text(lx, ly, "45°", fontsize=12, color="#2b6cb0", ha="center", va="center", fontweight="bold")
ax.text(rx, ry, "45°", fontsize=12, color="#2b6cb0", ha="center", va="center", fontweight="bold")

# Shooting arc (target positions)
ax.add_patch(Arc((0, 0), 2 * N, 2 * N, theta1=left_edge, theta2=right_edge, color="#d62828", lw=5))
ax.text(*pt(CENTER, N + 0.13), "shooting arc\n(target positions)", color="#d62828", fontsize=11,
        ha="center", va="top", fontweight="bold")

# Shooting distance dimension
dx, dy = pt(-118, N)
ax.annotate("", xy=(dx, dy), xytext=(0, 0),
            arrowprops=dict(arrowstyle="<->", color="#d62828", lw=1.3))
mx, my = pt(-118, 0.5)
ax.text(mx - 0.06, my, "N", color="#d62828", fontsize=13, ha="right", va="center", fontweight="bold")

# Target cell at the apex
ax.add_patch(Rectangle((-0.45, 0.02), 0.9, 0.32, facecolor="#e63946", edgecolor="black", lw=1.2))
ax.text(0, 0.18, "target CELL", color="white", fontsize=10, ha="center", va="center", fontweight="bold")
ax.plot(0, 0, "ko", ms=6)
ax.text(0.5, 0.05, "point directly below the\ncenter of the tag cluster", fontsize=9, va="center")

# Example robots and where auto fire drives them
robots = [
    ("A", (-100, 2.2), (-100, N), "inside cone, too far:\nmoves straight in"),
    ("B", (-62, 0.42), (-62, N), "inside cone, too close:\nmoves straight out"),
    ("C", (-12, 1.9), (right_edge, N), "outside cone:\ngoes to nearer end of arc"),
    ("D", (-172, 1.35), (left_edge, N), "outside cone:\ngoes to nearer end of arc"),
]
for name, (ra, rr), (ta, tr), label in robots:
    rxy, txy = pt(ra, rr), pt(ta, tr)
    ax.add_patch(FancyArrowPatch(rxy, txy, arrowstyle="-|>", mutation_scale=18, color="#222", lw=1.6,
                                 shrinkA=9, shrinkB=6))
    ax.add_patch(Rectangle((rxy[0] - 0.09, rxy[1] - 0.09), 0.18, 0.18, facecolor="#ffd166",
                           edgecolor="black", lw=1.2, zorder=5))
    ax.text(rxy[0], rxy[1], name, fontsize=9, ha="center", va="center", fontweight="bold", zorder=6)
    ax.plot(*txy, "o", color="#d62828", ms=8, markeredgecolor="black", zorder=6)
    off = {"A": (0.15, -0.05), "B": (0.14, 0.0), "C": (0.0, 0.17), "D": (0.0, 0.17)}[name]
    ha = "left" if name in ("A", "B") else "center"
    ax.text(rxy[0] + off[0], rxy[1] + off[1], label, fontsize=8.5, ha=ha,
            va="bottom" if name in ("C", "D") else "center")

ax.text(-2.9, -3.75, "N = shooting distance (SHOOTING_DISTANCE_IN). The cone has no maximum distance; dashed lines show it continues.\n"
        "Yellow squares: robot positions when the trigger is pulled.  Red dots: where auto fire drives the robot.",
        fontsize=9, color="#333")

ax.set_title("Auto fire: shooting cone and shooting arc (top-down view)", fontsize=14, fontweight="bold")
ax.set_xlim(-3.1, 3.1)
ax.set_ylim(-3.85, 0.75)
ax.set_aspect("equal")
ax.axis("off")
fig.tight_layout()
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "images", "autofire_cone.png"), dpi=150)
