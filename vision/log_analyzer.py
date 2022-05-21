import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import RegularPolygon
import numpy as np
import argparse

def hexagone(c, color, radius=0.075):
    return RegularPolygon((c[0], c[1]), numVertices=6, radius=radius, color=color)

parser = argparse.ArgumentParser(description="Plot the marker position seen from a camera logfile")
parser.add_argument("input", help = "Input logfile (vision_module.txt).")
args = parser.parse_args()

# Parse data
frames = []
reading_markers = False
with open(args.input, 'r') as f:
    current_frame = None
    for l in f:
        if l.startswith("#######"):
            if current_frame is not None:
                frames.append(current_frame)
            current_frame = {"Time": 0, "Angles": None, "Markers": []}
            reading_markers = False
        if l.startswith("Time:"):
            current_frame["Time"] = l.split("Time:")[1].split(" ")[0]
        if l.startswith("   Azimuth"):
            current_frame["Angles"] = l
        if l.startswith("Current markers in store:"):
            reading_markers = True
        if reading_markers and l.startswith("Marker"):
            try:
                current_marker = int(l.split("Marker")[1].split("\n")[0].split(" ")[0])
            except ValueError:
                continue
        if reading_markers and l.startswith("    TWM"):
            pos = [float(s) for s in l.split("    TWM")[1].split("\n")[0].split(" ") if len(s) > 1]
            current_marker = (current_marker, pos)
            current_frame["Markers"].append(current_marker)

# Plot
fig, ax = plt.subplots()
axfreq = plt.axes([0.25, 0.1, 0.65, 0.02])
plt.subplots_adjust(bottom=0.15)
freq_slider = Slider(ax=axfreq, label='Time', valmin=0, valmax=len(frames) -1, valinit=0, valstep = 1)

COLOR_MAP = {13: "b", 36:"g", 17:"tab:brown", 42:"y", 47:"r"}
def update(val):
    val = int(val)
    ax.clear()
    ax.plot([0, 0, 3, 3, 0], [0, 2, 2, 0, 0])
    frame = frames[val]
    ax.set_xlim(-0.5, 3.5)
    scale = (ax.transData.get_matrix()[0, 0] * 0.07)**2
    # scale = 35

    fig.suptitle(f"Frame {val}, t={frame['Time']}, {frame['Angles']}")
    for marker in frame["Markers"]:
        c = COLOR_MAP.get(marker[0], "#A0A0A0")

        h = hexagone((marker[1][0], marker[1][1]), c)
        ax.add_patch(h)
        # ax.plot(marker[1][0], marker[1][1], marker="h", markersize=scale, markerfacecolor=c, markeredgecolor=c)
        ax.annotate(marker[0], (marker[1][0], marker[1][1]))
    ax.set_xlim(-0.5, 3.5)

    ax.set_aspect("equal")
    ax.grid(True)
    fig.canvas.draw_idle()

freq_slider.on_changed(update)
update(0)
plt.show()
