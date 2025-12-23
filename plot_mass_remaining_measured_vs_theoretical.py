"""
Purpose
-------
Generates the comparison plot of measured versus theoretical particle
mass remaining as a function of particle diameter for the MGS-1 sieving
study. This script reproduces the figure used in the associated ASCE
journal manuscript.

Environment
-----------
Designed to run in Google Colab, Jupyter Notebook, or a standard
Python environment with NumPy and Matplotlib installed.

Output
------
Saves an SVG figure:
    mass_remaining_measured_vs_theoretical.svg
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter


# ============================================================
# 1) X-axis: particle diameters (µm)
#    Ordered from smallest to largest for log-scale plotting
# ============================================================
diam_um = np.array(
    [0.1, 1, 5, 10, 20, 40, 75, 100, 200, 1000],
    dtype=float
)


# ============================================================
# 2) Y-axis data: percent particle mass remaining
#    Values are entered from largest diameter (1000 µm)
#    to smallest (0.1 µm) and reversed for plotting.
# ============================================================

data_measured_txt = """
100
68.2
34.5
23.5
15.35
5.84
4.76
1.58
0
0
"""

data_theoretical_txt = """
100
78
62
52
42
24
12
8
2
0
"""


def parse_column(txt):
    """Convert a column of numeric text into a NumPy array."""
    return np.array([float(x) for x in txt.split()], dtype=float)


# Raw arrays correspond to diameters:
# [1000, 200, 100, 75, 40, 20, 10, 5, 1, 0.1] µm
y_meas_raw = parse_column(data_measured_txt)
y_theo_raw = parse_column(data_theoretical_txt)

# Reverse arrays to match ascending diameter order
y_meas = y_meas_raw[::-1]
y_theo = y_theo_raw[::-1]

# Sanity checks
if len(y_meas) != len(diam_um):
    raise ValueError(
        f"Measured data length ({len(y_meas)}) "
        f"does not match diameter array ({len(diam_um)})."
    )

if len(y_theo) != len(diam_um):
    raise ValueError(
        f"Theoretical data length ({len(y_theo)}) "
        f"does not match diameter array ({len(diam_um)})."
    )


# ============================================================
# 3) Plot
# ============================================================
plt.figure(figsize=(8, 3.5))

# Measured data: solid blue
plt.plot(
    diam_um,
    y_meas,
    linewidth=2,
    color="C0",
    label="Measured"
)

# Theoretical data: dashed red
plt.plot(
    diam_um,
    y_theo,
    "--",
    linewidth=2,
    color="C3",
    label="Theoretical"
)

ax = plt.gca()
ax.set_xscale("log", base=2)

# Define clean, readable tick locations
xmin, xmax = diam_um.min(), diam_um.max()
candidate_ticks = [0.1, 1, 5, 10, 20, 40, 75, 100, 200, 500, 1000]
xticks = [x for x in candidate_ticks if xmin * 0.9 <= x <= xmax * 1.1]
ax.set_xticks(xticks)
ax.xaxis.set_major_formatter(FormatStrFormatter("%.0f"))

plt.xlabel("Particle Diameter (µm)")
plt.ylabel("Percent Particle Mass Remaining (%)")

plt.xlim(xmin * 0.9, xmax * 1.1)
plt.ylim(0, 100)

plt.grid(True, which="both", linestyle="--", alpha=0.4)

plt.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, 1.12),
    ncol=2,
    frameon=False
)

plt.tight_layout()

# Save figure
plt.savefig(
    "mass_remaining_measured_vs_theoretical.svg",
    format="svg",
    bbox_inches="tight"
)

plt.show()
