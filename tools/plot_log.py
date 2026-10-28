"""Plot the test scenario log written by sim_main.

Usage (from the project folder):
    python3 tools/plot_log.py build/velocity_log.csv docs/velocity_step.png
"""
import csv
import sys

import matplotlib.pyplot as plt

log_file = sys.argv[1] if len(sys.argv) > 1 else "build/velocity_log.csv"
image_file = sys.argv[2] if len(sys.argv) > 2 else "docs/velocity_step.png"

columns = {}
with open(log_file) as f:
    for row in csv.DictReader(f):
        for name, value in row.items():
            columns.setdefault(name, []).append(float(value))

t_ms = [t * 1e3 for t in columns["time_s"]]

fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))

ax_v.plot(t_ms, columns["velocity_cmd_rad_s"], "k--", label="command")
ax_v.plot(t_ms, columns["velocity_true_rad_s"], label="true (simulated)")
ax_v.plot(t_ms, columns["velocity_est_rad_s"], ":", label="estimated (encoder)")
ax_v.set_ylabel("velocity [rad/s]")
ax_v.legend(loc="lower right")
ax_v.grid(True)

ax_i.plot(t_ms, columns["current_cmd_A"], label="current command")
ax_i.plot(t_ms, [-10 * x for x in columns["load_torque_Nm"]], "--",
          label="load torque / Kt (0.5 Nm = 5 A)")
ax_i.set_ylabel("current [A]")
ax_i.set_xlabel("time [ms]")
ax_i.legend(loc="upper right")
ax_i.grid(True)

for ax in (ax_v, ax_i):
    ax.axvline(10, color="gray", linewidth=0.8)
    ax.axvline(100, color="gray", linewidth=0.8)

fig.suptitle("Velocity step 0 -> 200 rad/s at 10 ms, 0.5 Nm load at 100 ms")
fig.tight_layout()
fig.savefig(image_file, dpi=120)
print("plot written to", image_file)
