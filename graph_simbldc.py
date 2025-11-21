import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

if len(sys.argv) < 2:
    print("Usage: python graph_simbldc.py <csv_file> [output_png]")
    sys.exit(1)

csv_file = sys.argv[1]
if not os.path.exists(csv_file):
    print(f"Error: CSV file not found: {csv_file}")
    sys.exit(1)

df = pd.read_csv(csv_file)

fig, ax1 = plt.subplots(figsize=(10, 6))

ax1.plot(df["step"], df["vel"], color="b", label="vel")
ax1.set_xlabel("Step")
ax1.set_ylabel("vel", color="b")
ax1.tick_params(axis="y", labelcolor="b")
ax1.set_ylim(auto=True)

ax2 = ax1.twinx()
ax2.plot(df["step"], df["pos"], color="g", label="pos")
ax2.set_ylabel("pos", color="g")
ax2.tick_params(axis="y", labelcolor="g")
ax2.set_ylim(auto=True)

ax3 = ax1.twinx()
ax3.spines["right"].set_position(("outward", 60))
ax3.plot(df["step"], df["output"], color="r", label="output")
ax3.set_ylabel("output", color="r")
ax3.tick_params(axis="y", labelcolor="r")
ax3.set_ylim(auto=True)

ax1.grid(True)
scenario_name = os.path.splitext(os.path.basename(csv_file))[0]
plt.title(f"SimBLDC Benchmark: {scenario_name}")

plt.tight_layout()

if len(sys.argv) >= 3:
    output_file = sys.argv[2]
    plt.savefig(output_file, dpi=150)
    print(f"Saved graph to {output_file}")
else:
    plt.show()

