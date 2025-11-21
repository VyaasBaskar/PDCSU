import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("sim_data.csv")

fig, ax1 = plt.subplots()

# Plot 'vel' on the first y-axis
ax1.plot(df["step"], df["vel"], color="b", label="vel")
ax1.set_xlabel("Time")
ax1.set_ylabel("vel", color="b")
ax1.tick_params(axis="y", labelcolor="b")

# Allow negative values on y-axis
ax1.set_ylim(auto=True)

# Plot 'pos' on the second y-axis
ax2 = ax1.twinx()
ax2.plot(df["step"], df["pos"], color="g", label="pos")
ax2.set_ylabel("pos", color="g")
ax2.tick_params(axis="y", labelcolor="g")
ax2.set_ylim(auto=True)

# Plot 'output' on the third y-axis
ax3 = ax1.twinx()
ax3.spines["right"].set_position(("outward", 60))  # Offset the third axis
ax3.plot(df["step"], df["output"], color="r", label="output")
ax3.set_ylabel("output", color="r")
ax3.tick_params(axis="y", labelcolor="r")
ax3.set_ylim(auto=True)

# Add grid and title
ax1.grid(True)
plt.title("Simulation Data: vel, pos, output")

# Add legends
lines1, labels1 = ax1.get_legend_handles_labels()
# lines2, labels2 = ax2.get_legend_handles_labels()
# lines3, labels3 = ax3.get_legend_handles_labels()
# plt.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, loc="upper left")

plt.tight_layout()
plt.show()
