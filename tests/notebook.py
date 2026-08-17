# %%
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

data_dir = Path("tests/data")
files = sorted(data_dir.glob("*.csv"))

config_cols = [
    "environment",
    "robots",
    "total_time",
]

robot_cols = [
    "id",
    "physical_time",
    "virtual_time",
    "alignment_time",
    "exploration_time",
    "distance_traveled",
]

total_time_df = pd.DataFrame(columns=config_cols)
robots_df = pd.DataFrame(columns=robot_cols)

for f in files:
    tdf = pd.read_csv(f, nrows=1)
    total_time_df = pd.concat([total_time_df, tdf]).reset_index(drop=True)

    df = pd.read_csv(f, skiprows=2)

    df["environment"] = tdf["environment"].iloc[0]
    df["robots"] = tdf["robots"].iloc[0]
    df["run"] = f.stem

    robots_df = pd.concat([robots_df, df]).reset_index(drop=True)

total_time_s = (
    total_time_df.groupby(["environment", "robots"])["total_time"]
    .agg(mean="mean", std="std", n="count", median="median")
    .reset_index()
)
# envs = total_time_s["environment"].unique()

robots_df.head()

# %% --- Total time ---
env = "room"

sub = total_time_s[total_time_s["environment"] == env].sort_values("robots")

x = sub["robots"].to_numpy(dtype=int)
y = sub["mean"].to_numpy(dtype=float)
yerr = sub["std"].fillna(0).to_numpy(dtype=float)

fig, ax = plt.subplots(figsize=(6.0, 4.5))
ax.bar(x, y, capsize=5, alpha=0.9)

for i, mean, std, n in zip(x, y, yerr, sub["n"]):
    label = f"{mean:.2f} ± {std:.2f}" # "\nn={n}"
    ax.annotate(label, xy=(i, mean + std), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Total time (s)")
ax.set_xticks(x)
ax.grid(True, axis="y", alpha=0.3)

fig.tight_layout()
plt.show()

# %% --- Distance Traveled ---
distance_s = (
    robots_df.groupby(["run", "environment", "robots"])["distance_traveled"]
    .sum()
    .groupby(["environment", "robots"])
    .agg(mean="mean", std="std", n="count", median="median")
    .reset_index()
)

sub = distance_s[distance_s["environment"] == env].sort_values("robots")

x = sub["robots"].to_numpy(dtype=int)
y = sub["mean"].to_numpy(dtype=float)

fig, ax = plt.subplots(figsize=(6.0, 4.5))
ax.bar(x, y, capsize=5, alpha=0.9)

for i, mean in zip(x, y):
    label = f"{mean:.2f}"
    ax.annotate(label, xy=(i, mean), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Distance traveled sum")
ax.set_xticks(x)
ax.grid(True, axis="y", alpha=0.3)

fig.tight_layout()
plt.show()
