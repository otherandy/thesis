# %%
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

data_dir = Path("tests/data")
files = sorted(data_dir.glob("*bots.csv"))

config_cols = [
    "environment",
    "robots",
    "radius",
    "start_x",
    "start_y",
    "strategy",
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

robots_df.head()

# %%

envs = total_time_s["environment"].unique()
envs

# %% --- Total time ---
env = "Polygon2"

sub = total_time_s[total_time_s["environment"] == env].sort_values("robots")

labels = sub["robots"].to_numpy(dtype=int)
x = np.arange(len(sub))
y = sub["mean"].to_numpy(dtype=float)
yerr = sub["std"].fillna(0).to_numpy(dtype=float)

fig, ax = plt.subplots(figsize=(6.0, 4.5))
ax.bar(x, y, capsize=5, alpha=0.9)

for i, mean, std, n in zip(x, y, yerr, sub["n"]):
    label = f"{mean:.2f} ± {std:.2f}"  # "\nn={n}"
    ax.annotate(label, xy=(i, mean), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Total time (s)")
ax.set_xticks(x, labels)
ax.grid(True, axis="y", alpha=0.3)

fig.tight_layout()
plt.show()

# %% --- Distance Traveled Sum ---
distance_by_run = (
    robots_df.groupby(["run", "environment", "robots"])["distance_traveled"]
    .sum()
    .groupby(["environment", "robots"])
    .agg(mean="mean")
    .reset_index()
)

sub = distance_by_run[distance_by_run["environment"] == env].sort_values("robots")

labels = sub["robots"].to_numpy(dtype=int)
x = np.arange(len(sub))
y = sub["mean"].to_numpy(dtype=float)

fig, ax = plt.subplots(figsize=(6.0, 4.5))
ax.bar(x, y, capsize=5, alpha=0.9)

for i, mean in zip(x, y):
    label = f"{mean:.1f}"
    ax.annotate(label, xy=(i, mean), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Distance traveled sum")
ax.set_xticks(x, labels)
ax.grid(True, axis="y", alpha=0.3)

fig.tight_layout()
plt.show()

# %% --- Distance Traveled Sum Split ---
distance_by_robot = (
    robots_df.groupby(["run", "environment", "robots", "id"])["distance_traveled"]
    .sum()
    .groupby(["environment", "robots", "id"])
    .agg(mean="mean")
    .reset_index()
)

sub = (
    distance_by_robot[distance_by_robot["environment"] == env]
    .pivot(index="robots", columns="id", values="mean")
    .sort_index()
    .fillna(0)
)

labels = sub.index.to_numpy(dtype=int)
x = np.arange(len(sub))
bottom = np.zeros(len(sub))

fig, ax = plt.subplots(figsize=(6.0, 4.5))
for robot_id in sub.columns:
    contribution = sub[robot_id].to_numpy(dtype=float)
    ax.bar(x, contribution, bottom=bottom, alpha=0.9, label=f"Robot {robot_id}")
    bottom += contribution

max_total = bottom.max() if len(bottom) else 0
ax.set_ylim(top=max_total * 1.1 if max_total > 0 else 1)

for i, total in zip(x, bottom):
    label = f"{total:.1f}"
    ax.annotate(label, xy=(i, total), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Distance traveled sum")
ax.set_xticks(x, labels)
ax.grid(True, axis="y", alpha=0.3)
# ax.legend(title="Robot", ncols=2)

fig.tight_layout()
plt.show()

# %% --- Mean Distance Traveled ---
distance_s = (
    robots_df.groupby(["environment", "robots"])["distance_traveled"]
    .agg(mean="mean")
    .reset_index()
)

sub = distance_s[distance_s["environment"] == env].sort_values("robots")

labels = sub["robots"].to_numpy(dtype=int)
x = np.arange(len(sub))
y = sub["mean"].to_numpy(dtype=float)

fig, ax = plt.subplots(figsize=(6.0, 4.5))
ax.bar(x, y, capsize=5, alpha=0.9)

for i, mean in zip(x, y):
    label = f"{mean:.2f}"
    ax.annotate(label, xy=(i, mean), ha="center", va="bottom")

ax.set_title(f"Environment: {env}")
ax.set_xlabel("Number of robots")
ax.set_ylabel("Mean distance traveled")
ax.set_xticks(x, labels)
ax.grid(True, axis="y", alpha=0.3)

fig.tight_layout()
plt.show()
