# %%
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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
robots_df = pd.DataFrame(columns=["environment", "robots"] + robot_cols)

for f in files:
    tdf = pd.read_csv(f, nrows=1)
    total_time_df = pd.concat([total_time_df, tdf]).reset_index(drop=True)

    df = pd.read_csv(f, skiprows=2)

    if df.shape[1] != len(robot_cols):
        raise ValueError(f"{f} has {df.shape[1]} columns, expected {len(robot_cols)}")

    df["environment"] = environment
    df["robots"] = robots

    robots_df = pd.concat([robots_df, df]).reset_index(drop=True)

total_time_s = total_time_df.groupby(["environment", "robots"])["total_time"].mean().reset_index()

robots_avg_df = robots_df.groupby(["environment", "robots", "id"]).mean().reset_index()

robots_avg_df

# %% --- Total time ---
envs = total_time_s["environment"].unique()

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(6.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = total_time_s[total_time_s["environment"] == env]

    x = sub["robots"].to_numpy(dtype=int)
    y = sub["total_time"].to_numpy(dtype=float)

    ax.bar(x, y, alpha=0.9)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, axis="y", alpha=0.3)

plt.tight_layout()
plt.show()
