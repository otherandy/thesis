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

# %%
avg_df = (
    df.groupby(["environment", "bots"], as_index=False)[data_cols]
    .mean()
    .sort_values(["environment", "bots"])
)

envs = sorted(avg_df["environment"].unique())
bots = sorted(avg_df["bots"].unique())

avg_df

# %%
timer_cols = [
    "total_time",
    "first_physical_time",
    "first_virtual_time",
    "first_alignment_time",
    "first_exploration_time",
]

fig, axes = plt.subplots(
    nrows=1, ncols=len(envs), figsize=(6.5 * len(envs), 4.5), sharey=False
)
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)

    x = sub.index.to_numpy(dtype=float).astype(int)

    for t in timer_cols:
        ax.plot(x, sub[t].values, marker="o", label=t)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(b)) for b in bots])

    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

axes[0].set_ylabel("Time")
plt.tight_layout()
plt.show()

# %% --- Total time ---
timer_col_total = "total_time"

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(6.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=[timer_col_total])

    x = sub.index.to_numpy(dtype=float).astype(int)
    y = sub[timer_col_total].to_numpy(dtype=float)

    ax.bar(x, y, alpha=0.9)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, axis="y", alpha=0.3)

plt.tight_layout()
plt.show()

# %%  --- Physical + Virtual ---
timer_cols_pv = ["first_physical_time", "first_virtual_time"]

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(7.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=timer_cols_pv)

    x = sub.index.to_numpy(dtype=float).astype(int)
    y_phys = sub["first_physical_time"].to_numpy(dtype=float)
    y_virt = sub["first_virtual_time"].to_numpy(dtype=float)

    if len(x) == 1:
        ax.bar(x, y_phys, alpha=0.9, label="first_physical_time")
        ax.bar(x, y_virt, alpha=0.9, bottom=y_phys, label="first_virtual_time")
    else:
        ax.bar(x, y_phys, alpha=0.9, label="first_physical_time")
        ax.bar(x, y_virt, alpha=0.9, bottom=y_phys, label="first_virtual_time")

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="best", fontsize=9)

plt.tight_layout()
plt.show()

# %% --- Alignment + Exploration ---
timer_cols_ae = ["first_alignment_time", "first_exploration_time"]

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(7.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=timer_cols_ae)

    x = sub.index.to_numpy(dtype=float).astype(int)
    y_align = sub["first_alignment_time"].to_numpy(dtype=float)
    y_explore = sub["first_exploration_time"].to_numpy(dtype=float)

    if len(x) == 1:
        ax.bar(x, y_align, alpha=0.9, label="first_alignment_time")
        ax.bar(x, y_explore, alpha=0.9, bottom=y_align, label="first_exploration_time")
    else:
        ax.bar(x, y_align, alpha=0.9, label="first_alignment_time")
        ax.bar(x, y_explore, alpha=0.9, bottom=y_align, label="first_exploration_time")

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="best", fontsize=9)

plt.tight_layout()
plt.show()

# %% --- Distance traveled ---
distance_col = "first_distance_traveled"

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(6.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=[distance_col])

    x = sub.index.to_numpy(dtype=float).astype(int)
    y = sub[distance_col].to_numpy(dtype=float)

    ax.bar(x, y, alpha=0.9)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Distance")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, axis="y", alpha=0.3)

plt.tight_layout()
plt.show()
