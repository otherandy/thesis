# %%
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("tests/data.csv")

timer_cols = [
    "physical_time",
    "virtual_time",
    "alignment_time",
    "exploration_time",
    "total_time",
]

avg_df = (
    df.groupby(["environment", "bots"], as_index=False)[timer_cols]
    .mean()
    .sort_values(["environment", "bots"])
)

envs = sorted(avg_df["environment"].unique())
bots = sorted(avg_df["bots"].unique())

avg_df

# %%
fig, axes = plt.subplots(
    nrows=1, ncols=len(envs), figsize=(6.5 * len(envs), 4.5), sharey=False
)
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)

    x = sub.index.to_numpy(dtype=float)

    for t in timer_cols:
        ax.plot(x, sub[t].values, marker="o", label=t)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of bots")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(b)) for b in bots])

    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

axes[0].set_ylabel("Average time")
plt.tight_layout()
plt.show()

# %% --- Total time ---
timer_cols_total = ["total_time"]

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(6.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=["total_time"])

    x = sub.index.to_numpy(dtype=float)
    y_total = sub["total_time"].to_numpy(dtype=float)

    ax.plot(x, y_total, marker="o", label="total_time")

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of bots")
    ax.set_ylabel("Average time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

plt.tight_layout()
plt.show()

# %%  --- Physical + Virtual ---
timer_cols_pv = ["physical_time", "virtual_time"]

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(7.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=timer_cols_pv)

    x = sub.index.to_numpy(dtype=float)
    y_phys = sub["physical_time"].to_numpy(dtype=float)
    y_virt = sub["virtual_time"].to_numpy(dtype=float)

    if len(x) == 1:
        ax.plot(x, y_phys, marker="o", label="physical_time")
        ax.plot(x, y_virt, marker="o", label="virtual_time")
    else:
        ax.stackplot(x, [y_phys, y_virt], labels=timer_cols_pv, alpha=0.9)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of bots")
    ax.set_ylabel("Average time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

plt.tight_layout()
plt.show()

# %% --- Alignment + Exploration ---
timer_cols_ae = ["alignment_time", "exploration_time"]

fig, axes = plt.subplots(nrows=1, ncols=len(envs), figsize=(7.0 * len(envs), 4.5))
if len(envs) == 1:
    axes = [axes]

for ax, env in zip(axes, envs):
    sub = avg_df[avg_df["environment"] == env].set_index("bots").reindex(bots)
    sub = sub.dropna(subset=timer_cols_ae)

    x = sub.index.to_numpy(dtype=float)
    y_align = sub["alignment_time"].to_numpy(dtype=float)
    y_explore = sub["exploration_time"].to_numpy(dtype=float)

    if len(x) == 1:
        ax.plot(x, y_align, marker="o", label="alignment_time")
        ax.plot(x, y_explore, marker="o", label="exploration_time")
    else:
        ax.stackplot(x, [y_align, y_explore], labels=timer_cols_ae, alpha=0.9)

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of bots")
    ax.set_ylabel("Average time")

    ax.set_xticks(x)
    ax.set_xticklabels([str(int(v)) for v in x])

    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

plt.tight_layout()
plt.show()
