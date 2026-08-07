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
robots_df = pd.DataFrame(columns=["environment", "robots"] + robot_cols)

for f in files:
    tdf = pd.read_csv(f, nrows=1)
    total_time_df = pd.concat([total_time_df, tdf]).reset_index(drop=True)

    df = pd.read_csv(f, skiprows=2)

    df["environment"] = tdf["environment"].iloc[0]
    df["robots"] = tdf["robots"].iloc[0]

    robots_df = pd.concat([robots_df, df]).reset_index(drop=True)

total_time_s = (
    total_time_df.groupby(["environment", "robots"])["total_time"].mean().reset_index()
)

robots_df.head()

# %% --- Total time ---
envs = total_time_s["environment"].unique()

fig, axes = plt.subplots(
    nrows=1,
    ncols=len(envs),
    figsize=(6.0 * len(envs), 4.5),
    squeeze=False,
)
axes = axes[0]

for ax, env in zip(axes, envs):
    sub = total_time_s[total_time_s["environment"] == env].sort_values("robots")

    x = sub["robots"].to_numpy(dtype=int)
    y = sub["total_time"].to_numpy(dtype=float)

    ax.bar(x, y, alpha=0.9)

    for i,j in zip(x,y):
        ax.annotate(str(np.round(j, decimals=5)), xy=(i, j), ha="center", va="bottom")

    ax.set_title(f"Environment: {env}")
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Total time (s)")
    ax.set_xticks(x)
    ax.grid(True, axis="y", alpha=0.3)

plt.tight_layout()
plt.show()

# %%
metrics = [
    "physical_time",
    "virtual_time",
    "alignment_time",
    "exploration_time",
    "distance_traveled",
]

plot_df = robots_df.melt(
    id_vars=["environment", "robots", "id"],
    value_vars=metrics,
    var_name="metric",
    value_name="value",
)

g = sns.catplot(
    data=plot_df,
    kind="box",
    x="robots",
    y="value",
    row="environment",
    col="metric",
    sharey=False,
    height=3.2,
    aspect=1.1,
)

g.set_axis_labels("Number of robots", "Value")
g.set_titles(row_template="Environment: {row_name}", col_template="{col_name}")

for ax in g.axes.flat:
    ax.grid(True, axis="y", alpha=0.3)

plt.tight_layout()
plt.show()
