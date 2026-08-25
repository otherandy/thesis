# %%
from pathlib import Path

import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

data_dir = Path("tests")
files = sorted(data_dir.glob("grid*.csv"))

for f in files:
    df = pd.read_csv(f)
    sns.heatmap(df, cbar=False, square=True, xticklabels=False, yticklabels=False)
    plt.show()
