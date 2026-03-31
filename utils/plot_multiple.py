import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from tkinter import Tk, filedialog


def plot_files_by_name():
    """
    Select exactly two CSV files and plot:
      - Top row: individual board total-force plots
      - Bottom row: summed total force

    Alignment is done using the shortest common synced_index.
    Required columns:
      - synced_index
      - total_force_N
    """

    root = Tk()
    root.withdraw()

    file_paths = filedialog.askopenfilenames(
        title="Select exactly 2 recorded board CSV files",
        filetypes=[("CSV files", "*.csv")]
    )

    file_paths = list(file_paths)

    if len(file_paths) != 2:
        print("Please select exactly 2 CSV files.")
        return

    dfs = []
    labels = []

    for path in file_paths:
        df = pd.read_csv(path)

        required = {"synced_index", "total_force_N"}
        missing = required - set(df.columns)
        if missing:
            raise ValueError(f"{os.path.basename(path)} is missing columns: {sorted(missing)}")

        # Keep only needed columns and sort by synced_index
        df = df[["synced_index", "total_force_N"]].copy()
        df = df.sort_values("synced_index").reset_index(drop=True)

        dfs.append(df)
        labels.append(os.path.basename(path))

    df1, df2 = dfs

    # Determine shortest common synced range
    max_common_synced_index = int(min(df1["synced_index"].max(), df2["synced_index"].max()))

    df1_common = df1[df1["synced_index"] <= max_common_synced_index].copy()
    df2_common = df2[df2["synced_index"] <= max_common_synced_index].copy()

    # Reindex on synced_index to ensure exact alignment
    df1_common = df1_common.set_index("synced_index")
    df2_common = df2_common.set_index("synced_index")

    common_index = df1_common.index.intersection(df2_common.index)
    if len(common_index) == 0:
        print("No overlapping synced_index values found between the two files.")
        return

    f1 = df1_common.loc[common_index, "total_force_N"].to_numpy()
    f2 = df2_common.loc[common_index, "total_force_N"].to_numpy()
    f_sum = f1 + f2
    x = common_index.to_numpy()

    # Figure layout
    fig = plt.figure(figsize=(14, 8))
    gs = GridSpec(2, 2, figure=fig, height_ratios=[1, 1.2])

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax_sum = fig.add_subplot(gs[1, :])

    # Top: individual plots
    ax1.plot(x, f1, label=labels[0])
    ax1.set_title(labels[0])
    ax1.set_xlabel("Synced Index")
    ax1.set_ylabel("Total Force (N)")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    ax2.plot(x, f2, label=labels[1])
    ax2.set_title(labels[1])
    ax2.set_xlabel("Synced Index")
    ax2.set_ylabel("Total Force (N)")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    # Bottom: summed plot
    ax_sum.plot(x, f_sum, label="Summed Total Force")
    ax_sum.set_title("Overall Force (Sum of Both Boards)")
    ax_sum.set_xlabel("Synced Index")
    ax_sum.set_ylabel("Total Force (N)")
    ax_sum.grid(True)
    ax_sum.legend(loc="upper right")

    fig.suptitle("Recorded Board Force Comparison (Synced Index Aligned)", fontsize=14)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_files_by_name()