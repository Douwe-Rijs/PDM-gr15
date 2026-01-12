import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------
# Load data
# -----------------------------
df = pd.read_csv("drone_hole_experiments.csv")
df = df[df["termination_reason"] != "exception"]

print("Loaded", len(df), "valid experiments")

# -----------------------------
# Derived metrics
# -----------------------------
df["hole_area"] = df["hole_width"] * df["hole_height"]

# -----------------------------
# Helper: binned success plot
# -----------------------------
def plot_binned_success(x, x_label, bins=10):
    binned = pd.cut(x, bins)
    success_rate = df.groupby(binned)["success"].mean()

    plt.figure()
    success_rate.plot(marker="o")
    plt.xlabel(x_label)
    plt.ylabel("Success rate")
    plt.title(f"Success rate vs {x_label}")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# =========================================================
# 1. Success rate vs hole WIDTH
# =========================================================
plot_binned_success(df["hole_width"], "Hole width [m]")


# =========================================================
# 2. Success rate vs hole HEIGHT
# =========================================================
plot_binned_success(df["hole_height"], "Hole height [m]")


# =========================================================
# 3. Success rate vs hole AREA
# =========================================================
plot_binned_success(df["hole_area"], "Hole area [m²]")


# =========================================================
# 4. Success rate vs number of WALLS
# =========================================================
walls_success = df.groupby("walls")["success"].mean()

plt.figure()
walls_success.plot(marker="o")
plt.xlabel("Number of walls")
plt.ylabel("Success rate")
plt.title("Success rate vs environment difficulty (walls)")
plt.grid(True)
plt.tight_layout()
plt.show()
# =========================================================
# 6. Flight time vs number of WALLS
# =========================================================
# -----------------------------
# Keep only successful runs for flight time analysis
# -----------------------------
df_success = df[df["success"] == True]

print("Successful runs:", len(df_success))
# =========================================================
# Flight time vs number of WALLS (successful runs only)
# =========================================================
walls_flight = df_success.groupby("walls")["flight_time"].agg(["mean", "std"])

plt.figure()
plt.errorbar(
    walls_flight.index,
    walls_flight["mean"],
    yerr=walls_flight["std"],
    marker="o",
    capsize=4
)
plt.xlabel("Number of walls")
plt.ylabel("Flight time [s]")
plt.title("Flight time vs environment difficulty (walls)\n(successful runs only)")
plt.grid(True)
plt.tight_layout()
plt.show()

# =========================================================
# 7. Flight time vs hole AREA (binned)
# =========================================================
# =========================================================
# Flight time vs hole AREA (successful runs only)
# =========================================================
def plot_binned_flight_time_success(x, x_label, bins=10):
    binned = pd.cut(x, bins)
    mean_time = df_success.groupby(binned)["flight_time"].mean()

    plt.figure()
    mean_time.plot(marker="o")
    plt.xlabel(x_label)
    plt.ylabel("Mean flight time [s]")
    plt.title(f"Flight time vs {x_label}\n(successful runs only)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


plot_binned_flight_time_success(df_success["hole_area"], "Hole area [m²]")



# =========================================================
# 5. 2D heatmap: hole width × hole height → success
# =========================================================
width_bins = np.linspace(df["hole_width"].min(), df["hole_width"].max(), 8)
height_bins = np.linspace(df["hole_height"].min(), df["hole_height"].max(), 8)

df["w_bin"] = pd.cut(df["hole_width"], width_bins)
df["h_bin"] = pd.cut(df["hole_height"], height_bins)

heatmap = df.pivot_table(
    values="success",
    index="h_bin",
    columns="w_bin",
    aggfunc="mean"
)

plt.figure(figsize=(8, 6))
plt.imshow(heatmap, origin="lower", aspect="auto")
plt.colorbar(label="Success rate")

plt.xticks(range(len(heatmap.columns)), heatmap.columns.astype(str), rotation=45)
plt.yticks(range(len(heatmap.index)), heatmap.index.astype(str))

plt.xlabel("Hole width bin [m]")
plt.ylabel("Hole height bin [m]")
plt.title("Success rate heatmap (hole width × height)")
plt.tight_layout()
plt.show()
