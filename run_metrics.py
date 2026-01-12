import random
import multiprocessing as mp
import pandas as pd
import numpy as np
from tqdm import tqdm

from drone_hole import run_sim

def worker_run_sim(params):
    """Run a single simulation safely in a subprocess."""
    try:
        # Copy params so we don't mutate the original
        sim_params = params.copy()

        # Extract bookkeeping fields
        run_id = sim_params.pop("run_id", None)
        seed = sim_params.pop("seed", None)

        # Set seeds (optional but recommended)
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)

        # Run simulation with ONLY valid arguments
        result = run_sim(**sim_params)

        # Re-attach metadata
        result.update(params)

        return result

    except Exception as e:
        return {
            **params,
            "flight_time": np.nan,
            "distance_to_goal": np.nan,
            "success": False,
            "collision": False,
            "termination_reason": "exception",
            "error": str(e),
        }


import itertools

def generate_experiments():
    experiments = []

    for walls, w, h, padding in itertools.product(
        WALLS, HOLE_WIDTHS, HOLE_HEIGHTS, PADDINGS
    ):
        for run_id in range(REPEATS):
            experiments.append({
                "gui": False,
                "walls": walls,
                "hole_width": w,
                "hole_height": h,
                "padding": padding,

                # bookkeeping
                "run_id": run_id,
            })

    return experiments



if __name__ == "__main__":
    WALLS = [2, 3, 4, 5, 6]
    HOLE_WIDTHS = [0.35, 0.45, 0.55, 0.65, 0.75]
    HOLE_HEIGHTS = [0.35, 0.45, 0.55, 0.65, 0.75]
    PADDINGS = [0.03, 0.05, 0.07]

    REPEATS = 5   # number of runs per configuration

    experiments = generate_experiments()
    num_workers = max(1, mp.cpu_count() - 1)

    print(f"Running {len(experiments)} sims with {num_workers} workers")

    results = []  # IMPORTANT: define outside try

    try:
        with mp.Pool(processes=num_workers) as pool:
            for result in tqdm(
                pool.imap_unordered(worker_run_sim, experiments),
                total=len(experiments)
            ):
                results.append(result)

    except KeyboardInterrupt:
        print("\nInterrupted by user — saving partial results...")

    finally:
        # Always save whatever we have
        if len(results) > 0:
            df = pd.DataFrame(results)
            df.to_csv("drone_hole_experiments.csv", index=False)

            print(f"Saved {len(df)} results to drone_hole_experiments.csv")
            print(df.head())
        else:
            print("No results to save.")
