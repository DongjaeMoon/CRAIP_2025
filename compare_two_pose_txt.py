#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path

import numpy as np

from evo.core import metrics, sync
from evo.tools import file_interface, log


def run_evo_analysis(gt_path: Path, pred_path: Path) -> None:
    """
    1) Load TUM trajectories
    2) Sync by timestamps (NO spatial alignment)
    3) Compute APE (translation_part) in Python with align disabled
    4) Save statistics + timewise errors
    5) Call evo_ape CLI (no align flags) for default visualization
    """
    log.configure_logging(verbose=True)

    # 1) Load trajectories (TUM format)
    traj_ref = file_interface.read_tum_trajectory_file(str(gt_path))
    traj_est = file_interface.read_tum_trajectory_file(str(pred_path))

    # 2) Synchronize timestamps ONLY (no spatial alignment)
    traj_ref_sync, traj_est_sync = sync.associate_trajectories(traj_ref, traj_est)
    print(
        f"Loaded {len(traj_ref_sync.poses_se3)} ref poses and "
        f"{len(traj_est_sync.poses_se3)} est poses after sync."
    )

    # 3) APE on translation only (TUM-style ATE), with alignment OFF
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)

    # Safety: explicitly disable any alignment/scale options if present
    if hasattr(ape_metric, "align"):
        ape_metric.align = False
    if hasattr(ape_metric, "correct_scale"):
        ape_metric.correct_scale = False
    if hasattr(ape_metric, "align_origin"):
        ape_metric.align_origin = False

    ape_metric.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric.get_all_statistics()

    print("\n=== APE (translation, no alignment) Statistics ===")
    for key in ["min", "max", "mean", "median", "rmse", "sse", "std"]:
        val = ape_stats.get(key, None)
        if val is not None:
            print(f"{key:>7}: {val:.6f}")

    # 4) Save statistics + timewise ATE
    out_dir = pred_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # 4-1) Scalar stats
    stats_path = out_dir / "ape_stats.txt"
    with stats_path.open("w") as f:
        f.write("APE (translation, no alignment) statistics\n")
        for key in ["min", "max", "mean", "median", "rmse", "sse", "std"]:
            val = ape_stats.get(key, None)
            if val is not None:
                f.write(f"{key}: {val:.9f}\n")
    print(f"Saved APE statistics to: {stats_path}")

    # 4-2) Timewise ATE: timestamp + error
    errors = ape_metric.error
    timestamps = traj_ref_sync.timestamps
    time_error_arr = np.vstack([timestamps, errors]).T

    timewise_path = out_dir / "ape_timewise.txt"
    np.savetxt(
        timewise_path,
        time_error_arr,
        fmt="%.9f",
        header="timestamp error(m)",
    )
    print(f"Saved timewise ATE to: {timewise_path}")

    # 5) Use evo_ape CLI for default visualization (no alignment flags!)
    plot_path = out_dir / "evo_ape_plot.png"
    results_zip = out_dir / "evo_ape_results.zip"

    cmd = [
        "evo_ape",
        "tum",
        str(gt_path),
        str(pred_path),
        "-v",                         # verbose
        "-r", "trans_part",           # translation-only APE (ATE-style)
        "--plot",                     # show evo's default plot
        "--plot_mode", "xy",          # 2D XY trajectory view
        "--save_plot", str(plot_path),
        "--save_results", str(results_zip),
    ]

    print("\nRunning evo_ape CLI for default visualization (no align flags)...")
    try:
        subprocess.run(cmd, check=True)
        print(f"Saved evo plot to: {plot_path}")
        print(f"Saved evo result zip to: {results_zip}")
    except FileNotFoundError:
        print(
            "[WARNING] 'evo_ape' command not found. "
            "Install evo CLI (pip install evo) or add it to PATH."
        )
    except subprocess.CalledProcessError as e:
        print("[WARNING] evo_ape failed:")
        print(e)


def main():
    parser = argparse.ArgumentParser(
        description="Compare two TUM pose TXT files with evo (no alignment) "
                    "and save stats + default plots."
    )
    parser.add_argument(
        "--gt_txt",
        type=str,
        required=True,
        help="Path to ground-truth TUM pose file",
    )
    parser.add_argument(
        "--pred_txt",
        type=str,
        required=True,
        help="Path to predicted TUM pose file",
    )

    args = parser.parse_args()
    gt_path = Path(args.gt_txt).expanduser().resolve()
    pred_path = Path(args.pred_txt).expanduser().resolve()

    run_evo_analysis(gt_path, pred_path)


if __name__ == "__main__":
    main()

