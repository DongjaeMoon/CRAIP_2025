#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import numpy as np


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw (rotation around Z axis) from quaternion (x, y, z, w)."""
    # standard ZYX yaw extraction
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Create quaternion for pure yaw rotation (roll = pitch = 0)."""
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def add_noise_to_trajectory(in_path: str, out_path: str,
                            pos_std: float = 0.02,
                            yaw_std_deg: float = 5.0):
    """
    Read TUM-format trajectory and add noise to x, y, yaw.

    - pos_std: std dev for x,y noise [meters]
    - yaw_std_deg: std dev for yaw noise [degrees]
    """
    in_file = Path(in_path)
    out_file = Path(out_path)

    if not in_file.exists():
        raise FileNotFoundError(f"Input file not found: {in_file}")

    yaw_std = math.radians(yaw_std_deg)

    out_file.parent.mkdir(parents=True, exist_ok=True)

    with in_file.open("r") as fin, out_file.open("w") as fout:
        for line in fin:
            stripped = line.strip()

            # Keep empty lines and comment lines as-is
            if not stripped or stripped.startswith("#"):
                fout.write(line)
                continue

            parts = stripped.split()
            # Expect exactly 8 numeric fields: t tx ty tz qx qy qz qw
            if len(parts) != 8:
                # If the format is unexpected, just copy the line
                # or you could raise an error instead.
                fout.write(line)
                continue

            t = float(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])

            # Extract current yaw
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            # Sample noise
            noise_xy = np.random.normal(0.0, pos_std, size=2)
            noise_yaw = np.random.normal(0.0, yaw_std)

            x_noisy = x + float(noise_xy[0])
            y_noisy = y + float(noise_xy[1])
            yaw_noisy = yaw + float(noise_yaw)

            # Rebuild quaternion as yaw-only (assuming 2D base motion)
            qx_n, qy_n, qz_n, qw_n = yaw_to_quaternion(yaw_noisy)

            # TUM format: t tx ty tz qx qy qz qw
            fout.write(
                f"{t:.9f} "
                f"{x_noisy:.6f} {y_noisy:.6f} {z:.6f} "
                f"{qx_n:.6f} {qy_n:.6f} {qz_n:.6f} {qw_n:.6f}\n"
            )

    print(f"Saved noisy trajectory to {out_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Add random noise to x, y, yaw of a TUM-format trajectory."
    )
    parser.add_argument(
        "input_txt",
        type=str,
        help="Input TUM-format trajectory file",
    )
    parser.add_argument(
        "output_txt",
        type=str,
        help="Output TUM-format trajectory file with noise",
    )
    parser.add_argument(
        "--pos_std",
        type=float,
        default=0.02,
        help="Std dev for x,y noise in meters (default: 0.02 = 2cm)",
    )
    parser.add_argument(
        "--yaw_std_deg",
        type=float,
        default=5.0,
        help="Std dev for yaw noise in degrees (default: 5 deg)",
    )

    args = parser.parse_args()
    add_noise_to_trajectory(args.input_txt, args.output_txt,
                            pos_std=args.pos_std,
                            yaw_std_deg=args.yaw_std_deg)


if __name__ == "__main__":
    main()

