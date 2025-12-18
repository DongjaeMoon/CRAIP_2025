#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Extract yaw (rotation around Z) from a quaternion (ROS convention x,y,z,w).
    """
    # Standard ZYX yaw extraction
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """
    Create a quaternion representing pure yaw (roll = pitch = 0).
    Returns (qx, qy, qz, qw).
    """
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def export_go1_pose_to_tum(bag_dir: str, out_path: str, topic: str = "/go1_pose"):
    """
    Read /go1_pose from a ROS2 bag and export to TUM format:
    timestamp tx ty tz qx qy qz qw

    - tx, ty come from pose.position.{x,y}
    - tz is forced to 0.0
    - yaw is extracted from the original quaternion and re-encoded
      as a yaw-only quaternion (roll=pitch=0).
    """
    bagpath = Path(bag_dir)
    out_file = Path(out_path)

    # Create typestore for legacy ROS2 bags if needed
    typestore = get_typestore(Stores.ROS2_FOXY)

    poses = []

    with AnyReader([bagpath], default_typestore=typestore) as reader:
        # Find the connection for /go1_pose
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise RuntimeError(f"Topic {topic} not found in bag {bag_dir}")

        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            # Timestamp in seconds (double)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Position (we ignore original z and set it to 0)
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = 0.0

            # Original quaternion
            qx_o = float(msg.pose.orientation.x)
            qy_o = float(msg.pose.orientation.y)
            qz_o = float(msg.pose.orientation.z)
            qw_o = float(msg.pose.orientation.w)

            # Extract yaw from original quaternion
            yaw = quaternion_to_yaw(qx_o, qy_o, qz_o, qw_o)

            # Rebuild yaw-only quaternion
            qx, qy, qz, qw = yaw_to_quaternion(yaw)

            poses.append((t, x, y, z, qx, qy, qz, qw))

    # Sort by timestamp just in case
    poses.sort(key=lambda p: p[0])

    # Write TUM formatted txt
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with out_file.open("w") as f:
        for t, x, y, z, qx, qy, qz, qw in poses:
            f.write(f"{t:.9f} {x:.6f} {y:.6f} {z:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    print(f"Saved {len(poses)} poses to {out_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Export /go1_pose from a ROS2 bag to TUM-format txt."
    )
    parser.add_argument(
        "bag_dir",
        type=str,
        help="ROS2 bag directory (folder containing metadata.yaml and *.db3)",
    )
    parser.add_argument(
        "out_path",
        type=str,
        help='Output txt path (e.g. "./trajectory1/go1_pose_gt.txt")',
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/go1_pose",
        help="Pose topic to export (default: /go1_pose)",
    )

    args = parser.parse_args()
    export_go1_pose_to_tum(args.bag_dir, args.out_path, args.topic)


if __name__ == "__main__":
    main()

