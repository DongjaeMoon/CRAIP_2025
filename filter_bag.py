import argparse
from pathlib import Path
from typing import cast

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys.stores import Stores, get_typestore
from rosbags.interfaces import ConnectionExtRosbag2


def remove_topics(src: Path, dst: Path, topics_to_remove):
    """Copy rosbag2 from src to dst, excluding given topics."""
    typestore = get_typestore(Stores.ROS2_FOXY)

    with Reader(src) as reader, Writer(dst, version=9) as writer:
        conn_map = {}

        # 1) Register all connections except the ones we want to drop
        for conn in reader.connections:
            if conn.topic in topics_to_remove:
                print(f"Skipping topic {conn.topic}")
                continue

            ext = cast(ConnectionExtRosbag2, conn.ext)
            new_id = writer.add_connection(
                conn.topic,
                conn.msgtype,
                typestore=typestore,
                serialization_format=ext.serialization_format,
                offered_qos_profiles=ext.offered_qos_profiles,
            )
            conn_map[conn.id] = new_id

        # 2) Copy messages for the kept connections
        kept_conns = [c for c in reader.connections if c.id in conn_map]

        for conn, timestamp, data in reader.messages(connections=kept_conns):
            writer.write(conn_map[conn.id], timestamp, data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Filter out topics from a ROS2 bag"
    )

    parser.add_argument(
        "--src",
        type=str,
        required=True,
        help="Path to input rosbag2 directory (e.g., trajectory_gt1)"
    )

    parser.add_argument(
        "--dst",
        type=str,
        required=True,
        help="Path to output rosbag2 directory (e.g., trajectory1)"
    )

    parser.add_argument(
        "--remove",
        nargs="+",
        default=["/go1_pose", "/go1_pose_2d"],   # ← Default topics removed
        help="Topics to remove (default: /go1_pose /go1_pose_2d)"
    )

    args = parser.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    topics_to_remove = set(args.remove)

    print(f"Input bag : {src}")
    print(f"Output bag: {dst}")
    print(f"Removing topics: {topics_to_remove}")

    remove_topics(src, dst, topics_to_remove)

