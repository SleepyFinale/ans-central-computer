#!/usr/bin/env python3
"""Pull occupancy-grid snapshots and the trajectory for a robot out of a rosbag.

Snapshots are taken at fixed fractions of the map messages, so an early one is a
genuinely partial view of the area and the last one is the final map.

Run with system Python after sourcing ROS 2.

Usage:
  python3 extract_maps.py data/bags/large2 --robot inky --topic map_live -o data/maps/large2_inky.npz
"""

import argparse

import numpy as np
import rosbag2_py
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.serialization import deserialize_message


FRACTIONS = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0)


def read_bag(path, map_topic, odom_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    maps = []
    odom = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == map_topic:
            maps.append(deserialize_message(data, OccupancyGrid))
        elif topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            odom.append((stamp * 1e-9, msg.pose.pose.position.x, msg.pose.pose.position.y))
    return maps, np.array(odom)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--robot', default='inky')
    parser.add_argument('--topic', default='map_live', help='map topic below /<robot>/')
    parser.add_argument('-o', '--output', required=True)
    args = parser.parse_args()

    maps, odom = read_bag(args.bag, f'/{args.robot}/{args.topic}', f'/{args.robot}/odom')
    if not maps:
        raise SystemExit(f'no /{args.robot}/{args.topic} messages in {args.bag}')

    arrays = {'odom': odom, 'fractions': np.array(FRACTIONS)}
    for k, fraction in enumerate(FRACTIONS):
        msg = maps[int(round(fraction * (len(maps) - 1)))]
        info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        arrays[f'grid_{k}'] = grid
        arrays[f'resolution_{k}'] = info.resolution
        arrays[f'origin_{k}'] = np.array([info.origin.position.x, info.origin.position.y])
        free = np.count_nonzero(grid == 0) * info.resolution ** 2
        print(f'  {fraction:4.0%}: {info.width}x{info.height}, free {free:5.1f} m2')
    np.savez_compressed(args.output, **arrays)
    print(f'{args.output}: {len(maps)} map messages, {len(odom)} odom samples')


if __name__ == '__main__':
    main()
