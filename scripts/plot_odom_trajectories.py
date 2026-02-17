#!/usr/bin/env python3
"""
Plot x-y trajectories from odometry topics in a rosbag2 file.
Extracts data from /blinky/odom and /pinky/odom and plots them together.
"""

import argparse
import sys
from pathlib import Path

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from nav_msgs.msg import Odometry
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as e:
    print(f"Error importing required packages: {e}")
    print("\nPlease install required packages:")
    print("  pip install matplotlib numpy")
    print("  (rosbag2_py should be available if ROS 2 is sourced)")
    sys.exit(1)


def extract_odom_data(bag_path, topic_name):
    """
    Extract x-y positions from odometry messages in a rosbag2 file.
    
    Args:
        bag_path: Path to the rosbag2 directory
        topic_name: Name of the odometry topic (e.g., '/blinky/odom')
    
    Returns:
        Tuple of (x_positions, y_positions, timestamps)
    """
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types to verify the topic exists
    topic_types = reader.get_all_topics_and_types()
    topic_exists = False
    for topic_type in topic_types:
        if topic_type.name == topic_name:
            topic_exists = True
            break
    
    if not topic_exists:
        print(f"Warning: Topic {topic_name} not found in bag file")
        return [], [], []
    
    x_positions = []
    y_positions = []
    timestamps = []
    
    # Read all messages and filter by topic name
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, Odometry)
            x_positions.append(msg.pose.pose.position.x)
            y_positions.append(msg.pose.pose.position.y)
            timestamps.append(timestamp)
    
    return x_positions, y_positions, timestamps


def plot_trajectories(blinky_data, pinky_data, output_file=None):
    """
    Plot x-y trajectories and pose over time (Time vs X, Time vs Y) for both robots.
    
    Args:
        blinky_data: Tuple of (x, y, timestamps) for blinky
        pinky_data: Tuple of (x, y, timestamps) for pinky
        output_file: Optional path to save the plot
    """
    blinky_x, blinky_y, blinky_t = blinky_data
    pinky_x, pinky_y, pinky_t = pinky_data

    # Elapsed time in seconds (common t0 = earliest timestamp across both)
    all_ts = (blinky_t or []) + (pinky_t or [])
    t0 = min(all_ts) if all_ts else 0
    blinky_elapsed = np.array([(t - t0) / 1e9 for t in blinky_t]) if blinky_t else np.array([])
    pinky_elapsed = np.array([(t - t0) / 1e9 for t in pinky_t]) if pinky_t else np.array([])

    duration_s = max(
        blinky_elapsed[-1] if len(blinky_elapsed) else 0,
        pinky_elapsed[-1] if len(pinky_elapsed) else 0,
    )

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # --- Top-left: X-Y trajectory ---
    ax_xy = axes[0, 0]
    if blinky_x and blinky_y:
        ax_xy.plot(blinky_x, blinky_y, 'b-', label='Blinky', linewidth=2, alpha=0.7)
        ax_xy.plot(blinky_x[0], blinky_y[0], 'bo', markersize=10, label='Blinky Start')
        ax_xy.plot(blinky_x[-1], blinky_y[-1], 'bs', markersize=10, label='Blinky End')
    if pinky_x and pinky_y:
        ax_xy.plot(pinky_x, pinky_y, 'r-', label='Pinky', linewidth=2, alpha=0.7)
        ax_xy.plot(pinky_x[0], pinky_y[0], 'ro', markersize=10, label='Pinky Start')
        ax_xy.plot(pinky_x[-1], pinky_y[-1], 'rs', markersize=10, label='Pinky End')
    ax_xy.set_xlabel('X Position (m)', fontsize=11)
    ax_xy.set_ylabel('Y Position (m)', fontsize=11)
    ax_xy.set_title(f'Trajectory (duration: {duration_s:.1f} s)', fontsize=12, fontweight='bold')
    ax_xy.legend(loc='best', fontsize=9)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis('equal')

    stats_text = []
    if blinky_x and blinky_y:
        blinky_dist = np.sum(np.sqrt(np.diff(blinky_x)**2 + np.diff(blinky_y)**2))
        stats_text.append(f'Blinky: {len(blinky_x)} pts, {blinky_dist:.2f}m')
    if pinky_x and pinky_y:
        pinky_dist = np.sum(np.sqrt(np.diff(pinky_x)**2 + np.diff(pinky_y)**2))
        stats_text.append(f'Pinky: {len(pinky_x)} pts, {pinky_dist:.2f}m')
    if stats_text:
        ax_xy.text(0.02, 0.98, '\n'.join(stats_text),
                  transform=ax_xy.transAxes, verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5), fontsize=8)

    # --- Top-right: Time vs X ---
    ax_tx = axes[0, 1]
    if len(blinky_elapsed) and len(blinky_x):
        ax_tx.plot(blinky_elapsed, blinky_x, 'b-', label='Blinky', linewidth=1.5, alpha=0.8)
    if len(pinky_elapsed) and len(pinky_x):
        ax_tx.plot(pinky_elapsed, pinky_x, 'r-', label='Pinky', linewidth=1.5, alpha=0.8)
    ax_tx.set_xlabel('Time (s)', fontsize=11)
    ax_tx.set_ylabel('X Position (m)', fontsize=11)
    ax_tx.set_title('Pose over time: X', fontsize=12, fontweight='bold')
    ax_tx.legend(loc='best', fontsize=9)
    ax_tx.grid(True, alpha=0.3)

    # --- Bottom-left: Time vs Y ---
    ax_ty = axes[1, 0]
    if len(blinky_elapsed) and len(blinky_y):
        ax_ty.plot(blinky_elapsed, blinky_y, 'b-', label='Blinky', linewidth=1.5, alpha=0.8)
    if len(pinky_elapsed) and len(pinky_y):
        ax_ty.plot(pinky_elapsed, pinky_y, 'r-', label='Pinky', linewidth=1.5, alpha=0.8)
    ax_ty.set_xlabel('Time (s)', fontsize=11)
    ax_ty.set_ylabel('Y Position (m)', fontsize=11)
    ax_ty.set_title('Pose over time: Y', fontsize=12, fontweight='bold')
    ax_ty.legend(loc='best', fontsize=9)
    ax_ty.grid(True, alpha=0.3)

    # --- Bottom-right: optional time span / duration summary ---
    ax_info = axes[1, 1]
    ax_info.axis('off')
    info_lines = [
        f'Run duration: {duration_s:.2f} s',
        f'Blinky: {len(blinky_x)} samples, t ∈ [0, {blinky_elapsed[-1]:.2f}] s' if len(blinky_elapsed) else 'Blinky: no data',
        f'Pinky:  {len(pinky_x)} samples, t ∈ [0, {pinky_elapsed[-1]:.2f}] s' if len(pinky_elapsed) else 'Pinky: no data',
    ]
    ax_info.text(0.1, 0.5, '\n'.join(info_lines), fontsize=11, verticalalignment='center',
                 family='monospace', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Plot x-y trajectories from odometry topics in a rosbag2 file'
    )
    parser.add_argument(
        'bag_path',
        type=str,
        help='Path to the rosbag2 directory'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output file path for the plot (e.g., trajectory.png). If not specified, displays interactively.'
    )
    parser.add_argument(
        '--blinky-topic',
        type=str,
        default='/blinky/odom',
        help='Blinky odometry topic name (default: /blinky/odom)'
    )
    parser.add_argument(
        '--pinky-topic',
        type=str,
        default='/pinky/odom',
        help='Pinky odometry topic name (default: /pinky/odom)'
    )
    
    args = parser.parse_args()
    
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag path does not exist: {bag_path}")
        sys.exit(1)
    
    # Initialize ROS 2
    # Set ROS_HOME to a writable directory if needed
    import os
    if 'ROS_HOME' not in os.environ:
        os.environ['ROS_HOME'] = str(Path.home() / '.ros')
    
    rclpy.init()
    
    try:
        print(f"Reading odometry data from {bag_path}...")
        print(f"  Extracting {args.blinky_topic}...")
        blinky_data = extract_odom_data(bag_path, args.blinky_topic)
        print(f"    Found {len(blinky_data[0])} messages")
        
        print(f"  Extracting {args.pinky_topic}...")
        pinky_data = extract_odom_data(bag_path, args.pinky_topic)
        print(f"    Found {len(pinky_data[0])} messages")
        
        if not blinky_data[0] and not pinky_data[0]:
            print("Error: No odometry data found in bag file")
            sys.exit(1)
        
        print("\nPlotting trajectories...")
        plot_trajectories(blinky_data, pinky_data, args.output)
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
