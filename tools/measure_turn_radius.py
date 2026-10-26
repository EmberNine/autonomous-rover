#!/usr/bin/env python3
"""Record the ground-truth path and fit a circle to measure the turning radius."""
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

RECORD_SECONDS = 20.0  # one full-lock lap at 0.2 m/s takes ~16.6 s


class PathRecorder(Node):
    def __init__(self):
        super().__init__('turn_radius_recorder')
        self.recorded_positions = []
        self.create_subscription(Odometry, '/odometry/ground_truth', self.record_position, 10)

    def record_position(self, odometry_message):
        position = odometry_message.pose.pose.position
        self.recorded_positions.append((position.x, position.y))


def fit_circle(points):
    """Least-squares circle fit: solve x² + y² + D·x + E·y + F = 0 for D, E, F."""
    x_values, y_values = points[:, 0], points[:, 1]
    design_matrix = np.column_stack([x_values, y_values, np.ones_like(x_values)])
    target = -(x_values**2 + y_values**2)
    (coef_d, coef_e, coef_f), *_ = np.linalg.lstsq(design_matrix, target, rcond=None)
    center_x, center_y = -coef_d / 2, -coef_e / 2
    radius = np.sqrt(center_x**2 + center_y**2 - coef_f)
    distances_from_center = np.hypot(x_values - center_x, y_values - center_y)
    rms_error = np.sqrt(np.mean((distances_from_center - radius) ** 2))
    return center_x, center_y, radius, rms_error


def main():
    rclpy.init()
    recorder = PathRecorder()
    stop_time = time.monotonic() + RECORD_SECONDS
    while time.monotonic() < stop_time:
        rclpy.spin_once(recorder, timeout_sec=0.1)
    points = np.array(recorder.recorded_positions)
    recorder.destroy_node()
    rclpy.shutdown()

    if len(points) < 20:
        print(f'Only {len(points)} samples. Is the sim running (not paused)?')
        return
    center_x, center_y, radius, rms_error = fit_circle(points)
    print(f'samples:      {len(points)}')
    print(f'center:       ({center_x:.3f}, {center_y:.3f}) m')
    print(f'radius:       {radius:.3f} m   (expected ~0.530)')
    print(f'fit RMS error: {rms_error * 1000:.1f} mm   (small = path really is a circle)')


if __name__ == '__main__':
    main()