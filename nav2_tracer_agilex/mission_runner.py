#!/usr/bin/env python3
"""
Autonomous waypoint mission runner.
Iterates hardcoded waypoints sequentially; skips failed waypoints autonomously.
Runs on Jetson — no human intervention required.
"""
import json
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math


# Waypoints hardcoded day before. Format: (x, y, yaw_deg)
# REPLACE with real measured coordinates from GLIM map
WAYPOINTS_XYZ: list[tuple[float, float, float]] = [
    # (x, y, yaw_degrees)
    # Example — replace with actual challenge waypoints
    (1.0,  0.0,  0.0),
    (2.0,  1.0,  90.0),
    (3.0,  1.0,  0.0),
    (0.0,  0.0,  180.0),  # home
]


def make_pose(x: float, y: float, yaw_deg: float, frame_id: str = 'map') -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.get_logger().info('MissionRunner: waiting for Nav2...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 active — starting mission')

    for i, (x, y, yaw) in enumerate(WAYPOINTS_XYZ):
        nav.get_logger().info(f'Waypoint {i+1}/{len(WAYPOINTS_XYZ)}: ({x:.2f}, {y:.2f}, {yaw:.0f}deg)')
        goal = make_pose(x, y, yaw)
        goal.header.stamp = nav.get_clock().now().to_msg()
        nav.goToPose(goal)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                nav.get_logger().info(f'  → distance remaining: {dist:.2f}m', throttle_duration_sec=2.0)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Waypoint {i+1} SUCCEEDED')
        else:
            nav.get_logger().warn(f'Waypoint {i+1} FAILED (result={result}) — skipping')
            continue

    nav.get_logger().info('Mission complete')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
