#!/usr/bin/env python3
"""
Record swing trajectory data for comparison between useTerrainPlanning=true/false
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import csv
import time

class SwingTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('swing_trajectory_recorder')

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Data storage
        self.data = []
        self.start_time = None

        # CSV file
        self.csv_file = open('/tmp/swing_trajectory_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'base_x', 'base_y', 'base_z',
            'FL_hip', 'FL_thigh', 'FL_calf',
            'FR_hip', 'FR_thigh', 'FR_calf',
            'RL_hip', 'RL_thigh', 'RL_calf',
            'RR_hip', 'RR_thigh', 'RR_calf'
        ])

        self.get_logger().info('Swing Trajectory Recorder started. Recording to /tmp/swing_trajectory_data.csv')

    def joint_state_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Extract joint positions
        joint_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_map[name] = msg.position[i]

        # Get specific joints (adjust names based on your robot)
        fl_hip = joint_map.get('FL_hip_joint', 0.0)
        fl_thigh = joint_map.get('FL_thigh_joint', 0.0)
        fl_calf = joint_map.get('FL_calf_joint', 0.0)

        fr_hip = joint_map.get('FR_hip_joint', 0.0)
        fr_thigh = joint_map.get('FR_thigh_joint', 0.0)
        fr_calf = joint_map.get('FR_calf_joint', 0.0)

        rl_hip = joint_map.get('RL_hip_joint', 0.0)
        rl_thigh = joint_map.get('RL_thigh_joint', 0.0)
        rl_calf = joint_map.get('RL_calf_joint', 0.0)

        rr_hip = joint_map.get('RR_hip_joint', 0.0)
        rr_thigh = joint_map.get('RR_thigh_joint', 0.0)
        rr_calf = joint_map.get('RR_calf_joint', 0.0)

        self.data.append({
            'time': current_time,
            'joints': [fl_hip, fl_thigh, fl_calf, fr_hip, fr_thigh, fr_calf,
                      rl_hip, rl_thigh, rl_calf, rr_hip, rr_thigh, rr_calf]
        })

    def odom_callback(self, msg):
        if len(self.data) > 0:
            # Update latest entry with base position
            self.data[-1]['base_x'] = msg.pose.pose.position.x
            self.data[-1]['base_y'] = msg.pose.pose.position.y
            self.data[-1]['base_z'] = msg.pose.pose.position.z

            # Write to CSV
            self.csv_writer.writerow([
                self.data[-1]['time'],
                self.data[-1]['base_x'],
                self.data[-1]['base_y'],
                self.data[-1]['base_z'],
            ] + self.data[-1]['joints'])

    def cleanup(self):
        self.csv_file.close()
        self.get_logger().info(f'Recording finished. Total samples: {len(self.data)}')

def main():
    rclpy.init()
    node = SwingTrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
