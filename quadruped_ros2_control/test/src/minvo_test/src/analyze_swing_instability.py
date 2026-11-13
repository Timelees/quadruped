#!/usr/bin/env python3
"""抓取并分析摆动相轨迹的重新优化问题"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from collections import defaultdict
import time

class SwingTrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('swing_trajectory_analyzer')

        # 存储每条轨迹的信息
        self.trajectories = defaultdict(lambda: defaultdict(list))
        self.update_count = defaultdict(int)
        self.start_time = self.get_clock().now()

        # 订阅所有脚的期望轨迹
        for leg in ['LF', 'LH', 'RF', 'RH']:
            self.create_subscription(
                MarkerArray,
                f'/legged_robot/desiredFeetTrajectory/{leg}',
                lambda msg, l=leg: self.callback(msg, l),
                10)

        self.get_logger().info('Swing Trajectory Analyzer started')
        self.get_logger().info('Waiting for trajectory data...')

    def callback(self, msg: MarkerArray, leg: str):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        for marker in msg.markers:
            if marker.type == Marker.LINE_STRIP and len(marker.points) > 0:
                # 使用轨迹的起始和结束时间来标识唯一轨迹
                points = [(p.x, p.y, p.z) for p in marker.points]
                start_pos = points[0]
                end_pos = points[-1]
                max_height = max(p[2] for p in points)

                # 使用起点、终点和最大高度作为轨迹特征
                traj_key = (
                    round(start_pos[0], 3),
                    round(start_pos[1], 3),
                    round(end_pos[0], 3),
                    round(end_pos[1], 3),
                    round(max_height, 3)
                )

                self.trajectories[leg][traj_key].append({
                    'time': current_time,
                    'points': points,
                    'id': marker.id
                })

                self.update_count[leg] += 1

    def analyze(self):
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('摆动相轨迹分析报告')
        self.get_logger().info('='*80)

        for leg in ['LF', 'LH', 'RF', 'RH']:
            if leg not in self.trajectories:
                self.get_logger().info(f'\n{leg}: 无数据')
                continue

            self.get_logger().info(f'\n{leg}: 总更新次数 = {self.update_count[leg]}')
            self.get_logger().info(f'{leg}: 唯一轨迹数量 = {len(self.trajectories[leg])}')

            # 找出被更新最多的轨迹
            sorted_trajs = sorted(
                self.trajectories[leg].items(),
                key=lambda x: len(x[1]),
                reverse=True
            )

            for i, (traj_key, traj_list) in enumerate(sorted_trajs[:5]):
                if len(traj_list) >= 2:
                    times = [t['time'] for t in traj_list]
                    self.get_logger().info(
                        f'  轨迹 {i+1}: 更新 {len(traj_list)} 次, '
                        f'时间范围 [{times[0]:.2f}s, {times[-1]:.2f}s]'
                    )

                    # 检查轨迹点是否在变化
                    first_points = np.array(traj_list[0]['points'])
                    last_points = np.array(traj_list[-1]['points'])

                    if len(first_points) == len(last_points):
                        diff = np.linalg.norm(last_points - first_points, axis=1)
                        max_diff = np.max(diff)
                        mean_diff = np.mean(diff)

                        self.get_logger().info(
                            f'    起点: ({first_points[0][0]:.3f}, {first_points[0][1]:.3f}, {first_points[0][2]:.3f})'
                        )
                        self.get_logger().info(
                            f'    终点: ({first_points[-1][0]:.3f}, {first_points[-1][1]:.3f}, {first_points[-1][2]:.3f})'
                        )
                        self.get_logger().info(
                            f'    最大高度: {np.max(first_points[:, 2]):.3f}m -> {np.max(last_points[:, 2]):.3f}m'
                        )
                        self.get_logger().info(
                            f'    轨迹变化: max={max_diff:.4f}m, mean={mean_diff:.4f}m'
                        )

                        if max_diff > 0.01:
                            self.get_logger().warn(
                                f'    ⚠️  警告: 相同特征的轨迹在发生变化！这会导致摆动相不稳定'
                            )

def main():
    rclpy.init()
    node = SwingTrajectoryAnalyzer()

    # 收集数据
    duration = 10.0
    start = time.time()

    try:
        while time.time() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # 分析数据
    node.analyze()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
