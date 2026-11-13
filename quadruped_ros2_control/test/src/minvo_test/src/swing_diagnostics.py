#!/usr/bin/env python3
"""诊断摆动相轨迹优化问题"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
from collections import defaultdict
import time
import sys

class SwingDiagnostics(Node):
    def __init__(self):
        super().__init__('swing_diagnostics')
        self.swing_trajectories = defaultdict(list)
        self.opt_trajectories = []
        self.subscription = self.create_subscription(
            MarkerArray,
            '/legged_robot/optimizedStateTrajectory',
            self.opt_trajectory_callback,
            10)

        # 订阅所有脚的期望轨迹
        for leg in ['LF', 'LH', 'RF', 'RH']:
            self.create_subscription(
                MarkerArray,
                f'/legged_robot/desiredFeetTrajectory/{leg}',
                lambda msg, l=leg: self.foot_trajectory_callback(msg, l),
                10)

        self.start_time = self.get_clock().now()
        self.get_logger().info('Swing Diagnostics Node Started')

    def foot_trajectory_callback(self, msg: MarkerArray, leg: str):
        """抓取每个脚的摆动相轨迹"""
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        for marker in msg.markers:
            if marker.type == Marker.LINE_STRIP:
                # 提取轨迹点
                points = [(p.x, p.y, p.z) for p in marker.points]
                if points:
                    self.swing_trajectories[leg].append({
                        'time': current_time,
                        'points': points,
                        'id': marker.id
                    })

                    # 只保留最近的数据
                    if len(self.swing_trajectories[leg]) > 100:
                        self.swing_trajectories[leg] = self.swing_trajectories[leg][-50:]

    def opt_trajectory_callback(self, msg: MarkerArray):
        """抓取优化后的状态轨迹"""
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.opt_trajectories.append({
            'time': current_time,
            'markers': len(msg.markers)
        })
        if len(self.opt_trajectories) > 100:
            self.opt_trajectories = self.opt_trajectories[-50:]

    def analyze_trajectories(self):
        """分析轨迹数据"""
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('摆动相轨迹诊断报告')
        self.get_logger().info('='*80)

        for leg in ['LF', 'LH', 'RF', 'RH']:
            data = self.swing_trajectories[leg]
            if not data:
                self.get_logger().info(f'\n{leg}: 无数据')
                continue

            self.get_logger().info(f'\n{leg}: 共 {len(data)} 条轨迹记录')

            # 分析轨迹变化
            if len(data) >= 2:
                # 检查同一id的轨迹是否在不断变化
                id_changes = defaultdict(list)
                for d in data:
                    id_changes[d['id']].append(d)

                for traj_id, traj_list in id_changes.items():
                    if len(traj_list) >= 2:
                        # 计算轨迹之间的差异
                        first_points = np.array(traj_list[0]['points'])
                        last_points = np.array(traj_list[-1]['points'])

                        if len(first_points) == len(last_points):
                            diff = np.linalg.norm(last_points - first_points, axis=1)
                            max_diff = np.max(diff)
                            mean_diff = np.mean(diff)

                            self.get_logger().info(
                                f'  轨迹ID {traj_id}: 更新 {len(traj_list)} 次, '
                                f'最大变化 {max_diff:.4f}m, 平均变化 {mean_diff:.4f}m'
                            )

                            # 检查Z轴变化（摆动高度）
                            z_first = first_points[:, 2]
                            z_last = last_points[:, 2]
                            z_diff = z_last - z_first
                            self.get_logger().info(
                                f'    Z轴变化: max={np.max(z_diff):.4f}m, '
                                f'min={np.min(z_diff):.4f}m, mean={np.mean(z_diff):.4f}m'
                            )

                            # 检查是否有异常大的变化
                            if max_diff > 0.05:  # 5cm
                                self.get_logger().warn(
                                    f'    警告: 轨迹变化过大 ({max_diff:.4f}m)！'
                                )

def main():
    rclpy.init()
    node = SwingDiagnostics()

    # 运行一段时间收集数据
    duration = 15.0  # 秒
    start = time.time()

    try:
        while time.time() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # 分析数据
    node.analyze_trajectories()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
