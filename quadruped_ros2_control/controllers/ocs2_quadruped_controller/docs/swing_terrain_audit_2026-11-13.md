# 地形摆动规划爬楼复现与修复（2026-11-13）

本轮遵循用户“不主动执行 colcon build 等构建命令”的要求。以下仿真均使用修改前安装库（mtime 14:57:13，SHA256 a8c18b0160aee3b0b48d9300cdf97489eb3c4e840f8aa55c806f20085199883e），不能当作本轮源码修复通过验证。

## 实际运行

世界 stair10cm，原始配置 useTerrainPlanning=true，MINVO，trackingWeight=25，ROS_DOMAIN_ID=47，GZ_PARTITION=swing_terrain_audit。通过真实 keyboard_input 的 PTY 输入 2→等待3秒→3→等待3秒→10次w（ly=0.5，对应目标前向速度0.175m/s）；x>=4.8m 或 |roll|>0.6 / |pitch|>0.7 时停车。自动驾驶脚本按 /odom 仿真时间切换，外层180秒墙钟超时。实际楼梯第一级中心 x=2.4m、长0.3m，前沿是2.25m。

- 原权重25：t=146.836、x=2.32285m、pitch=-0.71107rad 触发停车，随后翻倒；全段15次QP divergence/fallback，首次146.08。QP单次耗时中位数3.252ms、P99=6.528ms。
- 临时权重100：t=25.356、x=2.23593m、roll=-0.60740rad 触发停车，随后翻倒；6次QP divergence/fallback。该对照不足以说明增益与失稳的因果关系，但不支持直接增大跟踪权重作为修复。临时配置已逐字恢复原始版本。
- 原始数据位于工作区 log/swing_terrain_audit_20261113/：sim.log、controlled.csv、weight100.sim.log、weight100.csv、summary.json，baseline_bag/ 包含 odom、Ground_Truth、joint_states、control_input、预测足端、估计足端、地图、TF。第二轮只有日志和机身CSV，没有完整rosbag。
- feet.csv 从 currentState 的估计运动学足端得到，使用世界SDF盒体计算19mm足端球的距离。失稳前未发现超过5mm的穿入；约1–1.6mm负间隙可能来自估计/接触误差。未采集Gazebo接触力，不能声称本次已证实足端撞阶导致翻倒。desiredFeetTrajectory 是目标状态的运动学可视化，不等于QP名义样条；optimizedStateTrajectory 是MPC预测，也不等于物理实际轨迹。

## 确认的源码缺陷和本轮修复

1. **QP失败回退到XY=0。** createSwingHeightBSpline 只构建高度，XY控制点全为零。useTerrainPlanning=true 时，PreComputation 与 LeggedInterface 却对该样条继续施加三维软跟踪，位置误差增益20/s。机器人在x≈2.3m时，这意味着约46m/s量级的错误水平跟踪残差。回退前后的参考会突变，且未来horizon padding也有同样问题。现在给地形分支的名义样条补齐真实起落点之间的五次smoothstep XY，保持原Z完全不变；端点XY速度和加速度为零。false路径不变。日志证明翻倒前出现过QP回退，但尚无逐周期名义样条数据证明某个回退段最终被执行，因果链仍需复测。
2. **碰撞约束被静默丢弃。** solveQp 原先丢弃freeNorm<=0.1的所有约束，只验证剩余行。该阈值会丢弃端点附近仍可调整的碰撞约束，甚至固定端点的实际碰撞，verifySwingOutput=false时尤其不能发现。现在只把数值零自由系数当作常量行；常量行违反则返回失败；小非零行归一化后参与QP，成功返回前再检查所有原始约束。不可行端点不再伪报成功，因此某些地图误差场景的fallback次数可能增加。

3. **发散检查读取了上一轮解。** qpOASES init 的 xOpt 参数是初始猜测，不是输出缓冲；原先先检查 primal.norm() 再 getPrimalSolution，最后一轮若得到过大的可行修正可能直接被接受。现在先提取最新解，再检查有限性和原有1m修正范数上限。

没有把fallback称为无碰撞认证：它保留名义抬脚高度，但并不保证跨越所有障碍。QP只认证名义参考，三维软代价也不能保证实际足端净空；不能用上述修复直接宣称碰撞问题全部解决。

## 检查与后续验证边界

已执行 git diff --check；独立de Boor数值检查确认7控制点XY与五次smoothstep最大误差1.33e-15。在现有swing_qp_regression.cpp增加地图缺失回退、padding、非原点XY及端点速度用例，并把“固定端点穿地也应该求解成功”的旧断言改成应拒绝，同时保留合法端点可行性用例。这些C++用例尚未编译和执行。

下一次用户构建后，需先运行 swing_qp_regression，再用同一输入跑true/false配对爬楼；增加逐周期名义样条、实际足端跟踪误差和Gazebo接触事件记录，才能区分QP曲线、MPC执行偏差和碰撞。此次没有调整jerk权重或声称实现了全局最小jerk：当前目标惩罚的是相对名义曲线的导数修正；需要拿到执行曲线后再决定是否改变这一目标。


## 后续验证更新

用户随后明确授权编译与仿真。上述源码修复已随停车协调修复编译安装，7项相关回归通过；平地前进/左右转停车通过，但楼梯入口停车后继续爬楼仍侧翻。详见工作区 log/stop_fix_20261113/README.md，不应据此宣称整个地形规划链路已稳定。
