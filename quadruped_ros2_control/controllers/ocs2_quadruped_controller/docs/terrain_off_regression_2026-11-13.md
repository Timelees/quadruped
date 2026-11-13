# 关闭摆动轨迹优化路径的回归排查（2026-11-13）

## 结论与边界

远端 `git ls-remote` 确认 GitHub master 为 `22e9bbb0885e829bdceccd93472b777aed173cbb`，与工作区 HEAD 相同。

本次修复一个明确的非优化链路行为回归：MPC 的 `_footPlacementCbf` 被整体删除。关闭摆动 QP 不能关闭原本独立存在的落足 CBF。现已恢复 `useTerrainPlanning=false` 时的原落足 CBF 软代价、参数读取及编译项。开启地形规划时保留本次排查前的代价装配行为。

**未证明爬楼稳定性完全恢复。** 修复后有一次完成上楼，但后续受控运行仍侧倾；独立编译的原版在相同快速输入测试中也侧倾。不能把单次上楼当作稳定通过，也不能把每次侧倾都归因于 MINVO 改动。未进行足够重复次数、地图重放或逐项时序消融，不能声称已找到所有不稳定原因。

## 修复的具体改动

1. `PerceptiveLeggedInterface.cpp` 原来无条件删除四条腿的 `_footPlacementCbf` 注册，并删除 `getFootPlacementCbfConstraint()`。该项约束包含足端速度、落足区域剩余距离及其松弛导数，独立于名义高度样条/QP 开关。现仅在关闭地形规划时恢复注册，构造/参数读取函数直接恢复基线实现。
2. 控制器 `CMakeLists.txt` 删除了 `FootPlacementCBFConstraint.cpp`。恢复编译该实现。
3. Cyberdog `task.info` 删除了 `footPlacementCbf`。恢复基线 `mu=1e-2, delta=1e-4, gamma=5, relaxationDerivativeDt=1e-3`。`swingActivationProgress` 继续采用原有回退到普通落足配置的逻辑。

`footPlacementCbfTask` 的 WBC 参数读取在基线中已经注释，WBC 中也没有启用这项任务，故该配置删除不是本次运行回归，没有恢复失效配置。

## 完整链路检查

关闭优化时为：感知参考管理器 → 地形基座参考/凸落足区域 → 原名义高度 B 样条 → MPC 一维 Z 速度等式 + 普通落足软代价 + 落足 CBF 软代价 + 足端 SDF 碰撞软代价 → 原 WBC。

以下源码与基线逐字相同：WbcBase、WeightedWbc、TargetManager、GaitManager、SwitchedModelReferenceManager、FootPlacementConstraint、FootPlacementCBFConstraint、FootPlacementSafetySet、FootPlacementCBFCommon、PerceptiveLeggedPrecomputation，以及 SwingTrajectoryPlanner 的三个名义 update 重载。

B 样条求导 `.eval()` 改动单独用原版/当前实现编译后比较：7 控制点、5 次样条、0.3 s 摆动、起始时间 1/20/100 s，每段 101 个采样点的速度最大差为零。此检查不能覆盖所有可能输入，但不支持将该修复直接认定为本次原因。

仍存在且本轮保留的共享行为差异：地图/SDF 配对快照在参考生成前提交、初始 SDF 构建、正确获取目标输入、SoftConstraint 请求刷新法向约束预计算、可视化调度、MPC 异常停机、仿真参数覆盖。不能声称整条链路与基线逐字或时序完全相同。地图/SDF 同步修复用于避免求解中途写入共享场，不通过恢复并发写入来追求表面一致。

## 构建和回归

用户本轮明确授权构建及仿真，覆盖此前“不主动构建”的默认要求。

- `ocs2_quadruped_controller` 构建通过。
- 新增 `nominal_control_regression`，直接装配生产 `PerceptiveLeggedInterface`，逐腿检查名义 Z 等式、落足 CBF、三维跟踪和原落足/碰撞代价的注册。
- `swing_qp_regression --nominal-only` 检查地形提交顺序、样条导数，以及平地/上台阶/下台阶关闭优化时与名义路径的位置和速度一致、无地形/工作空间查询。
- CTest：`nominal_swing_regression`、`control_problem_false`、`control_problem_true` 共 3/3 通过。
- 全测试包构建被已有 `swing_cache_diagnostic` 的 Eigen 插件头文件搜索路径错误阻断；针对性目标已成功单独构建。
- 原完整 `swing_qp_regression` 在已有 `checkWorkspaceSeed` 失败，输出 `Workspace search rejected a reachable vertical line because nominal height was infeasible.`。本次没有修改开启优化的工作空间算法，也没有改该用例的断言来获得通过。

复跑：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select ocs2_quadruped_controller
colcon build --packages-select minvo_test --cmake-target nominal_control_regression --cmake-force-configure
colcon build --packages-select minvo_test --cmake-target swing_qp_regression
ctest --test-dir build/minvo_test -R '^(nominal_swing_regression|control_problem_false|control_problem_true)$' --output-on-failure
```

## 仿真观察（stair10cm，ROS_DOMAIN_ID=47）

全部通过真实 `ros2 run keyboard_input keyboard_input` 节点输入，记录 Gazebo `/odom`。没有实机控制。

| 运行 | 输入/条件 | 观察 |
| --- | --- | --- |
| 修复版第一次 | 2→3，逐步增加 w 至累计 10 次 | 通过 7 级台阶到平台，x=4.50 m 时 z=0.975 m；x 在 2.0–4.5 m 区间的最大绝对 roll/pitch 为 0.222/0.214 rad。继续行走越过平台边缘后跌落，触发姿态安全停机；不计为完整成功验收。 |
| 修复版受控快速 | 初始等待 5 s，2 后等待 3 s，3 后等待 3 s，10 次 w（0.175 m/s）；x≥4.8 m 或姿态阈值时停车 | 第一阶附近出现侧倾，x=2.225 m、roll=0.603 rad 时自动停车，之后站立恢复。 |
| GitHub 原版独立构建 | 相同快速输入及停车条件；原版控制器和 Cyberdog 配置在 /tmp 独立构建 | 同样侧倾，x=1.921 m、roll=0.606 rad 时自动停车，之后站立恢复。 |
| 修复版受控较缓 | 相同输入时序，改为 6 次 w（0.105 m/s） | 上楼途中侧倾，x=3.128 m、roll=-0.603 rad 时停车，随后倒地。未通过。 |

因此已确认且恢复的是落足 CBF 删除造成的控制问题变化；现有仿真数据不足以把整体稳定性退化归结为它，也不足以宣称所有问题均已修复。保留以上失败数据，下一步需固定地图和输入重放、比较实际 MPC 周期与参考/地图更新，以区分基线已有敏感性和共享时序改动的影响。

原始构建日志、回归输出、控制脚本及各轮 CSV 位于工作区 `log/terrain_off_regression_20261113/`。独立原版源码和构建产物在 `/tmp/terrain_off_audit/baseline/`。本轮仿真与键盘进程在结束时关闭。
