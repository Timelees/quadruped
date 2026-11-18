# 三维地形安全走廊验证

独立 `ament_cmake` 包，直接调用生产 `TerrainSafetyCorridor`、`PolynomialHull`、`GridMapTerrainInterface` 和 `SwingTrajectoryQpOptimizer`，不复制优化器。工作区 `src/terrain_safety_corridor_test` 是指向本目录的软链接，与现有 `src/minvo_test` 的包发现方式一致。

## 环境、数值验证、RViz

在工作区根目录执行。遵循当前任务要求，agent 不主动构建；下面的构建命令仅在用户明确要求构建时执行，其余命令使用已有可执行文件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select ocs2_quadruped_controller terrain_safety_corridor_test --parallel-workers 1
source install/setup.bash
ros2 run terrain_safety_corridor_test terrain_corridor_test --verify-only --output src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/data/final
colcon test --packages-select terrain_safety_corridor_test --event-handlers console_direct+
ros2 launch terrain_safety_corridor_test scene.launch.py scene:=bar
```

`scene:=flat|bar|pillars|stairs`；`rviz:=false` 仅发布。单独运行的节点链接实际控制器库，但不会发送机器人控制命令。地图话题 `/corridor_test/elevation_map`，走廊与轨迹 `/corridor_test/markers`。灰色线框为几何走廊，彩色线框和半透明表面为规划走廊，红线为优化后的球心轨迹。地图采用保守双线性插值面上界，而非直接把采样高程当成连续表面。

导出的 `*_planes.csv` 包含每个单元的时间、法向、`b_geom/b_plan/b_qp`；`*_vertices.csv` 是实际半空间交集的顶点；`*_trajectory.csv` 是原参考与优化后的球心轨迹；`*_terrain.csv` 是输入地图。

静态图片直接从这些 CSV 绘制：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/terrain_matplotlib /usr/bin/python3 \
  src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/plot_corridors.py \
  src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/data
```

## 几何与约束契约

1. **真实地形 → 保守占据柱**：`GridMapTerrainInterface::getPrisms()` 对插值面每个 patch 取四角高程最大值。对未知、NaN、地图外查询返回失败，不填成自由空间。
2. **膨胀障碍**：每个占据柱在 XY 和顶部膨胀 `footRadius + mapMargin`。这是球形足端 Minkowski 和的保守外包盒；忽略贴地柱的高度阈值已删除，所有占据柱（包括地面、低障碍和侧方障碍）参与构建。
3. **几何走廊**：每次摆动读取一次覆盖所有单元的保守占据柱快照，再按摆动方向的局部有界盒及每个障碍的严格分离半空间取交集。候选侧向平面必须将整段固定 XY 凸包置于规划侧，否则选择障碍顶部的上跨平面。各段生成的是三维凸多面体，保留 `geometricClearance` 的严格间隙。
4. **规划走廊**：所有单位法向半空间统一收缩 `trackingMargin`，即 `b_plan=b_geom-epsilon`。QP 再保留 `numericalMargin`。沿路径排列的单元可重叠，但不可漏掉多项式时间区间；每个公共时间端点同时满足两侧单元。

所有走廊数据使用**足端球心的世界坐标**；现有控制器样条使用**接触面参考坐标**，故 `p_c=p_ref+footRadius*e_z`。导出函数已计入此偏移，不需要下游再次膨胀半径。

对于 N 个控制点，`corridorControlPointConstraints()` 输出 `A*q<=b`，控制点向量顺序为 `[qx(0..N-1), qy(0..N-1), qz(0..N-1)]`。每个半空间对该段的六个 MINVO/Bernstein 顶点各产生一行：

```
a^T (G_i*q + r*e_z) <= b_plan - numericalMargin
```

生产优化器保留稳定的固定 XY、仅优化 Z 的结构：代入固定 XY 控制点，将上述矩阵精确投影到 Z 决策变量。侧向不可行、固定控制点不可行或最终任何一行越界均拒绝该优化结果，不删除半空间。求解后用完整 `A*q-b` 复核，输出 `corridorCertified/corridorMaxViolation`。

HOCBF 导出针对球心动力学 `p''=u+w`，几何屏障 `h=b_geom-a^T p`，线性类 K 增益 `k1,k2>0`：

```
(-a)^T u >= (k1+k2)*a^T v - k1*k2*h + support_W(a)
```

HOCBF 使用几何边界，MINVO 使用规划边界。该导出**没有接入 WBC 安全过滤**；`h>=0`、`h_dot+k1*h>=0` 的初始条件、跟踪误差管、控制可实现性及单元切换共同可行性仍需单独验证。

## 接触模式与结论边界

固定半径和非零常量误差球不能一直延伸到接触点：足端球体在接触地面时没有向下的位置误差空间。`contactPhase` 显式指定非接触认证区间 `[contactPhase,1-contactPhase]`，在此区间保留完整 `trackingMargin`；起落脚过渡沿用原来的接触包络、地形及小腿约束，**不获得本走廊的常量误差球保证**。

独立几何测试使用 `contactPhase=0.10`，在线 Cyberdog 配置使用 `0.25`（中间 50%）。后者来自实际仿真中早期 2 cm 恒定误差球造成过大修正的诊断，不能当作完整摆动安全证明。若要覆盖整个非接触时域，需要经执行层验证的时变误差集合及与触地模式相容的收缩方式；本实现没有用渐消裕量冒充常量误差球。

本方案是保守的高度地图单足几何走廊构建，不搜索横向拓扑路线。跨不过的高障碍、空走廊、地图未知及固定 XY 不可行会失败。优化器失败时，原有 `SwingTrajectoryPlanner` 仍回退到名义参考；**回退不带走廊认证**。`verifySwingOutput=false` 也不提供接触过渡段独立碰撞认证。实际机器人执行误差、动态障碍、悬垂结构及全身碰撞不由此模块证明。

## 已覆盖的检查

- 平地、跨越障碍、侧柱、逐级楼梯、负高程、斜向运动、弯曲路径、原地抬脚、1 mm 薄障碍。
- 独立地验证每个**完整凸多面体**与所有膨胀占据柱有严格分离平面；检查规划多面体加误差球仍在几何半空间内。
- MINVO/Bernstein 两种基的 3N 线性约束与直接顶点计算逐行一致；HOCBF 符号、增益、扰动项逐行一致。
- NaN、未知/地图外、空单元、非法误差预算拒绝。
- 生产 QP 在实际 `GridMapTerrainInterface` 四个地图场景上运行，检查六顶点、稠密轨迹及起终端位置/速度/加速度。

## Gazebo 楼梯与左侧平台复测

最终配置：楼梯 3/3、平台 3/3 独立启动通过；明细及原始失败记录见 [VALIDATION.md](VALIDATION.md)。完整机器人启动和参数说明以工作区根目录 `QUICK_START.md` 为入口。

```bash
CORRIDOR_TEST=src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test
python3 "$CORRIDOR_TEST/run_trial.py" --scenario stairs --domain 111 --speed .4 \
  --sim-seconds 100 --output "$CORRIDOR_TEST/data/stability/new_stairs_111.json"
python3 "$CORRIDOR_TEST/run_trial.py" --scenario platform --domain 112 --speed .4 \
  --sim-seconds 100 --output "$CORRIDOR_TEST/data/stability/new_platform_112.json"
```

脚本在独立 ROS_DOMAIN_ID/GZ_PARTITION 下新启动无界面 `stair10cm`，等待控制器激活；站立、trot、平台左转 90°、前进到目标、保持 3 s。成功检查四足都在目的面上，真值姿态稳定；并保留配置、源码哈希、launch 日志和 CSV。只清理自己启动的进程组。每轮使用新输出名。`runtime_probe.py` 仍可用于已启动仿真的短程 smoke test，不替代完整场景测试。

## 汇总与论文图表

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CORRIDOR_TEST/analyze_stability.py" \
  "$CORRIDOR_TEST/data/stability" --output "$CORRIDOR_TEST/data/final" \
  --final-trials stairs_capped_103 stairs_final_104 stairs_final_106 \
    platform_capped_105 platform_final_107 platform_final_108
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/terrain_matplotlib /usr/bin/python3 \
  "$CORRIDOR_TEST/plot_evidence.py" "$CORRIDOR_TEST/data/final" --output "$CORRIDOR_TEST/figures"
```

`data/final` 保存几何导出、MINVO 基准、全部试验指标和最终六轮汇总；`data/stability` 保存原始运行数据。五组图的源数据、图注和结论范围见 [figures/README.md](figures/README.md)。
