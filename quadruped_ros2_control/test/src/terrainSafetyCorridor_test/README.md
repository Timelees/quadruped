# 三维地形安全走廊验证

P0/P1 在线重规划、HOCBF、误差管、接近模式及最新 Gazebo 记录，见工作区根目录 `log/p0_validation/README.md` 和 `QUICK_START.md`。下文的 `data/final`、`data/stability` 为原几何与关闭过滤器的基线，不能替代新版本回归。

自动横障试验现从 `gazebo.launch.py` 默认原点出生，转向 −X 后接近 `x=-1.2` 的障碍；旧的 `x=-2` 出生、沿 +X 路线需显式传 `--x -2`。不传 `--obstacle-height` 使用原 world；显式 `.12` 生成顶部高程为 12 cm 的隔离副本。当前原 world 箱体中心为 z=3 cm、厚度为 10 cm，顶部实际为 8 cm，勿将箱体厚度当作地面以上高度。

独立 `ament_cmake` 包，直接调用生产 `TerrainSafetyCorridor`、`PolynomialHull`、`GridMapTerrainInterface` 和 `SwingTrajectoryQpOptimizer`，不复制优化器。工作区 `src/terrain_safety_corridor_test` 是指向本目录的软链接，与现有 `src/minvo_test` 的包发现方式一致。

## 环境、数值验证、RViz

在工作区根目录执行。遵循当前任务要求，agent 不主动构建；下面的构建命令仅在用户明确要求构建时执行，其余命令使用已有可执行文件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 nice -n 10 colcon build --packages-select ocs2_quadruped_controller terrain_safety_corridor_test --parallel-workers 1
source install/setup.bash
ros2 run terrain_safety_corridor_test terrain_corridor_test --verify-only --output src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/data/final
colcon test --packages-select terrain_safety_corridor_test --event-handlers console_direct+
ros2 launch terrain_safety_corridor_test scene.launch.py scene:=bar
```

`scene:=flat|bar|pillars|stairs`；`rviz:=false` 仅发布。地图话题为 `/corridor_test/elevation_map`，走廊、参考轨迹及优化轨迹为 `/corridor_test/markers`。节点调用生产库，不发送机器人控制指令。

### 新版显示含义

| 显示 | 含义 |
|---|---|
| 红色虚线、红点 | 地形修正前名义接触面参考轨迹；红点处足球与保守占据柱相交 |
| 蓝色实线 | 地形安全走廊 + MINVO QP 优化后的接触面参考轨迹 |
| 橙色线框 | 几何走廊下边界 |
| 灰色细线框 | 几何走廊上边界和侧边界 |
| 绿色线框 | 预留跟踪误差后的规划下边界 |
| 紫色线框 | 规划上边界 |
| 青色线框 | 规划侧边界；二维 XZ 图中为纵向截面边界 |

旧节点的红/绿/蓝框只是不同时间单元的轮换配色，灰框是几何走廊，旧红色实线为优化轨迹。不要与新版语义配色混用。

### 参考轨迹与场景

复用生产 `createSwingHeightBSpline()` 和在线相同的 XY 五次 smoothstep 控制点 `[0,0,0,0.5,1,1,1]`，起落点位置/速度/加速度保持不变。参数默认从安装目录 `cyberdog_description/config/ocs2/{task,gait}.info` 读取，支持 `--task FILE --gait FILE` 或 launch 的 `task:=FILE gait:=FILE`。读取 trot 第一相时长（当前 0.4 s）、摆高 0.075 m、起落速度 +0.10/−0.03 m/s、足半径 0.019 m，以及当前 QP/走廊参数。

展示的是**地形自适应摆高和 QP 修正之前**的名义曲线，便于验证避障修正。不是把已完成地形修正的在线轨迹再人为压低。

| 场景 | 起点 x → 落点 x (m)，y=0.10 m | 地形 |
|---|---|---|
| flat | 0.100 → 0.340 | 平地，无碰撞基线 |
| bar | 0.100 → 0.400 | x=0.200–0.230 m，10 cm 横障 |
| pillars | 0.105 → 0.205 → 0.385 → 0.565 → 0.745 | 与 minvo_test 相同的四个 6 cm 柱台，显示全部四次摆动 |
| stairs | 0.085 → 0.225 → 0.375 → 0.525 | 与 minvo_test 相同的 10/20/30 cm 台阶，显示全部三次摆动 |

柱台起落点与 `minvo_test` 一致；楼梯起落点相对其配置向踏面内侧移动 1–2 cm，给后一级障碍和当前 2 cm 跟踪管留空间。机器人参考参数仍使用当前 task/gait，未复制参考图的 0.3 s / 8.5 cm 参数。Bar 测试单独使用 `verticalExtent=0.14 m`、`maxCorrection=0.20 m`，为 10 cm 横障提供足够抬升搜索空间；这些是离线场景设置，不修改在线控制器配置。足半径 0.019 m、跟踪裕量 0.02 m 保持不变，实际值写入每段 metrics。

显示转换统一为 `p_display=p_c-r*e_z`，半空间同步为 `b_display=b-a_z*r`，地形保持原高程；因此显示上贴地，同时保持曲线与走廊约束关系等价。原始 CSV 仍保留球心坐标，避免下游重复转换。

起落高度由地图查询。地图分辨率 5 mm，碰撞检查使用四角最大值构成的保守插值 patch 上界。三个障碍场景的首段必须满足名义足球负净空和非零 QP 修正（后续段不强制发生碰撞）；优化曲线必须通过完整 MINVO 走廊矩阵复核、非接触区间稠密检查、全摆动 2001 点足球净空检查及端点 p/v/a 检查，否则节点失败，不发布成功轨迹。`output_certificate` 仍独立记录全摆动连续碰撞检查状态（0=safe、1=collision、2=unknown），稠密检查不替代全摆动连续证明。

### 新数据与逐地形二维图

**2026-11-18 已构建并通过四场景检查、colcon test 和 ROS MarkerArray 订阅核验。** 新数据在 `data/swing_sequences`，新版四张图及 `corridor_constraint.*` 已生成；`data/final` 的旧几何数据保留用于历史对照。

```bash
CORRIDOR_TEST=src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test
ROBOT_CONFIG=src/quadruped_ros2_control/descriptions/cyberdog_description/config/ocs2
# 以下运行命令需要已构建的新版测试节点；显式使用当前源码配置。
ros2 run terrain_safety_corridor_test terrain_corridor_test --scene all \
  --task "$ROBOT_CONFIG/task.info" --gait "$ROBOT_CONFIG/gait.info" \
  --output "$CORRIDOR_TEST/data/swing_sequences"
ros2 launch terrain_safety_corridor_test scene.launch.py scene:=stairs \
  task:="$ROBOT_CONFIG/task.info" gait:="$ROBOT_CONFIG/gait.info"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/terrain_matplotlib /usr/bin/python3 \
  "$CORRIDOR_TEST/plot_corridor_sections.py" "$CORRIDOR_TEST/data/swing_sequences" \
  --output "$CORRIDOR_TEST/figures"
```

生成 `corridor_constraint_{flat,bar,pillars,stairs}.{png,pdf,svg}`，每个地形一张图，含 XZ 截面和摆动相高度两个面板，图例标明全部边界。横障图同时复制为兼容入口 `corridor_constraint.{png,pdf,svg}`。PNG 为 600 dpi，PDF/SVG 保留可编辑文字。`plot_corridors.py DATA` 为同一绘图器的兼容入口；`plot_evidence.py` 也调用同一绘图函数。

`<scene>_swings.csv` 给出全部摆动的 stem 和顺序；每段单独导出数据，绘图器合并为一张场景图。`*_planes.csv` 导出 `b_geom/b_plan/b_qp`；`*_vertices.csv` 为半空间交集顶点；`*_trajectory.csv` 新增物理时间和原始/优化足球净空；`*_metrics.csv` 记录 schema=2、实际参数、净空及走廊残差。绘图器拒绝缺少新版指标或验证失败的数据，避免旧图被当成新实验结果。走廊证书范围由导出时间单元决定，当前机器人配置是 `[0.25,0.75]`，接触过渡没有常量误差球保证。

无需构建的绘图检查：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/terrain_matplotlib /usr/bin/python3 -m unittest \
  discover -s src/terrain_safety_corridor_test/test -v
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

解析几何回归使用 `contactPhase=0.10`；新版四场景管线读取在线配置，目前为 `0.25`（中间 50%）。后者来自实际仿真中早期 2 cm 恒定误差球造成过大修正的诊断，不能当作完整摆动安全证明。若要覆盖整个非接触时域，需要经执行层验证的时变误差集合及与触地模式相容的收缩方式；本实现没有用渐消裕量冒充常量误差球。

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
  "$CORRIDOR_TEST/plot_evidence.py" "$CORRIDOR_TEST/data/final" --corridor-data "$CORRIDOR_TEST/data/swing_sequences" --output "$CORRIDOR_TEST/figures"
```

`data/final` 保存几何导出、MINVO 基准、全部试验指标和最终六轮汇总；`data/stability` 保存原始运行数据。五组图的源数据、图注和结论范围见 [figures/README.md](figures/README.md)。

2026-11-18 截图显示更新：RViz 全部走廊线框统一为 1.5 mm（与紫色上框一致），包括绿色下框、侧框及几何外框；绿色保持高饱和、不透明，侧框/几何框保留较低透明度，关闭地形栅格线。已删除所有 TEXT_VIEW_FACING 文字，边界按 planning_lower/upper/sides 与 geometric_lower/upper/sides 分命名空间，可在 RViz 分别开关。二维图保留图例并同步绿色。重新启动 scene.launch.py 才会加载新版 RViz 配置；已经打开的旧 RViz 需要重新加载配置。
