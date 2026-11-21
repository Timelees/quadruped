# 固定触地时刻实验包

本包调用控制器的生产 C++ 样条、在线重规划、MINVO 走廊与激活实例化实现，并驱动原 `stair10cm` Gazebo 世界。算法机制试验与整机试验独立记录。结论、逐图用途和未验收项见 [验收报告](VALIDATION.md)。

## 构建与环境

从工作区根目录执行；仅在需要构建时运行下面的构建命令。构建与 Gazebo 仿真互斥，不修改用户的默认 task/gait/world 文件。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  nice -n 10 colcon build --executor sequential --parallel-workers 1 \
  --packages-select ocs2_quadruped_controller fixed_touchdown_experience \
  --allow-overriding ocs2_quadruped_controller
source install/setup.bash
```

`src/fixed_touchdown_experience` 是本包的 colcon 发现软链接。保留原工作区源代码目录；Gazebo 驱动复用同级 `terrainSafetyCorridor_test` 的采集、真值及生命周期脚本。ROS2/Gazebo 依赖沿用 QUICK_START，Python 使用 NumPy、Matplotlib；不依赖 SciPy。绘图对齐检查依赖已安装的 nature-figure skill；其他位置可设置 `NATURE_FIGURE_SKILL` 为其根目录。PDF 碰撞审查另需 PyMuPDF。普通 ROS2 本机终端不需要 Codex 的沙箱权限选项。

## 生产算法机制试验

```bash
EXP=src/quadruped_ros2_control/test/src/fixed_touchdown_experience
ros2 run fixed_touchdown_experience mechanism_experiment "$EXP/data/my_mechanism"
ros2 run fixed_touchdown_experience verify_data.py "$EXP/data/my_mechanism"
```

输出目录不得覆盖已有 `mechanism_scan.csv`。112 个条件为：障碍顶高 10/11/12/13 cm × 显露时刻 20/40/60/80/100/120/140 ms × 交付延迟 0/5/15/30 ms。固定摆动区间 `[0,0.4] s`，世界 X 从 −0.98 至 −1.32 m，障碍中心 −1.2 m、宽 0.03 m、长 1 m，足端球半径 19 mm。

本试验明确提供旧参考 p/v/a 作为受控激活边界，**不是状态估计或机器人执行试验**。局部配置 `correctionDuration=.16, minRemainingTime=.10, maxTrajectoryCorrection=.12, maxMarginFraction=.8, verticalExtent=.15`；其余采用 C++ 默认值。它不等同于机器人 task.info 的整套参数。默认机器人修正时长为 .12 s。

CSV 保留全部接受和拒绝；JSONL 保留每条接受轨迹的完整节点/控制点；trace 保存第一条接受轨迹的原生 C++ p/v/a；独立 NumPy 计算逐项交叉校验。C++ 程序还检查无效 IK、不满足工作空间的初值/终点、缺少分配加速度、名义安全但不确定族越界、完整/剩余非接触走廊一致性，以及目标无穿透与邻近同高障碍约束不因合并顺序而放松。

## Gazebo 单障碍试验

```bash
# 每次使用新前缀；脚本拒绝覆盖已有结果。
ros2 run fixed_touchdown_experience run_experiment.py \
  --output "$EXP/data/my_hocbf" --domain 181 --height .10 --speed .4

# 受控显露：冻结世界真值地图，在某条正在摆动的腿预测跨障时释放。
ros2 run fixed_touchdown_experience run_experiment.py \
  --output "$EXP/data/my_reveal" --domain 182 --height .10 --speed .4 \
  --reveal --truth-map --reveal-phase .025 --reveal-x -.60 --vertical-extent .15

# 用户允许的诊断消融；单独保存，不与 HOCBF 开启轮次混为一组。
ros2 run fixed_touchdown_experience run_experiment.py \
  --output "$EXP/data/my_reveal_off" --domain 183 --height .10 --speed .4 \
  --reveal --truth-map --reveal-phase 0 --reveal-x -.60 --vertical-extent .15 --safety off
```

默认 `--safety hocbf` 开启几何 HOCBF，关闭误差管；`full` 同时开启误差管。`--transport-delay .015` 注入真实 MRT 包等待，区别于仅修改预测时域。默认 seed=42；异步调度不保证逐位相同结果。

路线：原点站立→trot→转 180°→仅前进→远侧保持；前进阶段不输入偏航、俯仰或侧向修正。只测试原单障碍，不转到其他测试通道。世界写入隔离副本：`height` 表示障碍**真实顶面**，箱体中心设为 height/2；原文件箱体中心 .03、厚 .10 时真实顶面只有 .08，不能将其误报为 10 cm。未改动原 SDF。

受控显露释放前把障碍区域当平地，是人为打破地图保守性的压力干预，不能用于证明感知准确性或释放前安全。单障碍可由任一腿触发；记录 leg、观测摆动相位、原计划起止、地图发布和控制器提交时刻。低深度订阅优先及时触发，可能丢采样；分析使用真实时间戳和最大插值间隔，不填补长间断。

每轮保存 `.json/.csv`、`_wbc.csv`、`_gz_poses.csv`、`_reference.csv`、`.launch.log`、`.replans.json`、`.reveal.json`、`.protocol.json`、`.manifest.json`、`.world.sdf` 和隔离参数 profile。manifest 包含源文件、二进制、参数、世界 SHA256，跨版本不能合并成功率。`accepted=1` 的单腿准入日志不是整包激活，必须同时检查 `SWING_EVIDENCE` 和 `Swing replan applied`。

## 分析、绘图与回归

```bash
ros2 run fixed_touchdown_experience summarize_data.py "$EXP/data" \
  --output "$EXP/data/analysis/my_run_audit.json"
MPLCONFIGDIR=/tmp/fixed_touchdown_mpl ros2 run fixed_touchdown_experience plot_figures.py \
  --mechanism "$EXP/data/my_mechanism" --trials "$EXP/data/my_hocbf.json" \
  --output "$EXP/figure/my_figures"

build/ocs2_quadruped_controller/swing_replan_selfcheck
build/ocs2_quadruped_controller/execution_contract_selfcheck
build/ocs2_quadruped_controller/terrain_safety_corridor_selfcheck
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/test
```

图输出 PDF、可编辑 SVG、600 dpi PNG 和逐图图注。派生数据在 `data/analysis/<figure目录名>/`，包括 p/v/a 的全部 XYZ 数据。没有真实碰撞修复激活时，图 1/3/4/5 自动使用明确标识的离线生产算法试验；图 2/6 仍来自实际 Gazebo 数据，绝不拼出成功整机案例。全部 trial 用于图 2；若无修复激活，图 6 用第一个 trial 中非接触参考最接近障碍中心的一次完整摆动，无论它是否安全。

`data/good_data/algorithm_only` 与 `figure/good_data/algorithm_only` 仅备份合格的算法层证据，资格和来源见各目录说明。整机成功避障且同次摆动真实换轨未验收前，不放入整机 good_data。

## 五项补充实验（2026-11-21）

见 [实验结果、证据边界与复现命令](EXTENDED_EXPERIMENTS.md)。包含 6400 条局部生产算法扫描、Gazebo 原始 WBC 约束、五组 PDF/SVG/PNG；完整物理成功率仍未验证。入口 `ros2 run fixed_touchdown_experience run_extended.py --data 新目录 --figure 新目录`。
