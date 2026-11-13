# minvo_test

论文使用说明：[第5章：数据、图片含义与章节对应](docs/第5章_数据图片说明与章节对应.md)，包含CSV字段解释、图题建议、章节映射和结论边界。

独立 ROS 2 / ament_cmake 实验包，直接链接生产控制器中的 `BSpline`、`PolynomialHull`、`GridMapTerrainInterface`、`SwingTrajectoryQpOptimizer`、`SwingCollisionCertifier`。不复制样条或优化算法，不修改在线控制路径。

## 立即打开三维场景

先按下文“构建和测试”完成工作区构建。每个新终端加载工作区环境后启动：

```bash
cd /home/lee/Code/quadruped_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src
ROS_DOMAIN_ID=68 ROS_LOG_DIR=/tmp/minvo-scene-ros \
  ros2 launch minvo_test scene.launch.py scene:=pillars
```

`scene` 可选 `single`（单障碍）、`pillars`（四柱跨越摆动）、`stairs`（逐级上楼）。Ctrl-C 后更换参数重启；启动文件同时启动场景发布器与 RViz，无需另开终端。`rviz:=false` 可仅发布数据。专用话题为 `/minvo_scene/elevation_map`、`/minvo_scene/trajectories`，不会替换机器人地图。

- 工具栏选择 **Move Camera**；左键拖动旋转，中键拖动平移，滚轮缩放，Shift 配合鼠标可改变移动方向。
- 左下 **Views** 面板提供 `Top`、`Side XZ`、`Oblique` 预设，双击预设切换；也可选择 Orbit/XYOrbit 视图控制器。
- 参考为红色虚线；Bézier 为橙色；MINVO 为绿色；仅物理几何、不加跨越带的 MINVO 对照为蓝色。轨迹接近时会重叠，可在 Source Trajectories 的 Namespaces 中按方法开关显示。
- 黑球标记起落点，轨迹绘制的是足端接触点，端点直接贴合平台面（碰撞模型仍使用半径 19 mm 的足端球）。三维表面由高程数据构成，可自由绕场景查看。

RViz 配置包含 `Tools` 列表和视角预设，可使用上述工具交互查看场景。

## 目录

```text
minvo_test/
  config/   single.yaml、pillars.yaml、stairs.yaml、scene.rviz
  src/      场景节点、生产模块回归和配对基准测试
  launch/   scene.launch.py
  data/     执行后导出的 CSV、配置快照；benchmark / legacy_scenes 保留旧实验
  result/   绘图脚本生成的 PNG/PDF；旧结果也按子目录保留
  script/   plot_scenes.py、plot_minvo_planning.py
```

## 配置起落点与地形

直接修改 `config/*.yaml`，重新启动即可。launch 从工作区 `src` 目录运行时，优先读取源码中的配置，修改后不需要重新编译。其他目录启动时使用安装配置；可用 `config:=/绝对路径/scene.yaml` 指定源码文件。

```yaml
obstacles:
  # [xmin, xmax, ymin, ymax, height]，世界坐标，单位 m
  - [0.145, 0.175, -0.25, 0.25, 0.06]
swings:
  # start/goal 的 z 自动吸附到当地高程；第三项保留为 0。
  - start: [0.116, 0.10, 0.0]
    goal: [0.330, 0.10, 0.0]
    duration: 0.3
reference:
  swing_height: 0.085
  swing_time_scale: 0.20
  lift_off_velocity: 0.04
  touch_down_velocity: -0.04
```

参考轨迹直接复用在线控制器的 `createSwingHeightBSpline()`，采用五次、7 控制点 B 样条。中间时刻真实曲线高度满足 `z(T/2) = max(z_start,z_goal) + min(1,T/swing_time_scale)*swing_height`；该高度不是中间控制点高度。离地/触地速度也按同一比例缩放，端点加速度为零。默认值对齐 Cyberdog 的 task.info（85 mm、0.20 s、+0.04/-0.04 m/s）；地形基准取起落平台中较高者。测试只为高度样条嵌入起落点之间的 XY，优化器继续使用生产固定 XY 平滑进度。

已移除旧 `height_controls` 及 B→C 人为负高度。新参考可能本身就不碰撞，这是正常结果，不再人为压低参考以制造优化收益。

`map` 配置范围、中心和分辨率；`obstacles` 中重叠方柱取最大高程。楼梯由相邻不同高度的方柱组成。起落点须位于已知地图内，接触面应足够容纳足端球体。若与障碍或接触条件不兼容，节点报告失败并保留指标，不发布未经认证的回退轨迹。

`optimizer` 可调控制点数、目标采样数、细分数与净空峰值；这些值传给生产优化器。未加入 IK 或动力学约束，本包的认证对象是指定静态地图中的参考足端球体。

## 对齐参考资料的场景和图表

已阅读 `/home/lee/MINVO论文图参考/README.md` 和四张示意图。按该文档重做场景，而非把示意数值当实验结果：

- `single`：单个窄长方体，移动起落点使障碍靠近离地端，验证提前抬脚避障。
- `pillars`：四柱 A/B/C/D，均高 60 mm；地面→A→B→C→D 四次摆动，每次落点位于对应柱顶内，距后缘 25 mm。
- `stairs`：地面→第一级、第一级→第二级、第二级→第三级三次摆动，每次落在相邻踏面。

`pillars.yaml` 的 `crossing_bands` 是显式跨越高度策略：`[xmin,xmax,ymin,ymax,required_bottom_height]`。测试层使用虚拟占据柱把该高度策略交给已有连续凸包约束接口；物理地图与发布的高程图保持不变，并再次独立认证物理输出。只有几何避障不能推出“不下探狭缝”；蓝色 MINVO_geometry 组省略该策略，橙/绿两组采用相同策略以公平比较基。

当前默认场景中，蓝色几何组本身也没有违反柱顶跨越高度；策略组不代表在本例中取得了显著收益。指标记录 `gap_min_bottom` 和 `crossing_violation`，后者相对于配置跨越带高度。跨越约束的连续接受来自虚拟模型的生产认证，指标采样仅用于分析。

## 当前场景尺度与接触

以 Cyberdog 当前 `gait.info` 的 trot 单摆动 0.3 s 为时间尺度，保持足端半径 19 mm。`reference.info` 目标前进速度为 0.35 m/s；同一足端周期约 0.6 s，对应前进约 0.21 m，因此柱顶间采用 0.18 m 的前进步长作为测试尺度。

| 场景 | 几何与起落点 |
|---|---|
| 单障碍 | 障碍高 60 mm、沿前进方向宽 30 mm；从 x=0.116 m 地面起步，落在 x=0.330 m 地面，步长 0.214 m |
| 四柱 | 柱高 60 mm、沿前进方向宽 100 mm、相邻净间隙 80 mm；起落 x=0.105→0.205→0.385→0.565→0.745 m；首段步长 0.10 m，随后逐柱步长 0.18 m |
| 楼梯 | 级高 100 mm、踏面深 150 mm；起落点 x=0.095→0.245→0.395→0.545 m，分别位于地面和三个踏面内部 |

所有场景轨迹位于 y=0.10 m，代表同一侧的一只脚；支撑面沿 y 方向宽 0.50 m，可覆盖左右足端通道。起落点 z 自动吸附到地图中的接触平面实际高度。当前柱顶接触点距最近棱边 25 mm，楼梯接触点距下一台阶立面 25 mm，均通过生产接口的足端球体接触检查。节点会检查每个接触点的球体水平覆盖范围，若伸入更高邻近区域导致穿透则拒绝配置。

这是单足摆动几何实验，步长和障碍尺寸参考机器人配置，不等价于四足全身步态已可执行：没有确定机身运动、其他三足支撑时序、关节/力矩约束。

## 配色

已按要求改为**深蓝→蓝→青→浅绿→黄**，去掉原先 Rainbow 的品红色起点。三个场景共用 0–0.30 m 色标，因此相同高度跨场景颜色一致；最高 300 mm 台阶呈黄色。

配色由 YAML 的 `visualization.height_range` 和 `visualization.colors` 指定，颜色值为 RGB 0–255。节点调用已有 `grid_map::colorVectorToValue()` 生成标准 `color` 图层；RViz 采用 `Color Transformer: ColorLayer`、`Color Layer: color`，高度仍为 `elevation`。这样可以直接使用自定义多段渐变，不依赖插件内置 Rainbow。绘图读取执行时导出的配置快照并使用同样色标。

## 数据与绘图

默认从工作区 src 启动时，CSV 位于 `data/<scene>/`；`data_dir:=/绝对路径` 可更改。每次执行重写对应场景目录内同名数据，不影响其他场景：

- `map.csv`：实际 float 高程栅格。
- `trajectories.csv`：参考和优化的足端接触点位置，以及生产样条解析速度、加速度、地形与所需足端净空高度（包含球体棱边碰撞）。
- `hulls.csv`：每条曲线同一原始多项式段的 Bézier/MINVO 顶点。
- `spline_controls.csv`、`knots.csv`：生产样条原始数据，可重建每条轨迹。控制点、轨迹和包络 z 均使用同一足端接触点坐标。
- `metrics.csv`：成功标志、物理认证、密集测量净空、参考穿透、端点误差、目标值、规划耗时及狭缝指标；`certificate=0` 表示 ProvenSafe。新增 `reference_certificate`：0=ProvenSafe、1=CollisionDetected、2=Unknown；Unknown 不代表已证明安全或碰撞。`reference_clearance` 已与优化输出统一使用足端球体对保守栅格棱柱的竖直净空。
- `config.yaml`：本次输入快照。

不打开 RViz，批量生成并绘图：

```bash
cd /home/lee/Code/quadruped_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src
MINVO_PKG=quadruped_ros2_control/test/src/minvo_test
for scene in single pillars stairs; do
  ros2 run minvo_test minvo_scene "$MINVO_PKG/config/$scene.yaml" "$MINVO_PKG/data/$scene" --export-only
done
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/minvo-matplotlib /usr/bin/python3 \
  "$MINVO_PKG/script/plot_scenes.py"
```

本机系统 Matplotlib 的 3D 扩展与用户级 Matplotlib 版本不同，上述命令使用现成系统 NumPy/Matplotlib；脚本用标准库读 CSV，没有 Python 样条实现。`--data`、`--result` 可改输入/输出路径。

`result/<scene>_overview.png/svg` 按参考资料排列三维高程、XZ 碰撞侧视、XY 俯视；`*_swings_hulls.png/svg` 左侧各自轨迹及包络，右侧是同一 MINVO 曲线、同一分段的两类包络和控制点；`*_collision_detail.png/svg` 是碰撞局部图；`*_kinematics.png/svg` 给出一段 z/v/a。图片使用中文标注，优先选择宋体；本机没有微软宋体，使用思源宋体。可用 `--font /绝对路径/simsun.ttc` 指定宋体；SVG 保留文字而非转为路径。包络面积指标输出到 `data/hull_metrics.csv`。绘图中的包络是原始多项式段，QP 使用细分段，独立认证还可自适应细分，不能把粗段凸包相交直接解释为实际碰撞。

所有结果来自合成高程输入和生产模块，不声称是点云融合、Gazebo 或实机日志。MINVO 的紧致性必须按同曲线同分段比较，本包不强行保证每条曲线的 MINVO 投影面积更小。

## 构建和测试

在工作区根目录构建本包及其工作区依赖，使用标准 `build/`、`install/`、`log/` 目录：

```bash
cd /home/lee/Code/quadruped_control_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to minvo_test --symlink-install
source install/setup.bash
```

包源码保留在 `src/quadruped_ros2_control/test/src/minvo_test`。工作区提供符号链接 `src/minvo_test -> quadruped_ros2_control/test/src/minvo_test`，使 colcon 默认扫描能够发现嵌套在外层 `test` 包中的本包；请保留该链接，无需额外指定 `--base-paths`。不再依赖 `/tmp/minvo-prefix` 等临时安装。

构建后运行两项模块回归及三个场景导出测试：

```bash
cd /home/lee/Code/quadruped_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select minvo_test --event-handlers console_direct+
colcon test-result --test-result-base build/minvo_test --verbose
```

测试导出位于 `build/minvo_test/data/<scene>/`，不覆盖源码中的实验数据。要重新生成源码下的数据和图片，使用上文“数据与绘图”命令。旧 MINVO 测试已经迁入本包并从外层 CMake 移除；旧报告中的源文件位置和启动命令以本 README 为准。

本次验证结果以 `data/validation.csv`、`data/<scene>/smoothness.csv` 为准。当前共 20 条优化轨迹，单障碍和楼梯保留参考碰撞案例；逐柱场景四个落点正确接触柱顶；第一段起点接近 A 的前缘，参考发生足端球体棱边碰撞，优化提前抬脚，后面三段保持正常逐柱摆动。未重新启动 RViz GUI。

## 连续性与速度、加速度

`minvo_scene` 调用生产 B 样条接口，导出全部三维速度/加速度范数的密集采样峰值，以及内部节点两侧（间距 2*T*1e-9）的解析位置、速度、加速度差。五次 B 样条的内部单节点保证 C4 连续，QP 只修改控制点，故仍满足 C2；节点重数及两侧差值由程序检查，失败时停止导出。

每次摆动端点保持控制器的 +0.04/-0.04 m/s 垂直速度和零加速度。因此结论针对单段摆动内部，不声称与速度为零的支撑相或直接拼接另一段摆动时速度连续。真实接触切换沿用控制器定义，不能把四条图形曲线拼接成一条连续执行轨迹。

连续不等于峰值小，也不等于满足硬件限值。楼梯 0.3 s、起点距立面 25 mm 的测试会产生较大加速度；当前没有机器人给定的速度/加速度上限，也没有动力学可执行性认证。保留已有优化器；没有把未经通过的限幅实验合入生产代码。峰值均为密集采样测量，不是连续最大值证书。

## 绘图与复查

楼梯级高取 `gazebo.launch.py` 默认 `stair10cm.sdf` 的 0.10 m，踏面深 0.15 m。柱台和单障碍均高 0.06 m。运行导出后执行 `script/check_scenes.py` 检查接触点、中点高度、连续性及碰撞认证。

`plot_scenes.py` 仅生成 PNG 与 SVG。三维图从栅格高程构建平面和立面，立面沿高度以一格分辨率细分，每条带按实际 z 配色，避免整面继承顶部颜色；不改变碰撞地图。三个场景和 RViz 仍共用 YAML 色标（0–0.30 m、蓝到黄）。SVG 包含可编辑文字、线条和多边形；旧 PDF 不再生成。

同一轨迹的 Bezier/MINVO 包络仍按同一分段比较。正常逐柱参考可能已安全，两种优化曲线接近时不应宣称 MINVO 有显著的轨迹性能优势。

本次楼梯实测：MINVO 优化轨迹三维速度峰值约 3.715 m/s、加速度峰值约 186.986 m/s²。内部节点两侧最大位置差约 2.23e-9 m、速度差约 5.82e-8 m/s、加速度差约 6.93e-6 m/s²；这是两侧相距 6e-10 s 的有限差，结合内部单节点的 C4 结构确认内部连续性，不是加速度限幅认证。

柱体场景最新端点：x=0.105→0.205→0.385→0.565→0.745 m，y=0.10 m；起落 z 自动吸附支撑平面。仅调整端点，不更改 60 mm 柱高、85 mm 抬脚高度与 0.3 s 时长。第一段参考在保守栅格模型中的最小竖直净空约 -21.80 mm；12 条优化轨迹全部认证安全。名义柱体的球体碰撞见证由 check_scenes.py 复核，数据与 PNG/SVG 已更新。

## 单障碍完整轨迹与最佳局部分段包络

`script/plot_local_envelopes.py` 直接读取生产导出的 hulls/controls/knots/trajectories 数据，不在 Python 重新实现样条或基变换。现在仅绘制 single 场景，每次只输出两张单图，各含 PNG/SVG：

- `result/single_envelopes_full`：整条 MINVO 优化曲线在 XZ 平面上的所有原始分段的 Bézier/MINVO 包络叠加，等比例坐标。分段编号从 0 开始，图中标注分段面积总和（存在重叠时不等于并集面积）。
- `result/single_envelopes_best_span`：面积比最小的原始分段放大：将 XZ 投影刚性旋转至该段曲线弦方向，以沿弦距离和法向偏差为横纵轴（mm），放大法向显示比例并在图题注明。旋转不改变面积，面积指标仍按原始 XZ 投影计算。

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/minvo-matplotlib /usr/bin/python3 \
  quadruped_ros2_control/test/src/minvo_test/script/plot_local_envelopes.py
```

支持 `--swing 0`、`--method MINVO`、`--span 3`、`--font /绝对路径/simsun.ttc`。默认固定同一条 MINVO 优化曲线比较不同基的包络，避免把不同优化轨迹之间的差异当成包络紧致性。包含关系检查仅验证导出采样，不代替生产连续碰撞认证。CSV 保留所有原始分段指标，未选择的分段也不隐藏。

此次在固定 60 mm 障碍高度、85 mm 抬脚高度、0.3 s 时长下，试跑两轮起落点组合，得到 55 组成功求解且参考碰撞的候选。完整搜索结果保存于 `data/single_endpoint_search.csv`，按所有原始分段面积总和的 MINVO/Bezier 比值排序。选择 x=0.116→0.330 m，两个端点均位于地面，保持合理步长 0.214 m。只有配置发生变化，不改优化器或 MINVO 变换。

本次第 3 段（0.09–0.12 s）Bézier/MINVO 面积约 3.780/1.746 mm²，MINVO 减少 53.8%，优于旧端点的 51.1%。但完整轨迹分段面积总和分别约 332.93/401.13 mm²，MINVO 大约 20.5%；55 组候选中未找到整条轨迹的投影面积总和优于 Bézier 的情况。这组图能展示局部优势，不能据此宣称 MINVO 对任意曲线、任意投影都更紧。图中放大不改变这一结论。
