# MINVO 规划层：实现、开关与复现

> MINVO 测试现已迁入[独立 minvo_test 包](../../../test/src/minvo_test/README.md)，新的配置、源码路径和启动命令见该 README。以下保留本轮次的历史实验记录。


实现基于[理论方案](四足机器人_MINVO_HOCBF_纯理论与通用实现方案_Typora.md) §3.3、§4 与 §8.1。保护对象是静态指定高程模型中的参考足端球体；固定 XY、摆时和落脚点，仅优化高度。真实跟踪、整腿碰撞、动态障碍和 HOCBF 不属于本次实现范围。

## 结构与复用

沿用现有 `PlanarTerrainReceiver` 的求解前地图提交、`BSpline`、`SwingTrajectoryPlanner`、qpOASES、IK 高度截面和三维 MPC 软跟踪。新增以下模块，均直接供优化器与测试调用，没有工厂或新依赖：

| 模块 | 职责 |
|---|---|
| `PolynomialHull` | 用节点插入提取非零五次样条段；De Casteljau 精确子段；同一提取结果转换为 Bézier/MINVO |
| `TerrainGeometry` / `TerrainInterface` | 高度查询与完整覆盖的占据柱接口；GridMap 双线性块用四角最大值作保守柱顶；未知和越界不可认证 |
| `SwingCollisionCertifier` | §3.3 的凸包加球体与障碍柱分离、递归细分、三态结果 |
| `SwingTrajectoryQpOptimizer` | 固定 XY 的离散/Bézier/MINVO 净空 QP；保留首末位置、速度、加速度；独立输出几何复验 |

选择小模块而不是完整三维通用走廊框架：三维路径搜索、走廊图、后台重规划和新控制节点会扩大范围；当前固定 XY 下，已有高度上界可以直接形成凸半空间，复用小 QP 足以实现对照实验。

## 参考碰撞认证（§3.3）

1. 对原始五次样条的每个非零节点区间做节点插入，不通过采样拟合多项式。
2. 计算 Bézier/MINVO 凸包。高度样条存储 `z_center - footRadius`，认证时转换回球心；不重复膨胀半径。
3. 查询凸包水平范围加半径所覆盖的所有地形柱。GridMap 模型是已知四节点双线性插值，四角最大值不低估此模型；不声称不低估被传感器漏掉的真实障碍。
4. 使用凸包面法向、柱面法向和边叉积轴做分离投影。球体在单位轴上的支撑是 `radius`；占据柱向下无限延伸。任一有效轴分离即可排除该柱。没有找到分离轴只代表尚不能证明，不判碰撞。
5. 无法排除时做 De Casteljau 二分；达到深度/总段预算，或地图未知，返回 `Unknown`。
6. 只有找到实际曲线点穿入已知高度面，才返回 `CollisionDetected`。该拒绝见证不承担安全认证，采样通过不会返回安全。球体侧面风险也可能保持 `Unknown`，不会漏接收为安全。

`ProvenSafe` 要求全部区间与所有覆盖柱通过分离。端点接触允许配置的米制数值误差 `numericalTolerance=1e-9`；它不是跟踪或地图误差。MINVO 常数矩阵为已有 MIT ACL 五次矩阵，使用 [-1,1] 的降幂约定；测试覆盖基和、端点/驻点非负性及重构误差。这是浮点模型下的验证，没有实现机器可核验的精确有理/区间算术证明。

首尾接触子段使用 Bézier，避免 MINVO 外包络在恰好接触点制造不必要的负凸包高度。它是两种连续凸包约束的组合，不再用近似零根除法。

## 优化约束与接触

优化器使用同一精确段映射构造线性高度半空间；球体扫掠上界包括完整地形柱支持域。优化约束比 §3.3 的多方向分离检查保守：首版只允许向上越障，不能侧绕。

由于 XY 固定，Bézier/MINVO 两种高度模式共用每个子段的 Bézier XY 外包络查询走廊，只有高度约束的基不同。这样既保持保守性，也避免把不同地形查询范围误认为 MINVO 基的收益。`correctionCost` 记录完整二次修正目标，便于配对实验。

净空间隙为 `groundClearance * smoothstep(2*min(s,1-s))`，其中 `smoothstep(u)=10u³-15u⁴+6u⁵`。在 0.5 显式分段，完整包络仍是分段五次，可连同高度曲线一起作同基系数限制。两端间隙及一、二阶导数为零，与站立转摆动的零速度/加速度条件兼容。没有对每段只取间隙端点最小值。

接触点维持名义端点等式。删除了依据接触偏差全程下调地形的做法。若起终点与当前障碍模型不一致，返回未获得有效计划；不再伪造更低障碍。首版适合明确的平坦接触面；栅格保守上界在坡面或台阶边可能拒绝真实可接触点，应后续加入经过验证的局部接触模型，不能通过全程负偏移掩盖。

IK 仍是原数值截面及截面间线性插值，属于采样工作空间近似。`certificate.status` 仅认证参考球体几何，不能解释为连续关节可达、关节速度/力矩可行或实足安全。

## 配置与对照

Cyberdog `task.info` 的新字段取代旧 `useCbfUpdate`、`cbf*` 字段；旧名字不再读取。其他未设置新总开关的机器人配置默认走原名义路径。

| 字段 | 默认/当前值 | 行为 |
|---|---|---|
| `useTerrainPlanning` | Config 默认 false；Cyberdog true | false 完全绕开地形 QP，保持原名义规划与 Z 跟踪接口 |
| `terrainGeometryMode` | 2 | 0 离散采样；1 Bézier 高度半空间；2 MINVO 高度半空间 |
| `terrainSubdivisionsPerSpan` | 4 | 优化器每个原始段的细分数，向上取 2 的整数幂；0.5 包络切换处单独分段 |
| `checkSwingReference` | true | 对实际用于规划的三维名义参考运行 §3.3 检查；结果保存在 `referenceCertificate` |
| `verifySwingOutput` | true | 对输出再次独立做 §3.3 检查，只有 ProvenSafe 才接受 |
| `collisionHullBasis` | 1 | 0 Bézier；1 MINVO，控制独立认证器的凸包基础 |
| `collisionMaxDepth` | 10 | 每个原始段最多二分深度 |
| `collisionMaxSegments` | 4096 | 每次认证最多访问的段数；耗尽返回 Unknown |
| `terrainNumCurveSamples` | 60 | 目标与离散对照样点数；不是连续证书的判据 |
| `terrainGroundClearance` | 0.03 m | 分段五次间隙峰值 |
| `terrainElevationLayer` | elevation | 几何高程来源；不把 smooth_planar 自动当安全上界 |

要观察离散 QP 漏检，离线实验设 `terrainGeometryMode=0` 且 `verifySwingOutput=false`。此时 `valid()` 表示优化成功，证书仍为 Unknown，绝不等同于认证安全。保持输出复验开启时，离散 QP 可能被独立检查拒绝，这是预期结果。

参考检查可以单独调用 `SwingCollisionCertifier::check()`，不要求启用优化。模式枚举、段映射和地形接口均公开，便于外部比较；不必修改求解器实现。

## CBF 移除范围

摆动规划的 CBF 导数行、增益/窗口、共享松弛变量和旧 CBF 配置已移除。规划入口重命名为 `updateTerrain()` / `isTerrainPlanningEnabled()`。未启用的落足 CBF 工厂及其控制器构建入口也已移除，Cyberdog 中落足 CBF/WBC-CBF 配置已删除。

普通落脚几何代价、支撑约束、WBC、地图同步、估计器保持现有实现。历史独立 CBF 源文件/演示与旧评审文档作为历史材料保留，不链接到当前控制器；本次没有实现任何新的 CBF/HOCBF。

## 复现命令

下面使用已有工作区依赖，在 `/tmp` 中构建和安装本轮控制器，避免覆盖原安装库。需要已构建原有依赖。

```bash
cd /home/lee/Code/quadruped_control_ws/src
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /home/lee/Code/quadruped_control_ws/build/ocs2_quadruped_controller/colcon_command_prefix_build.sh
cmake -S quadruped_ros2_control/controllers/ocs2_quadruped_controller -B /tmp/minvo-cmake -DCMAKE_INSTALL_PREFIX=/tmp/minvo-prefix -DBUILD_TESTING=OFF
cmake --build /tmp/minvo-cmake -j2
cmake --install /tmp/minvo-cmake
cmake -S quadruped_ros2_control/test -B /tmp/minvo-tests -Docs2_quadruped_controller_DIR=/tmp/minvo-prefix/share/ocs2_quadruped_controller/cmake -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/minvo-tests --target minvo_planning_test swing_qp_regression -j2
ctest --test-dir /tmp/minvo-tests -R '^(minvo_planning_test|swing_qp_regression)$' --output-on-failure
/tmp/minvo-tests/minvo_planning_test quadruped_ros2_control/controllers/ocs2_quadruped_controller/docs/minvo_results_2026-11-11
python3 quadruped_ros2_control/test/src/plot_minvo_planning.py quadruped_ros2_control/controllers/ocs2_quadruped_controller/docs/minvo_results_2026-11-11
```

数据由生产 C++ 模块生成；Python 仅读取 CSV 绘图，不重新实现优化器。结果与分析见[实验报告](minvo_planning_results_2026-11-11.md)。当前活动摆动仍沿用原缓存策略，没有新增地图版本合同或剩余段重规划；几何证书仅适用于检查时的指定地图，不能外推到更新后的环境。

规划失败沿用现有异常上报路径，不退回未经认证的名义轨迹；本次没有新增安全停车或恢复控制器。部署前仍需闭环验证调用层如何响应失败。
