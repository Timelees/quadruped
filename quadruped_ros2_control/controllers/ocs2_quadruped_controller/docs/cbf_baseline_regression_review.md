# 原稳定版本对照与非 CBF 行为恢复

对照提交：2026-11-10 仓库公开后，已通过 GitHub 连接器读取 `master`，确认提交为 `22e9bbb0885e829bdceccd93472b777aed173cbb`（较稳定版本），与本地 HEAD 和 origin/master 一致。另直接读取了远端 SolverBase.cpp 的 preRun 调用顺序。

## 第三轮核查：剩余差异清单（2026-11-10）

再次从公开 GitHub 读取 master，仍为 `22e9bbb0885e829bdceccd93472b777aed173cbb`。本轮只补充核查记录，没有新增控制代码或参数修改；没有发现新的、有充分源码证据的默认 Gazebo 非 CBF 控制算法回归。此结论不等于运行效果已经相同。

### 已执行的无构建检查

- WBC 源码/头文件、TargetManager、GaitManager、SafetyChecker、初始化器、SwitchedModelReferenceManager 和控制器，共 15 个文件与 master 逐字一致。
- Cyberdog 描述（除 task.info）、Gazebo 配置、地形 world、命令输入包，共 49 个受版本控制的文件与 master 字节一致。包括实际仿真加载的 gazebo.yaml、gait.info、reference.info 和模型。
- gazebo.launch.py 的 Python AST 与 master 一致，现存 diff 只是注释。
- 当前仿真使用的 `KalmanFilterEstimate::update()` 与 master 逐字一致；StateOCS2::run 中策略计算、WBC 调用和所有电机命令写入与 master 一致，差异位于之后的可视化。
- 默认 elevation_mapping.yaml 未设置 sensor_max_range，新读取参数默认 infinity，不启用距离裁剪。
- 源码确认实机旧 YAML 不含统一 default_kp/default_kd，而 StateOCS2 只读取统一参数，缺省值为 0/6。当前 cyber_controllers.yaml 显式提供 50/1.5。这一项是实机路径的实际行为差异，不能误当作 Gazebo 配置差异。

### 关闭摆动 CBF 后仍然存在的差异

| 路径 | 仍保留的差异 | 范围与处理 |
| --- | --- | --- |
| 地图/SDF | 后台构建配对快照，在感知参考生成前提交；基线在回调直接更新 SDF，稍后独立提交地图 | 更新时序仍不同，保留以避免求解中修改 SDF；运行影响仍需验收 |
| 初始 SDF | 初始零地图上即构建 SDF；基线此调用被注释 | 只在首张真实地图提交前有行为差异；保留以避免空 SDF 及其极大距离默认值 |
| 目标输入 | getDesiredInput 替代误用 getDesiredState | 保留物理量纠正；基座代价和初始化器的输入参考来自 weightCompensatingInput；可视化也会读取目标输入 |
| B 样条求导 | 自身子段赋值增加 .eval() | 共用数学代码的正确性修复，保留；名义控制点、节点、update 算法未改 |
| 预计算 | SoftConstraint 请求也刷新摆动配置 | 关闭 CBF 时仍生成原一维 Z 配置；多了一种刷新请求，不改变约束定义 |
| 起跳高度 | 原固定减 0.019 改为减 footRadius 配置 | 当前配置及默认值均为 0.019，数值等价；手动改该字段会影响非 CBF 起跳高度 |
| 可视化 | 策略可视化限制到 50 Hz；落脚可视化改在 MPC 线程调用 | 改变调度与开销；不写控制目标。保留落脚可视化线程调整，避免并发读取正在修改的选择器 |
| MPC 异常 | 异常置失败标志，FSM 切 PASSIVE | 仅异常路径改变，保留显式停止，不能用持续执行旧策略冒充正常运行 |
| 估计器配置 | 节点 sim_mode 覆盖 task.info；真值接口读取前检查长度 | 默认仿真两者均 true，估计更新相同；实机 false 模式行为与旧配置不同 |
| 建图 | 新增 sensor_max_range 参数声明和读取 | 默认仿真无限制；新增实机 elevation_mapping_cyberdog.yaml 为 2.2 m，实机会裁剪远处点 |
| 实机控制 | 统一 PD 增益、IMU 协方差接口、新实机感知启动文件和配置 | 与 master 存在实际差异；默认 Gazebo 不加载这些文件。本轮不擅自撤销已有实机适配 |
| 构建配置 | grid_map_filters_rsl 未指定构建类型时默认 Release | 性能/构建产物可能不同；不是轨迹算法差异，本轮不改构建系统 |

其余 swing QP、IK 工作空间和三维跟踪改动由 useCbfUpdate 隔离；缺少 trackingWeight 的旧配置在关闭 CBF 时不会读取它。没有为追求 diff 为零而撤销正确性修复，也未新增判断分支。

本轮只执行源码/配置/AST 比较，没有构建或运行修改后的二进制。若仍有爬楼差距，现阶段应以同一地图、速度和配置的运行数据检验上述时序/启动差异，不能从静态比较继续推断一个尚未证实的控制根因。

## 公开仓库后继续比对：地形提交顺序

上轮保留的地图/SDF 成对提交仍有执行顺序错误。远端及本地 `SolverBase::preRun()` 都先调用 `ReferenceManager::preSolverRun()`，再调用各 synchronized module。此前将地形提交保留在 synchronized module，导致当前轮落脚点、基座参考和摆动轨迹读旧地图，随后足端碰撞代价读刚提交的新 SDF。关闭摆动 CBF 并不能隔离这种跨版本地图不一致。这是源码可确定的回归，尚不能量化其对爬楼稳定性的贡献。

本轮复用现有 `SolverSynchronizedModule` 接口，在 `PerceptiveLeggedReferenceManager::preSolverRun()` 中先提交地形，再调用原参考更新；控制器只把接收器交给该感知参考管理器，不再重复注册至 solver module 列表。保留回调后台构建、求解期间快照不变的方式，不修改 OCS2 全局调用顺序，也不增加 CBF 开关分支。接收器的 postSolverRun 原本为空。

另移除新增的控制/MPC 性能日志与计时。`Ocs2QuadrupedController.cpp` 已与 master 逐字相同。落脚可视化仍保留在 MPC 线程，避免读写落脚选择器的并发问题；原有异常停机处理、样条 Eigen 别名修复继续保留。

新增 `checkTerrainCommitOrder()`，通过现有 C++ 回归程序验证连续两次 preSolverRun 均先提交当轮地图再生成参考。已执行源码顺序/注册检查、控制器逐字比较、碰撞/落脚 CBF 文件比较、MPC/WBC/估计器配置去注释比较，全部通过；本次变更文件 diff 空白检查通过。未执行构建，新增 C++ 回归与楼梯仿真均未运行，不能宣称已完全恢复稳定提交的运行效果。

## 确认的回归

1. 全局 `model_settings.positionErrorGain` 从 0 改为 20。该增益不只用于摆动轨迹；`LeggedInterface::getZeroVelocityConstraint()` 也用它构造支撑足 Z 位置反馈。此前关闭摆动 QP 仍改变支撑约束。现恢复 0，CBF 三维摆动单独使用 `swing_trajectory_config.trackingPositionErrorGain=20`。
2. `getZpositionConstraint()` 经由三维接口统一加足半径，改变了原 Z 接口语义。现恢复原样条 Z；仅三维 CBF 接口返回足端球心高度。
3. 新增站姿候选和执行中锁定投影无开关隔离，关闭摆动 QP 仍改变地形投影与落脚评分。已移除，落脚选择实现与远端基线一致（只保留日志头文件前置声明相关的编译整理）。
4. 新增 IK 硬筛选把某个投影点的数值 IK 失败等同于整块地形不可达，并检查不确定的预测支撑末端。`infinity` 代价可使所有平面被排除，然后抛出 `No kinematically reachable terrain region`，终止 MPC。这不是地形本身不可达的证明。整套落脚/平面硬筛选及回调已撤销；摆动 QP 内的关节工作空间约束仍保留。
5. 摆动避障之外，原落脚 CBF 的相位实现也被替换。尤其 `FootPlacementCBFConstraint` 原有的相位索引与新共享函数不同，替换会改变摆动期间 MPC 落脚约束的激活，即使 `useCbfUpdate=false` 也受影响。本次恢复稳定提交中的原实现，以及 ReferenceManager 和 WBC 使用的原相位、激活和松弛逻辑。删除只验证新相位算法的检查，不再把新行为的检查通过当作原控制行为兼容的证据。
6. `trackingWeight` 原先不论开关都强制读取，缺少此新增字段的旧配置无法启动。现仅在开启三维摆动跟踪时读取；关闭时也不再构造摆动 QP 优化器。足半径的默认值恢复为原感知代码使用的 0.019 m，旧配置无需增加该字段即可保留原起跳高度偏移。

## 本次继续核查的其他因素

- Gazebo 使用 cyberdog 的 `config/gazebo.yaml`，仍为 `linear_kalman`、`sim_mode=true`。估计器中的新增节点参数覆盖不会改变这份仿真配置的模式；硬件配置的修改未覆盖回去。
- 本轮发现关闭摆动 CBF 后仍经过的三项差异：订阅队列由 10 改为 1；地图中心/缓冲索引匹配检查让移动地图跳过滤波，且删除了原滤波中心保持；SDF 的高度参数从 `maxElevation + 0.3` 改为固定 `0.3`。后者在 API 内会再次加到最大高程，改变切片范围和查询截断。原 `FootCollisionConstraint` 始终读取 SDF，故摆动 CBF 总开关不能隔离它们。本轮恢复上述基线行为，不增加开关或回退分支。它们是已确认的行为差异，尚无运行证据证明其中某项就是爬楼失败的唯一原因。
- 保留地图/SDF 成对提交的并发修复，避免回调线程在一次求解途中替换 SDF。本轮进一步修正提交顺序，见上节；仍需仿真验收。
- 本次优先恢复用户已验证版本的滤波语义。原按矩阵下标融合移动地图、保留上次中心的处理本身存在世界坐标对应风险；本轮不顺带重写该算法。若后续单独修正，需针对地图移动验收，不能仅以源码“更合理”当作步态兼容证据。
- 保留 B 样条导数中 Eigen 自赋值的 `.eval()`，避免对自身子段赋值时的数据别名问题。名义轨迹控制点、节点和 update 生成逻辑未改变。
- 保留目标输入通过 `getDesiredInput()` 获取的更正。原 `getDesiredState()` 取错了物理量；当前基座跟踪代价使用 `weightCompensatingInput()` 构造输入参考，没有直接采用该目标输入作为力参考。
- 保留 MPC 异常后停止并切入 passive 的处理；撤掉错误处理只会掩盖规划线程停止。

## 本次源码核验

上一轮只修改 `PlanarTerrainReceiver.cpp`、现有 `swing_qp_regression.cpp` 和本文。沿用现有订阅回调、pending 缓冲、preSolverRun 同步模式及 assert/异常式回归检查；不新增依赖、架构或配置分支。

新增回归覆盖 +0.1 m、-0.1 m 和平地的完整摆动，比较关闭 CBF 的位置/速度与名义规划器，并用会抛异常的 workspace 和地形查询计数验证没有调用优化链路。该 C++ 用例尚未构建/执行。已执行无构建的源码函数对照检查：全部三个名义 update 重载、updateFootSchedule、Z 高度接口、五个地图接收/滤波函数、落脚相位与松弛函数均与基线相同；原 FootCollisionConstraint 和落脚 CBF 实现也相同。本次修改文件的 `git diff --check` 通过。

使用 Git HEAD `22e9bbb` 逐文件/逐函数比较通过：`FootPlacementCBFCommon.h/.cpp`、`FootPlacementCBFConstraint.cpp` 全文件相同；ReferenceManager 的原相位函数、`getFootPlacementFlags()` 和 `getFootPlacementRelaxation()` 相同；所有名义摆动 update 重载和原 Z 位置接口相同；ConvexRegionSelector 除日志 include/析构定义外相同。

按照用户最新 AGENTS.md 的“不主动执行 colcon build 等构建命令”，本次未构建、未使用旧二进制冒充修改后的仿真验证。以下历史构建和运行记录均发生在本次继续恢复之前，不能验收本次修改后的爬楼表现。

## 开关

- `useCbfUpdate=false`：不运行摆动 QP，使用原名义 B 样条和 Z 速度等式。当前默认恢复为此值。
- `useCbfUpdate=true`：运行摆动 QP，MPC 跟踪三维参考。
- `enableCbfConstraint=false`：仅移除 QP 的 CBF 导数条件；地形、工作空间 QP 和三维跟踪仍在运行，不能用它作为完整禁用摆动优化的开关。

用户报告中的 `Swing QP solved` 和 `Swing optimized` 说明对应进程仍启用了摆动 QP。控制器启动时加载配置，编辑后需要重新启动进程；配置路径也应为实际加载的机器人 task.info。

## 验证边界

历史轮次曾构建控制器与测试包并通过当时检查；不代表本次修改。`log/cbf_off_baseline_runtime.log` 中没有摆动 QP 规划和 MPC 线程错误，状态采样持续到仿真 `60.581 s`，最终站高 `0.271 m`、姿态接近水平，但未到达台阶。后续四次 `w` 的短验证未能获得 `controller_manager` 服务且无状态样本，不计为成功的运行验证。完整楼梯和离散地形尚未重新验收。

此前 4 m 平地通过的结果对应撤销前版本，不能外推成当前 CBF 爬楼通过。本文的恢复旨在消除对原非 CBF 行为的干扰，尚不构成当前 CBF 爬楼成功的结论。工作树还保留其他已有修改（地图同步、估计器等），未整体覆盖为远端版本；落脚摆动相位计算现已恢复基线。
