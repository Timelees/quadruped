# 四足机器人摆动相 CBF + QP 系统建模与实现核验

> 核验日期：2026-11-09。依据当前工作区源码；本次在原核验基础上完成实现改进与生产代码回归测试，使用 colcon 构建；`useCbfUpdate` 保持 false，没有进行仿真或实机试验。建模核验参考 gnc-autonomy；实现改进使用 control-systems 的样条边界与二次规划理论，并按 engineering-guardrails 复用现有结构、集中校验与删除静默回退。CBF 的适用条件另以文末原始论文核对。

## 1. 核验结论与代码边界

当前系统包含三个不同层次，不能把它们统称为“已经实现闭环安全的 CBF-QP 控制器”。

| 层次 | 变量与约束 | 当前实现性质 | 入口 |
|---|---|---|---|
| 摆动参考轨迹 | 高度 B 样条控制点；地形高度与一阶 CBF 配点约束 | 凸 QP；固定 XY 曲线；加密配点、逐步增补违反约束并复验全部行 | `SwingTrajectoryQpOptimizer::optimizeSwingTrajectory` |
| MPC 落足区域 | 预测状态、输入对应的足端位置和速度；多边形边界 CBF | `StateInputSoftConstraint` + `RelaxedBarrierPenalty`，允许违反；不是独立足端 QP | `FootPlacementCBFConstraint`、`PerceptiveLeggedInterface` |
| WBC 二阶区域 CBF | 广义加速度对应的二阶不等式 | 存在公共构造函数；当前控制器源码未找到调用，WBC 配置加载和任务声明被注释 | `FootPlacementCBFCommon::buildSecondOrderInequality`、`WbcBase` |

**判断：高度 QP 的核心矩阵与一阶参考曲线约束一致；已保留端点位置/速度、统一地图插值、加密并复验约束、消除静默回退和相位索引错误。不能据此推出真实机器人连续时间避障。公共二阶函数仍遗漏非线性松弛量的二阶导数，且未接入 WBC，本次没有扩展这一层。**

以下路径均相对于控制器包根目录 `ocs2_quadruped_controller/`，可直接定位实现：

- [高度 QP](../src/interface/constraint/SwingTrajectoryQPOptimizer.cpp)：建模、目标、全部内部样点约束、qpOASES 求解。
- [B 样条](../src/interface/constraint/BSpline.cpp)：节点、基函数、解析导数。
- [摆动 Planner](../src/interface/constraint/SwingTrajectoryPlanner.cpp)：QP 调用、显式失败上报与关闭分支。
- [落足 CBF](../src/perceptive/constraint/FootPlacementCBFConstraint.cpp)：一阶残差与线性近似。
- [MPC 注册](../src/perceptive/interface/PerceptiveLeggedInterface.cpp)：软约束包装。
- [区域几何](../src/perceptive/common/FootPlacementSafetySet.cpp)：平面半空间及世界坐标变换。
- [公共 CBF](../src/perceptive/common/FootPlacementCBFCommon.cpp)：一阶计算、二阶不等式、公共相位查询。
- [参考管理器](../src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp)：松弛函数、更新参考。
- [垂向跟踪接口](../src/interface/LeggedRobotPreComputation.cpp)：使用高度和垂向速度参考。

已有 [历史核验记录](swing_cbf_qp_review.md) 记录过导数补齐等修改；本报告以本次实际读取的源码为准，不把历史记录中的测试结果视为本次运行证据。

## 2. 坐标、时钟与变量

使用世界系足端位置 \(p^W=[x,y,z]^T\)，地形为静态高程 \(g(x,y)\)。输入地图必须与世界系一致；接口自身没有完成坐标系校验。

设起跳、触地时刻为 \(t_0,t_f\)，摆动时长 \(T=t_f-t_0>0\)，归一化相位为

\[
s=(t-t_0)/T\in[0,1].
\]

设 B 样条次数为 \(d\)，控制点数为 \(n\)。配置字段 `splineOrder` 实际传给 `BSpline` 的 degree，默认值 3 表示三次。定义行向量 \(B(s)\in\mathbb R^{1\times n}\)，固定水平控制点 \(c_x,c_y\)，优化变量 \(c\in\mathbb R^n\)：

\[
x(s)=B(s)c_x,\quad y(s)=B(s)c_y,\quad z(s)=B(s)c.
\]

`interpolateXY` 将线性插值点用作水平控制点，但最终水平曲线是 \(B(s)c_x,B(s)c_y\)。它位于起终点的直线段上，**通常不按相位匀速**；均匀排列控制点不是 Greville 横坐标。代码通过解析 \(B'c_x,B'c_y\) 计算沿线速度，因此 QP 内部链式求导仍一致。

相位导数与物理时间导数关系：

\[
\dot z=B'c/T,\quad\ddot z=B''c/T^2,\quad z^{(3)}=B'''c/T^3.
\]

QP 查询的 XY 并没有经当前高度参考接口约束为实际足端 XY。实际 MPC 水平运动可偏离这条查图路径，这是跨层模型误差。

## 3. 高度参考的安全函数与线性约束

### 3.1 几何安全函数

最大附加间隙为 \(d_{\max}\)，定义

\[
d(s)=d_{\max}\sin^2(\pi s),\quad
h(s,c)=B(s)c-g(x(s),y(s))-d(s).
\]

当前目标是样点处 \(h\ge0\)。此处间隙是竖直距离，不是到地形的最短欧氏距离；没有直接包含足部半径、姿态、地图误差和跟踪误差。

目标函数保持配置的 N 个样点；安全约束使用 8(N−1)+1 个均匀样点，额外加入 CBF 起止相位并去重。所有内部约束样点加入

\[
B_jc\ge r_j,\quad r_j=g(x_j,y_j)+d(s_j).
\]

首末样点不加入地形 QP 行，但在输出复验中检查；首尾各两个高度控制点固定。夹持节点使首末控制点对应端点位置（浮点实现末端采用左极限）。因此位置和速度边界均精确保留，端点地形一致性由输出复验检查。

### 3.2 一阶 CBF 配点条件

在静态且可微的地形假设下：

\[
\dot h=\frac{B'c-\nabla g^T[x',y']^T-d'}{T},\qquad
d'=\pi d_{\max}\sin(2\pi s).
\]

施加 \(\dot h+\gamma h\ge0\)，乘以正数 \(T\)，得到

\[
\boxed{(B'_j+\gamma T B_j)c\ge
\nabla g_j^T[x'_j,y'_j]^T+d'_j+\gamma T r_j.}
\]

与 `optimizeControlPointsZ` 当前代码一致：已包含 `clearanceDerivative`，已使用解析 `generateSplineDerivativeBasis`。不应再把“遗漏间隙导数”列为当前缺陷。

CBF 只在 `enableCbfConstraint` 为真且 \(s_j\in[s_a,s_b]\) 时加入，默认窗口 \([0.18,0.82]\)，默认 \(\gamma=3\,s^{-1}\)。矩阵 CBF 残差等于 \(T(\dot h+\gamma h)\)，与高度行残差的物理尺度不同；统一 `1e-8` 数值阈值并不表示统一物理误差限。

对于 \(h>0\)，增大 \(\gamma\) 允许更快接近边界；对于 \(h<0\)，恢复要求更强。不能把增大它概括为“更加安全”或“更早抬脚”。

### 3.3 有限时间触地与切换

若在整个下降段持续满足有限常数 \(\gamma\) 的条件，且 \(h(t_a)>0\)，则

\[
h(t)\ge h(t_a)e^{-\gamma(t-t_a)}>0.
\]

它不能在有限触地时刻变为零。因此末段关闭该条件是合理的混合阶段设计，但窗口外只能依赖几何约束和跟踪；不能声称完整摆动相由指数 CBF 保证。窗口激活时还必须检查初始安全状态。

## 4. QP 目标与求解器模型

### 4.1 名义曲线与目标抬升包络

名义高度样条被重新拟合为 \(c_r\)，固定首尾各两个控制点以保留端点高度和物理时间速度；对其余控制点做采样最小二乘，加 1e-9 对角正则。删除了拟合失败时改用直线高度的回退。

记 \(z_b=Bc_r\)，原名义样点高度为 \(z_r\)。先求 \(\ell=\max(r-z_b,0)\)，对正抬升点作高斯最大值传播，再进行两次三点平滑，每次不低于原始必需抬升。最终目标为

\[
z_\star=\max(z_b+\widetilde\ell,z_r).
\]

这是构造目标的启发式，不是额外安全约束，也不保证输出高于整条名义曲线。

### 4.2 与源码对应的二次代价

令 \(D_k\) 为样点序列的未归一化 \(k\) 阶前向差分矩阵，\(C_k\) 为控制点差分矩阵。代价为

\[
\begin{aligned}
J(c)={}&w_c\|c-c_r\|^2+w_p\|Bc-z_\star\|^2\\
&+w_v\|D_1(Bc-z_\star)\|^2+w_a\|D_2(Bc-z_\star)\|^2\\
&+w_j\|D_3Bc\|^2+w_{c2}\|C_2c\|^2+w_{c3}\|C_3c\|^2
+\varepsilon\|c\|^2.
\end{aligned}
\]

配置权重大于零的项才启用；\(\varepsilon=\max(0,\texttt{hessianRegularization})\)。展开为

\[
\min_c\ \tfrac12c^THc+f^Tc,\quad Ac\ge b,
\quad c_0=z_0,\quad c_{n-1}=z_f,\quad c_1=c_{r,1},\quad c_{n-2}=c_{r,n-2}.
\]

源码的 \(H=2M+2\varepsilon I\)、\(f=-2q\) 与平方范数展开一致。所有保留项半正定，\(\varepsilon>0\) 时严格凸；正则为零时不能无条件声称唯一解。约束对 \(c\) 仿射，是因为 XY、地图查询值与梯度在本次 QP 中固定。

“velocity/acceleration/jerk”项目前是差分代理量，没有除以 \(\Delta t^k\)；控制点差分也不等于物理 jerk。目标数值依赖采样数、节点数和摆动时长。若要表示时间积分，应使用解析导数映射或 \(D_k/\Delta t^k\)，配合积分权重 \(\Delta t\)，重新标定权重和量纲。

### 4.3 边界消元、约束生成和返回语义

对夹持节点向量 U，端点速度给出

\[
c_1=z_0+\frac{T\dot z_0(U_{d+1}-U_d)}d,\qquad
c_{n-2}=z_f-\frac{T\dot z_f(U_n-U_{n-1})}d.
\]

将四个固定控制点写成向量 \(c_b\)，自由变量为 \(u\)，有 \(c=c_b+Eu\)。QP 缩为

\[
H_f=E^THE,\quad f_f=E^T(f+Hc_b),\quad A_f=AE,\quad b_f=b-Ac_b.
\]

默认 15 个控制点只需求解 11 个自由变量；不需要给固定点设置相等上下界。构造子问题前按自由变量行法向长度缩放，但最终验收使用未缩放原始行残差。

求解器先计算无约束极小值，每轮检查**全部加密约束**，选取最违反的一行加入工作集，用 qpOASES reliable 设置求解子问题。整个过程共用 `qpMaxWorkingSetRecalculations` 预算，不为每轮重置。若子问题最优解满足全体行，则它也为完整凸 QP 的最优解：子问题给出完整问题的目标下界，而该解又属于完整可行集。没有删掉 CBF 后再试，也没有靠代表点替代全量验收。

- 只有解全有限且完整原始行残差不低于 −1e-8 才接受。
- 输出样条在同一加密相位集上，直接通过位置/物理速度重新计算高度和 CBF，包含首末高度及窗口边界；CBF 复验阈值为 −1e-8 m/s。
- `BSpline::evaluateDerivative` 的节点向量缩短使用 `.eval()`，避免自身子段赋值时的 Eigen 别名问题。
- `collisionFree=true` 仍只表示这些离散验收通过，不是连续曲线/实际足部无碰撞的认证。
- 地图插值 stencil 越界、缺层或含未知值直接报告异常，不再返回零高程或零梯度。
- Planner 在 QP/复验失败时抛出带腿号和摆动时间的异常；缺少 3D 位置序列通过 `.at()` 上报。没有静默输出未验收的名义曲线。
- `isCbfUpdateEnabled()` 只反映配置开关；开关为真而未设置地形接口会显式报错。开关为假仍走原始 update 路径。

现有 MPC 线程会捕获异常、记录警告并结束 MPC 更新线程；这**不是新的机器人安全停机策略**，不能由此推断关节控制已停机。重新启用前应验证上层状态机对 MPC 更新停止的处理。

Cyberdog 配置 `useCbfUpdate=false` 保持不变；`cbfNumCurveSamples=120` 用于目标函数，对应加密后的 953 条内部高度行、611 条 CBF 行，共 1564 行，默认 11 个自由变量。当前 80 次预算在回归地形上足够，但不保证所有地形实时可行。落足 CBF 软代价独立于此开关。

## 5. perceptive 落足区域 CBF

### 5.1 半空间与松弛模型

平面多边形先形成 \(A_Pp_P+b_P\ge0\)，向内缩 \(m\) 米时每行常数减去 \(m\|a_i\|\)。经世界到平面的刚体变换，得到

\[
A_W=A_PS R_{PW},\quad b_W=b_P+A_PS t_{PW},
\quad S=[I_2\ 0].
\]

源码的旋转和平移写法与此一致。半空间表示沿平面法线无限延伸的区域，不约束足端到支撑平面的高度。行法向未归一化，边长会改变代价尺度；退化边、共线点和极短边应在几何入口剔除。

设松弛距离 \(\rho(t)\)，每行安全函数为

\[
h_i=a_i^Tp+b_i+\rho\|a_i\|.
\]

固定区域时

\[
\dot h_i=a_i^Tv+\dot\rho\|a_i\|,\quad
r_i=\dot h_i+\gamma h_i.
\]

`getValue` 和 `getLinearApproximation` 的函数值及状态/输入导数与该式一致，前提是区域参数、松弛量与时间作为外部给定量，求导时不随当前状态变化。`ConstraintOrder::Linear` 不能理解为整个运动学约束全局线性。

MPC 的足端速度直接依赖预测输入，因而此层使用一阶条件合理；对 WBC 广义加速度输入，位置约束通常为二阶，需另建 HOCBF。

### 5.2 已修正：激活查询的相位索引错位

OCS2 `lookup::findIndexInTimeArray` 用 `lower_bound`，对于事件数组 \([1,2,3]\)，\(t=1.8\) 对应 mode 索引 1。修改前 `FootPlacementCBFConstraint.cpp` 的私有相位函数把 \([event_k,event_{k+1})\) 映射为 mode 索引 \(k\)，少了 1。

具体反例：事件为 \([1,2,3]\)，某腿接触标志为 `[true,false,true,true]`，\(t=1.8\)：

- 真实处于 mode 1 的摆动，参考管理器/公共函数得到相位 0.8。
- 私有函数得到 mode 0 并因其接触而返回 -1。
- `isActive` 的 `!isContact` 虽然为真，但 `isLateSwing` 为假，导致本应开启的晚摆动 CBF 未激活。

此反例不依赖浮点事件边界。现已删除约束和参考管理器中的重复实现，统一使用公共头文件中的 contactFlagStocks 重载；原 selector 重载也转调该实现。起跳事件取左侧接触模式，触地事件取左侧摆动模式，与 OCS2 lower_bound 约定一致。连续多个无接触 mode 使用完整摆动区间，不再伪造 1 ms 时长。缺少任一侧切换边界的截断摆动返回 -1；恢复该类摆动的相位仍需上一周期的真实事件信息。

### 5.3 软惩罚与时变安全集

注册路径明确使用 relaxed barrier，负残差可以产生有限代价，因此 \(r_i\ge0\) 没有硬保证。扩大代价权重不能替代可行性证明。落足位置也以软约束形式注册。

实际松弛模型为

\[
\rho=\rho_{\max}(1-s)^q,\quad
\dot\rho=-\frac{q\rho_{\max}}T(1-s)^{q-1},\quad
\ddot\rho=\frac{q(q-1)\rho_{\max}}{T^2}(1-s)^{q-2}
\]

（摆动内部）。当前 \(\dot\rho\) 用前向差分估计，会跨触地或区域切换，且其截断误差不受 CBF 裕度补偿。建议直接使用解析分段导数并规定边界平滑性。

若区域本身随时间变化，则完整一阶导数还需

\[
\dot h_i=\dot a_i^Tp+a_i^Tv+\dot b_i+
\dot\rho\|a_i\|+\rho\frac{a_i^T\dot a_i}{\|a_i\|}.
\]

当前只适用于固定区域片段；地图/目标多边形跳变时应做混合集切换可行性检查，不能对跳变直接套光滑 CBF 定理。

## 6. 二阶 CBF 公共函数核验

对固定半空间，设广义速度 \(\nu\)，足端加速度为

\[
a_{foot}=J\dot\nu+\dot J\nu,
\quad \psi_1=\dot h+\gamma h.
\]

正确的二阶条件为

\[
\dot\psi_1+\xi\psi_1=
 a_i^T(J\dot\nu+\dot J\nu)+\ddot\rho\|a_i\|
 +(\gamma+\xi)\dot h_i+\xi\gamma h_i\ge0.
\]

对应 WBC 的上界形式：

\[
-a_i^TJ\dot\nu\le
 a_i^T\dot J\nu+\ddot\rho\|a_i\|
 +\gamma\dot h_i+\xi\psi_{1,i}.
\]

`buildSecondOrderInequality` 的符号、\(-a_i^TJ\) 和其他项正确，但没有接收/加入 \(\ddot\rho\)。仅在固定区域且 \(\ddot\rho=0\) 时与完整模型一致。对当前 \(q=2\) 的松弛，摆动内部遗漏的是正项，通常更保守；在分段边界或其他松弛曲线下不能一概认定保守。

另外 `ecbfResidualMin` 是 **令广义加速度为零时的常数项最小值**，不是已求解 WBC 的实际残差。真实残差应为 \(f_{upper}-D\,z_{wbc}\)，并注明使用命令加速度还是真实估计加速度。

当前搜索只发现该函数的声明和定义，没有找到控制器内的调用；`WbcBase` 相关任务声明和参数加载已注释。因此配置文件中的 `footPlacementCbfTask` 不能作为二阶保护已生效的证据。

即使接通二阶约束，前向不变性还要求激活时 \(h_i\ge0\)、\(\psi_{1,i}\ge0\)，以及输入可行、模型假设成立；不能把区域外恢复直接称为原安全集不变性。

## 7. 已处理问题与后续边界

| 类别 | 本次处理 | 仍需解决 |
|---|---|---|
| 相位 | 三处重复实现合并，修复错位和虚构时长 | 跨预测窗缺失起跳/触地事件的恢复 |
| 样条 | 固定端点位置和速度，修复导数别名 | 正在执行时刻的拼接连续性、端点加速度 |
| 地图 | 同一四栅格双线性曲面解析求高度/梯度，拒绝未知 stencil | 梯度跨栅格边界不连续；地图误差、延迟和坐标系约束 |
| 离散安全 | 目标与安全网格分开；加密约束、完整行验收、输出复算 | 仍没有严格连续区间证明，窄障碍可能漏检 |
| 失败 | 删除归零查图、拟合直线回退、名义曲线静默回退 | 上层安全停机/改步态/重规划策略 |
| 数值求解 | 固定边界消元、行缩放、完整集分离检查、共用迭代预算 | 更广地形下的预算与实时尾延迟 |
| 工程边界 | 构造期集中校验；删除内部不可能维度分支和重复可达性判断 | 未新增速度、加速度、腿长、关节、力矩限制 |

保留现有目标权重与启发式抬升包络，避免在缺少机器人执行指标时同时改变参数量纲。差分仍为形状代理，改成时间积分目标后必须重新标定权重；本次未实施。固定 XY 查图与实际 XY 偏差、足部体积、2.5D 地图对悬垂表达能力、MPC 软惩罚性质也仍存在。

本次没有接入新的 WBC 保护层、放宽物理限制或增加依赖。对于连续性、输入限制和误差膨胀的后续数学扩展，见下节；已实施的端点位置/速度约束不再属于待办项。

## 8. 可采用的升级模型

### 8.1 保持凸性的最小改动

保留固定 XY 和高度控制点变量；前两行边界条件本次已实施，后两行物理限制尚未添加：

\[
\begin{aligned}
&B(0)c=z_0,\ B(1)c=z_f,\\
&B'(0)c/T=v_{z0},\ B'(1)c/T=v_{zf},\\
&v_z^-\le B'_jc/T\le v_z^+,\\
&a_z^-\le B''_jc/T^2\le a_z^+.
\end{aligned}
\]

再用一致的地图插值和验证器检查连续区间。增加约束可能导致不可行，应优先松弛性能目标，保持障碍约束含义，并由上层决定变更落足点或摆动时间；不能悄悄删除 CBF 后仍报告安全。

若静态高程具有已知水平 Lipschitz 常数 \(L_g\)，实际水平误差有界 \(\|e_{xy}\|\le\epsilon_{xy}\)，垂向误差 \(|e_z|\le\epsilon_z\)，地图误差 \(\epsilon_g\)，可先给参考间隙增加

\[
m\ge\epsilon_z+L_g\epsilon_{xy}+\epsilon_g.
\]

这只在误差界和地形正则性真实成立时有效，还需另计延迟与区间采样误差；台阶处不能直接使用有限 \(L_g\)。

### 8.2 若目标是实际足端闭环安全

建立足端双积分器近似：\(\dot p=v,\dot v=u+w\)，将 \(u\) 映射到 WBC 可实现加速度域。对地形安全函数 \(h=z-g(x,y)-d(t)\)：

\[
\ddot h=u_z-\nabla g^Tu_{xy}
-v_{xy}^T\nabla^2g\,v_{xy}-\ddot d
\]

（此式先忽略扰动，静态光滑地形）。再施加

\[
\ddot h+(\gamma+\xi)\dot h+\gamma\xi h\ge\eta,
\]

其中 \(\eta\) 应由明确的扰动界构造，而不是任意安全常数。给定当前状态与地图导数时该条件对加速度仿射；但与完整刚体动力学、接触、力矩限制联立后的可行性必须验证。若模型不是可控足端双积分器，不能直接把 \(u\) 当作任意可实现输入。

此方案属于新增闭环安全层，区别于当前参考样条 QP。没有 CLF 条件的跟踪平方代价不构成 CLF；若增加 CLF，其下降率和可行性也需单独建模。

## 9. 验证范围与复现

新增 [生产路径回归程序](../../../test/src/swing_qp_regression.cpp)，直接调用生产优化器和样条，不复制 QP 公式。使用现有 test 包的可执行程序风格，加入 CTest 入口。

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
source build/ocs2_quadruped_controller/colcon_command_prefix_build.sh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select ocs2_quadruped_controller --executor sequential
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select test --cmake-target swing_qp_regression --executor sequential
ctest --test-dir build/test -R '^swing_qp_regression$' --output-on-failure
```

检查范围：

1. 起跳/触地左右极限、原 1.8 秒错位反例、跨多个 mode 的连续摆动、截断摆动。
2. 非单位时间区间的端点速度与位置/速度有限差分，覆盖一阶、二阶输出导数。
3. 双线性曲面的高度/解析梯度、GridMap 环形缓冲区移动、越界与 NaN 查询。
4. 平地、6 cm 凸起（T=0.4 s）、4 cm 凸起（T=0.7 s）使用 80 次共同求解预算；核对端点位置/速度、所有约束点的 CBF 与独立 2001 点高度检查。
5. 禁用开关保持普通站立参考；启用且缺地形接口显式失败；非法增益/零时长拒绝；低于地形的端点不能成为有效输出。

开发期对生产源码的独立编译运行中，三种地形分别生成约 15/12/5 条工作行，消耗 22/14/7 次迭代；这些工作行只是求解过程，最终始终验收全部 1564 行。所有加密 CBF 点通过；独立网格最小高度为 0，点间最小 CBF 残差分别约 −4.1e-6、−5.0e-4、−1.6e-4 m/s。回归对点间误差设置 1 mm/s 的场景检查限，**不把该测试限当作生产约束放宽量，也不声称点间严格满足 CBF**。

此前仅增加复验、仍用粗网格求解时，在平地上检出约 −5.6e-5 m/s 的点间违反。直接把全部加密行交给冷启动求解器又出现迭代耗尽，因此采用当前的约束生成法；固定边界消元和数值缩放一并减少求解冗余。没有通过增加预算或丢弃安全行让测试通过。

构建与最终 CTest 结果见文末执行记录。测试不证明机器人实际跟踪误差界、闭环稳定性或连续避障，也没有执行仿真/实机。更广的地图、真实足端轨迹和四腿同时更新尾延迟需在启用配置前验证。

## 10. 理论参考与保证边界

- [Ames 等：Control Barrier Function Based Quadratic Programs for Safety Critical Systems](https://arxiv.org/abs/1609.06408)：CBF 将安全集前向不变性转为输入条件；当前样条参数 QP 与实际控制输入 QP 必须区分。
- [Xiao、Belta：Control Barrier Functions for Systems with High Relative Degree](https://arxiv.org/abs/1903.04706)：高相对阶条件对应一系列集合的交集，不应遗漏辅助初始集及输入限制。
- [Breeden、Garg、Panagou：Control Barrier Functions in Sampled-Data Systems](https://arxiv.org/abs/2103.03677)：采样控制安全需要专门的区间条件。本文对象含分段常值控制，不能直接充当本项目样条配点安全证明。

本报告中的具体代码缺陷与改进排序来自当前源码和公式推导；论文提供方法论，不构成对此机器人实现的安全认证。


## 11. 本次执行记录与工程取舍

- 控制器 Release 构建、链接及安装成功；完整 colcon build 和指定控制器目标构建均返回 0。
- test 包的 `swing_qp_regression` 目标构建成功；CTest 为 1/1 通过，约 0.09 秒。
- Release 产物直接运行：平地约 2.25 ms、6 cm 凸起约 1.56 ms、4 cm 凸起约 1.23 ms。这是单次本机观测，不是实时上界或四腿尾延迟测试。
- 本地检查确认配置开关仍为 false、原有 Cyberdog 80 次预算不变；文档链接、公式分隔符和新增修改的空白检查通过。
- 仅沿用 Eigen、qpOASES 和现有测试包，未增加第三方依赖。复用三处重复的相位逻辑；边界消元取代固定变量等值约束；保留外部输入校验，删除内部不可能维度分支和静默降级。
- 直接对 1564 行冷启动会耗尽旧预算；增加预算会扩大时间上限。当前采用约束生成并验收完整集合，保留原预算，示例仅需 7–22 次迭代；复杂度是一个有限预算内的增补循环。
- 默认 shell 加载自身工作区 underlay 时，曾在 colcon 收尾挂起并中断；使用 ROS 与控制器已有依赖前缀环境完成构建。代码无需增加环境兼容分支。
