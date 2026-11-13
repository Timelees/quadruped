# 基于 Elevation Mapping、MINVO 与 HOCBF 的四足机器人摆动相足端连续时间安全规划与跟踪

> **研究设计与工程实施方案（2026-11-11）**  
> 定位：规划层采用五次 B-Spline + 分段 MINVO 几何安全约束；跟踪安全接口层采用 HOCBF-QP；以 Elevation Mapping / grid_map 提供地形信息，面向通用四足机器人摆动足端轨迹规划与安全跟踪架构。  
> **状态说明**：本文是拟实施的理论与工程设计，不代表代码已实现或获得实机安全性证明。所有模块均以独立理论与通用实现方案描述，不依赖特定现有项目代码；需要实测验证的条件另行列出。

---

## 0. 核心结论与边界

**层次必须清楚：**

- **MINVO 规划层**：给定安全地形模型、局部安全走廊和五次 B-Spline，将每个多项式段转换成可包围整段轨迹的 MINVO 凸包；通过所有凸包顶点位于安全走廊内部这一**充分条件**，约束整个连续时间参考轨迹的几何安全。它**不等于** CBF。
- **HOCBF 跟踪接口层**：读取**实际**足端位置/速度、参考轨迹状态和局部光滑安全函数，使用有相对阶的控制屏障条件修正名义控制命令。它提供的是在**特定动力学、初始状态、输入可行性、估计误差和实施条件下**的安全性分析，不能替代实际机器人全身动力学验证。
- **控制执行层**：在机器人关节控制器或全身控制（WBC）接口中落实安全命令。若 HOCBF 输出的是足端加速度，而控制器不能直接稳定实现该加速度，则不能宣称理论安全保证已自动转移到真机。
- **地图不等于真实世界**：Elevation Mapping 为 2.5D 表面高度模型，可能无法表示悬空障碍、台阶下沿、垂直/多层曲面、传感器盲区。地图模型支持的无碰撞结论与真实环境无碰撞结论应分开表述。

### 0.1 总体数据流

```text
RGB-D/LiDAR/深度点云 + 位姿/TF
                  │
                  ▼
         elevation_mapping
  grid_map: elevation/variance/时间戳/有效性
                  │
                  ▼
         TerrainSnapshotAdapter
     坐标对齐 + 地图快照 + 不确定性
          ┌───────┴────────┐
          ▼                ▼
  GeometrySafetyModel  DifferentiableSafetyField
  栅格占据上界/障碍凸分解   光滑 h、∇h、∇²h
          │                │
          ▼                │
  B-Spline 名义轨迹        │
          │                │
  粗检 → 连续时间证书       │
          │                │
     安全 ────┐           │
     风险 ────▼           │
  分段 Bézier/MINVO        │
  固定凸安全走廊 + QP       │
  轨迹验证/提交/切换        │
          │                │
          └────────┬───────┘
                   ▼
          HOCBF 安全跟踪接口
   实际足端状态 + 名义指令 + 安全场
                   │
              QP/可行性检查
                   │
                   ▼
          机器人控制器 / WBC
                   │
              实机执行/反馈
```

---

## 1. 系统假设、坐标与安全定义

### 1.1 坐标系统与状态

采用惯性世界坐标系 $W$，重力方向为 $-z_W$。所有高程值和足端位置最终转换到同一个 $W$ 坐标系。机器人基座系 $B$、地图系 $M$、足端系 $F$ 使用 TF 在**对应时间戳**求变换，而非无条件使用最新 TF。

第 $l$ 条腿的实际足端位置、速度与加速度：

$$
\mathbf p_l=[x_l,y_l,z_l]^T,\quad \mathbf v_l=\dot{\mathbf p}_l,\quad \mathbf a_l=\ddot{\mathbf p}_l.
$$

参考轨迹：$\mathbf p_{r,l}(t),\mathbf v_{r,l}(t),\mathbf a_{r,l}(t)$。摆动起点 $t_s$、预定落脚时刻 $t_f$，当前时刻 $t_c$，剩余时间 $T_r=t_f-t_c$。

### 1.2 足端几何与终端接触

足端可先用球体 $\mathcal B(r_f)$ 进行保守包络；实际脚掌更适合使用朝向相关的凸多面体或支撑函数。**所有安全约束都应针对足端体积而不是单一脚尖点。**

对离地阶段定义正安全间隙；对**落脚接触阶段**，应使用单独的接触终端条件，不能在落脚时刻仍要求 $z-H\ge r_f+d_{\mathrm{clear}}>r_f$ 并同时要求接触，这会产生逻辑冲突。实际可采用“摆动中段无碰撞 + 端点接触域 + 接触模式切换”的分阶段规范。

### 1.3 安全证明的三个层次

1. **几何层**：在保守地图和固定的静态障碍模型下，所有时间段的足端包络与障碍分离。
2. **简化动态层**：双积分足端模型、连续时间 HOCBF 及可行控制集成立时，屏障集合前向不变。
3. **真实系统层**：需要将关节运动学/动力学、采样保持、执行误差、地图和状态估计误差纳入鲁棒裕量，才可以提出更强的实机安全结论。

---

## 2. 从 Elevation Mapping 到统一 TerrainSnapshot

### 2.1 ROS / grid_map 数据入口

ANYbotics 的 elevation_mapping 以 `grid_map_msgs/GridMap` 表示机器人局部 2.5D 地图；常见话题：`/elevation_mapping/elevation_map_raw` 和 `/elevation_mapping/elevation_map`。前者通常带 `elevation`、`variance` 等原始层；后者融合地图的层名可能不同，例如 `elevation`、`upper_bound`、`lower_bound`。**必须运行时读取 `getLayers()`，不得假定融合地图必然含 `variance`。**

推荐使用 `grid_map::GridMapRosConverter::fromMessage(...)` 或项目对应 ROS 版本的等价转换接口；获取网格分辨率、地图朝向/中心、frame_id、time stamp、有效单元与层名。grid_map 使用环形缓冲，不要直接假设矩阵行列号就是世界 x/y 顺序，应使用 `getPosition(index, xy)` / `getIndex(xy, index)`。

> 下述为与特定代码仓库无关的接口级伪代码；ROS1 与 ROS2 的消息接口及转换 API 可能不同，应在实现时根据选定版本适配。

```cpp
struct TerrainCell {
    double elevation{NAN};
    double variance{NAN};
    double upperBound{NAN};
    bool valid{false};
};

struct TerrainSnapshot {
    std::string frameId;
    int64_t stampNs{0};
    double resolution{0.0};
    Eigen::Vector2d center{0.0, 0.0};
    grid_map::GridMap map;             // 使用快照，避免优化中读到撕裂数据
    bool hasVariance{false};
    bool hasUpperBound{false};
};

TerrainSnapshot makeSnapshot(const grid_map::GridMap& incoming) {
    TerrainSnapshot s;
    s.map = incoming;
    s.frameId = incoming.getFrameId();
    s.stampNs = incoming.getTimestamp();
    s.resolution = incoming.getResolution();
    s.center = incoming.getPosition();
    s.hasVariance = incoming.exists("variance");
    s.hasUpperBound = incoming.exists("upper_bound");
    return s;
}
```

### 2.2 查询必须具备的接口

| 字段 | 来源 | 规划用途 | HOCBF 用途 |
|---|---|---|---|
| 高度 `elevation` | 原始或融合地图 | 障碍表面 | 构建 $h$ |
| 方差 `variance` | 通常原始地图 | 风险裕量 | 鲁棒裕量 |
| `upper_bound` | 若实际存在 | 保守地形上界候选 | 不能直接当可微函数 |
| `valid/NaN` | grid_map | 未知区域禁用认证 | 不允许运行未经验证的屏障约束 |
| 地图分辨率 | grid_map | 膨胀、栅格覆盖 | 光滑拟合尺度 |
| 时间戳 + frame_id | 消息元数据 | 时空一致性 | 计算过期地图和坐标误差 |
| 地形梯度/Hessian | 新建局部光滑模型 | 可选 | HOCBF 核心 |
| 凸障碍物/安全走廊 | 新建几何模块 | MINVO 核心 | 可选采用局部平面障碍屏障 |

### 2.3 地形不确定性和足端几何膨胀

在置信假设允许时，使用保守高度候选：

$$
H^+(x,y)=\widehat H(x,y)+k_\sigma\sigma_H(x,y)+\delta_{\mathrm{registration}}+\delta_{\mathrm{staleness}}.
$$

其中 $\sigma_H=\sqrt{\max(0,\mathrm{variance})}$。如果某一层已是经过统计定义的上界，则应核实其统计含义，**不要再不加区分地重复叠加方差增益**。$k_\sigma$ 的概率解释依赖误差分布和校准；固定安全裕量应通过实验界定。

另外，地图高度格子的单元内最大值并不必然等于格子中心高度；为了构造可信上界，需要使用下列一种或组合：

- 经校准的感知最大误差 + 栅格覆盖上界；
- 对每个栅格对应的三维占据柱体进行保守建模；
- 已知高度场 Lipschitz 常数 $L_H$，利用 $H(x)\le H(x_c)+L_H\|x-x_c\|$ 建立单元内上界；
- 对未知区域标为不可认证，触发减速、重新感知或重新选落脚点。

### 2.4 地形转换为**两套不同数据**

**用于 MINVO 的几何数据**：

```cpp
struct ConvexPolytope {
    Eigen::MatrixXd A;   // A*x <= b，统一世界系
    Eigen::VectorXd b;
};
struct TerrainGeometry {
    std::vector<ConvexPolytope> obstacles;
    std::vector<ConvexPolytope> safeCorridors;
    uint64_t mapVersion;
};
```

来源：对轨迹附近的栅格计算保守高度上包络，或者构造局部三维占据柱体并凸分解、生成位于已认证自由空间内的局部凸走廊。对于高台阶，不宜把整个路径范围压成一个最大高度，必须随分段建立不同空间走廊。

**用于 HOCBF 的可微数据**：

```cpp
struct SmoothSafetyFieldSample {
    double h;
    Eigen::Vector3d gradH;    // 若 h=z-H(x,y)-d，gradH=[-Hx,-Hy,1]
    Eigen::Matrix3d hessianH;
    bool differentiable;
    bool certifiedConservative;
};
```

来源：在已知有效、安全的局部区域内，用 $C^2$ 二维样条 / 平滑核 / 多项式曲面拟合 $\tilde H(x,y)$，然后**增加拟合误差的上界** $\epsilon_{\mathrm{fit}}$：

$$
H_{\mathrm{CBF}}(x,y)=\tilde H(x,y)+\epsilon_{\mathrm{fit}}(x,y)+\delta_{\mathrm{track}}.
$$

如果 $\epsilon_{\mathrm{fit}}$ 是空间变化的，它自身也必须足够光滑才能在 HOCBF 中求导；最简单可靠的局部实现是在有效小区内用经验证的常数上界。**平滑本身可能削低台阶高度，不能把平滑后的高度直接当安全上界。**

### 2.5 高程图的特殊局限：侧向/悬空碰撞

对于脚尖绕台阶侧面，2.5D 充分条件可把障碍表示为自地面基准延伸至 $H^+(x,y)$ 的柱体，属于保守模型；但对台阶下沿、悬空横杆或多层几何，同一个 (x,y) 只有一个高度，无法准确表达三维障碍。此时需要额外点云/TSDF/ESDF/三维占据地图，不能只靠 elevation map 做真实三维安全认证。

---

## 3. B-Spline 参考轨迹与正确的连续时间碰撞判断

### 3.1 三维参考轨迹统一表示与时间参数化

设足端摆动相采用五次 B-Spline 参考轨迹：

$$
\mathbf p_r(t)=\sum_{j=0}^{N-1}N_{j,5}(t)\mathbf Q_j,
\qquad t\in[t_s,t_f],
$$

其中 $\mathbf p_r(t)=[x_r(t),y_r(t),z_r(t)]^T$，$\mathbf Q_j\in\mathbb R^3$。若采用水平与竖直方向独立参数化的实现，也须在连续碰撞认证之前统一为具有共同物理时间的分段多项式轨迹，保证各段位置及其所需导数可以正确求值。只在高度方向调整轨迹属于受限的抬脚避障，不能等价于完整三维绕障。

规划与跟踪层均应使用同一世界坐标系、相同摆动起止时刻及一致的导数定义。

### 3.2 快速风险预检（不能当安全证明）----感觉可以不要，节省这部分采样的计算消耗

对 $t_k=t_s+k\Delta t$ 采样，并查询：

$$
d_k=z_r(t_k)-H^+(x_r(t_k),y_r(t_k))-r_f-d_{\mathrm{clear}}(t_k).
$$

如果有 $d_k<0$，即可直接确认**相对于该地图与足端球形模型**采样处存在几何冲突；若所有 $d_k\ge0$，则**不能**据此判断连续时间安全。对高度地图的 NaN / 失效区，标记 `UNKNOWN`，而不是 `SAFE`。

推荐三态返回：`RISK`、`PROVEN_SAFE`、`UNKNOWN`；其中 `RISK` 表示已发现或未排除的风险，`PROVEN_SAFE` 仅由可靠的连续时间几何证书产生。

### 3.3 可靠的分段几何碰撞证书

对所有非零 B-Spline 节点区间和随后自适应细分区间 $I_i$：

1. 提取区间精确多项式（例如通过节点插入得到 Bézier 段）；
2. 构造包围整段曲线的 Bézier 或经验证的 MINVO 控制点凸包 $\mathcal H_i$；
3. 按足端球体 $\mathcal B(r_f)$ 做 Minkowski 膨胀；
4. 根据完整栅格覆盖的保守高度或三维障碍物凸体检测 $(\mathcal H_i\oplus\mathcal B)\cap\mathcal O_k$；
5. 若分离，给出**该段相对于地图模型的连续时间安全证书**；若相交，不能直接认为曲线一定碰撞，进行时间细分/更精确几何检测；
6. 达到细分深度、超时或地图未知仍无法排除碰撞时标记 `UNKNOWN` 并触发优化/保守处理。

**两种容易混淆的结果**：凸包与障碍相交 ≠ 曲线与障碍相交；采样没有撞 ≠ 整段没撞。

### 3.4 球体与高度图的严格上包络检测（高度优化的首版）

对于区间 $I_i$，建立其水平 **足端扫掠区域** $\Omega_i$：水平轨迹包围 + 脚半径 + 地图/跟踪误差裕量。对 $\Omega_i$ 的所有有效栅格计算经过认证的高度上界：

$$
\bar H_i\ge \sup_{(x,y)\in\Omega_i}H_{\mathrm{true/cons}}(x,y).
$$

若该段 z 曲线的 MINVO 全部高度控制点满足

$$
\boxed{v^z_{i,j}\ge \bar H_i+r_f+d_i,\quad j=0,\ldots,5,}
$$

则在**指定包络与高度上界假设下**整段满足高度净空。此充分条件很保守，靠近抬脚/落脚接触点时需要缩短区间、改变走廊或切换到接触边界模型。

**数值注意**：`max(H_grid)` 如果只针对网格中心而不是轨迹与整个栅格的交叠范围，不足以支持严格上界结论。

---

## 4. 规划层 MINVO：数学模型、约束和求解

### 4.1 精确分段与基变换

将第 $i$ 个非零区间归一化为 $u\in[0,1]$。五次多项式：

$$
\mathbf p_i(u)=\mathbf C_i[1,u,u^2,\ldots,u^5]^T.
$$

MINVO 基：$\boldsymbol\alpha_{MV}(u)=A_{MV}^{(5)}[1,u,\ldots,u^5]^T$。如果按 **控制点为列** 的约定：

$$
\mathbf p_i(u)=V_i^{MV}\boldsymbol\alpha_{MV}(u),\quad
V_i^{MV}=\mathbf C_i(A_{MV}^{(5)})^{-1}.
$$

从已知 Bézier 控制点 $V_i^{Be}$ 转换：

$$
\boxed{V_i^{MV}=V_i^{Be}A_{Be}^{(5)}(A_{MV}^{(5)})^{-1}.}
$$

若用 B-Spline 局部控制点直接转换：

$$
V_i^{MV}=V_i^{BS}A_{BS,i}^{(5)}(A_{MV}^{(5)})^{-1}.
$$

这里 $A_{BS,i}^{(5)}$ 取决于局部节点区间/归一化规则，不要把仅适用于均匀内部区间的矩阵直接用于所有端点区间。

**工程必做**：确认五次 MINVO 基函数在 $[0,1]$ 上非负、和为 1；不能将三次 MADER 矩阵直接用于五次。五次矩阵须从官方论文/代码的对应阶数生成并验证。若暂时无法可靠取得五次 MINVO 基，**先使用五次 Bézier 凸包**建立完整正确基线，后续替换。

### 4.2 MINVO 凸包性质

在上述基非负且分解单位条件下：

$$
\mathbf p_i(u)=\sum_{j=0}^5\alpha_j(u)\mathbf v_{i,j},\quad
\alpha_j\ge0,\quad\sum_j\alpha_j=1,
$$

因此：

$$
\boxed{\mathbf p_i([0,1])\subseteq\mathcal H_i=\operatorname{conv}\{\mathbf v_{i,0},\ldots,\mathbf v_{i,5}\}.}
$$

对于三维五次多项式，凸包顶点可能多于 4，不应在几何实现中强行把它当四顶点四面体。

### 4.3 固定安全走廊 + 线性 MINVO 约束（优先）

局部走廊 $\mathcal C_i=\{\mathbf x:A_i\mathbf x\le\mathbf b_i\}$ 是 **已经证实不与障碍物相交** 的凸自由空间。对所有 MINVO 控制点：

$$
\boxed{A_i\mathbf v_{i,j}\le\mathbf b_i,\qquad j=0,\ldots,5.}
$$

若 $\mathbf v_{i,j}=T_{i,j}\mathbf q$ 且 $\mathbf q$ 为优化控制点，则：

$$
\boxed{A_iT_{i,j}\mathbf q\le\mathbf b_i.}
$$

这组约束对控制点**线性**，适合当前 QP 求解器。足端半径可通过对安全走廊进行内缩（半空间第 k 面的法向量 $\mathbf a_k$：$b'_k=b_k-r_f\|\mathbf a_k\|$）或按精确足端支持函数处理。

**走廊交界**：相邻分段轨迹在连接时间必须连续；若走廊互不相接，轨迹可能不可行。规划前检查相邻安全走廊交集/可通行性。



### 4.5 目标函数

令 $\mathbf q$ 为控制点堆叠向量、$\mathbf q_{ref}$ 为名义控制点。可采用：

$$
J=w_p\int\|p(t;q)-p_r(t)\|^2dt+
 w_v\int\|\dot p-\dot p_r\|^2dt+
 w_a\int\|\ddot p-\ddot p_r\|^2dt+
 w_j\int\|p^{(3)}\|^2dt+
 w_q\|q-q_{ref}\|^2.
$$

固定节点、固定时间参数时，上式可写成 $\tfrac12q^THq+f^Tq+c$。加入：

$$
\begin{aligned}
\min_q\quad&\tfrac12q^THq+f^Tq\\
\text{s.t.}\quad&A_{eq}q=b_{eq}\quad \text{（起终点、速度、必要的加速度）},\\
&A_{MV}q\le b_{MV}\quad \text{（连续时间几何）},\\
&A_{lim}q\le b_{lim}\quad \text{（线性或线性化限制）}.
\end{aligned}
$$

速度、加速度约束可对**导数曲线**另建凸包并对导数控制点施加范数界；关节限位需通过 IK/雅可比约束审查，不应只使用足端笛卡尔限幅替代。

### 4.6 接触边界和初始状态约束

标准条件：

$$
p(t_s)=p_{lift},\quad \dot p(t_s)=v_{lift},\quad
p(t_f)=p_{touch},\quad \dot p(t_f)=v_{touch}.
$$

在重规划 $t_c$ 时：

$$
\boxed{p_{new}(t_c)=p_{meas}(t_c),\quad
\dot p_{new}(t_c)=v_{meas}(t_c),\quad
\ddot p_{new}(t_c)=a_{switch},}
$$

这里加速度初值是否取估计值或命令值取决于执行器切换协议；测量噪声较大时不应强行将差分加速度设为严格等式。终端接触速度及摆动时间如果无法保持安全可行，应由步态/落脚规划协调，而不是在 QP 中随意放松障碍安全约束。

### 4.7 地形自适应分段

触发策略：对每一段计算包围凸包与地形障碍物的分离证书；距离明显富余的段保留；风险高/凸包相交但实曲线可能安全的段做二分（对多项式段用 De Casteljau/精确子段提取）。限制最大细分层数 $D_{max}$、总段数 $M_{max}$ 和求解预算；预算耗尽且无法认证，则结果是 **UNKNOWN/INFEASIBLE**，绝不能当安全。

### 4.8 地图更新后的轨迹提交协议

规划线程以地图版本 $v$ 读取快照；求解器输出 $(q^*,v)$；发布/切换前用最新地图版本进行碰撞复检。若地图已改变并影响扫掠区域，则重新验证或不提交。此“check–recheck”是避免用过期地图提交新轨迹的重要工程机制，不是解决所有延迟/感知漏检问题的万能保证。

---

## 5. HOCBF 跟踪接口层：理论推导

### 5.1 足端双积分模型（简化控制层）

在足端任务空间先建立：

$$
\boxed{\dot p=v,\quad\dot v=u+w,}
$$

其中 $u$ 是可指令的**任务空间加速度**，$w$ 代表未建模误差或扰动。实际实现若只有关节位置命令，必须提供足端加速度到关节命令/力矩的合理映射与可实现性验证。

对跟踪器，可定义 PD + 前馈名义指令：

$$
\boxed{u_{nom}=a_r+K_p(p_r-p)+K_d(v_r-v).}
$$

**HOCBF 安全层只修改这个名义指令，不负责重新选择整条轨迹的终点或改变接触模式。**

### 5.2 局部可微地形安全函数

选取经保守校准的 $C^2$ 高度上包络 $H_c(x,y)$，在**摆动的非接触阶段**定义：

$$
\boxed{h(p)=z-H_c(x,y)-r_f-d_s.}
$$

安全集 $\mathcal C_0=\{(p,v):h(p)\ge0\}$。此形式对过顶避障适用；针对台阶前立面和侧向绕障时，**只靠表面高度障碍 h 不够灵活**，可使用局部安全走廊的多个平面屏障 $h_k(p)=b_k-a_k^Tp-r_f\|a_k\|$ 或逐个障碍的可微距离安全函数。

若安全间隙随时间/相位变化，应改用 $h(p,t)$ 并保留 $\partial_t h,\partial_{tt}h,\partial_t\nabla h$ 等所有项。**为首版严谨实现，建议在单个非接触安全阶段使用常数 $d_s$，端点采用接触模式切换。**

### 5.3 相对阶二与高阶屏障

设二维地形梯度 $g_H=[H_x,H_y]^T$、Hessian $D^2H_c$。有：

$$
\dot h=v_z-g_H^Tv_{xy}.
$$

$$
\boxed{\ddot h=u_z-g_H^Tu_{xy}-v_{xy}^TD^2H_c\,v_{xy}}
$$

（上式先取 $w=0$；若有扰动，需加上 $\nabla h^Tw$）。

设 $\alpha_1,\alpha_2>0$，定义：

$$
\psi_0=h,\quad \psi_1=\dot h+\alpha_1h.
$$

二阶 HOCBF 条件：

$$
\dot\psi_1+\alpha_2\psi_1\ge0
\quad\Longleftrightarrow\quad
\boxed{\ddot h+(\alpha_1+\alpha_2)\dot h+\alpha_1\alpha_2h\ge0.}
$$

代入得到对 $u$ 的仿射约束：

$$
\boxed{[-H_x,-H_y,1]u\ge
v_{xy}^TD^2H_c\,v_{xy}-(\alpha_1+\alpha_2)\dot h-\alpha_1\alpha_2h.}
$$

若初始 $h(t_0)\ge0$ **且** $\psi_1(t_0)\ge0$，局部安全函数满足正则性、系统正确、控制输入始终可行且连续满足屏障条件，则 $\{h\ge0,\psi_1\ge0\}$ 前向不变。这一结论不等于从任意 $h\ge0$ 初始状态都能自动恢复安全，亦不能保证地图突然更新后当前状态仍在新安全集合内。

### 5.4 若采用安全走廊平面 HOCBF（优先工程方案）

走廊面 $a_k^Tp\le b_k-r_f\|a_k\|$，定义：

$$
h_k(p)=(b_k-r_f\|a_k\|)-a_k^Tp.
$$

$$
\dot h_k=-a_k^Tv,\quad\ddot h_k=-a_k^Tu.
$$

则：

$$
\boxed{-a_k^Tu-(\alpha_1+\alpha_2)a_k^Tv+\alpha_1\alpha_2 h_k\ge0.}
$$

**优势**：没有地形 Hessian；和 MINVO 规划层使用同一局部安全走廊，空间安全语义一致。这是首版 HOCBF 最值得采用的接口。**限制**：安全走廊若频繁切换且新走廊不包含当前状态/速度对应的 HOCBF 可行集，前向不变性条件不自动保持；要设计走廊重叠与切换验证。

### 5.5 HOCBF-QP

$$
\boxed{\begin{aligned}
\min_{u}\;&\tfrac12(u-u_{nom})^TW(u-u_{nom})\\
\mathrm{s.t.}\;&A_{cbf}(p,v)u\le b_{cbf}(p,v),\\
&u_{min}\le u\le u_{max},\\
&\text{（可选）关节空间映射后的可实现性约束}.
\end{aligned}}
$$

可设置**名义跟踪**为软目标，但不要通过允许 HOCBF 的碰撞安全约束无限松弛来掩盖不可行。若必须设计松弛变量，应对其使用进行安全级别区分，并明确：含正松弛的解**不再具有原约束的屏障安全保证**。

### 5.6 约束可行性、关节可实现性和接触模式

- 对每步求解，先检查 HOCBF + 输入限幅有无可行解；失效时向上层发送安全事件，触发落脚点/步态/时间调整或经验证的恢复动作。
- 双积分模型的 u 是任务空间加速度。关节实现需近似满足 $a_f=J(q)\ddot q+\dot J(q,\dot q)\dot q$，再结合力矩、摩擦/接触与全身动力学约束；仅仅限幅 $u$ 不足以保证全身可执行。
- 在摆动末端，接触不是屏障失效：应在合法接触域/接触判据中切换到 stance 模式和对应接触约束。确保终端高度与 $d_s$ 不冲突。
- 若地图突变使 $h<0$ 或 $\psi_1<0$，HOCBF 的前向不变性前提不满足；需单独讨论恢复可行性，而不能宣称自动保持安全。

### 5.7 鲁棒版本（推荐后期补充）

若可证明 $\|w\|\le\bar w$，则 HOCBF 中控制通道的最坏影响满足：

$$
\nabla h^Tw\ge-\|\nabla h\|\bar w.
$$

因此可在名义 HOCBF 右端增加 $\|\nabla h\|\bar w$ 保守裕量。对地图/定位误差，还需在 $h$ 中使用经校准的几何上包络，而不能把所有误差都混为动力学扰动。采样保持 HOCBF 还需按控制周期、延迟、状态变化率推导额外离散实现裕量；只在离散时刻检查连续公式**不是**无条件连续时间保证。

---

## 6. 两层接口的关键合同（Safety Contract）

```cpp
struct SafetyPlanContract {
    uint64_t mapVersion;
    double validFrom;
    double validUntil;
    std::vector<ConvexPolytope> safeCorridors;
    // 每段对应的合法时间区间、含脚半径和误差的走廊
    bool geometryCertified;
    double certifiedMargin;
};

struct FootReferenceState {
    Eigen::Vector3d p, v, a;
    double time;
    int legId;
};

struct HOCBFResult {
    Eigen::Vector3d uSafe;
    bool feasible;
    double minBarrier;
    double minPsi1;
    double qpSolveMs;
};
```

**规划层交付给安全层的内容**不仅是一个 B-Spline：还需要地图版本、安全走廊、对应时间区间和脚几何/安全余量。**安全层必须使用实际状态**，并先检查 $p,v$ 在当前走廊的 HOCBF 可行域中。如果当前实际状态已经偏出该走廊，不能直接套用上一轮走廊屏障。

### 6.1 轨迹安全和跟踪安全并非简单相加

MINVO 凸包包含的是**参考中心曲线**，HOCBF 约束的是**实际足端运动**。两者之间的耦合需满足至少以下之一：

1. 规划走廊预留 tracking tube 的空间余量，并验证实际误差被约束在该 tube 内；或
2. 跟踪 HOCBF 直接作用于包含真实足端尺寸的安全走廊；或
3. 结合以上两者，并设计安全域切换与末端接触机制。

仅证明参考轨迹安全，不能推出实际足端安全；仅 HOCBF 可行，也不能确保预定时间能够准确落脚。

---

## 7. 在线系统时序与事件触发重规划

### 7.1 运行时状态机

```text
STANCE
  │ 摆动计划到来
  ▼
PREPARE_SWING ──地图未知/几何不可行──> REPLAN_FOOTHOLD / SAFE_RECOVERY
  │ 地图有效 + 规划成功
  ▼
SWING_ACTIVE ── 地图更新/风险上升 ──> REPLAN_REMAINING
  │                                  │
  │                         轨迹通过复检 + 切换可行
  │<─────────────────────────────────┘
  │ 计划终端满足接触域/真实接触
  ▼
TOUCHDOWN → STANCE
```

### 7.2 事件触发条件

按整段碰撞证书、地图版本/时间、跟踪误差和求解剩余时间判断：

$$
\mathrm{trigger} = (\mathrm{mapRisk}>\theta_r)
\vee(\|e_p\|>\theta_p)
\vee(\mathrm{corridorExpired})
\vee(\mathrm{mapVersionAffected}).
$$

不要在每个控制周期无条件求解完整轨迹 QP；地图更新频率、轨迹规划频率、HOCBF 控制频率可分别设置并实测。

### 7.3 剩余轨迹重规划

对 $[t_c,t_f]$ 精确裁剪原多项式段，建立新的初值约束；计算剩余地图有效区域与走廊；重新求解、独立验证几何安全、运动学可行及末端条件；检查地图版本并原子替换生效轨迹。规划失败时**不得自动退回已判有碰撞风险的原始轨迹**。

### 7.4 提交时间与实时性

设计算、验证、发布、切换上界为 $T_{solve},T_{verify},T_{publish},T_{switch}$，需满足

$$
T_{solve}+T_{verify}+T_{publish}+T_{switch}<T_r
$$

并留出最小闭环响应时间；此时间条件是必要条件而非充分的安全保证。应记录平均/P95/P99 耗时与最大迭代次数，而不是只记录平均时间。

---

## 8. 通用实现架构与开发流程

### 8.1 模块划分与职责

1. **TerrainSnapshotAdapter**(已有)：订阅并缓存带时间戳、坐标系和版本号的 Elevation Mapping 快照；检查有效层、方差、地图边界和数据时效。
2. **TerrainGeometryBuilder**（无）：在地图误差上界及足端几何包络下生成保守地形高度上包络、凸障碍分解和时空安全走廊。
3. **SwingReferenceGenerator**（部分有）：生成三维五次 B-Spline 参考轨迹、解析边界条件与运动学约束。
4. **PiecewisePolynomialExtractor**（无）：精确提取每个非零节点区间对应的多项式；支持节点插入与后续自适应细分。
5. **MinvoBasisTransform**（无）：对匹配的次数/区间实施 MINVO 基变换，提供控制点、凸包与数值一致性检验；没有已验证的高阶变换时以 Bézier 凸包为保守基线。
6. **ContinuousCollisionCertifier**（需要找）：依据地图保守几何和足端尺寸判定 `PROVEN_SAFE`、`COLLISION_DETECTED`、`UNKNOWN`，不能将未知区域自动视为安全。
7. **MinvoTrajectoryOptimizer**（无）：构建跟踪、平滑、端点、运动限制及 MINVO 走廊约束，在固定走廊下使用 QP 求解并重新认证。
8. **SafetyPlanManager**（应该有）：保存参考轨迹及其地图版本、安全走廊、有效时间和证明裕量；事件触发重规划、检查切换连续性、在提交前复检。
9. **HOCBFSafetyFilter**（无）：输入实际足端状态、名义控制量与当前有效安全集合；构建高阶屏障约束并输出可实现的安全命令或不可行状态。
10. **RobotCommandInterface**（有）：将安全命令映射到腿部运动学/全身控制框架，并监测执行误差及控制频率。

### 8.2 推荐的数据接口

```text
TerrainSnapshot:
  map_version, timestamp, frame_id, resolution
  elevation, uncertainty, validity, conservative_upper_bounds

SwingTrajectoryRequest:
  swing_start/end, initial_foot_state, target_contact_state
  foothold, nominal_spline, kinematic_limits

GeometrySafetyCertificate:
  result: PROVEN_SAFE | COLLISION_DETECTED | UNKNOWN
  map_version, valid_time_range, conservative_corridors
  clearance_margin, foot_geometry, error_budget

OptimizedSwingPlan:
  spline, derivative_access, valid_time_range
  terminal_conditions, geometry_certificate

HOCBFInput:
  measured_foot_position/velocity, nominal_acceleration
  active_safe_region, map_timestamp, control_limits

HOCBFOutput:
  safe_acceleration_or_equivalent_command
  feasible, minimum_h, minimum_psi1, solver_status
```

### 8.3 规划层实现顺序

1. 统一地图和足端状态坐标系，并从高程图创建只读快照。
2. 对参考轨迹进行离散快速风险预筛；预筛不能替代连续安全认证。
3. 提取 B-Spline 的精确多项式区间，将各区间转为 Bézier 或经验证的 MINVO 控制点。
4. 构造包含地形不确定性、足端尺寸和跟踪误差预算的保守障碍模型。
5. 对所有区间进行几何证书检查；安全则输出参考轨迹与证书，碰撞或未知则进入优化。
6. 在当前时间窗口内构造局部安全走廊，施加 MINVO 控制点不等式及端点、速度、加速度、运动学条件。
7. 求解 QP；对候选轨迹再次进行连续时间碰撞认证，并在最新地图快照中复检。
8. 通过证书后原子提交；不可行或未知时触发落脚点/摆动时序调整及安全恢复，不提交未经验证的轨迹。

### 8.4 跟踪层实现顺序

1. 每个控制周期获取实际足端位置、速度和接触模式，查询当前地图及有效安全走廊。
2. 依据名义参考加速度和足端跟踪误差计算名义控制指令。
3. 根据光滑地形屏障或局部平面屏障计算 $h$、$\psi_1$ 及对应 HOCBF 约束。
4. 校验初始安全条件、走廊/地图版本、控制输入可实现集合和数值有效性。
5. 求解 HOCBF-QP，将安全输出转换为可执行的关节或全身控制命令。
6. 监测跟踪误差、QP 可行性和采样延迟；不满足条件时按照上层安全协议执行恢复动作。

**接口设计要点：** MINVO 规划层输出的几何安全证书并不自动成为 HOCBF 的动态安全证明。二者通过统一的安全走廊、地图版本、时间区间、跟踪误差预算以及控制可行域建立可审查的安全契约。

---

## 9. 验证计划与消融实验

### 9.1 单元测试（优先级最高）

| 测试 | 输入 | 必须验证的输出 |
|---|---|---|
| 坐标系转换 | 不同 TF 和地图平移 | 足端与地图在同一系、时间戳一致 |
| 地图有效性 | NaN、边界、老旧地图 | `UNKNOWN` 而非误判安全 |
| 几何上界 | 台阶边、斜坡、噪声 | 上包络不低估已知合成真值 |
| 分段多项式 | 五次、非均匀节点、端点 | 与原样条在全区间数值一致 |
| MINVO 转换 | 随机五次多项式 | 轨迹重构一致、基非负且求和为 1 |
| 连续碰撞 | 两采样点之间细小障碍 | 采样漏检可复现、连续证书不误判 |
| QP | 窄走廊/不可行终端 | 正确区分成功、不可行、未知 |
| HOCBF | 平面、斜坡、曲面 | 不等式符号正确、初始集条件检查 |
| 末端接触 | 坡面/台阶落脚 | 不将合法接触判成永久不可行 |
| 地图版本 | 求解过程中地图更新 | 必须复检后提交 |

### 9.2 对比组

| 编号 | 规划 | 安全跟踪 | 目的 |
|---|---|---|---|
| B0 | 原始 B-Spline | 常规跟踪控制 | 原始基线 |
| B1 | 离散采样地形 QP | 常规跟踪控制 | 对照采样间隔漏检 |
| B2 | 分段 Bézier 走廊 QP | 常规跟踪控制 | 连续时间约束基线 |
| B3 | 分段 MINVO 走廊 QP | 常规跟踪控制 | MINVO 独立收益 |
| B4 | 分段 MINVO 走廊 QP | HOCBF | 安全跟踪独立收益 |
| B5 | 地形自适应 MINVO | HOCBF | 全方法收益 |

**公平条件**：相同参考轨迹、地形地图、足端包络、安全裕量、终端条件、QP 时间预算和随机地形。分段/优化变量数量不同必须单独报告。

### 9.3 场景与指标

场景：高度突然变化的单台阶、不同间距凸起、狭窄走廊、斜坡/凹凸、悬空障碍（验证 2.5D 局限）、摆动中途新地图、地图局部空洞、足端跟踪扰动、终端落脚点附近障碍。

关键指标：连续时间碰撞率（高频仿真 + 必要的几何认证）、最小净空、规划成功率、平均/P95/P99 耗时、最大抬脚高度、轨迹长度、jerk、终端位置/速度误差、关节限位违反率、HOCBF 干预范数 $\|u_{safe}-u_{nom}\|$、QP 不可行率、地图失效触发率。

仅用凸包体积比不能直接证明实质运动收益。若只优化 z，三维凸包可能退化，应该比较净空保守性、可行率和优化开销，而不是不适当比较三维体积。

### 9.4 可检验的研究假设

- H1：在相同几何安全要求下，MINVO 约束比同分段 Bézier 约束允许更多有效避障轨迹（需要验证，不保证必然成立）。
- H2：地形风险自适应分段相较固定高密度分段，以更低计算成本达到类似安全认证率。
- H3：存在跟踪误差/扰动时，HOCBF 安全接口能够降低实际足端安全违反，但应同时报告不可行率与跟踪/落脚性能代价。
- H4：使用剩余摆动时间的事件触发重规划，在地图更新场景提高成功率，且不破坏真实接触时序。

---

## 10. 研究创新性边界与推荐论文表述

**不宜作为独立创新的表述**：“首次将 CBF 和 MINVO 结合”“MINVO 能保证连续曲线不碰撞”“Elevation Mapping 获取局部高度”。MINVO/MADER、HOCBF 与地形感知足端规划均有已有工作。

更适合形成研究贡献的方向：

1. **地形不确定性驱动的安全区域构造与自适应分段**：解决离散高度图到连续时间多项式凸包的保守映射及实时性问题。
2. **规划-跟踪统一安全契约**：明确参考 MINVO 轨迹安全证书、地图版本、接触相位及 HOCBF 可行域的接口关系，分析安全切换条件。
3. **剩余摆动时域内安全可行重规划**：解决已执行轨迹、终端落脚时间和安全约束的相互冲突，并对不可行情况给出可验证的上层协调机制。

推荐论文方法名（工作名）：**Terrain-Aware MINVO Trajectory Planning with HOCBF Safety Filtering for Quadruped Swing Motion**。不要在未完成相关文献系统检索前宣称首次提出。

---

## 11. 研发里程碑（按依赖关系）

- **M0：定义算法接口与假设**：明确三维参考轨迹、动力学模型、足端包络、接触时序、优化变量、控制可实现集合与安全证明范围。
- **M1：TerrainSnapshot**：能读取 `elevation`、可用的 `variance` / `upper_bound`、NaN、TF、时间戳，并可记录重放。
- **M2：连续时间几何基线**：五次 B-Spline → 精确分段 Bézier → 障碍/地形保守上界 → `PROVEN_SAFE/UNKNOWN`；优先用合成台阶做测试。
- **M3：MINVO-QP**：验证五次 MINVO 基、插入 QP 线性约束、规划后再认证；与 Bézier 对比。
- **M4：事件触发重规划**：实现剩余摆动轨迹优化、状态连续切换、地图版本复检与失败恢复策略。
- **M5：HOCBF 平面安全过滤**：在任务空间可实现的控制接口上应用局部平面走廊；验证模型、可行域与控制命令转换。
- **M6：鲁棒性与实机**：加入局部曲面 HOCBF、输入误差/采样裕量，系统消融与实机实验。

> 最小可行原型：**M0–M3**，用于比较离散地形约束、Bézier 凸包和 MINVO 凸包的几何保守性与规划性能。**M5** 才引入明确动力学模型下的 HOCBF，并检验动态安全前提。

---

## 12. 关键参考资料（已核对的公开来源）

1. Fankhauser P. et al. Robot-Centric Elevation Mapping with Uncertainty Estimates, CLAWAR 2014. 官方项目：https://github.com/ANYbotics/elevation_mapping 。该仓库给出原始/融合地图话题、层与地形不确定性背景。
2. ANYbotics, `grid_map`: https://github.com/ANYbotics/grid_map 。多层二维栅格地图、圆环缓冲与 C++ 地图访问。
3. Tordesillas J., How J. P. MINVO Basis: Finding Simplexes with Minimum Volume Enclosing Polynomial Curves. *Computer-Aided Design*, 151, 103341, 2022. 官方仓库：https://github.com/mit-acl/minvo ；论文：https://www.sciencedirect.com/science/article/pii/S0010448522000975 。
4. Tordesillas J., How J. P. MADER: Trajectory Planner in Multiagent and Dynamic Environments. *IEEE Transactions on Robotics*, 38(1), 463–476, 2022. 项目：https://acl.mit.edu/projects/multi-agent-trajectory-planning ；论文 DOI：10.1109/TRO.2021.3080235 。
5. Xiao W., Belta C. High-Order Control Barrier Functions. *IEEE Transactions on Automatic Control*, 67(7), 3655–3662, 2022. DOI：10.1109/TAC.2021.3105491 ；https://ieeexplore.ieee.org/document/9516971/ 。


---

## 附录 A：开发前必须回答的问题

1. 轨迹优化采用三维控制点还是固定水平路径上的单自由度抬脚高度？HOCBF 层采用什么动态模型与控制输入？
2. 三维 B-Spline 是否统一时间参数化？能否精确提取分段多项式及其导数？
3. 实际 elevation map 的 layer 列表、frame、时间戳、分辨率和方差字段是什么？
4. 地形里是否存在 2.5D 无法表示的负障碍/悬空障碍？
5. 足端姿态/几何尺寸是什么？脚掌是球形近似还是凸多面体？
6. 目标机器人控制架构能够在哪个接口实现任务空间安全加速度、关节加速度或力矩修正？
7. 摆动相可否延长、可否临时更换落脚点？这些权限决定不可行问题能否正确处理。
8. 地图更新、QP 重规划、HOCBF 控制环的真实频率与延迟是多少？

## 附录 B：几个常见理论错误（论文写作自检）

- **错误**：离散采样点没碰撞，所以曲线连续时间安全。**改正**：必须提供覆盖整段的凸包/区间证明或经验证的连续碰撞检测。
- **错误**：MINVO 控制点在地形表面上方，必然足端整个球体安全。**改正**：还需要明确空间扫掠范围、脚体半径和保守地形模型。
- **错误**：对离散 elevation map 直接计算梯度与 Hessian 就能应用二阶 HOCBF。**改正**：构造 $C^2$ 且保守的局部高度函数，或使用局部平面走廊 HOCBF。
- **错误**：给轨迹 QP 加几何高度约束就叫 CBF-QP。**改正**：CBF 必须有状态演化模型和屏障导数不等式。
- **错误**：只满足 $h\ge0$ 就可以保证二阶 HOCBF 永远安全。**改正**：需要 $\psi_1\ge0$ 以及持续可行性等前提。
- **错误**：HOCBF 在每个采样时刻满足就自动得到任意时刻安全。**改正**：需要 sampled-data 鲁棒性与延迟分析。
- **错误**：摆动末端还能坚持正离地高度同时完成接触。**改正**：接触切换需独立终端安全条件。
- **错误**：优化失败就使用原始可能碰撞的轨迹。**改正**：触发上层安全恢复/重选落脚点/步态调整。
