# MINVO 首次 trot 不可行核查

本轮暂时取消 MPC 的 `_footPlacementCbf` soft constraint 注册；保留普通落脚区域、足端碰撞代价及摆动 QP 内的 CBF/MINVO。未修改 `useCbfUpdate=false` 的轨迹生成路径，也未修改 task.info 开关、增益或关节限位。

## 数值证据与修正

复现日志：工作区 `log/minvo_trot_active_rows.log`。按 `2`、`3`，尚未发送 `w` 即失败：MINVO 起始行 1724 的名义残差约 -3.41447e-6 m，自由控制点行范数约 9.9592e-6，随后迭代将修正量放大并与工作空间约束冲突。status=37 时仍有 183 次预算，不是迭代预算耗尽。

MINVO 基函数没有 B 样条钳位端点的单点插值性质。直接要求首个 MINVO 凸包的所有顶点高于地形，会对安全的接触曲线产生过强条件。原端点因子分解只在端点与地形下界近似相等时启用，微米级正净空就会绕过它。

现在首尾子段先确认固定端点满足该子段地形下界，然后对 `z(s)-z_endpoint` 作端点因式分解，对商多项式施加 MINVO 非负证书。该条件仍是充分条件；没有移除首尾子段避障或放松几何约束。商多项式最高四次，使用 degree-4 MINVO；原五次曲线仍使用 degree-5 MINVO。

同时修正两处不一致：接触配准改用与凸包一致的球体地形上界；端点多项式变换同时变换常数下界，避免非零世界高程下错误约束。

接触配准本来就允许地图与接触面注册偏差。因此几何证书针对配准后的高度场，不能解释为对真实地形的无误差保证。

## 验证范围

构建 `ocs2_quadruped_controller` 和 `test`，运行生产实现的 `swing_qp_regression`。新增零/正/负世界高程、3 mm 地图邻域起伏、1.5 μm 正端点净空的回归；执行中摆动仍启用 CBF。CBF 关闭回归对上下台阶与平地逐点比较高度和速度，并确认不访问地形或 IK workspace。

在 `stair10cm`、`useCbfUpdate=true`、真实 keyboard_input 的四次独立运行中，完成 `2→3→单次w`，未出现 QP failure 或 MPC error：

| 工作区 log 前缀 | 最后仿真时刻 | 最后机身 x | 说明 |
| --- | ---: | ---: | --- |
| minvo_trot_workspace_diag | 40.771 s | -0.610 m | 常规切换 |
| minvo_trot_workspace_retry | 47.981 s | -0.905 m | 常规切换 |
| minvo_trot_workspace_third | 34.621 s | -0.469 m | 常规切换 |
| minvo_trot_fast_switch | 31.971 s | -0.534 m | 站立后约 1 s 切换 trot |

每个前缀均有 `.log`、`_pose.jsonl`、`_truth.jsonl`。上述运行证明这四次切换不再触发报告的停机，**不证明前进跟踪正确**：机器人在单次 w 后向后漂移，尚未通过正常行走和楼梯验收。

另一次较早运行 `minvo_trot_endpoint_fix` 暴露起跳附近 workspace 下界高于固定端点约 1.7 mm；后续四次未重现，但没有专门修复未来机身预测/落脚点的 IK 不一致，不能宣称所有初始状态下均可行。失败诊断保留 required/nominal/free_norm、MINVO 起始索引及 workspace 截面，便于继续核查。
