# 接触时序论文实验索引（2026-11-20）

当前状态：六个实验包、离线扫描和最新10cm物理对照已完成，当前侧走6/6通过；在线执行收益尚未验收。
所有负向结果和未触发试验均保留；“有功能包/有图”不等于实验主张已验收。
使用 Python + nature-figure 导出，按 engineering-guardrails 复用原试验启动、地图干预及几何分析代码。

| 独立ROS2包 | 本轮数据 | 可支持的结论 | 验收边界 |
|---|---|---|---|
| [lateral_stability_test](lateral_stability_test/README.md) | 当前版本左右各3次，6/6通过 | 修正水平指令中的倾斜/竖直耦合；当前1m横移+10s保持协议可复现 | 有限路线/速度回归，不能外推为所有侧向运动稳定保证 |
| [swing_update_comparison_test](swing_update_comparison_test/README.md) | 四方法225配对、27场景族6075条件；多版本显露诊断 | 固定准入62/225、修复碰撞6/90；27场景族当前修复120/1647；局部可变期限修复12/90 | 已运行同步显露，但完整准入目标激活收益未验收；局部可变期限未同步改全身接触计划 |
| [swing_contact_timing_test](swing_contact_timing_test/README.md) | 当前10cm障碍3配置×3种子，通过1/3、2/3、1/3，全部零换轨 | 可记录真实几何确认接触及计划误差，不能由参考终端条件替代物理触地 | 缺失率均衡、可变时序全身对照、统计重复 |
| [swing_update_boundary_test](swing_update_boundary_test/README.md) | 120状态条件；225高度/时间条件 | 高度热力图明确分开准入比例与原轨迹碰撞修复；生成有限场景repair_boundary.csv | 更大范围可比较需求参数、执行层成功率与拒绝原因细分 |
| [swing_whole_body_test](swing_whole_body_test/README.md) | 逐文件SHA复用同9次物理试验，新增独立试验0 | 模型CoM、姿态、求解支撑力、跨障和效率有完整测量 | 当前无稳定性能增益，不支持固定时序减少全身扰动 |
| [swing_continuous_safety_test](swing_continuous_safety_test/README.md) | 180相同走廊配对，另有11/21/41控制点匹配QP | 41控制点、3点离散检查有33个球体碰撞；MINVO连续组146个接受解密集检查无碰撞 | 连续认证对离散漏检已有正证据；仍不支持MINVO比Bernstein更少保守 |

各包 `data/` 保存原始数据及派生表，`figure/` 保存图。
当前控制器SHA256前缀 `d1f069055aeb`；7961–7963侧走6/6通过，滚转峰值0.02483–0.03480rad。旧版 `5f1b423022da` 的7901–7903结果单独保留。状态偏移扫描当前27/120准入，旧版26/120；采集manifest不会在重绘时改写。
历次710x/750x/770x/780x及失败均按版本保留。每轮manifest为二进制版本依据，不能跨版本合并通过率。
全身包通过逐文件SHA256显式复用时序包数据，不能叠加独立样本量。
种子固定不等于ROS异步调度完全确定。

## 入口

工作区先source ROS与install。各包 `run_experiment.py` 默认三种子，支持`--analyze-only`和`--domain`，不会自行构建。
安装后使用 `ros2 run <包名> run_experiment.py`；源码更新尚未安装时，使用 `python3 src/quadruped_ros2_control/test/<包名>/run_experiment.py`。
显露使用comparison包的`run_reveal.py`；现有二进制状态扫描使用boundary包的`run_state_boundary.py`。
重采以新种子保存新轮次；确定性CSV存在时程序拒绝覆盖，须先完整保留旧批次。

用户先授权构建两个实验包，后明确授权构建 `ocs2_quadruped_controller` 与这两个实验包。实际构建、自检、扫描日志在 `log/paper_experiments_20261120/`。
控制器局部改进：候选曲线首次按走廊边界精确细分；激活状态偏差门槛与轨迹QP修正额度解耦，前者仍4cm、后者复用terrainMaxCorrection；重复更新避免再次按地图细分以限制规模。
同步显露的测试标记与地图一起提交，未带标记的正常地图沿用配置；地形订阅保留最新一个快照，减少历史地图积压。
旧生产扫描、逐步诊断及新版数据分别保留，不混合二进制版本。最新哈希以每轮manifest为准。

## 验证与下一步

Python接触与几何确认、现有地图/延迟分析测试可用：

```bash
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/quadruped_mpl python3 -m unittest discover \
  -s src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/test -p 'test_*.py'
```

基线与新增图均保存plot-area对齐、PDF字形及碰撞审查结果；新增多控制点比较、碰撞修复与双面板高度热力图已通过审查。
共享绘图代码使用实际源码拼接副本做静态检查，见 `log/paper_experiments_20261120/figure_source_audit_bundle.py`；副本不作为运行源。
未指定期刊，导出约183 mm宽、600dpiPNG及编辑型PDF/SVG，无TIFF；每轮点直接呈现、不画伪重复置信区间。

旧版5f1b423022da物理时序/全身对照7901–7903已完成：Frozen/Online/Online+filter分别通过1/3、2/3、0/3；完整过滤组两轮摔倒、一轮终点不稳，九轮实际换轨均0。其汇总/图按 `qp_final_7901_7903/` 归档。同步显露目标摆动激活验证继续单独审查，不能将静态已知障碍路线通过替代在线更新成功。
当前d1f069055aeb的7961–7963批次三组分别通过1/3、2/3、1/3，全部零换轨。每轮几何确认接触22–142个，接触绝对误差中位数10–15ms、P95为20–60.5ms；通过轮次模型CoM RMS为2.34–2.51mm。缺失覆盖与任务长度不同，不据此认定方法优越。时序/全身包均保存 `workspace_derivative_7961_7963/` 快照。
同一控制器7971低速显露诊断覆盖7/8/10cm×三方法九轮，准入组目标摆动仍0/3激活。7/8cm候选实际传输50/35ms后速度偏差1.171/0.945m/s，超过原0.40m/s门槛；直接组两次实际换轨均未完成目标LF跨障。详见comparison包带 `_v0.4` 的原始数据、summary与图。该结果说明当前缺口在规划—实际激活之间，不能用离线QP准入数补足。
7981小障碍2/4/5cm九轮全部路线通过、真实跨障；准入组目标激活仍0/3。7991的软代价缩放尝试仍零目标激活，已撤回并恢复原源码与二进制SHA。预设摆动时长.6/.8s的8001/8002共九轮均摔倒，含两轮显露前失败；每轮固定接触计划，不属于可变触地方法B。全部配置、失败与图见comparison包，不跨版本/速度/步态合并样本。

当前最关键的未完成项是未来接续与策略交付时间对齐：候选从规划初始时刻改变参考，策略实际交付前仍执行旧参考。已实现并实测测量Hermite前缀、旧执行参考前缀、固定前缀变量消元及独立在线走廊细分，8011–8042六批共54轮均未产生准入组目标摆动实际激活收益。这些实验修改已撤回，恢复已验证的`d1f069055aeb`源码与配置；详见[传输诊断](swing_update_comparison_test/TRANSPORT_DIAGNOSTICS.md)。独立扫描也确认传输预算压缩可行更新范围。除此以外，还缺全身可变触地对照，MINVO比Bernstein减少保守性的主张也没有正证据。不能将这些项标成论文实验验收完成。
10cm末期未知障碍本身可能不可行；拒绝更新能保证不接受坏候选，但旧参考也可能撞障，尚需停止/改接触计划等明确后备策略才能声称任务级安全。
误差管与HOCBF的未闭合执行保证继续按用户要求暂不作为本轮完成条件，不能因此在论文中认定已证明。

最终恢复与验证：三个已授权包均完成CMake目标构建与安装，控制器安装库SHA与上述`d1f069055aeb`完全一致；18个已记录的运行时C++源码/头文件逐项匹配。两项C++自检、实际TargetManager指令回归通过，Python测试26项通过、1项跳过。安装扫描器900行、连续比较360行、历史传输重放900行，排除耗时列后均匹配各自原始结果，独立样本数未增加。63张根目录图通过PDF字形和碰撞审查，多面板对齐通过，单面板不适用；新增8041/8042图均已逐面板目视检查。详见`log/paper_experiments_20261120/final_validation.json`。
