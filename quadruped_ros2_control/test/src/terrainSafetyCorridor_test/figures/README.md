# 论文图表、图注与源数据

2026-11-18。所有曲线和计数来自本工作区生产模块导出或保留的 Gazebo 数据。使用现有 Python/NumPy/Matplotlib，不使用示意值代替测量。PDF/SVG 保留文字，PNG 为 600 dpi。双栏图宽 182.88 mm，单栏标定图宽 91.44 mm；投稿前按目标期刊版心放置并复查实际字高。

## 证据链与可直接使用的图注

### 1. 几何下界、规划收缩与轨迹

文件：`corridor_constraint.pdf/svg/png`。

**图注建议：** 分段地形安全走廊及足端球心轨迹。(a) 横障场景的 XZ 截面；灰色区域为几何走廊截面，蓝色区域为收缩跟踪裕量后的规划走廊，灰色实心块为真实障碍。(b) 同一轨迹在归一化摆动相中的高度。橙色虚线为名义参考，蓝色实线为生产 QP 优化结果。规划集与几何集相差 20 mm 位置误差预算，顶面由每段参考凸包扩展 80 mm 后收缩得到。显示范围内的各单元边界均来自实际导出半空间。

源：`../data/final/bar_{planes,trajectory,terrain,vertices}.csv`，调用生产走廊与 QP。轨迹为球心 `p_ref+0.019 e_z`，不是接触面高度。独立包非接触区间为 `[0.1,0.9]`；图中起落脚曲线仍显示完整 `[0,1]`，无走廊阴影部分不具有常量误差球证书。该 QP 同时受既有地形、小腿、工作空间及平滑目标影响，本图展示约束满足，不能单独量化走廊带来的因果增益。

### 2. 连续约束的漏检消除与 MINVO 下界紧致性

文件：`minvo_comparison.pdf/svg/png`。

**图注建议：** 离散配点与连续凸包约束的对比。(a) 27 个确定性障碍几何中，离散方法得到 22 个解，其中 6 个仍发生碰撞；Bézier 与 MINVO 各得到 18 个经过独立碰撞认证的解，其余 9 个报告未求解。堆叠柱分别显示认证通过、未认证及未求解数量。(b) 对同一解析曲线族，MINVO 投影下界相对真实最小值的间隙为 29.1906 mm，Bézier 为 50 mm；该曲线族的间隙缩小 20.8094 mm。

源：`../data/final/minvo/metrics.csv` 和 `hull_bounds.csv`；生产 `minvo_planning_test` 生成。几何为高度 `[0.06,0.10,0.14] m` ×宽度 `[0.002,0.010,0.040] m` ×中心 `[0.217,0.253,0.289] m`。每方法计时重复 5 次，分类仅使用 repeat=0；重复计时不增加几何样本量。未认证的 6 个解均有负净空。该基准未显示 MINVO 相对 Bézier 的成功数优势；41.62% 是 (b) 特定曲线族的下界间隙改善，不是任意曲线的统一比例。此处合成地形不等同于 Gazebo 楼梯。

### 3. 感知高程标定

文件：`map_calibration.pdf/svg/png`；数字明细 `calibration_source.json`。

**图注建议：** 仿真感知链修正前后的地图高度偏差。在 ROS 仿真时间 5–12 s 内，对两轮共同可见的三个踏面采样位置计算地图高程相对真实平面的中位误差。旧链路在 100、200、300 mm 踏面上的误差为 9.49、16.21、27.35 mm，修正后为 −0.90、0.002、0.008 mm。修正包含一致的相机投影和取消经验高度补偿。

源：`../data/stability/stairs_map_91_map.csv` 与 `stairs_camera_96_map.csv`。每帧站点先对 3×3 局部有效栅格取中位数，再在该时间窗聚合；旧/新每站点分别 14/9 帧。仅画两轮共同可见站点，位置与可见范围不同。这里只展示确定性仿真标定诊断，不将帧数当独立重复，不画伪造的置信区间，也不是单独相机参数的消融。

### 4. 计划支撑与真实接触诊断

文件：`support_diagnosis.pdf/svg/png`；选取事件与统计 `support_source.json`。

**图注建议：** 右前腿在第二级踏面上的支撑建立诊断。(a,c) 旧失败记录；(b,d) 最终配置记录。时间零点为 RF 首次在该踏面进入计划支撑相，阴影为随后 0.35 s 的诊断窗口。上排为估计足端中心距真实踏面加半径的高度差，下排为实际足端传感器力。旧记录在计划支撑后仍离地且几乎不承载；最终配置在该事件后建立有效接触。

源：`../data/stability/stairs_repeat_89_wbc.csv`、`stairs_final_104_wbc.csv`。确定性选取规则：RF 进入 mode=6 的首个切换，且估计 RF x∈(2.55,2.85) m；展示 `[-0.10,+0.35] s` 内全部保留样本，不平滑。第二级面 0.20 m，加球半径 0.019 m；足端位置来自状态估计下的运动学，力来自传感器，不是 WBC 优化力。事件时间分别 27.676/27.216 s；支撑窗口中位 gap 为 32.74/−0.19 mm，中位力 0.68/28.50 N。它诊断缺少支撑，没有证明一切失败均由冲击引起；旧新配置多项不同，不能作为单变量消融。

### 5. 楼梯与左平台重复通过

文件：`closed_loop_validation.pdf/svg/png`。

**图注建议：** 最终配置在 10 cm 七级楼梯和 12.5 cm 左侧平台上的独立重复测试。上排为机身高度，下排为 roll 随前进距离的变化。每条线对应一次新启动测试，三次楼梯与三次平台均满足四足到达目标面并稳定保持 3 s 的成功判据。平台测试在前进前使用 trot 完成左转 90°。曲线取 Gazebo 真值，不平滑。

源：`../data/final/stability_summary.json` 选定最终六轮，以及 `../data/stability/<trial>.csv`。楼梯 Trial 1/2/3 对应 103/104/106，平台对应 105/107/108。绘制全部 forward/hold 阶段，初始化、站立和原地转向不在横轴前进曲线内；整轮姿态峰值仍保留在汇总 CSV。每次新启动是实验单位，n=3/场景；这是当前路线和速度的观察结果，不是总体通过率保证。结果联合反映感知、规划与控制配置，不是 MINVO 或走廊的单独消融。全部失败历史另列，未丢弃后以历史总试验声称六次全通过。

## 统计定义和认证范围

汇总的足端 RMS 是四足运动学位置相对 MPC desired foot 的三维跟踪 RMS，不是对 MINVO 曲线直接求误差。接触事件定义为 sensor force 上穿 5 N，下降速度取前 35 ms 最大向下速度，力峰值取后 40 ms；采样间隔不均匀，均使用真实时间窗。

最终六轮 47,206 次时域规划包含 10,752 次名义回退（22.8%）。回退不带走廊认证；在线常量 2 cm 跟踪球只覆盖 `[0.25,0.75]`。HOCBF 约束已导出但未进入 WBC。图表不构成整个执行轨迹、接触切换或全身动力学的形式化安全证明。

## 复现

从工作区根目录、已加载 ROS 环境的终端运行：

```bash
CORRIDOR_TEST=src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CORRIDOR_TEST/analyze_stability.py" \
  "$CORRIDOR_TEST/data/stability" --output "$CORRIDOR_TEST/data/final" \
  --final-trials stairs_capped_103 stairs_final_104 stairs_final_106 \
    platform_capped_105 platform_final_107 platform_final_108
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/terrain_matplotlib /usr/bin/python3 \
  "$CORRIDOR_TEST/plot_evidence.py" "$CORRIDOR_TEST/data/final" --output "$CORRIDOR_TEST/figures" \
  --audit-tools /home/lee/.codex/skills/nature-figure/scripts
```

`--audit-tools` 是可选的本机制图 QA 工具路径，不是控制器运行依赖。几何和 MINVO CSV 再生成命令见根目录 `QUICK_START.md`。

## 图表 QA

- 已逐图、逐面板检查全部五组 PNG；修正了图例裁剪和诊断曲线纵轴裁剪；最终窗口内保留全部数据。
- 四组多面板图的最终物理对齐审计通过，容差 1.5 pt，详见 `*.alignment.json`。单面板标定图不适用。
- 源码检查 0 FAIL；唯一 WARN 是没有 TIFF。交付格式为 PDF、SVG 和 600 dpi PNG，不声称已满足某特定期刊的 TIFF 提交要求。
- 五份最终 PDF 的文字审计均通过（最小字高 6.5–7 pt），日志在 `qa/*.text.json`。
- PDF 自动碰撞审计未执行成功：当前系统及已提供的 Python 运行时均缺 PyMuPDF，返回 NOT AUDITABLE / exit 2。按 [nature-figure 技能](/home/lee/.codex/skills/nature-figure/SKILL.md) 的要求 “NOT AUDITABLE … report the dependency/PDF blocker and do not claim collision validation”，保留 `qa/*.collision.log` 与 `qa/audit_status.json`，不标记自动碰撞审计通过。未为图表 QA 改动项目依赖环境。
