# 六张实验图

图1/3/4/5：生产算法离线验证，尚未取得整机同次摆动换轨成功；图2/6：最终版本Gazebo实测。详细数据口径与结论边界见 ../../VALIDATION.md。每图有PDF、SVG、600dpi PNG和caption.txt；图的源数据在 ../../data/analysis/final/。

- [图1 摆动中途轨迹更新](fig1_trajectory_update.png)：离线旧轨迹碰撞、新轨迹正净空，固定终端0.4s。
- [图2 固定触地时序](fig2_fixed_touchdown_timing.png)：两个独立启动内全部合格接触观测，无可变触地对照。
- [图3 连续时间碰撞检查](fig3_continuous_safety.png)：稀疏点遗漏碰撞，生产MINVO行独立复核。
- [图4 终端边界保持](fig4_terminal_boundary.png)：参考z/vz/az；原参考不是无约束消融。
- [图5 激活边界接续](fig5_state_splicing.png)：受控边界、未对齐候选与实例化结果。
- [图6 参考与实际执行](fig6_reference_execution.png)：真实诊断，当前未满足正净空和20mm跟踪预算。

QA：所有最终PDF文字与碰撞审查通过，多面板对齐门通过；逐图人工检查完整、无遮挡。白底、蓝橙绿、虚线原参考、淡网格、p/v/a纵向面板沿用提供样例。数据曲线没有平滑或艺术修饰。图6保留原生采样折线，不补出理想执行效果。

唯一源代码审查提示为PNG没有TIFF；本交付是矢量线图，PDF/SVG为主文件，PNG仅预览，因此保留该提示。QA报告在qa/，总览图review_contact_sheet.png不作为单张论文主图。
