# RViz 截图显示与重跑记录（2026-11-18）

显示更新仅涉及颜色、透明度、线宽、文本与地图网格。绿色规划下界不透明、线宽3.5 mm；侧框降至0.6 mm、alpha=0.20；几何上框/侧框0.5 mm、alpha=0.18。移除全部文本 marker，保留两条轨迹、碰撞点和全部走廊边界。半空间和优化算法不变。

使用工作区当前 task/gait 重跑四场景九段摆动；当前 swingHeight=0.085 m（此前为0.075 m），这是工作区已有配置变更，本次未修改。全部走廊矩阵、2001点净空、端点p/v/a及连续碰撞检查通过，证书均为ProvenSafe。详细指标见summary.csv，源文件校验见source_manifest.json，日志见pipeline.log。Bar仍10 cm，柱台4段、楼梯3段，接触参考坐标及安全半径不变。

plot_corridor_sections.py 已生成每地形一张PNG/PDF/SVG及兼容corridor_constraint文件。二维图保留图例，只有RViz删除文字。面板对齐与PDF字号检查通过；自动PDF重叠审计因缺少PyMuPDF未完成。旧data/final与历史闭环数据已从当前工作区删除，本次不恢复这些文件，因此不重绘依赖历史闭环记录的其他证据图。

ROS MarkerArray订阅检查见rviz_marker_check.json，验证全部场景无文本、绿色下界样式、弱化侧框和各段贴地起落点。未声称进行GUI截图核验。
