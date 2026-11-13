# CBF/continuous safety perspective: verified literature (2026-11-09)

检索词组：swing + control barrier + foot trajectory；legged + barrier + obstacle；CBF sampled data；HOCBF；tracking error bound；Bernstein continuous collision safety。已按题名存在性检索，再打开原始 arXiv、作者 PDF 或机构 repository 核验题名作者与摘要。下列未声明数值结论。VERIFIED-A 表示打开全文核验，VERIFIED-B 表示打开原始摘要页核验。不是穷尽检索，也不能由未找到完全同机制推出“首次”。

## 最接近与安全落足相关

1. **Ruben Grandia, Andrew J. Taylor, Aaron D. Ames, Marco Hutter. Multi-Layered Safety for Legged Robots via Control Barrier Functions and Model Predictive Control. ICRA 2021.**
   - https://arxiv.org/abs/2011.00032 ; 作者全文 https://andrewjamestaylor.github.io/assets/paper_materials/grandia2021multi_layered_safety_for_legged_robots_via_control_barrier_functions_and_model_predictive_control/paper.pdf ; DOI https://doi.org/10.1109/ICRA48506.2021.9561510
   - VERIFIED-A。原文 Eq.13 对 swing feet 设置随时间收缩并收敛到落足区域的多面体安全集；Eq.14 在 kino-dynamic MPC 加一阶 CBF，Eq.15 WBC 中再加导数约束，基于全动力学实现高频追踪。ANYmal stepping stones 仿真和实机。
   - 很高新意威胁：“把 CBF 放入足式机器人 MPC/WBC，保证安全落足”不是新故事。差异：当前项目若是固定 XY 的 z-height spline 系数 QP，作用对象为参考曲线而非 full-order safety torque；目标为途中地形净空而非收缩落足多边形。
   - 原文重要限制：Eq.16 硬件摆动腿模型误差加积分补偿，作者明确 τ 可能不满足 Eq.15；提醒 formal assumptions 与硬件实际安全不同。
2. **Quan Nguyen, Ayonga Hereid, Koushil Sreenath, Jessy W. Grizzle, Aaron D. Ames. 3D Dynamic Walking on Stepping Stones with Control Barrier Functions. CDC 2016.**
   - 原文 https://mae.osu.edu/sites/default/files/2021-06/nguyen20163d.pdf ; 机构记录 https://authors.library.caltech.edu/records/kby2t-rma02 ; DOI https://doi.org/10.1109/CDC.2016.7798370
   - VERIFIED-A。CLF-QP + CBF-QP 控制 full nonlinear hybrid walking，约束步长步宽引导 swing foot 到离散落脚点；验证为 DURUS 数值模拟，不要写成实机实验。
   - 中高威胁：“CBF 修正摆动足轨迹”的宽泛表述早已存在；区别在途中碰撞净空/低维 spline 优化，不能靠换机器人就称新方法。
   - 元数据小冲突：Caltech 记录把 Quan 写为 Quang、作者顺序与PDF有差异，采用原PDF署名，不写页码以规避记录页码冲突。
3. **Charles Khazoom, Daniel Gonzalez-Diaz, Yanran Ding, Sangbae Kim. Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions. Humanoids 2022.**
   - https://arxiv.org/abs/2207.00692 ; MIT机构记录 https://dspace.mit.edu/entities/publication/1208949f-4e4a-4bf4-b909-fd2f83aa762a ; IEEE https://ieeexplore.ieee.org/abstract/document/10000235
   - VERIFIED-B（原始摘要+MIT记录）。CBF-WBC 对 MIT Humanoid full underactuated dynamics 下自碰撞避障，能修正不可行高层 footstep/swing trajectories；摘要说明 effectiveness validated in simulation。
   - 高机制威胁，低任务重合：对象是自碰撞，非地形净空；但“高层轨迹不安全，CBF 低层修正”的思路不是新。
4. **Lizhi Yang, Blake Werner, Massimiliano de Sa, Aaron D. Ames. CBF-RL: Safety Filtering Reinforcement Learning in Training with Control Barrier Functions. ICRA 2026（原页明确 accepted）；预印本首发 2025-10-16，v6 2026-06-22.**
   - https://arxiv.org/abs/2510.14959 ; 全文 https://arxiv.org/html/2510.14959v6
   - VERIFIED-A。训练时 CBF filter 与 barrier-inspired reward 双机制，在 G1 验证安全障碍避让/爬楼；部署时无 runtime filter。它是相邻学习路线，不是在线 spline QP。
   - 不能把“无 runtime filter”写成无条件真实全阶连续安全保证：全文声明重点为经验 safer behavior，Section II 离散安全主要在采样时刻、inter-sample 另引文讨论。可作为2026前沿参照而不是强行当作同任务基线。

## 安全保证与鲁棒性基础

5. **Aaron D. Ames, Xiangru Xu, Jessy W. Grizzle, Paulo Tabuada. Control Barrier Function Based Quadratic Programs for Safety Critical Systems. IEEE TAC 2017（预印本2016）.**
   - https://arxiv.org/abs/1609.06408 ; DOI https://doi.org/10.1109/TAC.2016.2638961
   - VERIFIED-B。CBF 前向不变性、CLF 性能与实时 QP 调和，汽车示例；不能将任意参考曲线上的有限配点不等式直接升级为闭环机器人安全定理。
6. **Joseph Breeden, Kunal Garg, Dimitra Panagou. Control Barrier Functions in Sampled-Data Systems. IEEE Control Systems Letters 6:367–372, 2022（online/预印本2021）.**
   - https://arxiv.org/abs/2103.03677 ; 作者PDF https://websites.umich.edu/~dpanagou/assets/documents/JBreeden_LCSS21.pdf ; DOI https://doi.org/10.1109/LCSYS.2021.3076127
   - VERIFIED-B。固定步长、分段常值控制的前向不变性充分条件，并使条件对控制仿射以用于 QP。当前 spline 的 collocation 并不等于它的 sampled-data controller，但它足以否定“离散样点满足 ⇒ 自动连续安全”的推理。
7. **Wei Xiao, Calin Belta. High-Order Control Barrier Functions. IEEE TAC 67(7):3655–3662, 2022（online2021）.**
   - 作者全文 https://calinbelta.com/wp-content/uploads/2023/12/High-Order_Control_Barrier_Functions.pdf ; DOI https://doi.org/10.1109/TAC.2021.3105491
   - VERIFIED-A。高相对阶安全约束、控制限制与可行性，HOCBF 递归安全集合。当前 spline coefficients 为决策变量，不因为曲线二阶导存在便可称 torque/acceleration HOCBF。若扩展真实加速度输入，要显式考虑初始高阶安全集合、力矩限幅、可行性。
   - 另有早期 CDC2019 **Control Barrier Functions for Systems with High Relative Degree**, arXiv https://arxiv.org/abs/1903.04706，别与上面TAC题名年份混写。
8. **Shishir Kolathaya, Aaron D. Ames. Input-to-State Safety With Control Barrier Functions. IEEE Control Systems Letters 3(1):108–113, 2019（预印本2018）.**
   - https://arxiv.org/abs/1803.03035 ; DOI https://doi.org/10.1109/LCSYS.2018.2853698
   - VERIFIED-B。输入扰动下允许安全集膨胀，偏离程度受扰动幅度界定。可为 tracking-error-aware safety margin 提供理论参考；原论文不是足端跟踪误差管算法。
9. **Sylvia L. Herbert, Mo Chen, SooJean Han, Somil Bansal, Jaime F. Fisac, Claire J. Tomlin. FaSTrack: A Modular Framework for Fast and Guaranteed Safe Motion Planning. CDC 2017.**
   - https://arxiv.org/abs/1703.07373 ; 机构记录 https://pure.kaist.ac.kr/en/publications/fastrack-a-modular-framework-for-fast-and-guaranteed-safe-motion-/ ; DOI https://doi.org/10.1109/CDC.2017.8263867
   - VERIFIED-B。简化规划动力学+安全追踪控制器+有保证的 tracking-error bound，考虑高阶动力学和扰动，示例为10D quadrotor/3D RRT。对“规划安全+跟踪误差膨胀”创新路线构成既有原理，因此新意应是接触混合、足端/连杆几何、在线误差界与deadline可行性的结合，而不是单独加固定裕度。

## 补充与排除

- **Magnus Gaertner, Marko Bjelonic, Farbod Farshidian, Marco Hutter. Collision-Free MPC for Legged Robots in Static and Dynamic Scenes. ICRA 2021.** https://arxiv.org/abs/2103.13987 ，VERIFIED-B。ESDF+moving cylinders+relaxed barrier cost+机器人动力学/摩擦/运动学；不要将其 relaxed barrier penalty 称 CBF 硬安全约束。摘要的whole-body表述不意味着覆盖每段摆动腿，须全文碰撞几何核验，未据搜索第三方“只躯干”描述下断言。
- **Online Terrain-Aware Bipedal Gait Generation via Manifold Projection and Optimization-Guided Motion Libraries**：只检得 Eurekamag 二手条目，声称 tightened DTCBF + swing foot trajectory optimization，但未拿到作者/DOI/原始出版社页面，按协议 UNVERIFIABLE，不进入正式参考文献。不因标题吻合判定重复；建议独立补检。
- Bernstein Polynomial-Based Method for Solving Optimal Trajectory Generation Problems（Sensors 2022,22(5),1869）检索发现，但 MDPI 429/PMC captcha，作者与全文未核验，本轮不作为已核验引用。连续时间凸包/多项式证书应由主代理其他来源补齐。

## 对创新与实验的建议（推论，不是论文原结论）

最值得检验的核心反事实：在同样 spline 参数化、同样加密地形高度约束与平滑目标下，加 CBF 项是否确实降低真实碰撞/跟踪误差，还是只对 already safe 曲线再加保守约束？必须以 geometry-only QP 为首要消融。若只提高峰值抬脚高度，先与自适应抬脚高度 baseline 比较。若不能优于这两者，停止把 CBF 本身作为主要新意。

可投稿的新贡献组合应聚焦：连续区间证书（非简单加密）+带跟踪/地图误差界的几何margin+触地终端/接触切换可行性。一阶指数 CBF hdot+gamma h>=0 从 h0>0 出发，只能下界指数衰减，因此固定正净空集不可在有限时刻达到h=0触地；应定义时变safe tube/terminal contact set并证明可行转换，而非简单关闭CBF后宣称全摆动安全。
