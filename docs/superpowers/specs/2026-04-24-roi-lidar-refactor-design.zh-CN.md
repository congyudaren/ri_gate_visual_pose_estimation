# ROI LiDAR 重构设计

日期：2026-04-24
状态：待用户评审草案
范围：重构 ROI 与 LiDAR 角点恢复链路，使 ROI 只提供结构 mask，LiDAR 负责在线恢复前表面角点

## 目标

将当前的 ROI 加角点管线重构为一条“结构优先”的管线：

- ROI 保留现有稳定的图像侧角点提取逻辑，但只把它用于生成结构 mask。
- LiDAR 负责在线恢复前表面几何。
- 主输出改为相机系下的前表面 4 个角点。
- Debug 输出仍然保留 2 个直接解算得到的上角点。

重构后的系统需要匹配真实传感条件：

- 目标是单个、几何已知的框架结构，
- 前表面近似正对传感器，
- 几何尺寸已知，只允许小范围误差，
- LiDAR 支持点稀疏且分布不均，
- `top_beam` 在初始化后允许阶段性失效，
- tracking 比一次性 batch 求解更重要。

## 背景

当前实现把以下职责混在一起：

- 图像侧角点查找，
- corner ROI 生成，
- lookback 点云累计，
- 每个角点的直接支持点聚合，
- 最终 3D 角点发布。

这套设计已经不再匹配新的方法方向。

目标物体是由两个 `1m x 1m x 1m` 框架上下叠放组成的单个结构。对于前表面：

- 总物理高度为 `2m`，
- 前表面宽度由已知框架几何固定，
- 仍然采用“前表面大致正对传感器”的先验，
- 允许小姿态误差，
- 当前相机相对常规图像语义是反装的，
- 当前相机与 LiDAR 外参已经对齐，不需要额外修正。

因此图像侧结构语义必须按物理含义定义，而不能按图像上下左右定义：

- `top_beam` 表示前表面的物理上横杆，
- `left_post` 和 `right_post` 表示物理左柱和右柱，
- 由于相机反装，这些物理语义不等于图像直观上的上/左/右。

场景假设也刻意保持收敛：

- 场景中只有一个目标框架，
- 不需要通用多目标 tracking，
- 模块内部可以做时序 tracking，但只需要维护一个目标状态。

## 范围内

- 保留现有稳定的 ROI 角点提取逻辑，作为图像侧几何先验。
- 将 ROI 输出从 `corner_rois` 退化为 `structure_rois`。
- 从图像角点导出 `left_post`、`right_post`、`top_beam` 三个结构 mask。
- 新增一个只负责点筛选与点集净化的 LiDAR 数据处理层。
- 新增一个三结构的在线 tracking 层。
- 新增一个恢复层，输出相机系下的前表面 4 个角点。
- 新增 2 个直接解算上角点的 debug 输出。
- 支持不依赖实时相机与雷达的离线验证。

## 范围外

- 不做完整物体的 6DoF 位姿求解。
- 不做通用多目标关联，也不引入泛化的 `track_id` 基础设施。
- 这一层不输出完整结构的 12 个角点。
- 不把完整 3D 框架 mesh 或显式边列表作为主 API。
- 方法验证不依赖实时同步采集的相机/LiDAR 数据。
- 不重写当前稳定的图像侧角点检测器。
- 不要求 ROI 产出三根结构的精确像素级语义分割。

## 选定方案

采用三段式“结构优先”设计：

1. ROI 从现有图像角点结果中导出 `left_post`、`right_post`、`top_beam` 三个结构 mask。
2. LiDAR 数据处理层将投影点落入这三个结构 mask 中，并完成点筛选。
3. LiDAR tracking 与恢复层在线更新三根结构状态，并在每帧从状态中恢复前表面角点。

### 为什么选这个方案

- 保留了当前已经稳定的图像侧部分，而不是推倒重来。
- 将几何恢复职责转移到用户希望负责它的 LiDAR 侧。
- 匹配真实可观测量：两根柱子的稀疏支持点，加上可选可失效的上横杆。
- 把 tracking 作为主逻辑，而不是 lookback window 的副作用。
- 主 API 保持小而清晰，同时保留足够的 debug 能力。

## ROI 层

### ROI 的职责

ROI 负责：

- 在图像中检测单个目标框架，
- 用现有稳定逻辑恢复图像侧 4 个角点，
- 将这些角点重映射为物理结构语义，
- 从这些角点导出 3 个结构 mask。

ROI 不负责：

- 以角点语义作为主输出，
- 决定最终 3D 角点位置，
- 做时序 tracking，
- 用 LiDAR 信息反向修正图像结果。

### ROI 内部方法

选定的方法是对当前稳定角点路径做“语义退化”，而不是替换：

1. 保留当前 `bbox -> 4 image corners` 逻辑。
2. 按物理框架语义重解释这些角点。
3. 将相关角点两两连线，得到 3 条物理结构线。
4. 对每条线按法向膨胀，生成结构 ROI mask。

这样做保留了稳定的图像侧部分，只改变它最终输出的语义层级。

### 物理语义重映射

如果当前角点顺序是图像语义的 `TL / TR / BL / BR`，那么在当前反装相机下，物理结构映射为：

- `top_beam = BL -> BR`
- `left_post = TR -> BR`
- `right_post = TL -> BL`

这个映射必须固定下来，保证后续所有阶段都按物理语义工作。

### ROI 输出

ROI 到 LiDAR 的接口应当被结构语义彻底替换。新的结构对象至少应包含：

- 目标 `bbox`，
- `left_post_mask`，
- `right_post_mask`，
- `top_beam_mask`，
- 每个结构各自的 `valid`，
- 可选的结构线端点，用于 debug 与可视化，
- 来源信息，例如该结构线来自 refined corners 还是 fallback corners。

旧的 corner-ROI 接口不再并行保留。

### ROI 有效性规则

ROI 应当宽松。

只要目标 `bbox` 存在，并且角点链路还能给出可用几何，三个结构 mask 就应当发出，即使它们比较粗。是否有足够支持点可以恢复 3D 角点，应由下游 LiDAR 阶段决定。

## LiDAR 数据处理层

### 职责

数据处理层只做点筛选，不做几何恢复，也不输出角点。

它的任务是把投影点证据变成更干净的三类结构点集：

- `left_post_points_filtered`
- `right_post_points_filtered`
- `top_beam_points_filtered`

### 输入

每帧输入包括：

- 当前 ROI 给出的结构 mask，
- 图像坐标系中的投影点证据，
- 深度值，
- 时间戳。

该层可以使用短时 lookback window，但这个窗口只服务于点证据收集。

### 方法

对每个结构独立执行：

1. 收集落在该结构 mask 内的原始投影点，
2. 估计前表面最近的深度主峰，
3. 去除背景点，
4. 去除可能属于后方杆件的点，
5. 输出清洗后的点集以及诊断计数。

目标几何提供了关键深度先验：

- 框架由 `1m` 杆件构成，
- 后方结构应当大致出现在前表面之后 `1m` 处，
- 因此后方杆件剔除应当使用 `1.0m +/- tolerance` 的先验，而不是通用聚类方法。

### 点数规模先验

即使经过清洗，支持点仍然很稀疏。一个代表性的 `1s` 窗口里，过滤后的点大约只有：

- 左柱 `26+` 个点，
- 右柱 `26+` 个点，
- 上横杆 `14+` 个点。

因此这一层必须保守：

- 不做激进几何子聚类，
- 不在深度分离后继续做重复硬裁剪，
- 不在这里拟合直线，
- 不在这里做最终几何语义判定。

恢复层需要保留这些点以维持鲁棒性。

## LiDAR 恢复架构

### 顶层结构

LiDAR 恢复路径拆成两个内部层次：

1. 结构 tracking，
2. 前表面恢复。

它们是同一个 solver 内部的职责拆分，不建议第一版拆成独立 ROS 节点。

### 为什么是在线 tracking 而不是 batch 求解

求解器不应当等“攒够数据以后”再一次性跑三层求解。

正确方式是：

- 每个新输入帧都更新 `left_post`、`right_post`、`top_beam` 的状态，
- 最终恢复层每帧都消费这三个结构状态，
- lookback window 只充当短时记忆，而不是 batch 回放池。

这更贴合真实需求，因为 tracking 本身就是核心问题。

## 单目标 tracking 模型

由于场景中只存在一个目标框架，solver 只维护一个 `FaceTrack`。

不需要通用多目标关联。

`FaceTrack` 包含：

- `left_post_state`
- `right_post_state`
- `top_beam_state`
- 当前被 tracking 的前表面输出

每帧输入都只更新这一个对象。

## 单帧观测

本设计中的 `observation` 指“恢复层根据过滤后的点集在单帧中提炼出来的结构观测”。

它不是原始点集本身，也不是长期状态。

### 柱子观测

对每根柱子，单帧观测包含：

- `support_count`
- `x_obs`
- `z_obs`
- `y_visible_min`
- `y_visible_max`
- `x_dispersion`
- `z_dispersion`
- `front_peak_confidence`
- `top_side_sample_present`

其中 `top_side_sample_present` 是从柱子物理顶部侧打到的一点弱证据，不代表该帧已经可靠地恢复了顶部边。

### 上横杆观测

对 `top_beam`，单帧观测包含：

- `support_count`
- `y_top_obs`
- `z_obs`
- `x_span`
- `z_dispersion`
- `front_peak_confidence`

### 单帧观测状态

每个单帧观测都归类为：

- `observed`
- `weak`
- `missing`

对柱子而言：

- `observed` 表示点数、深度主峰、`x/z` 集中性足够好，可以信任 `x_obs` 与 `z_obs`，
- `weak` 表示有点，但只能弱更新 tracking，
- `missing` 表示这一帧不应更新该结构的几何状态。

对 `top_beam` 而言：

- `observed` 表示点数、跨度、前表面深度一致性足够好，可作为顶部锚点，
- `weak` 表示可以帮助维持 tracking，但不能作为首次初始化的顶部锚点，
- `missing` 表示当前帧不存在可用的上横杆结构观测。

## 结构状态更新

### 基本规则

每个结构状态都维护一个覆盖最近 `1s` 的 observation deque。

这个 deque 存的是单帧结构观测，不是 raw points 回放池。

### 柱子状态内容

每个柱子状态保存：

- 最近观测 deque，
- `x_state`
- `z_state`
- `y_top_candidate_state`
- `confidence`
- `freshness`
- `lost_age`
- `initialized`
- `top_initialized`

其中 `y_top_candidate_state` 是柱子侧的“顶部候选” tracking 状态，不是直接的顶部边测量值。

### 上横杆状态内容

`top_beam_state` 保存：

- 最近观测 deque，
- `y_top_state`
- `z_state`
- `x_span_state`
- `confidence`
- `freshness`
- `lost_age`
- `initialized`

### 按观测状态更新

若结构观测为 `observed`：

- 将其加入 deque，
- 以高权重更新几何，
- 清零 `lost_age`，
- 刷新 `freshness`，
- 提升 `confidence`。

若为 `weak`：

- 也加入 deque，
- 只允许小幅修正状态，
- 不可用于首次初始化，
- 只部分恢复 freshness 与 confidence。

若为 `missing`：

- 不更新几何，
- 衰减 freshness 与 confidence，
- 增加 `lost_age`。

### 从 deque 估计当前状态

对每根柱子：

- `x_state` 与 `z_state` 来自近期 `x_obs` / `z_obs` 的加权稳健中心，
- `observed` 观测给全权重，
- `weak` 观测给降低后的权重，
- `missing` 不参与估计。

第一版建议使用 weighted median 这类稳健中心，而不是参数化滤波器。

对 `top_beam`：

- `y_top_state` 来自 `y_top_obs` 的加权稳健中心，
- `z_state` 与 `x_span_state` 以同样方式从近期有效横杆观测估计。

### 顶部侧样本处理

单帧里柱子顶部侧的点可能极少，无法稳定定义顶部点。因此：

- 单帧柱子顶部侧样本只作为弱证据，
- `y_top_candidate_state` 要通过时序累计来估计，
- 它主要用于 `top_beam` 临时失效时维持 tracking，
- 不应当单独作为完整模型首次初始化的顶部锚点。

## 模型初始化规则

完整前表面模型只能在至少有一帧成功初始化 `top_beam_state` 之后建立。

一旦第一次成功初始化完成：

- 后续帧允许 `top_beam` 失效，
- 模型可以继续依赖左右柱状态与历史顶部信息维持在 `tracking`。

这个规则避免系统在只有极弱柱子顶部样本时凭空“发明”出一条顶部边。

## 前表面恢复

### 恢复层主输入

恢复层读取：

- `left_post_state`
- `right_post_state`
- `top_beam_state`
- 历史 tracking 的顶部信息
- 固定几何先验

固定几何先验为：

- 已知的前表面宽度，
- 前表面高度 `H = 2m`。

### 低自由度求解状态

恢复层使用低自由度状态：

- `x_left`
- `z_left`
- `x_right`
- `z_right`
- `y_top`

这样既保留了“基本正对”的简化，也允许左右柱落在不同深度上。

### `y_top` 来源优先级

最终 `y_top` 的来源优先级为：

1. 有效的 `top_beam_state`，
2. 左右柱的顶部候选状态融合，
3. 上一时刻 tracking 下来的 `y_top`。

这意味着 `top_beam` 是强顶部锚点，但不是每一帧都必需。

### 角点恢复

先恢复前表面顶部两个角：

- `top_left = (x_left, y_top, z_left)`
- `top_right = (x_right, y_top, z_right)`

然后用物理高度先验补全底边：

- `bottom_left = top_left + height_prior`
- `bottom_right = top_right + height_prior`

高度偏移的符号应当在实现中跟随当前相机系坐标约定，但其物理语义固定为：

- `top_left`
- `top_right`
- `bottom_left`
- `bottom_right`

这 4 个角点构成该层唯一的主几何输出。

## 主输出与 Debug 输出

### 主输出

主结果只包括：

- 相机系下的前表面 4 个角点，
- 顺序按物理语义固定：
  - `top_left`
  - `top_right`
  - `bottom_left`
  - `bottom_right`

主 API 不发布完整面模型、显式边、也不再发布 2D 图像 `bbox`。

### Debug 输出

为了调试，还需要额外发布：

- 2 个直接解算得到的上角点，
- 它们的来源与状态，
- tracking 诊断信息。

这样可以区分哪些角点是直接观测得到的，哪些只是先验补全。

### 当前阶段不做的事

虽然下游未来可能需要完整 12 个角点，但这一层当前只做到：

- 主输出 4 个前表面角点，
- debug 输出 2 个直接解算的上角点。

从 4 角点扩展到 12 角点属于后续阶段。

## 角点状态语义

内部诊断中，每个角点应带有以下语义状态：

- `observed`
- `inferred`
- `invalid`

预期行为是：

- 顶部两个角在有直接顶部支持时通常为 `observed`，
- 当顶部来自柱子侧历史或 tracking 时，顶部角可能退化为 `inferred`，
- 底部角通常为 `inferred`，因为它们依赖 `2m` 高度先验补全，
- 模型不可用时角点为 `invalid`。

## 解状态

对外的解状态简化为：

- `tracking`
- `invalid`
- `lost`

### tracking

`tracking` 表示：

- 当前模型可用，
- 当前输出允许混合新鲜观测与历史 tracking 状态，
- 基本几何校验通过。

### invalid

`invalid` 表示：

- 模型还没有完成初始化，
- 或当前证据不足以给出可信输出。

### lost

`lost` 表示：

- 最近曾经有有效 tracking 结果，
- 但当前已经无法继续维持可信输出。

第一版不再额外定义 `ready` 状态。

## 几何校验

只有当基本几何校验通过时，模型才能以 `tracking` 状态发布：

- 顶边宽度与已知前表面宽度一致，
- 左右柱深度差合理，
- 当 `top_beam` 有效时，顶部一致性检查通过，
- 结构状态不过期，
- tracking 置信度足以避免发布明显陈旧的“幻觉”结果。

第一版应优先使用简单硬约束，先挡住明显错误的几何。

## 离线验证

该重构应支持不依赖实时相机与 LiDAR 话题的验证。

### 验证模式

采用 `static-scene hybrid offline validation`：

- ROI 在一张静止场景参考 RGB 图像上运行，
- LiDAR tracking 与恢复在录制好的投影点数据上运行，
- 两者虽然不是同步采集，但由于框架位置未变，因此接受为可兼容输入。

这不是时序同步测试，而是在静态场景假设下做方法验证。

### 选定资产

用于 ROI 的参考 RGB 图像：

- [analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png](/home/sy/code/ws_fastlio_nx/analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png)

用于 LiDAR tracking 与恢复的投影点证据：

- [analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz](/home/sy/code/ws_fastlio_nx/analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz)

该 `.npz` 内包含：

- `uv`
- `depth`
- `stamp`

结合相机内参，可以从这些字段重构离线流程所需的相机系点样本。

### 离线验证应验证的内容

- ROI 产生的三个结构 mask 是否符合物理语义。
- 三个结构过滤后的点数是否仍然合理。
- 三个结构状态是否能在无实时传感器的情况下持续更新。
- 模型是否只会在至少一次成功初始化 `top_beam` 后建立。
- 当 `top_beam` 后续变弱或消失时，solver 是否仍可保持 `tracking`。
- 前表面 4 个角点在相机系下是否稳定。
- debug 上角点输出是否与直接支持证据一致。

### 离线验证不验证的内容

- 实时话题同步，
- ROS 实时运行时序，
- 动态场景鲁棒性，
- 通用多目标 tracking。

## 实现影响

预计会改到的区域：

- ROI 的结构语义消息定义，
- `roi_generator_node.py`，
- 从现有角点结果导出结构线与结构 mask，
- LiDAR 侧点筛选逻辑，
- 单目标在线结构状态管理，
- 最终 4 角点与 2 个 debug 上角点输出，
- 离线验证 helper 与测试。

可能被淘汰或降级的部分：

- 以 corner ROI 作为主契约的输出，
- 以每个角点直接聚点求解为核心的 LiDAR 逻辑。

## 风险与权衡

主要权衡：

- 该设计刻意保留强几何先验，而不是做通用无约束 3D 模型求解。
- `top_beam` 对首次初始化仍然是必需的，这会让首个成功时刻依赖顶部证据质量。
- tracking 允许在弱观测期间维持输出，因此诊断信息必须足够强。

缓解方式：

- 保留 2 个直接解算上角点的 debug 输出，
- 保留 `tracking / invalid / lost` 以及置信度诊断，
- 离线验证重点检查状态转换与结构正确性，
- 继续复用当前稳定的图像侧角点逻辑，而不是替换它。

## 推荐下一步

基于本设计编写实现计划，至少覆盖：

1. 将 corner-based ROI 契约替换为 structure masks，
2. 用角点连线加法向膨胀生成 ROI 结构 mask，
3. 实现 LiDAR 点筛选层，
4. 实现单目标三结构状态及其更新规则，
5. 恢复 4 个前表面角点并发布 2 个 debug 上角点，
6. 基于选定 RGB 图像与 `.npz` 资产搭建离线验证路径。
