# Tool Manual | 工具说明

## What This Tool Does | 工具作用

The tool provides an offline workflow for inspecting, correcting, and exporting loop-closure constraints in LiDAR pose graphs.

本工具提供一个离线工作流，用于检查、修正并导出激光雷达位姿图中的闭环约束。

Typical use cases:

典型用途包括：

- difficult automatic loop closures in repetitive indoor scenes
- replacing a weak existing loop edge with a manually validated one
- adding several manual loop edges and optimizing them together

- 室内重复结构场景中的自动闭环困难
- 用手工验证后的结果替换质量较差的已有闭环边
- 连续添加多条手工闭环边并统一优化

## Main UI Areas | 主要界面区域

| Area | Description | 中文说明 |
|---|---|---|
| Trajectory panel | 2D graph view for node and edge selection | 2D 位姿图，用于选节点和选边 |
| Point Cloud Review | Embedded point-cloud viewer for preview, manual alignment, and GICP inspection | 内嵌点云窗口，用于预览、手动初值调整和 GICP 检查 |
| Control panel | Summary, delta, registration settings, and actions | 参数、摘要、手工增量和操作按钮 |
| Graph Changes | Session-based list of accepted graph edits | 工作会话中的图改动列表 |
| Execution Log | Runtime log panel | 运行日志面板 |

## Workflow | 基本流程

### A. Add a new manual loop edge | A. 新增手工闭环边

1. Load a session.
2. In the trajectory panel, switch to `Pick Nodes`.
3. Select two nodes. The tool normalizes them to `source_id > target_id`.
4. Preview the point clouds.
5. Adjust the source pose if needed.
6. Run `GICP` or `Auto Yaw Sweep`.
7. If the result is good, click `Add Manual`.
8. After collecting the desired changes, click `Optimize`.

1. 加载 session。
2. 在轨迹图切换到 `Pick Nodes`。
3. 选择两个节点，工具会自动规范为 `source_id > target_id`。
4. 查看点云预览。
5. 如有需要，手动调整 source 初值。
6. 运行 `GICP` 或 `Auto Yaw Sweep`。
7. 若结果满意，点击 `Add Manual`。
8. 收集完需要的改动后，点击 `Optimize`。

### B. Replace an existing loop edge | B. 替换已有闭环边

1. Switch to `Pick Edges`.
2. Select an existing loop edge.
3. Inspect or manually align the source cloud.
4. Run `GICP`.
5. Click `Replace Edge`.
6. The working graph disables the old edge and uses the new manual result as its replacement.

1. 切换到 `Pick Edges`。
2. 选择一条已有闭环边。
3. 检查或手动调整 source 点云。
4. 运行 `GICP`。
5. 点击 `Replace Edge`。
6. 工作图会禁用旧边，并使用新的手工结果作为替换约束。

## Working vs Original | Working 与 Original

- `Original`: read-only baseline graph.
- `Working`: editable graph used for subsequent preview, matching, and optimization.

- `Original`：只读基线图。
- `Working`：后续预览、匹配和优化实际使用的可编辑工作图。

Every successful optimization updates the `Working` trajectory revision.

每次优化成功后，`Working` 轨迹版本都会更新。

## Target Cloud Modes | Target 点云构造模式

### 1. Temporal Window

- default mode
- target cloud = `target_id ± N` frames
- all frames are transformed to the map frame and merged directly

- 默认模式
- target 点云 = `target_id ± N` 帧
- 所有帧先变换到 map 系再直接拼接

### 2. RS Spatial Submap

- keeps the online RS-style local-submap logic
- selects spatial neighbors near the target node, with a source-to-target minimum time gap

- 保留在线 RS 风格的局部子图逻辑
- 按 target 附近的空间邻居选帧，并对 source 和 target 施加最小时间间隔过滤

## Point Cloud Editing | 点云编辑

The tool is intentionally optimized for the common ground-robot workflow.

本工具有意针对地面机器人最常见的手工闭环流程进行了收敛设计。

### Default edit modes | 默认编辑模式

- `XY+Yaw`: the main mode for horizontal alignment
- `Z`: used occasionally from a side view to move the whole source cloud up or down

- `XY+Yaw`：主要模式，用于平面内对齐
- `Z`：偶尔从侧视图整体上移或下移 source 点云

### Mouse behavior | 鼠标操作

#### View mode

- Left drag: orbit camera
- Right drag: pan camera
- Wheel: zoom

- 左键拖动：旋转视角
- 右键拖动：平移视角
- 滚轮：缩放

#### Edit mode

- The editable object is always the `source` cloud.
- `target` remains fixed.
- In `XY+Yaw` + `Drag=XY`: left drag changes source `x/y`.
- In `XY+Yaw` + `Drag=Yaw`: left drag changes source `yaw`.
- In `Z`: left vertical drag changes source `z`.
- Right drag still pans the camera.
- `Alt + Left drag` or middle drag still rotates the camera.
- Wheel always zooms the camera.

- 可编辑对象始终是 `source` 点云。
- `target` 始终固定。
- `XY+Yaw` + `Drag=XY`：左键拖动修改 source 的 `x/y`。
- `XY+Yaw` + `Drag=Yaw`：左键拖动修改 source 的 `yaw`。
- `Z`：左键上下拖动修改 source 的 `z`。
- 右键仍用于平移视角。
- `Alt + 左键拖动` 或中键仍用于旋转视角。
- 滚轮始终用于缩放视角。

Manual dragging and the right-side `Manual Delta` controls are synchronized both ways.

点云拖拽和右侧 `Manual Delta` 控件是双向同步的。

## Auto Yaw Sweep | 自动 Yaw 遍历

This feature targets ground robots whose main unknown is usually yaw.

这个功能主要面向地面机器人，因为这类场景下不确定量通常主要是 yaw。

Behavior:

功能逻辑：

- the current manual delta is used as the base pose
- only yaw seeds are swept
- each seed first updates the point-cloud preview
- GICP is then run from that seed
- the best result is chosen using overlap quality and RMSE

- 以当前手工 delta 作为基准位姿
- 仅遍历 yaw 初值
- 每个初值都会先刷新点云预览
- 然后从该初值运行 GICP
- 最终按重叠质量和 RMSE 选出最佳结果

## Graph Changes | 图改动列表

The bottom table stores accepted changes in the current session.

底部表格用于保存当前工作会话中已经接受的图改动。

Types:

类型包括：

- `Manual Add`
- `Replace Existing Loop`
- `Disable Existing Loop`

Common statuses:

常见状态包括：

- `Accepted`
- `Applied`
- `Disabled`

Notes:

说明：

- `Replace Existing Loop` is shown as a single logical row.
- Replacement operations are treated as one atomic user action for undo / redo semantics.

- `Replace Existing Loop` 在表格中会聚合为单行业务动作。
- 替换操作在撤销语义上被视为一次原子动作。

## Exported Files | 导出文件

When you optimize and export, the tool generates:

执行优化和导出后，工具会生成：

- edited input `g2o`
- manual constraint CSV
- optimized `pose_graph.g2o`
- optimized `optimized_poses_tum.txt`
- `global_map_manual_imu.pcd`
- `trajectory.pcd`
- `pose_graph.png`
- `manual_loop_report.json`

## Practical Tips | 实用建议

- Start with `XY+Yaw` and only use `Z` when the source cloud is obviously too high or too low.
- Use `Temporal Window` for most manual work.
- Run optimization after a small number of accepted changes, instead of accumulating too many edits first.
- If you are revising an existing loop edge, prefer `Replace Edge` over adding a duplicate manual edge.

- 优先使用 `XY+Yaw`，只有当 source 整体明显过高或过低时再切到 `Z`。
- 大多数手工配准场景建议使用 `Temporal Window`。
- 建议每累计少量已接受改动后就优化一次，而不是一次性堆积过多编辑。
- 若你是在修订已有闭环边，优先使用 `Replace Edge`，避免叠加重复约束。
