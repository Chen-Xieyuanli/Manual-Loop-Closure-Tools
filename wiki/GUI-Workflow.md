# GUI Workflow

## Main Layout | 主界面结构

The GUI is divided into four main areas:

界面主要分为四块：

- trajectory panel
- point-cloud review panel
- right-side control tabs (`Summary` / `Advanced`)
- bottom tabs (`Graph Changes` / `Execution Log`)

- 轨迹面板
- 点云预览面板
- 右侧控制页签（`Summary` / `Advanced`）
- 底部页签（`Graph Changes` / `Execution Log`）

## Trajectory Panel | 轨迹面板

- `Nodes` is used to select source / target node pairs.
- `Edges` is used to inspect or revise existing loop edges.
- `Working` and `Original` let you compare the current edited graph against the baseline graph.
- `Ghost` overlays the other trajectory as a lightweight reference.

- `Nodes` 用于选择 source / target 节点对。
- `Edges` 用于检查或修订已有闭环边。
- `Working` 与 `Original` 可对比当前编辑图与原始图。
- `Ghost` 会以轻量叠加方式显示另一套轨迹。

## Point Cloud Review | 点云预览

- `Preview`, `Final`, and `Compare` control the source/target display mode.
- `Top`, `Side-Y`, and `Side-X` are camera presets.
- In edit mode, source is the only editable object; target stays fixed.

- `Preview`、`Final`、`Compare` 控制点云显示模式。
- `Top`、`Side-Y`、`Side-X` 是相机预设。
- 在编辑模式下，只有 source 可编辑，target 始终固定。

## Right Control Tabs | 右侧控制页

### Summary

- Summary cards
- Delta
- Registration
- Actions

### Advanced

- shared variance settings
- backend selection
- export map voxel

## Bottom Tabs | 底部页签

- `Graph Changes` records accepted manual edits and disabled/restored loop actions.
- `Execution Log` stores optimization and GICP logs.

- `Graph Changes` 记录已接受的手工改动及禁用/恢复动作。
- `Execution Log` 用于查看优化与 GICP 日志。
