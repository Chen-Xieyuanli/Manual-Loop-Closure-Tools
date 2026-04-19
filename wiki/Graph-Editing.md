# Graph Editing

## Editing Philosophy | 编辑逻辑

The tool distinguishes between:

工具明确区分以下几种图编辑动作：

- adding a new manual loop
- replacing an existing loop edge
- disabling an existing loop edge
- restoring a disabled edge

- 新增手工闭环边
- 替换已有闭环边
- 禁用已有闭环边
- 恢复已禁用边

## Add Manual Loop | 新增手工闭环边

Use this when the graph does not already contain the constraint you want.

适用于图中原本不存在你想加入的那条约束。

Workflow:

流程：

1. pick a source / target pair
2. refine the initial pose if necessary
3. run GICP
4. verify the result in the viewer
5. click `Add`

## Replace Existing Loop | 替换已有闭环边

Use this when an existing loop edge is present but unreliable.

适用于已有闭环边存在，但你认为约束不够可靠的情况。

Workflow:

流程：

1. switch to `Edges`
2. select the existing loop
3. run GICP from the current preview seed
4. click `Replace`

Internally this keeps the original graph untouched, disables the selected existing loop in the working graph, and applies the new manual result instead.

内部实现上不会改写原始图，而是在 working graph 中禁用原边，再应用新的手工约束结果。

## Disable / Restore | 禁用 / 恢复

- `Disable` removes an existing loop from the working graph only.
- `Restore` brings the disabled loop back into the working graph.

- `Disable` 只在 working graph 中移除已有闭环边。
- `Restore` 会把被禁用的边重新加入 working graph。

## Export Behavior | 导出语义

Export is based on the working graph state after optimization.

导出结果基于优化后的 working graph 状态。

This means:

这意味着：

- disabled loops are excluded
- replacement loops are exported through the accepted working graph state
- original input files are not overwritten

- 被禁用的闭环边不会进入最终导出
- 替换后的闭环以 working graph 的接受状态导出
- 原始输入文件不会被覆盖
