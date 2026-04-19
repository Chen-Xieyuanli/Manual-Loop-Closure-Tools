# FAQ

## Is ROS required?

No for normal use.

正常使用不需要。

The standalone tool is designed around a Python-first workflow. ROS / catkin are only needed if you want to keep the legacy C++ optimizer as a fallback.

## Why are there both `Original` and `Working` trajectories?

`Original` is the immutable baseline.

`Original` 是不可变的基线结果。

`Working` is the graph currently being edited and re-optimized.

`Working` 是当前被编辑和重新优化的图。

## Does `Replace` overwrite the original edge?

Not in the input graph.

不会改写输入图本身。

It replaces the selected edge only inside the working session semantics.

它只会在 working session 语义中替换当前边。

## Why is Python slower than C++ in parity tests?

The Python backend prioritizes easier deployment and GUI integration. The current parity results show much simpler installation at the cost of higher optimization runtime on the tested sessions.

Python backend 的重点是更低的部署门槛和更自然的 GUI 集成。当前 parity 结果表明，它在测试数据上通常会比 C++ 更慢，但安装和维护明显更简单。

## Where should I start reading?

Start from:

建议阅读顺序：

1. [Quick Start](Quick-Start.md)
2. [GUI Workflow](GUI-Workflow.md)
3. [Graph Editing](Graph-Editing.md)
4. [Troubleshooting](Troubleshooting.md)
