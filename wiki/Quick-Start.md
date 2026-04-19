# Quick Start

## Recommended Path | 推荐路径

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make venv
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

For normal usage, the Python backend is the default path.

默认情况下，工具优先使用 Python backend。

## Minimum Input | 最小输入

Your session should already contain:

你的 session 通常需要已经包含：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

## Typical Flow | 典型流程

1. Load a session.
2. Inspect the trajectory.
3. Pick node pairs or an existing loop edge.
4. Preview source and target clouds.
5. Run GICP.
6. Add / replace / disable graph changes.
7. Optimize the working graph.
8. Export the final result.

1. 加载 session。
2. 检查轨迹。
3. 选择节点对或已有闭环边。
4. 预览 source 和 target 点云。
5. 运行 GICP。
6. 新增 / 替换 / 禁用图改动。
7. 优化 working graph。
8. 导出最终结果。

## Helpful Links | 快速链接

- [GUI Workflow](GUI-Workflow.md)
- [Graph Editing](Graph-Editing.md)
- [Troubleshooting](Troubleshooting.md)
