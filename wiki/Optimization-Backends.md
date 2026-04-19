# Optimization Backends

## Python First | Python 优先

The standalone project now uses the Python optimizer as the default backend.

独立项目现在默认使用 Python 优化后端。

Why:

原因：

- simpler installation
- no mandatory ROS / catkin requirement
- consistent integration with the PyQt + Open3D GUI
- validated parity against the legacy C++ backend

- 安装更简单
- 不再强制依赖 ROS / catkin
- 与 PyQt + Open3D GUI 的集成更自然
- 已与 legacy C++ backend 做过一致性验证

## Legacy C++ Fallback | Legacy C++ 回退后端

The C++ backend is still supported as an optional fallback.

C++ backend 仍保留为可选 fallback。

Use it when:

适用情况：

- you already have the legacy backend environment
- you want direct parity checks against historical runs
- you need a fallback when Python GTSAM is temporarily unavailable

## Parameter Consistency | 参数一致性

The Python backend follows the same runtime-parameter precedence used by the legacy optimizer:

Python backend 遵循与 legacy optimizer 相同的参数优先级：

1. explicit CLI / GUI options
2. `runtime_params.yaml`
3. validated offline defaults

## Output Files | 导出结果

Both backends export:

两种 backend 都会导出：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `global_map_manual_imu.pcd`
- `trajectory.pcd`
- `pose_graph.png`
- `manual_loop_report.json`
