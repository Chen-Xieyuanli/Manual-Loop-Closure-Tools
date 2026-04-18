# Installation Guide | 安装说明

## Scope | 适用范围

This guide targets Ubuntu 20.04 + ROS Noetic and matches the versions currently used to test this repository.

本说明面向 Ubuntu 20.04 + ROS Noetic，并与当前仓库已经验证过的版本保持一致。

## Tested Versions | 已测试版本

| Dependency | Version | Notes |
|---|---:|---|
| Python | 3.10.16 | GUI environment |
| Open3D | 0.19.0 | GUI point-cloud viewer |
| PyQt5 | 5.15.10 | GUI |
| NumPy | 1.24.4 | GUI |
| SciPy | 1.14.1 | GUI / transforms |
| Matplotlib | 3.10.8 | Trajectory view |
| ROS | Noetic | Backend build/runtime |
| catkin_tools | 0.9.4 | Backend build |
| GCC | 9.4.0 | Backend build |
| CMake | 3.25.0 | Backend build |
| OpenCV | 4.2.0 | Backend |
| PCL | 1.10.0 | Backend |
| GeographicLib | 1.50.1 | Backend |
| GTSAM | 4.3.0 | Backend |

## 1. Clone the Repository | 1. 克隆仓库

```bash
cd ~/my_git
git clone git@github.com:JokerJohn/Mannual-Loop-Closure-Tools.git
cd Mannual-Loop-Closure-Tools
```

## 2. Install System Dependencies | 2. 安装系统依赖

Run the helper script after your ROS Noetic apt source is available.

在 ROS Noetic 软件源已经配置好的前提下，运行辅助脚本安装系统依赖。

```bash
bash scripts/install_ubuntu20.sh
```

What the script installs:

脚本会安装以下依赖：

- build tools: `build-essential`, `cmake`, `git`, `pkg-config`
- ROS packages: `roscpp`, `rospy`, `rosbag`, `tf`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `image_transport`, `cv_bridge`, `pcl_conversions`, `visualization_msgs`, `std_srvs`
- system libraries: `libpcl-dev`, `libopencv-dev`, `libboost-all-dev`, `libgeographic-dev`, `libzstd-dev`
- utilities: `python3-pip`, `python3-venv`, `python3-catkin-tools`

## 3. Create the Python Environment | 3. 创建 Python 环境

Recommended default path:

默认推荐方式：

```bash
make venv
source .venv/bin/activate
```

This uses the version-pinned [requirements.txt](../requirements.txt) and creates a local virtual environment under `.venv/`.

这条路径会使用带版本号约束的 [requirements.txt](../requirements.txt)，并在仓库根目录创建 `.venv/` 本地虚拟环境。

Manual equivalent:

手动等价命令：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

Alternative with conda:

也可使用 conda：

```bash
conda env create -f environment.yml
conda activate manual-loop-closure
```

## 4. Install GTSAM | 4. 安装 GTSAM

The backend optimizer was tested with `GTSAM 4.3.0`.

后端优化器当前使用 `GTSAM 4.3.0` 进行了测试。

If `GTSAMConfig.cmake` is already available under `/usr/local/lib/cmake/GTSAM` or another CMake prefix, you can skip this step.

如果系统里已经存在 `/usr/local/lib/cmake/GTSAM` 或其他 CMake 前缀下的 `GTSAMConfig.cmake`，可以跳过此步骤。

Otherwise, install GTSAM 4.2+ and make sure CMake can find it through one of:

否则请安装 GTSAM 4.2+，并确保 CMake 能通过以下任一方式找到它：

- `CMAKE_PREFIX_PATH`
- `GTSAM_DIR`
- a default system location such as `/usr/local/lib/cmake/GTSAM`

## 5. Build the Backend Optimizer | 5. 编译后端优化器

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make backend
```

This builds:

该步骤会编译：

- package: `manual_loop_closure_backend`
- binary: `manual_loop_optimize`

Expected binary path after a successful build:

编译成功后的二进制路径通常为：

```bash
backend/catkin_ws/devel/lib/manual_loop_closure_backend/manual_loop_optimize
```

## 6. Verify the Environment | 6. 检查环境

```bash
make env-check
```

This script prints Python package versions, ROS / catkin presence, common GTSAM CMake paths, and backend build hints.

该脚本会打印 Python 包版本、ROS / catkin 状态、常见 GTSAM CMake 路径以及后端构建提示。

## 7. Launch the GUI | 7. 启动 GUI

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

Or:

或者：

```bash
python launch_gui.py --g2o /path/to/pose_graph.g2o
```

## 8. Expected Input Layout | 8. 输入目录结构

```text
mapping_session/
├── key_point_frame/
│   ├── 0.pcd
│   ├── 1.pcd
│   └── ...
├── pose_graph.g2o
└── optimized_poses_tum.txt
```

The tool also supports sessions where `pose_graph.g2o` and `optimized_poses_tum.txt` are stored under the latest timestamp subdirectory, while `key_point_frame/` remains at the session root.

工具也支持这样的 session：`pose_graph.g2o` 和 `optimized_poses_tum.txt` 位于最新时间戳子目录下，而 `key_point_frame/` 仍位于 session 根目录。

## Troubleshooting | 故障排查

### Open3D cannot be imported | 无法导入 Open3D

- Make sure you launched the GUI from the tested conda or venv environment.
- The GUI will try to re-launch itself with a compatible Python interpreter if possible.

- 请确认 GUI 在已安装依赖的 conda 或 venv 环境中启动。
- GUI 会尽量自动切换到可导入依赖的 Python 解释器。

### Backend optimizer not found | 找不到后端优化器

- Re-run `bash scripts/build_backend_catkin.sh`
- Or set `MANUAL_LOOP_OPTIMIZER_BIN=/absolute/path/to/manual_loop_optimize`

- 重新执行 `bash scripts/build_backend_catkin.sh`
- 或手动设置 `MANUAL_LOOP_OPTIMIZER_BIN=/absolute/path/to/manual_loop_optimize`

### GTSAM not found by CMake | CMake 找不到 GTSAM

Export a CMake prefix before building:

构建前先导出 CMake 前缀：

```bash
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```

## Related Documentation | 相关文档

- [Tool Manual / 工具说明](TOOL_README.md)
- [Project Overview / 项目总览](../README.md)
