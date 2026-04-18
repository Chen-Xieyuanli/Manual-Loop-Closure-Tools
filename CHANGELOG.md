# Changelog

All notable changes to this project will be documented in this file.

本项目的重要变更会记录在本文件中。

## [0.1.0] - 2026-04-18

### Added

- Standalone bilingual open-source project layout for the manual loop closure tool.
- PyQt GUI entrypoint and extracted backend catkin workspace.
- Detailed English / Chinese installation and tool documentation.
- Version-pinned `requirements.txt` and optional conda environment.
- One-command helper scripts for venv creation, environment checking, backend building, and Ubuntu dependency installation.
- README assets, screenshots, and workflow illustrations.
- GitHub Actions smoke-check workflow.

### Changed

- Adapted GUI path discovery and optimizer lookup for the standalone repository.
- Simplified backend build dependencies so the offline optimizer no longer requires Open3D CMake integration.

### Validated

- `python3 -m py_compile`
- `python launch_gui.py --help`
- `python scripts/check_env.py`
- `bash scripts/build_backend_catkin.sh`

## [Unreleased]

- Reserved for future changes.
