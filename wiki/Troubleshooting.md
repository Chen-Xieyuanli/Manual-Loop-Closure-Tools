# Troubleshooting

## GUI Opens but Point Clouds Look Wrong

Check:

- whether the selected session has the correct `g2o`, `TUM`, and `key_point_frame` files
- whether `source` and `target` are selected in the intended order
- whether the point-cloud mode is `Temporal Window` or `RS Spatial Submap`

检查：

- session 是否包含正确的 `g2o`、`TUM` 和 `key_point_frame`
- `source` / `target` 是否按预期选择
- 当前点云模式是否为 `Temporal Window` 或 `RS Spatial Submap`

## Python Optimizer Not Available

Run:

```bash
python scripts/check_env.py
```

and verify that Python GTSAM can be imported.

然后确认 Python GTSAM 能正常导入。

## C++ Backend Missing

This is expected for Python-only usage.

对于 Python-only 使用路径，这属于正常情况。

Only install the C++ backend if you explicitly need fallback or parity checks.

只有在你明确需要 fallback 或 parity 对比时，才需要安装 C++ backend。

## Exported Map Too Sparse

Check the export map voxel in `Advanced`.

请检查 `Advanced` 中的导出地图体素参数。

- `0.0` means no export downsampling
- values larger than `0.0` will reduce map density

## Session Browser Opens at Home

The GUI remembers the last successful session path. If it still resets unexpectedly, verify that the session was actually loaded successfully before reopening the file dialog.

GUI 会记住最近一次成功加载的 session 路径。如果仍然回到 home，请先确认上一次 session 是否真的加载成功。
