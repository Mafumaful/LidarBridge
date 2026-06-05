# paths

这个目录用于保存路径生成器导出的路径文件。

- `generated_path.yaml`：关键点和 cubic spline 采样路径，适合 C++ 用 `yaml-cpp` 读取
- `generated_path.csv`：采样路径表格，适合简单 C++ 文件解析或调试查看

仓库里默认也放了一份可直接联调的示例 `generated_path.*`，你后续在路径生成器里按 `s` 保存后会覆盖它们。
