# FMM UBODT delta 参数说明

## 含义

delta 是最短路径距离的**上界（Upper Bound）**。UBODT 生成时对路网每个节点执行 Dijkstra 搜索，只记录最短路径距离 <= delta 的节点对。

## 单位

与路网文件的空间单位一致：

| 路网坐标系 | 单位 | 推荐值 |
|---|---|---|
| 投影坐标系（如 UTM） | 米 | 3000 |
| WGS84 经纬度 | 度 | 0.03（≈3.3km，1度≈111km） |

度与米的换算：`3000米 = 3000 / 113200 ≈ 0.0265度`

## 选择依据

1. **GPS 采样频率**（最关键）：采样间隔越大，相邻点间距越大，delta 需要越大
2. **UBODT 文件大小**：delta 与文件大小呈超线性关系，过大会导致生成时间过长
3. **匹配失败率**：匹配时出现 "unmatched" 警告说明 delta 太小，需增大

## 本项目建议

室内步行 PDR 场景，步行速度约 1-1.5 m/s，推荐 delta 范围 2000-3000 米（WGS84 下 0.02-0.03 度）。

## 参考

- FMM GitHub: https://github.com/cyang-kth/fmm
- FMM Wiki 配置文档: https://fmm-wiki.github.io/docs/documentation/configuration/
- FMM Issue #138 中作者对 delta 换算的说明
