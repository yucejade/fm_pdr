# UBODT 生成功能开发文档

## 开发计划

### 需求
在 PDRTest 程序中增加参数，支持输入路网地图文件并输出 UBODT 文件。

### 约束
1. **绝对不修改原有代码逻辑** — 只允许新增代码，已有函数的行为和流程不可变更
2. **新增参数不可与原有参数冲突** — 短选项、长选项名称均不可重复

### 修改文件清单

| 文件 | 修改类型 | 说明 |
|------|----------|------|
| `src/fm_pdr.h` | 新增 | 添加 `fm_pdr_generate_ubodt()` C 接口声明 |
| `src/fm_pdr.cpp` | 新增 | 添加 FMM 头文件引用和 `fm_pdr_generate_ubodt()` 实现 |
| `example/plain_c/main.c` | 新增 | 添加 `--network`/`--ubodt`/`--delta` 参数解析和 UBODT 生成分支 |
| `example/plain_c/CMakeLists.txt` | 无修改 | 已有 FMMLIB 链接，无需变更 |
| `README.md` | 新增 | 添加 UBODT 生成使用示例 |

### 新增命令行参数

| 短选项 | 长选项 | 类型 | 默认值 | 说明 |
|--------|--------|------|--------|------|
| `-n` | `--network` | 字符串 | 无 | 输入路网 shapefile 文件路径 |
| `-u` | `--ubodt` | 字符串 | 无 | 输出 UBODT 文件路径 |
| `-l` | `--delta` | 浮点数 | 0.02 | UBODT 上界参数 |

当同时提供 `--network` 和 `--ubodt` 时，进入 UBODT 生成模式。

### 新增 C API

```c
int fm_pdr_generate_ubodt( const char* network_file, const char* output_file, double delta );
```

内部流程：
1. `FMM::NETWORK::Network` 加载 shapefile（字段名：fid/u/v）
2. `FMM::NETWORK::NetworkGraph` 构建图
3. `FMM::MM::UBODTGenAlgorithm::generate_ubodt()` 生成 UBODT 文件（CSV 格式，启用 OpenMP）

### 迭代检查结果

| 检查项 | 结果 |
|--------|------|
| 原有 PDR 训练/预测/实时模式代码未被修改 | 通过 |
| 原有参数 -c/-t/-d/-x/-y/-o/-r/-e/-h 行为不变 | 通过 |
| 新增短选项 n/u/l 与原有无冲突 | 通过 |
| 新增长选项 network/ubodt/delta 与原有无冲突 | 通过 |
| show_help 中原有帮助文本未被修改（已修复一处误改：--raw-data-dir 被误改为 --save-pdr-data，已还原） | 通过 |

## 测试结论

### 编译测试

| 步骤 | 命令 | 结果 |
|------|------|------|
| 编译 PDR 库 | `./make.sh pdr` | 无错误 |
| 编译测试程序 | `./make.sh test` | 无错误 |

### 功能测试

| 项目 | 内容 |
|------|------|
| 测试命令 | `./PDRTest --network ./network/edges.shp --ubodt ./ubodt.txt --delta 0.02` |
| 路网数据 | `/home/kevin/yuce/project/fmm/build/python/network/edges.shp` |
| 路网规模 | 77 节点，224 条边 |
| 输出文件 | `ubodt.txt`，5705 行，144237 字节 |
| 执行耗时 | 0.049 秒 |
| 执行状态 | Status: success |

### 输出文件样例

```
source;target;next_n;prev_n;next_e;distance
0;76;1;59;0;0.015491
0;59;1;58;0;0.015072
0;57;1;58;0;0.0151281
0;12;1;10;0;0.016121
```

### 结论

UBODT 生成功能开发完成，编译和功能测试均通过。
