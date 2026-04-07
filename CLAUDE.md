# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供代码仓库的工作指导。

## 项目概述

用于树莓派 5 室内导航的 PDR（行人航位推算）库，使用 TDK40607P IMU 和 MMC56x3 磁力计。基于 [NJU-AML2022 PDR 项目](https://github.com/nju-aml2022/Pedestrian-Dead-Reckoning-PDR) 从 Python 移植到 C++。

## 构建命令

```bash
./make.sh rebuild      # 清理并重新构建所有内容（第三方库 + pdr + 测试）
./make.sh build        # 不清理直接构建所有内容
./make.sh clean        # 删除 build 目录
./make.sh thirdparty   # 仅构建第三方依赖
./make.sh pdr          # 仅构建 PDR 库
./make.sh test         # 仅构建测试程序
```

构建输出位于 `build/package/`：
- `bin/` - 可执行文件（PDRTest, mag_calib4）
- `lib/` - 静态库（libpdr.a）
- `include/FmPDR/` - 头文件
- `conf/` - 配置文件

## 运行测试

```bash
# 训练 PDR 模型
./build/package/bin/PDRTest --train-dir "./test_data/train_data"

# 从数据文件预测轨迹
./build/package/bin/PDRTest --dataset-dir "./test_data/sensor_data" --output-path "./Trajectory.csv"

# 使用传感器实时预测
./build/package/bin/PDRTest -x 32.11199920 -y 118.9528682 --output-path "./Trajectory.csv"

# 磁力计校准流程
./build/package/bin/mag_calib4 -t 0 -d "./mag_calib_data" -i -1  # 采集数据
./build/package/bin/mag_calib4 -t 1 -d "./mag_calib_data/Magnetometer.csv" -o "./mag_calib_data/mag_calibration.csv"  # 生成校准文件
./build/package/bin/mag_calib4 -t 2 -c "./mag_calib_data/mag_calibration.csv" -i -1  # 验证校准
```

## 架构

### 核心库 (`src/`)

库通过 `fm_pdr.h` 提供 C API 用于集成。主要内部组件：

- **`CFmPDR`** (`pdr.h/cpp`) - PDR 算法主控制器
- **`CFmStepPredictor`** (`step_predictor.h/cpp`) - 使用加速度计峰值检测进行步态检测；使用 dlib 机器学习模型（线性回归）进行步长预测
- **`CFmDirectionPredictor`** (`direction_predictor.h/cpp`) - 融合磁力计和重力旋转向量，使用 Butterworth 滤波进行方向预测
- **`CFmDataManager`** (`data_manager.h/cpp`) - 管理传感器数据（加速度计、线性加速度、陀螺仪、磁力计、GPS）
- **`CFmMergeDirectionStep`** (`merge_direction_step.h/cpp`) - 将方向和步态预测合并为轨迹

### 数据流

1. **训练阶段**: `fm_pdr_init_with_file()` 传入训练数据路径
   - 加载传感器和 GPS 数据，训练步长模型，保存到 `model.dat`

2. **预测阶段**: `fm_pdr_start()` → `fm_pdr_predict()` 循环 → `fm_pdr_stop()`
   - 步态检测 → 步长估计 → 方向预测 → 轨迹更新

### FMM 模块 (`src/fmm/`)

使用 FMM 库进行地图匹配集成，将轨迹匹配到路网。包含：
- `fmm_algorithm` - 核心地图匹配算法封装
- `ubodt` - 上界起点终点表处理

### 设备驱动 (`src/device/`)

- **TDK40607P/** - IMU 传感器驱动
- **MMC56x3/** - 磁力计驱动

### 校准 (`src/calibration/`)

磁力计校准算法，用于硬磁/软磁校正：
- `magnetometer-calibration` - 主校准逻辑
- `SixParametersCorrector` - 6 参数校正模型
- `SoftAndHardIronCalibration` - 硬磁/软磁补偿

## 配置

配置文件位于 `src/conf/config.json`，控制算法参数：

| 参数 | 说明 |
|------|------|
| `sample_rate` | 采样频率 (Hz)，默认 50 |
| `pdr_duration` | PDR 时间窗口（秒），默认 4 |
| `model_name` | 机器学习模型类型 ("Linear")，默认 "Linear" |
| `model_file_name` | 模型文件路径，默认 "model.dat" |
| `butter_wn` | Butterworth 滤波器截止频率，默认 0.0035 |

## 第三方依赖

通过 `make.sh thirdparty` 自动构建：
- **OpenBLAS** - 线性代数
- **Eigen** - C++ 线性代数模板库
- **Fusion** - 传感器融合
- **IIR1** - 数字滤波（Butterworth）
- **GeographicLib** - 地理坐标计算
- **DLib** - 机器学习（步长回归）
- **FMM** - 快速地图匹配（依赖 GDAL, PROJ, GEOS, SQLite）

## 集成

库导出：
- 头文件: `include/FmPDR/fm_pdr.h` (C API)
- 库文件: `lib/libpdr.a`

参见 `example/plain_c/main.c` 集成示例。

## 代码规范

- C++17 标准
- 对外集成使用 C API (`fm_pdr.h`)
- 内部类使用 `CFm` 前缀
- Debug 构建包含 `-Wall -gdwarf-2 -fstack-protector-all -g`
- Release 构建使用 `-O2`
