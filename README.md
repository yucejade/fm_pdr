# Main Page {#mainpage}

# PDR算法库

## 项目概述
本项目基于树莓派5硬件平台，结合TDK40607P(IMU)和MMC56x3磁力计，采用行人航迹跟踪(PDR)方式实现惯性导航功能和程序演示。
本项目参考了[NJU-AML2022的PDR项目](https://github.com/nju-aml2022/Pedestrian-Dead-Reckoning-PDR/tree/main#)，并将其从Python环境移植到C++语言环境中。

## 主要特性

- 支持数据文件方式和实时传感器数据采集模式
- 提供完整的C语言API接口
- 支持模型训练与轨迹预测

## 示例程序运行

### 训练PDR模型
```bash
./PDRTest --train-dir "./test_data/train_data"
```

### 根据数据文件预测航迹
```bash
./PDRTest --dataset-dir "./test_data/sensor_data" --output-path "./Trajectory.csv"
```

### 接入传感器实时预测航迹
```bash
./PDRTest -x 32.11199920 -y 118.9528682 --output-path "./Trajectory.csv"
```

### 获取磁力计原始数据
```bash
./mag_calib4 -t 0 -d "./mag_calib_data" -i -1
```

### 根据磁力计数据文件生成校准参数
```bash
./mag_calib4 -t 1 -d "./mag_calib_data/Magnetometer.csv" -o "./mag_calib_data/mag_calibration.csv"
```

### 展示校准前后的实时磁力计数据
```bash
./mag_calib4 -t 2 -c "./mag_calib_data/mag_calibration.csv" -i -1
```

## 编译构建
```bash
./make.sh rebuild
```

### 编译输出内容
构建完成后，在build/package目录下生成：
```text
build/package/
├── bin/                   # 可执行文件
│   ├── PDRTest            # 主测试程序
│   └── mag_calib4         # 磁力计校准测试程序
│   └── [其它测试程序...]   # 其它测试程序
├── conf/                  # 帮助文档
│   └── config.json        # 配置文件
├── docs/                  # 帮助文档
├── lib/                   # 静态库文件
│   ├── libpdr.a           # PDR核心库
│   └── [第三方库文件...]
└── include/               # 头文件
    ├── fm_pdr.h           # PDR接口头文件
    └── [第三方头文件...]
```

## 第三方依赖库
构建过程会自动编译以下依赖库：
- OpenBLAS: 线性代数计算库
- Eigen: C++模板库，用于线性代数运算
- Fusion: 传感器融合库
- IIR1: 数字滤波器库
- GeographicLib: 地理坐标计算库
- DLib: 机器学习库
- RapidCSV: CSV文件解析库
- RapidJSON: JSON解析库
- Libgpiod: GPIO设备控制库
- ConcurrentQueue: 并发队列库

## 集成说明
编译后，在include目录下生成fm_pdr.h头文件，lib目录下生成libpdr.a库文件用于提供给第三方集成。
本算法大致工作过程如下：
- 准备阶段
    1. 采集传感器原始数据
    2. 校准磁力计数据，并将校准结果以文件形式输出
    3. 训练算法模型，并将训练结果以文件形式输出
- 行人航迹预测(PDR)
    1. 输入内容为配置文件，原始传感器数据，磁力计校准结果，模型训练结果
    2. 根据输入内容结合PDR算法预测行人位置和方向


### 配置文件说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `sample_rate` | 50 | 采样频率（Hz） |
| `pdr_duration` | 4 | PDR持续时间（秒） |
| `model_name` | "Linear" | 使用的模型名称 |
| `model_file_name` | "model.dat" | 模型文件名 |
| `clean_start` | 4 | 开始清理的点数 |
| `clean_end` | 0 | 结束清理的点数 |
| `default_east_point` | 50 | 默认东向点 |
| `move_average` | 10 | 移动平均窗口大小 |
| `min_distance` | 20 | 最小距离（米） |
| `distance_frac_step` | 4.0 | 距离分数步长 |
| `optimized_mode_ratio` | 0.95 | 优化模式比例 |
| `butter_wn` | 0.0035 | 巴特沃斯滤波器截止频率 |
| `least_start_point` | 50 | 最少起始点数 |

这些配置项通常通过`conf/config.json`文件进行调整。

## API文档
请参考package/docs目录
