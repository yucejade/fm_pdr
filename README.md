# Main Page {#mainpage}

# PDR算法库

## 项目概述

本项目参考了[NJU-AML2022的PDR项目](https://github.com/nju-aml2022/Pedestrian-Dead-Reckoning-PDR/tree/main#)，并将其从Python环境移植到C++语言环境中。

## 主要特性

- 支持数据文件方式和实时传感器数据采集
- 提供完整的C语言API接口
- 支持模型训练与轨迹预测

## 示例程序运行

### 训练模型
```bash
./PDRTest --train-dir "./test_data/train_data"
```

### 数据文件模式
```bash
./PDRTest --dataset-dir "./test_data/sensor_data" --output-path "./Trajectory.csv"
```

### 接入传感器的实时模式
```bash
./PDRTest -x 32.11199920 -y 118.9528682 --output-path "./Trajectory.csv"
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
│   └── PDRTestFromFile    # 文件模式测试程序
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

## API文档
请参考package/docs目录
