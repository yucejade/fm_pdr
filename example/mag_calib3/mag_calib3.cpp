#include <iostream>
#include <cstdlib>    // 用于生成随机数据
#include <Eigen/Core>
#include "SoftAndHardIronCalibration.h"  // 校准器类
#include "SixParametersCorrector.h"      // 校正器类（含文件读写）

using namespace Boardcore;
using namespace Eigen;

// --------------------------
// 1. 模拟磁力计数据结构（需与实际传感器一致）
// --------------------------
// struct MagnetometerData {
//     float x;  // 原始X轴数据（μT）
//     float y;  // 原始Y轴数据（μT）
//     float z;  // 原始Z轴数据（μT）
// };

int main() {
    // --------------------------
    // 2. 初始化校准器
    // --------------------------
    SoftAndHardIronCalibration calib;

    // --------------------------
    // 3. 模拟磁力计数据（需覆盖球面所有方向，建议100+点）
    // --------------------------
    std::cout << "正在模拟磁力计数据（100个点）..." << std::endl;
    for (int i = 0; i < 100; ++i) {
        MagnetometerData data;
        // 生成[-100, 100]μT的随机球面点（模拟传感器旋转）
        data.magneticFieldX = static_cast<float>(rand()) / RAND_MAX * 200 - 100;
        data.magneticFieldY = static_cast<float>(rand()) / RAND_MAX * 200 - 100;
        data.magneticFieldZ = static_cast<float>(rand()) / RAND_MAX * 200 - 100;
        // 喂入校准器（累积数据）
        calib.feed(data);
    }

    // --------------------------
    // 4. 计算校准参数（软铁增益+硬铁偏移）
    // --------------------------
    SixParametersCorrector result = calib.computeResult();
    std::cout << "校准完成！" << std::endl;

    // --------------------------
    // 5. 保存校准参数到文件（注意：需修正原toFile方法的笔误）
    // --------------------------
    const std::string param_file = "mag_calib_params.csv";
    if (result.toFile(param_file)) {
        std::cout << "校准参数已保存到：" << param_file << std::endl;
    } else {
        std::cerr << "保存校准参数失败！" << std::endl;
        return 1;
    }

    // --------------------------
    // 6. 从文件加载校准参数（验证保存的正确性）
    // --------------------------
    SixParametersCorrector loaded_corrector;
    if (loaded_corrector.fromFile(param_file)) {
        std::cout << "成功加载校准参数！" << std::endl;
        // 打印加载的参数（可选，用于验证）
        std::cout << "加载的硬铁偏移（b）: " << loaded_corrector.getb().transpose() << " μT" << std::endl;
        std::cout << "加载的软铁增益（A）: " << loaded_corrector.getA().transpose() << "（无单位）" << std::endl;
    } else {
        std::cerr << "加载校准参数失败！" << std::endl;
        return 1;
    }

    // --------------------------
    // 7. 示例：使用校准参数校正原始数据
    // --------------------------
    // 假设的原始磁力计数据（从传感器读取）
    MagnetometerData raw_data(0.0f, 120.5f, -80.3f, 50.2f);
    // 转换为Eigen向量（符合correct方法的输入要求）
    Vector3f raw_vec(raw_data.magneticFieldX, raw_data.magneticFieldY, raw_data.magneticFieldZ);
    // 调用校正方法（公式：校正后 = (原始数据 - 偏移) × 增益）
    Vector3f corrected_vec = loaded_corrector.correct(raw_vec);

    // 输出校正结果
    std::cout << "\n=== 数据校正示例 ===" << std::endl;
    std::cout << "原始数据: " << raw_vec.transpose() << " μT" << std::endl;
    std::cout << "校正后数据: " << corrected_vec.transpose() << " μT" << std::endl;

    return 0;
}
