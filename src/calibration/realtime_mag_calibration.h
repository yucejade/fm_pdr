#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace chrono;

class RealTimeMagCalibrator
{
private:
    vector< Vector3d >       window;             // 滑动窗口（保存最新N个数据）
    int                      windowSize;         // 窗口大小（推荐40）
    double                   calibrateInterval;  // 校准间隔（秒）
    steady_clock::time_point lastCalibrateTime;  // 上次校准时间
    Matrix3d                 softIron;           // 当前软铁矩阵
    Vector3d                 hardIron;           // 当前硬铁偏移
    bool                     isCalibrated;       // 校准状态

    // 新增：数据有效性检查
    bool isValidData(const Vector3d& mag)
    {
        // 检查异常值（类似 -3276.8 的无效数据）
        if (mag.x() < -1000 || mag.x() > 1000 ||
            mag.y() < -1000 || mag.y() > 1000 || 
            mag.z() < -1000 || mag.z() > 1000) {
            return false;
        }
        
        // 检查数据是否过于接近零（传感器故障）
        double magnitude = mag.norm();
        if (magnitude < 1.0) {
            return false;
        }
        
        return true;
    }

    // 新增：检查数据点分布是否足够分散
    bool isDataWellDistributed()
    {
        if (window.size() < 9) return false; // 需要足够的数据点
        
        // 计算数据点的协方差矩阵
        Vector3d mean = Vector3d::Zero();
        for (const auto& point : window) {
            mean += point;
        }
        mean /= window.size();
        
        Matrix3d covariance = Matrix3d::Zero();
        for (const auto& point : window) {
            Vector3d diff = point - mean;
            covariance += diff * diff.transpose();
        }
        covariance /= window.size();
        
        // 检查协方差矩阵的条件数（反映数据分布）
        JacobiSVD<Matrix3d> svd(covariance);
        double cond = svd.singularValues()(0) / svd.singularValues()(2);
        
        // 如果条件数太大，说明数据点过于集中在某个方向
        return cond < 1000.0;
    }

    bool computeParameters()
    {
        int n = window.size();
        if (n < 9) return false; // 窗口大小需≥9

        // 新增：检查数据分布
        if (!isDataWellDistributed()) {
            std::cerr << "Warning: 数据点分布不足，请在不同方向移动设备\n";
            return false;
        }

        // 1. 构造设计矩阵M（n行×10列，对应椭球方程参数）
        MatrixXd M(n, 10);
        for (int i = 0; i < n; ++i) {
            double x = window[i].x(), y = window[i].y(), z = window[i].z();
            M.row(i) << x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, 1;
        }

        // 2. SVD求解最小二乘解（取V的最后一列，对应最小特征值）
        JacobiSVD<MatrixXd> svd(M, ComputeFullV);
        VectorXd p = svd.matrixV().col(9);

        // 🔧 修改：确保所有二次项系数为正（椭球方程要求）
        if (p[0] < 0 || p[1] < 0 || p[2] < 0) {
            p = -p;
        }

        // 3. 构造二次项矩阵A（对称，包含交叉项）
        Matrix3d A;
        A << p[0], p[3]/2, p[4]/2,  // x²项、xy项（半值）、xz项（半值）
             p[3]/2, p[1], p[5]/2,  // xy项（半值）、y²项、yz项（半值）
             p[4]/2, p[5]/2, p[2];  // xz项（半值）、yz项（半值）、z²项

        // // 🔧 修改：检查A是否可逆（数据点足够分散）
        // double detA = A.determinant();
        // if (fabs(detA) < 1e-6) {
        //     std::cerr << "Warning: 数据点过于集中，无法拟合椭球(" << std::to_string(detA) << ")\n";
        //     return false;
        // }

        // 4. 构造线性项向量B（x、y、z的一次项系数）
        Vector3d B(p[6], p[7], p[8]);

        // 5. 计算硬铁偏移（H = -0.5 * A⁻¹ * B）
        Vector3d H = -0.5 * A.inverse() * B;

        // 6. 计算椭球半径平方（R² = 0.25*Bᵀ*A⁻¹*B - 常数项）
        double R2 = 0.25 * B.transpose() * A.inverse() * B - p[9];
        // 🔧 修改：检查R²有效性（椭球必须存在）
        if (R2 <= 0) {
            std::cerr << "Warning: 拟合的椭球不存在（R²≤0）！\n";
            return false;
        }

        // 7. 构造软铁矩阵（S = A / R²，对称正定）
        Matrix3d S = A / R2;
        // 🔧 修改：检查S是否正定（符合物理意义）
        Eigen::SelfAdjointEigenSolver<Matrix3d> eigensolver(S);
        if (eigensolver.eigenvalues().minCoeff() <= 0) {
            std::cerr << "Warning: 软铁矩阵非正定，拟合失败！\n";
            return false;
        }

        // 8. 更新校准参数（仅当所有验证通过时）
        softIron = S;   // 完整软铁矩阵（非对角阵）
        hardIron = H;   // 正确硬铁偏移
        return true;
    }
public:
    // 构造函数：指定窗口大小和校准间隔
    RealTimeMagCalibrator( int winSize = 40, double interval = 1.0 ) : windowSize( winSize ), calibrateInterval( interval ), isCalibrated( false )
    {
        if ( winSize < 9 )
            throw invalid_argument( "Window size must be >=9" );
        softIron.setIdentity();
        hardIron.setZero();
        lastCalibrateTime = steady_clock::now();
    }

    // 喂入原始数据（维护滑动窗口）
    void feed( const Vector3d& raw )
    {
        // 新增：数据有效性检查
        if (!isValidData(raw)) {
            std::cerr << "Warning: 跳过无效数据点 (" << raw.x() << ", " << raw.y() << ", " << raw.z() << ")\n";
            return;
        }

        window.push_back( raw );
        if ( (int)window.size() > windowSize )
        {
            window.erase( window.begin() );  // 移除 oldest 数据
        }
    }

    // 检查是否需要校准（定时触发）
    bool needCalibrate()
    {
        auto               now     = steady_clock::now();
        duration< double > elapsed = now - lastCalibrateTime;
        bool time_status = elapsed.count() >= calibrateInterval;
        bool window_status = ((int)window.size() >= windowSize);
        
        return time_status && window_status;
    }

    // 执行校准（返回是否成功）
    bool calibrate()
    {
        if ( computeParameters() )
        {
            lastCalibrateTime = steady_clock::now();
            isCalibrated      = true;
            return true;
        }
        // 校准失败，恢复未校准状态
        isCalibrated = false;
        return false;
    }

    bool calibrateHardIronAndScale() {
        // 方法：假设软铁矩阵为对角阵，只校准硬铁偏移和缩放因子

        if (window.size() < 9) return false;

        // 1. 计算硬铁偏移（数据点的中心）
        Vector3d sum = Vector3d::Zero();
        for (const auto& point : window) {
            sum += point;
        }
        Vector3d H = sum / window.size();

        // 2. 计算各轴的标准差作为缩放因子
        Vector3d variance = Vector3d::Zero();
        for (const auto& point : window) {
            Vector3d diff = point - H;
            variance.x() += diff.x() * diff.x();
            variance.y() += diff.y() * diff.y();
            variance.z() += diff.z() * diff.z();
        }
        variance /= window.size();

        // 3. 计算平均半径
        double avgRadius = 0;
        for (const auto& point : window) {
            Vector3d diff = point - H;
            avgRadius += diff.norm();
        }
        avgRadius /= window.size();

        // 4. 构造对角软铁矩阵
        Matrix3d S = Matrix3d::Zero();
        S(0,0) = avgRadius / sqrt(variance.x());
        S(1,1) = avgRadius / sqrt(variance.y());
        S(2,2) = avgRadius / sqrt(variance.z());

        // 避免过度缩放
        for (int i = 0; i < 3; ++i) {
            if (S(i,i) < 0.1 || S(i,i) > 10.0) {
                S(i,i) = 1.0;  // 恢复为1
            }
        }

        // 5. 验证参数
        Vector3d test = S * (window[0] - H);
        if (test.norm() < 0.1 || test.norm() > 10.0) {
            return false;
        }

        softIron = S;
        hardIron = H;

        double magnitude = std::sqrt(S(0,0) * S(0,0) + S(1,1) * S(1,1) + S(2,2) * S(2,2));
        std::cout << "简化校准成功！硬铁偏移: (" << H.transpose() 
                  << "), 缩放因子: (" << S(0,0) << ", " << S(1,1) << ", " << S(2,2) << ", " << magnitude << ")\n";

        return true;
    }

    // 应用当前校准参数
    Vector3d apply( const Vector3d& raw )
    {
        // 未校准则返回原始数据
        if ( ! isCalibrated )
            return raw;
        return softIron * ( raw - hardIron );
    }

    // 获取当前参数（用于输出）
    Matrix3d getSoftIron() const
    {
        return softIron;
    }
    Vector3d getHardIron() const
    {
        return hardIron;
    }
    bool isReady() const
    {
        return isCalibrated;
    }
};