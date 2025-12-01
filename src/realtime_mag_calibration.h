#include <Eigen/Dense>
#include <chrono>
#include <vector>

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

    // 内部函数：用窗口数据计算校准参数（9参数椭球拟合）
    bool computeParameters()
    {
        int n = window.size();
        if ( n < 9 )
            return false;

        // 构造设计矩阵M（n行×10列）
        MatrixXd M( n, 10 );
        for ( int i = 0; i < n; ++i )
        {
            double x = window[ i ].x(), y = window[ i ].y(), z = window[ i ].z();
            M.row( i ) << x * x, y * y, z * z, x * y, x * z, y * z, x, y, z, 1;
        }

        // SVD求解最小二乘解
        JacobiSVD< MatrixXd > svd( M, ComputeFullV );
        VectorXd              p = svd.matrixV().col( 9 );
        if ( p[ 0 ] < 0 )
            p = -p;

        // 提取软铁矩阵（简化为对角阵，适合实时）
        softIron << sqrt( p[ 0 ] ), 0, 0, 0, sqrt( p[ 1 ] ), 0, 0, 0, sqrt( p[ 2 ] );

        // 提取硬铁偏移
        Vector3d GHI( p[ 6 ], p[ 7 ], p[ 8 ] );
        hardIron = -0.5 * softIron.inverse() * GHI;

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
        return false;
    }

    // 应用当前校准参数
    Vector3d apply( const Vector3d& raw )
    {
        if ( ! isCalibrated )
            return raw;  // 未校准则返回原始数据
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