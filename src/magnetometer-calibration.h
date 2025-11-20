#include <Eigen/Dense>
#include <ctime>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include <string>

using namespace std;
using namespace Eigen;
using namespace rapidjson;

// 校准结果结构体
struct CalibrationResult
{
    Matrix3d m_Q;          // 3x3校准矩阵（椭球到球的转换矩阵）
    Vector3d m_n;          // 3维偏移向量（椭球中心）
    double   m_d;          // 缩放因子（球半径）
    string   m_timestamp;  // 校准时间戳（ISO格式，如"2025-11-19 14:30:00"）
};

class CFmMagnetometerCalibration
{
public:
    CFmMagnetometerCalibration( const string& calibration_path );
    CFmMagnetometerCalibration( const string& mag_path, const string& calibration_path );
    ~CFmMagnetometerCalibration();

    // 校准函数
    void Calibration( double &mag_x, double &mag_y, double &mag_z );

private:
    CalibrationResult m_result;
    Eigen::VectorXd m_b;
    Eigen::MatrixXd m_Ainv;

private:
    // 核心校准函数
    void Calibration( const string& mag_path, const string& calibration_path );
    void FitEllipsoid( Eigen::VectorXd& magX, Eigen::VectorXd& magY, Eigen::VectorXd& magZ );
    void ComputeError( VectorXd& magX, VectorXd& magY, VectorXd& magZ );

    // 读取磁力计文件数据
    void ReadMagneticSensorData( const std::string& filePath, std::vector< double >& magXVec, std::vector< double >& magYVec, std::vector< double >& magZVec );

    // 保存/加载校准结果
    void SaveCalibrationResult( const string& filename );
    void LoadCalibrationResult( const string& filename );

    // 生成时间戳
    string GetTimestamp();    
};
