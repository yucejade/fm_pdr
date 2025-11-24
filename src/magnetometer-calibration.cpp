#include "magnetometer-calibration.h"
#include "exception.h"
#include <Spectra/GenEigsSolver.h>
#include <iostream>
#include <rapidcsv.h>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Spectra;

CFmMagnetometerCalibration::CFmMagnetometerCalibration( const string& calibration_path )
{
    LoadCalibrationResult( calibration_path );

    auto Qinv = m_result.m_Q.inverse();
    m_b       = -1 * ( Qinv * m_result.m_n );
    m_Ainv    = ( 1 / ( std::sqrt( m_result.m_n.transpose() * ( Qinv * m_result.m_n ) - m_result.m_d ) ) * m_result.m_Q.sqrt() ).real();
}

CFmMagnetometerCalibration::CFmMagnetometerCalibration( const string& mag_path, const string& calibration_path )
{
    Calibration( mag_path, calibration_path );
}

CFmMagnetometerCalibration::~CFmMagnetometerCalibration() {}

void CFmMagnetometerCalibration::Calibration( double& mag_x, double& mag_y, double& mag_z )
{
    Eigen::VectorXd h( 3 );
    h << mag_x, mag_y, mag_z;

    Eigen::VectorXd h_b  = h - m_b;
    auto            hHat = m_Ainv * h_b;

    mag_x = hHat[0];
    mag_y = hHat[1];
    mag_z = hHat[2];
}

void CFmMagnetometerCalibration::Calibration( const string& mag_path, const string& calibration_path )
{
    std::vector< double > magXVec;
    std::vector< double > magYVec;
    std::vector< double > magZVec;

    ReadMagneticSensorData( mag_path, magXVec, magYVec, magZVec );

    Eigen::VectorXd mag_x = Eigen::Map< Eigen::VectorXd, Eigen::Unaligned >( magXVec.data(), magXVec.size() );
    Eigen::VectorXd mag_y = Eigen::Map< Eigen::VectorXd, Eigen::Unaligned >( magYVec.data(), magYVec.size() );
    Eigen::VectorXd mag_z = Eigen::Map< Eigen::VectorXd, Eigen::Unaligned >( magZVec.data(), magZVec.size() );

    FitEllipsoid( mag_x, mag_y, mag_z );

    SaveCalibrationResult( calibration_path );
}

void CFmMagnetometerCalibration::ReadMagneticSensorData( const std::string& filePath, std::vector< double >& magXVec, std::vector< double >& magYVec, std::vector< double >& magZVec )
{
    // 1. 创建CSV文档对象（指定表头行为第0行）
    rapidcsv::Document doc( filePath, rapidcsv::LabelParams( 0 ) );

    // 2. 按列名提取数据（自动转换为double类型）
    magXVec = doc.GetColumn< double >( "X (µT)" );  // 提取"X (µT)"列的所有数据
    magYVec = doc.GetColumn< double >( "Y (µT)" );  // 提取"Y (µT)"列的所有数据
    magZVec = doc.GetColumn< double >( "Z (µT)" );  // 提取"Z (µT)"列的所有数据
}

void CFmMagnetometerCalibration::FitEllipsoid( Eigen::VectorXd& magX, Eigen::VectorXd& magY, Eigen::VectorXd& magZ )
{
    // 椭球拟合至少需要9个点，新增输入校验
    const int num_points = magX.size();
    if ( num_points < 9 )
        throw std::invalid_argument( "Ellipsoid fitting requires at least 9 data points." );

    // 1. 构造椭球方程的设计矩阵D
    Eigen::VectorXd a1  = magX.array().square();
    Eigen::VectorXd a2  = magY.array().square();
    Eigen::VectorXd a3  = magZ.array().square();
    Eigen::VectorXd a4  = 2 * ( magY.array() * magZ.array() );
    Eigen::VectorXd a5  = 2 * ( magX.array() * magZ.array() );
    Eigen::VectorXd a6  = 2 * ( magX.array() * magY.array() );
    Eigen::VectorXd a7  = 2 * magX;
    Eigen::VectorXd a8  = 2 * magY;
    Eigen::VectorXd a9  = 2 * magZ;
    Eigen::VectorXd a10 = Eigen::VectorXd::Ones( num_points );

    Eigen::MatrixXd D( 10, num_points );
    D << a1.transpose(), a2.transpose(), a3.transpose(), a4.transpose(), a5.transpose(), a6.transpose(), a7.transpose(), a8.transpose(), a9.transpose(), a10.transpose();

    // 2. 构造约束矩阵C
    Eigen::MatrixXd C( 6, 6 );
    C << -1, 1, 1, 0, 0, 0, 1, -1, 1, 0, 0, 0, 1, 1, -1, 0, 0, 0, 0, 0, 0, -4, 0, 0, 0, 0, 0, 0, -4, 0, 0, 0, 0, 0, 0, -4;

    // 3. 计算散点矩阵S及子矩阵
    Eigen::MatrixXd S   = D * D.transpose();
    Eigen::MatrixXd S11 = S.topLeftCorner( 6, 6 );
    Eigen::MatrixXd S12 = S.topRightCorner( 6, 4 );
    Eigen::MatrixXd S21 = S.bottomLeftCorner( 4, 6 );
    Eigen::MatrixXd S22 = S.bottomRightCorner( 4, 4 );

    // 4. 求解约束特征值问题
    Eigen::MatrixXd                                              temp = C.inverse() * ( S11 - S12 * S22.inverse() * S21 );
    Spectra::DenseGenMatProd< double >                           op( temp );
    Spectra::GenEigsSolver< Spectra::DenseGenMatProd< double > > eigs( op, 1, 6 );  // 求解1个最大实部特征值

    eigs.init();
    eigs.compute( Spectra::SortRule::LargestReal );

    // 5. 提取特征向量
    if ( eigs.info() != Spectra::CompInfo::Successful )
        throw std::runtime_error( "Ellipsoid fitting failed: Eigenvalue computation unsuccessful." );

    Eigen::VectorXd u1 = eigs.eigenvectors().real();  // 特征向量（6维）
    Eigen::VectorXd u2 = -S22.inverse() * S21 * u1;   // 补充向量（4维）
    Eigen::VectorXd u( 10 );                          // 合并为10维向量
    u << u1, u2;

    // 6. 填充校准结果到result
    m_result.m_Q << u( 0 ), u( 5 ), u( 4 ),  // 3x3校准矩阵（对称矩阵）
        u( 5 ), u( 1 ), u( 3 ), u( 4 ), u( 3 ), u( 2 );

    m_result.m_n << u( 6 ), u( 7 ), u( 8 );  // 3维偏移向量（椭球中心）
    m_result.m_d = u( 9 );                   // 缩放因子（球半径）

    // 7. 生成时间戳
    m_result.m_timestamp = GetTimestamp();

    // 8. 计算误差
    ComputeError( magX, magY, magZ );
}

void CFmMagnetometerCalibration::ComputeError( Eigen::VectorXd& magX, Eigen::VectorXd& magY, Eigen::VectorXd& magZ )
{
    auto Qinv = m_result.m_Q.inverse();

    Eigen::VectorXd b = -1 * ( Qinv * m_result.m_n );

    // std::cout << "b : = " << get_shape(b) << std::endl;

    Eigen::MatrixXd Ainv = ( 1 / ( std::sqrt( m_result.m_n.transpose() * ( Qinv * m_result.m_n ) - m_result.m_d ) ) * m_result.m_Q.sqrt() ).real();

    // std::cout<< "tmp: " << Ainv << std::endl;

    Eigen::VectorXd calibX( magX.size() );
    Eigen::VectorXd calibY( magY.size() );
    Eigen::VectorXd calibZ( magZ.size() );

    float totalError = 0;
    for ( int i = 0; i < magY.size(); i++ )
    {
        Eigen::VectorXd h( 3 );
        h << magX( i ), magY( i ), magZ( i );

        Eigen::VectorXd h_b  = h - b;
        auto            hHat = Ainv * h_b;

        auto  mag   = hHat.transpose().dot( hHat );
        float error = ( mag - 1 ) * ( mag - 1 );
        totalError += error;
    }

    std::cout << "Total Error " << totalError << std::endl;
}

// 生成时间戳（格式：YYYY-MM-DD HH:MM:SS）
std::string CFmMagnetometerCalibration::GetTimestamp()
{
    time_t now = time( nullptr );
    char   buf[ 20 ];
    strftime( buf, sizeof( buf ), "%Y-%m-%d %H:%M:%S", localtime( &now ) );
    return buf;
}

void CFmMagnetometerCalibration::SaveCalibrationResult( const string& filename )
{
    // 1. 创建JSON根对象
    Document doc;
    doc.SetObject();
    Document::AllocatorType& allocator = doc.GetAllocator();

    // 2. 添加时间戳（必须非空，由getTimestamp生成）
    Value timestamp( kStringType );
    timestamp.SetString( m_result.m_timestamp.c_str(), allocator );
    doc.AddMember( "timestamp", timestamp, allocator );

    // 3. 添加m_Q（3x3矩阵→JSON二维数组）
    Value m_Q( kArrayType );
    for ( int i = 0; i < 3; ++i )
    {
        Value row( kArrayType );
        for ( int j = 0; j < 3; ++j )
            row.PushBack( m_result.m_Q( i, j ), allocator );

        m_Q.PushBack( row, allocator );
    }
    doc.AddMember( "m_Q", m_Q, allocator );

    // 4. 添加m_n（3维向量→JSON一维数组）
    Value m_n( kArrayType );
    for ( int i = 0; i < 3; ++i )
        m_n.PushBack( m_result.m_n( i ), allocator );
    doc.AddMember( "m_n", m_n, allocator );

    // 5. 添加m_d（double→JSON数值）
    doc.AddMember( "m_d", m_result.m_d, allocator );

    // 6. 写入JSON文件
    FILE* fp = fopen( filename.c_str(), "w" );
    if ( ! fp )
        throw SensorException( SensorException::CALIBRATION_SAVE_ERROR, "Error: 无法打开文件 " + filename );

    char                      buffer[ 65536 ];  // 缓冲区（根据文件大小调整）
    FileWriteStream           ws( fp, buffer, sizeof( buffer ) );
    Writer< FileWriteStream > writer( ws );

    // 检查JSON写入是否成功
    if ( ! doc.Accept( writer ) )
    {
        fclose( fp );
        throw SensorException( SensorException::CALIBRATION_SAVE_ERROR, "Error: 写入JSON文件 " + filename + " 失败" );
    }

    fclose( fp );
}

void CFmMagnetometerCalibration::LoadCalibrationResult( const string& filename )
{
    // 1. 读取JSON文件
    FILE* fp = fopen( filename.c_str(), "r" );
    if ( ! fp )
        throw SensorException( SensorException::CALIBRATION_LOAD_ERROR, "Error: 无法打开文件 " + filename );

    char           buffer[ 65536 ];
    FileReadStream rs( fp, buffer, sizeof( buffer ) );
    Document       doc;
    doc.ParseStream( rs );
    fclose( fp );

    // 2. 严格验证JSON格式
    if ( ! doc.IsObject() || ! doc.HasMember( "timestamp" ) || ! doc[ "timestamp" ].IsString() || ! doc.HasMember( "m_Q" ) || ! doc[ "m_Q" ].IsArray() || doc[ "m_Q" ].Size() != 3 || ! doc.HasMember( "m_n" ) || ! doc[ "m_n" ].IsArray() || doc[ "m_n" ].Size() != 3 || ! doc.HasMember( "m_d" )
         || ! doc[ "m_d" ].IsDouble() )
        throw SensorException( SensorException::CALIBRATION_LOAD_ERROR, "Error: JSON格式无效或参数缺失" );

    // 3. 验证m_Q的每个行元素是否为3维数组
    const Value& m_Q_json = doc[ "m_Q" ];
    for ( int i = 0; i < 3; ++i )
    {
        if ( ! m_Q_json[ i ].IsArray() || m_Q_json[ i ].Size() != 3 )
            throw SensorException( SensorException::CALIBRATION_LOAD_ERROR, "Error: m_Q第" + to_string( i + 1 ) + "行不是3维数组（无效维度）" );
    }

    // 4. 提取时间戳
    m_result.m_timestamp = doc[ "timestamp" ].GetString();

    // 5. 提取m_Q（3x3矩阵→JSON二维数组）
    for ( int i = 0; i < 3; ++i )
    {
        const Value& row_json = m_Q_json[ i ];
        for ( int j = 0; j < 3; ++j )
        {
            if ( ! row_json[ j ].IsDouble() )
                throw SensorException( SensorException::CALIBRATION_LOAD_ERROR, "Error: m_Q(" + to_string( i + 1 ) + "," + to_string( j + 1 ) + ")不是数字（无效值）" );

            m_result.m_Q( i, j ) = row_json[ j ].GetDouble();  // 固定大小，无越界风险
        }
    }

    // 6. 提取m_n（3维向量→JSON一维数组）
    const Value& m_n_json = doc[ "m_n" ];
    for ( int i = 0; i < 3; ++i )
    {
        if ( ! m_n_json[ i ].IsDouble() )
            throw SensorException( SensorException::CALIBRATION_LOAD_ERROR, "Error: m_n第" + to_string( i + 1 ) + "个元素不是数字（无效值）" );

        m_result.m_n( i ) = m_n_json[ i ].GetDouble();  // 固定大小，无越界风险
    }

    // 7. 提取m_d（double→JSON数值）
    m_result.m_d = doc[ "m_d" ].GetDouble();
}