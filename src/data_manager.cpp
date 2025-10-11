#include "data_manager.h"
#include "fm_pdr.h"
#include <GeographicLib/Geodesic.hpp>

using namespace rapidcsv;

CFmDataManager::CFmDataManager() : m_config( nullptr ), m_data_type( DATA_TYPE_FILE ), m_train_data_size( 0 ) {}
CFmDataManager::CFmDataManager( const PDRConfig& config, DataType type, size_t train_data_size ) : m_config( &config ), m_data_type( type ), m_train_data_size( train_data_size )
{
    FusionOffsetInitialise( &m_offset, config.sample_rate );
    FusionAhrsInitialise( &m_ahrs );

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .convention            = FusionConventionEnu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * ( unsigned int )config.sample_rate, /* 5 seconds */
    };
    FusionAhrsSetSettings( &m_ahrs, &settings );
}
CFmDataManager::~CFmDataManager() {}

Eigen::MatrixXd CFmDataManager::get_gravity_with_ahrs( Eigen::MatrixXd& accelerometer, Eigen::MatrixXd& gyroscope, Eigen::MatrixXd& magnetometer )
{
    const int rows = accelerometer.rows();
    MatrixXd  gravity( rows, 3 );

    for ( int i = 0; i < rows; ++i )
    {
        FusionVector acc  = { { static_cast< float >( accelerometer( i, 0 ) ), static_cast< float >( accelerometer( i, 1 ) ), static_cast< float >( accelerometer( i, 2 ) ) } };
        FusionVector gyro = { { static_cast< float >( gyroscope( i, 0 ) ), static_cast< float >( gyroscope( i, 1 ) ), static_cast< float >( gyroscope( i, 2 ) ) } };
        FusionVector mag  = { { static_cast< float >( magnetometer( i, 0 ) ), static_cast< float >( magnetometer( i, 1 ) ), static_cast< float >( magnetometer( i, 2 ) ) } };

        // Apply calibration
        gyro = FusionCalibrationInertial( gyro, m_gyroscopeMisalignment, m_gyroscopeSensitivity, m_gyroscopeOffset );
        acc  = FusionCalibrationInertial( acc, m_accelerometerMisalignment, m_accelerometerSensitivity, m_accelerometerOffset );
        mag  = FusionCalibrationMagnetic( mag, m_softIronMatrix, m_hardIronOffset );

        // Update gyroscope offset correction algorithm
        gyro = FusionOffsetUpdate( &m_offset, gyro );

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        const float deltaTime = 1.0f / m_config->sample_rate;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate( &m_ahrs, gyro, acc, mag, deltaTime );

        // Get gravity vector
        const FusionVector grav = FusionAhrsGetGravity( &m_ahrs );
        gravity( i, 0 )         = grav.axis.x;
        gravity( i, 1 )         = grav.axis.y;
        gravity( i, 2 )         = grav.axis.z;
    }

    return gravity;
}

void CFmDataManager::set_location_output( const Eigen::MatrixXd& trajectory )
{
    const int       n               = trajectory.rows();
    Eigen::MatrixXd location_output = Eigen::MatrixXd::Constant( n, 8, std::numeric_limits< double >::quiet_NaN() );

    // 填充矩阵数据
    for ( int i = 0; i < n; ++i )
    {
        const int idx             = i;
        location_output( idx, 0 ) = trajectory( i, 0 );  // 时间戳
        location_output( idx, 1 ) = trajectory( i, 1 );  // 经度→纬度
        location_output( idx, 2 ) = trajectory( i, 2 );  // 维度→经度
        location_output( idx, 5 ) = trajectory( i, 3 );  // 运动方向
    }

    // 保存成 CSV 文件
    const vector< string > col_names = { "Time (s)", "Latitude (°)", "Longitude (°)", "Height (m)", "Velocity (m/s)", "Direction (°)", "Horizontal Accuracy (m)", "Vertical Accuracy (°)" };
    save_to_csv( location_output, "Location_output.csv", col_names );
}

void CFmDataManager::eval_model( const Eigen::MatrixXd& trajectory ) const
{
    // 检查是否有有效位置数据
    if ( ! m_have_location_true )
    {
        cout << "No location valid" << endl;
        return;
    }

    // 计算并获取各项评估指标
    double dist_error = get_dist_error( trajectory );
    double dir_error  = get_dir_error( trajectory );
    double dir_ratio  = get_dir_ratio( trajectory );

    // 输出评估结果
    cout << "Distances error: " << dist_error << endl;
    cout << "Direction error: " << dir_error << endl;
    cout << "Direction ratio: " << dir_ratio << endl;
}

bool CFmDataManager::save_to_csv( const MatrixXd& matrix, const string& filename, const vector< string >& col_names )
{
    // 验证列数匹配
    const int cols = matrix.cols();
    if ( cols != static_cast< int >( col_names.size() ) )
        return false;

    // 创建rapidcsv文档对象
    Document doc;

    // 设置列名
    for ( size_t c = 0; c < col_names.size(); ++c )
        doc.SetColumnName( c, col_names[ c ] );

    // 将Eigen矩阵转换为按列存储的vector<vector>
    const int                  rows = matrix.rows();
    vector< vector< string > > col_data( cols );

    // 按列提取数据（rapidcsv需要按列存储的数据）
    for ( int c = 0; c < cols; ++c )
    {
        col_data[ c ].reserve( rows );
        for ( int r = 0; r < rows; ++r )
        {
            string val;

            if ( std::isnan( matrix( r, c ) ) )
            {
                val.clear();  // 0.0转为空字段
            }
            else
            {
                std::ostringstream oss;
                // oss << std::uppercase << std::scientific << std::setprecision(9) << matrix(r, c);
                oss << std::setprecision( 9 ) << matrix( r, c );
                val = oss.str();
            }
            col_data[ c ].push_back( val );
        }
    }

    // 设置列数据
    for ( int c = 0; c < cols; ++c )
        doc.SetColumn( c, col_data[ c ] );

    // 保存CSV文件
    doc.Save( filename );

    return true;
}

// 注意插值相关处理
double CFmDataManager::get_dir_error( const Eigen::MatrixXd& trajectory ) const
{
    // 初始化方向误差列表
    vector< double > dir_list;

    // 获取有效数据范围
    int start_index = m_train_data_size;
    int end_index   = m_time_location_true.size();

    // 循环计算每个点的方向误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // cout << "Valid Direction: " << m_direction_true[i] << ", Output Direction: " <<positions[ i ].direction << endl;

        // 计算原始差值
        double raw_diff = abs( m_direction_true[ i ] - trajectory(i, 3) );

        // 计算循环差值（考虑角度周期性）
        double cyclic_diff = 360.0 - raw_diff;

        // 取两者中的较小值（实际角度差）
        double dir_error = min( raw_diff, cyclic_diff );

        // 将当前点的误差添加到列表
        dir_list.push_back( dir_error );
    }

    // 检查是否有有效数据点
    if ( dir_list.empty() )
    {
        cout << "No valid direction data points for error calculation" << endl;
        return -1.0;  // 返回错误值
    }

    // 计算平均误差
    double sum = 0.0;
    for ( double error : dir_list )
        sum += error;
    double average_error = sum / dir_list.size();

    // 返回平均方向误差
    return average_error;
}

double CFmDataManager::get_dir_ratio( const Eigen::MatrixXd& trajectory, double diff ) const
{
    // 初始化方向误差列表
    vector< double > dir_list;

    // 获取有效数据范围
    int start_index = m_train_data_size;
    int end_index   = m_time_location_true.size();

    // 循环计算每个点的方向误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // cout << "Valid Direction: " << m_direction_true[ i ] << ", Output Direction: " << trajectory(i, 3) << endl;

        // 计算原始差值
        double raw_diff = abs( m_direction_true[ i ] - trajectory(i, 3) );

        // 计算循环差值（考虑角度周期性）
        double cyclic_diff = 360.0 - raw_diff;

        // 取两者中的较小值（实际角度差）
        double dir_error = min( raw_diff, cyclic_diff );

        // 将当前点的误差添加到列表
        dir_list.push_back( dir_error );
    }

    // 检查是否有有效数据点
    if ( dir_list.empty() )
    {
        cout << "No valid direction data points for ratio calculation" << endl;
        return -1.0;  // 返回错误值
    }

    // 计算满足条件的比例
    int count = count_if( dir_list.begin(), dir_list.end(),
                          [ diff ]( double error )
                          {
                              return error <= diff;
                          } );

    double ratio = static_cast< double >( count ) / dir_list.size();

    // 返回比例值
    return ratio;
}

double CFmDataManager::get_dist_error( const Eigen::MatrixXd& trajectory ) const
{
    // 2. 获取Geodesic实例（使用WGS84椭球体）
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // 3. 初始化距离列表
    vector< double > dist_list;

    // 4. 获取有效数据范围
    int start_index = m_train_data_size;
    int end_index   = m_time_location_true.size();

    // 5. 循环计算每个点的距离误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // 5.1 获取有效点坐标
        double lat_valid = m_latitude_true[ i ];
        double lon_valid = m_longitude_true[ i ];

        // 5.2 获取输出点坐标
        double lat_output = trajectory(i, 1);
        double lon_output = trajectory(i, 2);

        // cout << "Valid: (" << lat_valid << "," << lon_valid << "), Output: (" << lat_output << "," << lon_output << ")" << endl;

        // 5.3 计算大地距离（geodesic distance）
        double distance_meters;
        geod.Inverse( lat_valid, lon_valid, lat_output, lon_output, distance_meters );

        // 5.4 将距离添加到列表
        dist_list.push_back( distance_meters );
    }

    // 6. 检查是否有有效数据点
    if ( dist_list.empty() )
    {
        cout << "No valid location data points for distance error calculation" << endl;
        return -1.0;  // 返回错误值
    }

    // 7. 计算平均误差
    double sum = 0.0;
    for ( double dist : dist_list )
        sum += dist;
    double average_error = sum / dist_list.size();

    // 8. 返回平均距离误差（单位：米）
    return average_error;
}
