#include "fm_pdr.h"
#include "data_buffer_loader.h"
#include "data_file_loader.h"
#include "data_manager.h"
#include "exception.h"
#include "fm_device_wrapper.h"
#include "json_operator.h"
// #include "magnetometer-calibration.h"
#include "SensorData.h"
#include "SixParametersCorrector.h"
#include "fmm_app.h"
#include "fmm_config.h"
#include "pdr.h"
#include "trajectory_manager.h"
#include <Eigen/Core>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fmm/mm/fmm/ubodt_gen_algorithm.hpp>
#include <fmm/network/network.hpp>
#include <fmm/network/network_graph.hpp>
#include <fstream>
#include <iostream>
#include <moodycamel/concurrentqueue.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

using namespace Boardcore;

typedef enum _FmPDRStatus
{
    PDR_STOPPED,
    PDR_RUNNING
} FmPDRStatus;

typedef struct _NewPosition
{
    Eigen::MatrixXd position;
    Eigen::MatrixXd matched_position;
} NewPosition;

typedef struct _FmPDRHandler
{
    std::string        m_config_dir;        // 配置文件目录
    PDRConfig          m_config;            // PDR配置
    FMMConfig          m_fmm_config;        // FMM配置
    FMMApp             m_fmm;               // FMM模型
    CFmPDR             m_pdr;               // PDR句柄
    StartInfo          m_si;                // 起点信息
    CFmDataManager*    m_data_loader;       // 数据加载器
    char*              m_sensor_data_path;  // PDR数据文件路径
    fm_device_handle_t m_device_handle;     // 设备操作句柄
    // CFmMagnetometerCalibration*                  m_mag_calibration;   // 磁力计校准句柄
    SixParametersCorrector*                                        m_loaded_corrector;  // 矫正器句柄
    int                                                            m_status;            // 0:停止,1:启动
    std::thread                                                    m_worker;            // 子线程句柄
    moodycamel::ConcurrentQueue< std::vector< Eigen::MatrixXd* > > queue;               // 轨迹队列

    // 注意：创建PDR对象时，不能使用传入参数config，需要全局生命周期的m_config
    _FmPDRHandler( const PDRConfig& config, const FMMConfig& fmm_config, const CFmDataManager& train_data, Eigen::MatrixXd& train_position )
        : m_config( config ), m_fmm_config( fmm_config ), m_fmm( fmm_config ), m_pdr( m_config, train_data, train_position ), m_data_loader( nullptr ), m_sensor_data_path( nullptr ), m_loaded_corrector( nullptr ), m_status( PDR_STOPPED )
    {
        memset( &m_device_handle, 0x00, sizeof( m_device_handle ) );
    }
    _FmPDRHandler( const PDRConfig& config, const FMMConfig& fmm_config ) : m_config( config ), m_fmm_config( fmm_config ), m_fmm( fmm_config ), m_pdr( m_config ), m_data_loader( nullptr ), m_sensor_data_path( nullptr ), m_loaded_corrector( nullptr ), m_status( PDR_STOPPED )
    {
        memset( &m_device_handle, 0x00, sizeof( m_device_handle ) );
    }
} FmPDRHandler;

static int eigenToPDRTrajectory( const Eigen::MatrixXd& predict_trajectories, PDRTrajectory** trajectories )
{
    const unsigned long n = predict_trajectories.rows();
    if ( n == 0 )
    {
        *trajectories = nullptr;
        return 0;
    }

    std::unique_ptr< PDRTrajectory > new_traj;
    try
    {
        // 预分配内存
        new_traj = std::make_unique< PDRTrajectory >();

        // 直接将指针指向 Eigen 矩阵的列数据
        new_traj->time      = const_cast< double* >( predict_trajectories.col( 0 ).data() );
        new_traj->x         = const_cast< double* >( predict_trajectories.col( 1 ).data() );
        new_traj->y         = const_cast< double* >( predict_trajectories.col( 2 ).data() );
        new_traj->direction = const_cast< double* >( predict_trajectories.col( 3 ).data() );
        new_traj->length    = n;
        new_traj->ptr       = ( void* )&predict_trajectories;

        *trajectories = new_traj.release();
    }
    catch ( const std::bad_alloc& e )
    {
        // 内存分配失败
        throw MemoryException( MemoryException::ALLOC_FAILED, "Convert PDRTrajectory error" + std::string( e.what() ) );
    }
    catch ( ... )
    {
        // 处理其他异常
        throw;
    }
    return n;
}

// 配套的析构函数
static void free_trajectory( PDRTrajectory* trajectory ) noexcept
{
    if ( ! trajectory )
        return;
    trajectory->time      = nullptr;
    trajectory->x         = nullptr;
    trajectory->y         = nullptr;
    trajectory->direction = nullptr;
    trajectory->length    = 0;
    delete static_cast< Eigen::MatrixXd* >( trajectory->ptr );
    delete trajectory;
}

bool file_exists( const std::string& file_path )
{
    struct stat buffer;
    return ( stat( file_path.c_str(), &buffer ) == 0 );
}

std::vector< std::vector< double > > read_csv_file( const std::string& file_path )
{
    std::vector< std::vector< double > > data;
    std::ifstream                        file( file_path );

    if ( ! file.is_open() )
        throw FileException( FileException::OPEN_FAILED, file_path.c_str() );

    std::string line;
    bool        first_line = true;

    while ( std::getline( file, line ) )
    {
        // 跳过空行和标题行
        if ( line.empty() || first_line )
        {
            first_line = false;
            continue;
        }

        std::vector< double > row;
        std::stringstream     ss( line );
        std::string           cell;

        while ( std::getline( ss, cell, ',' ) )
        {
            try
            {
                row.push_back( std::stod( cell ) );
            }
            catch ( const std::exception& )
            {
                // 转换失败，跳过该单元格
                continue;
            }
        }

        if ( ! row.empty() )
        {
            if ( data.empty() )
            {
                // 初始化列向量
                data.resize( row.size() );
            }

            // 将数据添加到对应的列
            for ( size_t i = 0; i < row.size() && i < data.size(); ++i )
            {
                data[ i ].push_back( row[ i ] );
            }
        }
    }

    file.close();
    return data;
}

void allocate_sensor_arrays( PDRSensorData* sensor_data, int length )
{
    if ( length <= 0 )
        return;

    sensor_data->acc_time = new double[ length ]();
    sensor_data->acc_x    = new double[ length ]();
    sensor_data->acc_y    = new double[ length ]();
    sensor_data->acc_z    = new double[ length ]();

    sensor_data->lacc_time = new double[ length ]();
    sensor_data->lacc_x    = new double[ length ]();
    sensor_data->lacc_y    = new double[ length ]();
    sensor_data->lacc_z    = new double[ length ]();

    sensor_data->gyr_time = new double[ length ]();
    sensor_data->gyr_x    = new double[ length ]();
    sensor_data->gyr_y    = new double[ length ]();
    sensor_data->gyr_z    = new double[ length ]();

    sensor_data->mag_time = new double[ length ]();
    sensor_data->mag_x    = new double[ length ]();
    sensor_data->mag_y    = new double[ length ]();
    sensor_data->mag_z    = new double[ length ]();

    sensor_data->length = length;
}

void allocate_true_arrays( PDRTrueData* true_data, int length )
{
    if ( length <= 0 )
        return;

    true_data->time_location       = new double[ length ]();
    true_data->latitude            = new double[ length ]();
    true_data->longitude           = new double[ length ]();
    true_data->height              = new double[ length ]();
    true_data->velocity            = new double[ length ]();
    true_data->direction           = new double[ length ]();
    true_data->horizontal_accuracy = new double[ length ]();
    true_data->vertical_accuracy   = new double[ length ]();

    true_data->length = length;
}

void cleanup_pdr_data( PDRData* pdr_data )
{
    if ( ! pdr_data )
        return;

    PDRSensorData* sensor_data = &pdr_data->sensor_data;
    if ( sensor_data->acc_time )
        delete[] sensor_data->acc_time;
    if ( sensor_data->acc_x )
        delete[] sensor_data->acc_x;
    if ( sensor_data->acc_y )
        delete[] sensor_data->acc_y;
    if ( sensor_data->acc_z )
        delete[] sensor_data->acc_z;

    if ( sensor_data->gyr_time )
        delete[] sensor_data->gyr_time;
    if ( sensor_data->gyr_x )
        delete[] sensor_data->gyr_x;
    if ( sensor_data->gyr_y )
        delete[] sensor_data->gyr_y;
    if ( sensor_data->gyr_z )
        delete[] sensor_data->gyr_z;

    if ( sensor_data->mag_time )
        delete[] sensor_data->mag_time;
    if ( sensor_data->mag_x )
        delete[] sensor_data->mag_x;
    if ( sensor_data->mag_y )
        delete[] sensor_data->mag_y;
    if ( sensor_data->mag_z )
        delete[] sensor_data->mag_z;

    memset( sensor_data, 0x00, sizeof( PDRSensorData ) );

    PDRTrueData* true_data = &pdr_data->true_data;
    if ( true_data->time_location )
        delete[] true_data->time_location;
    if ( true_data->latitude )
        delete[] true_data->latitude;
    if ( true_data->longitude )
        delete[] true_data->longitude;
    if ( true_data->height )
        delete[] true_data->height;
    if ( true_data->velocity )
        delete[] true_data->velocity;
    if ( true_data->direction )
        delete[] true_data->direction;
    if ( true_data->horizontal_accuracy )
        delete[] true_data->horizontal_accuracy;
    if ( true_data->vertical_accuracy )
        delete[] true_data->vertical_accuracy;

    memset( true_data, 0x00, sizeof( PDRTrueData ) );
}

static std::vector< double > ptr_to_vector( double* ptr, unsigned long len )
{
    if ( ! ptr || len <= 0 )
        return {};
    return std::vector< double >( ptr, ptr + len );
}

static bool validate_sensor_data( double* time, double* x, double* y, double* z, unsigned long len )
{
    return ( len > 0 ) && time && x && y && z;
}

static bool validate_true_data( double* time_location, double* latitude, double* longitude, double* height, double* velocity, double* direction, double* horizontal_accuracy, double* vertical_accuracy, unsigned long len )
{
    return ( len > 0 ) && time_location && latitude && longitude && height && velocity && direction && horizontal_accuracy && vertical_accuracy;
}

std::string quote_header( const std::string& header )
{
    return "\"" + header + "\"";
}

static void append_to_csv( const std::string& full_path, const std::vector< std::pair< std::string, std::vector< double > > >& columns )
{
    // 检查所有列数据长度是否一致，空列自动跳过检查
    size_t expected_size = 0;
    for ( const auto& col : columns )
    {
        if ( ! col.second.empty() )
        {
            expected_size = col.second.size();
            break;
        }
    }
    for ( const auto& col : columns )
    {
        if ( ! col.second.empty() && col.second.size() != expected_size )
            throw DataException( DataException::COLUMN_INCONSISTENT, "Column '" + col.first + "' has size " + std::to_string( col.second.size() ) + " but expected " + std::to_string( expected_size ) );
    }

    std::ifstream test( full_path );
    bool          file_exists = test.good();
    test.close();

    std::ios_base::openmode mode = file_exists ? std::ios::app : std::ios::out;
    std::ofstream           outfile( full_path, mode );

    if ( ! outfile.is_open() )
        throw FileException( FileException::OPEN_FAILED, "Cannot open file: " + full_path );

    // 当写入文件不存在时（即：不需要追加写入），写入列头
    if ( ! file_exists )
    {
        for ( size_t i = 0; i < columns.size(); ++i )
        {
            outfile << columns[ i ].first;
            if ( i < columns.size() - 1 )
                outfile << ",";
        }
        outfile << "\n";
    }

    // 设置固定小数格式并保留8位小数
    outfile << std::fixed << std::setprecision( 8 );

    // 写入数据，空列填充空字符串
    for ( size_t i = 0; i < expected_size; ++i )
    {
        for ( size_t j = 0; j < columns.size(); ++j )
        {
            // 允许空列并自动处理
            if ( ! columns[ j ].second.empty() )
                outfile << columns[ j ].second[ i ];

            // 空列不输出数值
            if ( j < columns.size() - 1 )
                outfile << ",";
        }
        outfile << "\n";
    }

    // 最后恢复默认
    outfile.unsetf( std::ios_base::fixed );
}

static int read_correct_data( FmPDRHandler* hdl, int is_first, SensorData* sensor_data, PDRData* pdr_data )
{
    const int count = hdl->m_config.sample_rate * hdl->m_config.pdr_duration;
    int       ret;

    ret = fm_device_read( hdl->m_device_handle, is_first, count, 1, sensor_data );
    if ( ret != 0 )
    {
        std::cerr << "Sensor data reading failed." << std::endl;
        return -1;
    }

    // 校准磁力计数据
    for ( int i = 0; i < count; ++i )
    {
        const double&    timestamp = sensor_data->sensor_data.acc_time[ i ];
        const double&    mag_x     = sensor_data->sensor_data.mag_x[ i ];
        const double&    mag_y     = sensor_data->sensor_data.mag_y[ i ];
        const double&    mag_z     = sensor_data->sensor_data.mag_z[ i ];
        MagnetometerData raw_data( timestamp, mag_x, mag_y, mag_z );
        Vector3f         raw_vec( raw_data.magneticFieldX, raw_data.magneticFieldY, raw_data.magneticFieldZ );

        // 计算模长
        // double magnitude_before = std::sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
        // std::cout << "校准前数据：(" << mag_x << "," << mag_y << "," << mag_z << "," << magnitude_before << ")" << std::endl;

        // hdl->m_mag_calibration->Calibration( mag_x, mag_y, mag_z );

        // double magnitude_after = std::sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
        // std::cout << "校准后数据：(" << mag_x << "," << mag_y << "," << mag_z << "," << magnitude_after << ")" << std::endl;

        // mag_x *= sensor_data.sensor_data.mag_x[ i ];
        // mag_y *= sensor_data.sensor_data.mag_y[ i ];
        // mag_z *= sensor_data.sensor_data.mag_z[ i ];

        // 调用校正方法（公式：校正后 = (原始数据 - 偏移) × 增益）
        Vector3f corrected_vec = hdl->m_loaded_corrector->correct( raw_vec );

        // 输出校正结果
        // std::cout << "\n=== 数据校正示例 ===" << std::endl;
        // std::cout << "原始数据: " << raw_vec.transpose() << " μT" << std::endl;
        // std::cout << "校正后数据: " << corrected_vec.transpose() << ", " << corrected_vec.norm() << " μT" << std::endl;

        sensor_data->sensor_data.mag_x[ i ] = corrected_vec[ 0 ];
        sensor_data->sensor_data.mag_y[ i ] = corrected_vec[ 1 ];
        sensor_data->sensor_data.mag_z[ i ] = corrected_vec[ 2 ];
    }

    // 转换为PDRData结构
    pdr_data->sensor_data = sensor_data->sensor_data;

    return 0;
}

void free_trajectory_vector( std::vector< PDRTrajectory* >* traj_vec )
{
    if ( traj_vec )
    {
        // 释放vector内的每个PDRTrajectory*
        for ( auto traj : *traj_vec )
            free_trajectory( traj );

        // 释放vector本身
        delete traj_vec;
    }
}

static void do_pdr( FmPDRHandler* hdl )
{
    TrajectoryManager tm( 4 );
    SensorData        sensor_data;
    PDRData           pdr_data;
    bool              is_first_data      = true;
    int               ret;
    int               last_origin_rows   = 0;
    int               last_matched_rows  = 0;

    memset( &sensor_data, 0x00, sizeof( sensor_data ) );
    memset( &pdr_data, 0x00, sizeof( pdr_data ) );

    while ( hdl->m_status == PDR_RUNNING )
    {
        try
        {
            // 使用固定缓存模式读取传感器数据
            ret = read_correct_data( hdl, is_first_data, &sensor_data, &pdr_data );
            if ( ret != 0 )
            {
                std::cerr << "Sensor data reading failed." << std::endl;
                continue;
            }

            // 将sensor_data数据追加的形式保存到csv文件中，方便调试和验证
            if ( hdl->m_sensor_data_path )
            {
                int ret = fm_pdr_save_pdr_data( ( char* )hdl->m_sensor_data_path, &pdr_data );
                if ( ret != 0 )
                {
                    std::cerr << "Failed to save data." << std::endl;
                    continue;
                }
            }

            CFmDataBufferLoader data_loader( hdl->m_config, 0, pdr_data );

            if ( is_first_data )
            {
                hdl->m_si     = hdl->m_pdr.start( hdl->m_si.x0, hdl->m_si.y0, data_loader );
                is_first_data = false;
            }

            // 追加最近测量（配置项pdr_duration）的每步轨迹点到tm，tm记录导航开始到目前为止所有轨迹
            tm.append( hdl->m_pdr.pdr( hdl->m_si, data_loader ) );

            // 按配置项match_duration设置的时间间隔稀释轨迹点（完整轨迹，FMM匹配需要）
            Eigen::MatrixXd full_origin = tm.process_trajectory( hdl->m_config.match_duration );

            // 将完整稀释轨迹输入FMM模型，得到预测轨迹
            Eigen::MatrixXd full_matched = hdl->m_fmm.match( full_origin );

            int cur_origin_rows = full_origin.rows();
            int cur_matched_rows = full_matched.rows();

            // 只推入新增的增量行到队列
            if ( cur_origin_rows > last_origin_rows )
            {
                Eigen::MatrixXd* origin_trajectory = new Eigen::MatrixXd(
                    full_origin.bottomRows( cur_origin_rows - last_origin_rows ) );
                Eigen::MatrixXd* predict_trajectories = new Eigen::MatrixXd(
                    full_matched.bottomRows( cur_matched_rows - last_matched_rows ) );

                std::vector< Eigen::MatrixXd* > trajectory;
                trajectory.push_back( origin_trajectory );
                trajectory.push_back( predict_trajectories );

                if ( origin_trajectory->rows() > 0 && predict_trajectories->rows() > 0 )
                    hdl->queue.enqueue( trajectory );

                last_origin_rows  = cur_origin_rows;
                last_matched_rows = cur_matched_rows;
            }
        }
        catch ( const PDRException& e )
        {
            std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
            continue;
        }
        catch ( const std::exception& e )
        {
            std::cerr << "[StdError] " << e.what() << std::endl;
            continue;
        }
        catch ( ... )
        {
            std::cerr << "[Unknown Error]" << std::endl;
            continue;
        }
    }

    // 释放设备读取缓存
    fm_device_free_sensor_data( sensor_data );
}

int fm_pdr_init_with_file( char* config_dir, char* train_file_path, PDRHandler* handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! config_dir || ! handler )
        return PDRException::PARAMETER_ERROR;
    if ( ! train_file_path && trajectories_array )
        return PDRException::PARAMETER_ERROR;

    PDRTrajectory*                                                             trajs = nullptr;
    int                                                                        ret   = PDRException::SUCCESS;
    std::unique_ptr< PDRTrajectoryArray, decltype( &fm_pdr_free_trajectory ) > trajectories_guard( trajectories_array, fm_pdr_free_trajectory );

    try
    {
        const std::string&              config_path     = std::string( config_dir ) + "//" + "config.json";
        PDRConfig                       config          = CFmJSONOperator::readPDRConfigFromJson( config_path.c_str() );
        const std::string&              fmm_config_path = std::string( config_dir ) + "//" + "fmm_config.xml";
        FMMConfig                       fmm_config( fmm_config_path );
        std::unique_ptr< FmPDRHandler > h;

        if ( ! fmm_config.validate() )
        {
            throw FileException( FileException::OPEN_FAILED, fmm_config_path.c_str() );
        }

        if ( train_file_path )
        {
            CFmDataFileLoader                                data_loader( config, ( size_t )-1, train_file_path );
            std::unique_ptr< std::vector< PDRTrajectory* > > trajectories_vector = std::make_unique< std::vector< PDRTrajectory* > >();
            Eigen::MatrixXd*                                 train_trajectories  = new Eigen::MatrixXd();
            h                                                                    = std::make_unique< FmPDRHandler >( config, fmm_config, data_loader, *train_trajectories );

            if ( trajectories_array )
            {
                ret = eigenToPDRTrajectory( *train_trajectories, &trajs );
                trajectories_vector->push_back( trajs );

                trajectories_array->array = trajectories_vector->data();
                trajectories_array->count = 1;
                trajectories_array->ptr   = trajectories_vector.release();
            }
        }
        else
        {
            h = std::make_unique< FmPDRHandler >( config, fmm_config );
        }

        h->m_config_dir = config_dir;
        *handler        = static_cast< PDRHandler >( h.release() );
        trajectories_guard.release();
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }

    return ret;
}

int fm_pdr_get_config( PDRHandler handler, PDRConfig* config )
{
    if ( ! handler )
        return PDRException::PARAMETER_ERROR;

    FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
    config            = &hdl->m_config;

    return PDRException::SUCCESS;
}

int fm_pdr_start( PDRHandler handler, PDRPoint* start_point, char* raw_data_path )
{
    if ( ! handler || ! start_point )
        return PDRException::PARAMETER_ERROR;

    int           ret = PDRException::SUCCESS;
    FmPDRHandler* hdl = nullptr;

    try
    {
        hdl = reinterpret_cast< FmPDRHandler* >( handler );
        if ( hdl->m_status )
            return PDRException::ALREADY_RUNNING;

        std::unique_ptr< char[], decltype( &free ) > sensor_data_path_guard( strdup( raw_data_path ), free );

        hdl->m_si.x0            = start_point->x;
        hdl->m_si.y0            = start_point->y;
        hdl->m_sensor_data_path = sensor_data_path_guard.get();
        hdl->m_status           = PDR_RUNNING;

        // 集成驱动
        std::unique_ptr< FmDeviceHandle, decltype( &fm_device_uninit ) > device_guard( nullptr, fm_device_uninit );

        ret = fm_device_init( hdl->m_config.sample_rate, &hdl->m_device_handle );
        if ( ret != 0 )
            return PDRException::DEVICE_INIT_ERROR;
        device_guard.reset( hdl->m_device_handle );

        std::unique_ptr< SixParametersCorrector > corrector_guard( new SixParametersCorrector() );
        const std::string                         mag_calib_path = hdl->m_config_dir + "//mag_calibration.csv";

        if ( ! corrector_guard->fromFile( mag_calib_path ) )
            throw FileException( FileException::DIR_NOT_EXIST, mag_calib_path.c_str() );

        hdl->m_loaded_corrector = corrector_guard.release();
        hdl->m_worker           = std::thread( do_pdr, hdl );

        sensor_data_path_guard.release();
        device_guard.release();
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }
    return ret;
}

int fm_pdr_start_with_file( PDRHandler handler, char* sensor_file_path )
{
    if ( ! handler || ! sensor_file_path )
        return PDRException::PARAMETER_ERROR;

    int           ret = PDRException::SUCCESS;
    FmPDRHandler* hdl = nullptr;

    try
    {
        hdl = reinterpret_cast< FmPDRHandler* >( handler );
        std::unique_ptr< CFmDataFileLoader >         data_loader_guard( new CFmDataFileLoader( hdl->m_config, 0, sensor_file_path ) );
        std::unique_ptr< char[], decltype( &free ) > sensor_path_guard( strdup( sensor_file_path ), free );

        // VectorXd pos_x          = hdl->m_data_loader->get_true_data( TRUE_DATA_FIELD_LATITUDE );
        // VectorXd pos_y          = hdl->m_data_loader->get_true_data( TRUE_DATA_FIELD_LONGITUDE );
        // double   x0             = pos_x[ 0 ];
        // double   y0             = pos_y[ 0 ];

        double x0 = 123.45001915081717;
        double y0 = 41.71512697704802;
        // double x0 = 123.456094;
        // double y0 = 41.71748;

        hdl->m_si               = hdl->m_pdr.start( x0, y0, *data_loader_guard );
        hdl->m_data_loader      = data_loader_guard.release();
        hdl->m_sensor_data_path = sensor_path_guard.release();
        hdl->m_status           = PDR_RUNNING;
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }
    return ret;
}

int fm_pdr_predict( PDRHandler handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! handler || ! trajectories_array )
        return PDRException::PARAMETER_ERROR;

    int ret                  = PDRException::SUCCESS;
    using TrajectoryVecGuard = std::unique_ptr< std::vector< PDRTrajectory* >, decltype( &free_trajectory_vector ) >;
    TrajectoryVecGuard trajectories_vector_guard( new std::vector< PDRTrajectory* >(),  // 初始化时分配vector
                                                  free_trajectory_vector                // 自定义删除器
    );
    Eigen::MatrixXd*   origin_trajectory    = nullptr;
    Eigen::MatrixXd*   predict_trajectories = nullptr;
    PDRTrajectory*     origin_trajs         = nullptr;
    PDRTrajectory*     matched_trajs        = nullptr;

    try
    {
        FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );

        // 根据是否创建设备句柄判断PDR模式
        if ( ! hdl->m_device_handle )
        {
            TrajectoryManager tm( 4 );
            tm.append( hdl->m_pdr.pdr( hdl->m_si, *hdl->m_data_loader ) );

            origin_trajectory    = new Eigen::MatrixXd( tm.process_trajectory( hdl->m_config.match_duration ) );
            predict_trajectories = new Eigen::MatrixXd( hdl->m_fmm.match( *origin_trajectory, true ) );

            ret = eigenToPDRTrajectory( *origin_trajectory, &origin_trajs );
            trajectories_vector_guard->push_back( origin_trajs );
            ret = eigenToPDRTrajectory( *predict_trajectories, &matched_trajs );
            trajectories_vector_guard->push_back( matched_trajs );

            trajectories_array->array = trajectories_vector_guard->data();
            trajectories_array->count = trajectories_vector_guard->size();
            trajectories_array->ptr   = trajectories_vector_guard.release();
        }
        else
        {
            // 取出所有增量
            std::vector< Eigen::MatrixXd* > origin_deltas;
            std::vector< Eigen::MatrixXd* > matched_deltas;

            while ( true )
            {
                std::vector< Eigen::MatrixXd* > trajectory;
                if ( ! hdl->queue.try_dequeue( trajectory ) )
                    break;
                origin_deltas.push_back( trajectory[ 0 ] );
                matched_deltas.push_back( trajectory[ 1 ] );
            }

            // 合并所有增量为单个矩阵
            if ( ! origin_deltas.empty() )
            {
                int total_origin = 0, total_matched = 0;
                for ( auto* m : origin_deltas )  total_origin += m->rows();
                for ( auto* m : matched_deltas ) total_matched += m->rows();

                Eigen::MatrixXd* merged_origin  = new Eigen::MatrixXd( total_origin, 4 );
                Eigen::MatrixXd* merged_matched = new Eigen::MatrixXd( total_matched, 4 );

                int off_o = 0, off_m = 0;
                for ( size_t i = 0; i < origin_deltas.size(); ++i )
                {
                    int rows_o = origin_deltas[ i ]->rows();
                    int rows_m = matched_deltas[ i ]->rows();
                    merged_origin->block( off_o, 0, rows_o, 4 )  = *origin_deltas[ i ];
                    merged_matched->block( off_m, 0, rows_m, 4 ) = *matched_deltas[ i ];
                    off_o += rows_o;
                    off_m += rows_m;
                    delete origin_deltas[ i ];
                    delete matched_deltas[ i ];
                }

                ret = eigenToPDRTrajectory( *merged_origin, &origin_trajs );
                trajectories_vector_guard->push_back( origin_trajs );
                ret = eigenToPDRTrajectory( *merged_matched, &matched_trajs );
                trajectories_vector_guard->push_back( matched_trajs );
            }

            // 转换为C结构体传出
            trajectories_array->array = trajectories_vector_guard->data();
            trajectories_array->count = trajectories_vector_guard->size();
            trajectories_array->ptr   = trajectories_vector_guard.release();
        }
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }
    return ret;
}

int fm_pdr_save_trajectory_data( char* file_path, PDRTrajectoryArray* trajectories_array )
{
    // 参数有效性校验
    if ( ! file_path || ! trajectories_array )
        return PDRException::PARAMETER_ERROR;

    int ret = PDRException::SUCCESS;

    try
    {
        for ( unsigned int i = 0; i < trajectories_array->count; ++i )
        {
            PDRTrajectory* trajectories = trajectories_array->array[ i ];
            if ( ! trajectories )
                return PDRException::NONE;

            // 数据指针完整性校验
            if ( ! trajectories->length || ! trajectories->time || ! trajectories->x || ! trajectories->y || ! trajectories->direction )
                return DataException::EMPTY_ERROR;

            // 构建符合append_to_csv要求的列数据结构
            std::vector< std::pair< std::string, std::vector< double > > > columns = { { quote_header( "Time (s)" ), { trajectories->time, trajectories->time + trajectories->length } },
                                                                                       { quote_header( "Longitude (°)" ), { trajectories->x, trajectories->x + trajectories->length } },
                                                                                       { quote_header( "Latitude (°)" ), { trajectories->y, trajectories->y + trajectories->length } },
                                                                                       { quote_header( "Height (m)" ), {} },
                                                                                       { quote_header( "Velocity (m/s)" ), {} },
                                                                                       { quote_header( "Direction (°)" ), { trajectories->direction, trajectories->direction + trajectories->length } },
                                                                                       { quote_header( "Horizontal Accuracy (m)" ), {} },
                                                                                       { quote_header( "Vertical Accuracy (°)" ), {} } };

            // 调用核心写入逻辑
            append_to_csv( file_path, columns );
        }
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }
    return ret;
}

void fm_pdr_free_trajectory( PDRTrajectoryArray* trajectories_array )
{
    if ( ! trajectories_array )
        return;

    free_trajectory_vector( static_cast< std::vector< PDRTrajectory* >* >( trajectories_array->ptr ) );
}

int fm_pdr_stop( PDRHandler handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! handler || ! trajectories_array )
        return PDRException::PARAMETER_ERROR;

    try
    {
        FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
        if ( hdl->m_status == PDR_STOPPED )
            return PDRException::CALL_ERROR;

        hdl->m_status = PDR_STOPPED;
        if ( hdl->m_worker.joinable() )
            hdl->m_worker.join();
        delete hdl->m_loaded_corrector;
        hdl->m_loaded_corrector = nullptr;
        fm_device_uninit( hdl->m_device_handle );

        // 返回无锁队列中剩余的行人航迹
        return fm_pdr_predict( handler, trajectories_array );
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
    }
    return PDRException::SUCCESS;
}

void fm_pdr_uninit( PDRHandler* handler )
{
    if ( ! handler || ! ( *handler ) )
        return;

    FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( *handler );

    // 以hdl->m_loaded_corrector判断是否为实时PDR模式
    if ( hdl->m_status != PDR_STOPPED && hdl->m_loaded_corrector )
    {
        PDRTrajectoryArray ta;
        fm_pdr_stop( handler, &ta );
        fm_pdr_free_trajectory( &ta );
    }
    free( hdl->m_config.model_name );
    free( hdl->m_config.model_file_name );
    delete hdl->m_data_loader;
    free( hdl->m_sensor_data_path );
    delete hdl;
    hdl = nullptr;
}

int fm_pdr_save_pdr_data( char* dir_path, PDRData* pdr_data )
{
    if ( ! dir_path || ! pdr_data )
        return PDRException::PARAMETER_ERROR;

    int ret = PDRException::SUCCESS;

    try
    {
        std::string dir_path_name( dir_path );

        // 创建目录（如果不存在）
        int ret = mkdir( dir_path_name.c_str(), 0755 );
        if ( ret != 0 && errno != EEXIST )
            throw FileException( FileException::CREATE_FAILED, dir_path_name.c_str() );

        PDRSensorData* sensor_data = &pdr_data->sensor_data;

        // 处理加速度计数据
        if ( validate_sensor_data( sensor_data->acc_time, sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > acc_columns = { { quote_header( "Time (s)" ), ptr_to_vector( sensor_data->acc_time, sensor_data->length ) },
                                                                                           { quote_header( "X (m/s^2)" ), ptr_to_vector( sensor_data->acc_x, sensor_data->length ) },
                                                                                           { quote_header( "Y (m/s^2)" ), ptr_to_vector( sensor_data->acc_y, sensor_data->length ) },
                                                                                           { quote_header( "Z (m/s^2)" ), ptr_to_vector( sensor_data->acc_z, sensor_data->length ) } };

            std::string acc_path = dir_path_name + "/Accelerometer.csv";
            append_to_csv( acc_path, acc_columns );
        }

        // // 处理线性加速度计数据
        // if ( validate_sensor_data( sensor_data->lacc_time, sensor_data->lacc_x, sensor_data->lacc_y, sensor_data->lacc_z, sensor_data->length ) )
        // {
        //     std::vector< std::pair< std::string, std::vector< double > > > lacc_columns = { { quote_header("Time (s)"), ptr_to_vector( sensor_data->lacc_time, sensor_data->length ) },
        //                                                                                     { quote_header("X (m/s^2)"), ptr_to_vector( sensor_data->lacc_x, sensor_data->length ) },
        //                                                                                     { quote_header("Y (m/s^2)"), ptr_to_vector( sensor_data->lacc_y, sensor_data->length ) },
        //                                                                                     { quote_header("Z (m/s^2)"), ptr_to_vector( sensor_data->lacc_z, sensor_data->length ) } };

        //     std::string lacc_path = dir_path_name + "/LinearAccelerometer.csv";
        //     append_to_csv( lacc_path, lacc_columns );
        // }

        // 处理陀螺仪数据
        if ( validate_sensor_data( sensor_data->gyr_time, sensor_data->gyr_x, sensor_data->gyr_y, sensor_data->gyr_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > gyr_columns = { { quote_header( "Time (s)" ), ptr_to_vector( sensor_data->gyr_time, sensor_data->length ) },
                                                                                           { quote_header( "X (rad/s)" ), ptr_to_vector( sensor_data->gyr_x, sensor_data->length ) },
                                                                                           { quote_header( "Y (rad/s)" ), ptr_to_vector( sensor_data->gyr_y, sensor_data->length ) },
                                                                                           { quote_header( "Z (rad/s)" ), ptr_to_vector( sensor_data->gyr_z, sensor_data->length ) } };

            std::string gyr_path = dir_path_name + "/Gyroscope.csv";
            append_to_csv( gyr_path, gyr_columns );
        }

        // 处理磁力计数据
        if ( validate_sensor_data( sensor_data->mag_time, sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > mag_columns = { { quote_header( "Time (s)" ), ptr_to_vector( sensor_data->mag_time, sensor_data->length ) },
                                                                                           { quote_header( "X (µT)" ), ptr_to_vector( sensor_data->mag_x, sensor_data->length ) },
                                                                                           { quote_header( "Y (µT)" ), ptr_to_vector( sensor_data->mag_y, sensor_data->length ) },
                                                                                           { quote_header( "Z (µT)" ), ptr_to_vector( sensor_data->mag_z, sensor_data->length ) } };

            std::string mag_path = dir_path_name + "/Magnetometer.csv";
            append_to_csv( mag_path, mag_columns );
        }

        // 保存GPS数据
        PDRTrueData* true_data = &pdr_data->true_data;
        memset( true_data, 0x00, sizeof( PDRTrueData ) );

        // 处理GPS数据
        if ( validate_true_data( true_data->time_location, true_data->latitude, true_data->longitude, true_data->height, true_data->velocity, true_data->direction, true_data->horizontal_accuracy, true_data->vertical_accuracy, true_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > gps_columns = {
                { quote_header( "Time (s)" ), ptr_to_vector( true_data->time_location, true_data->length ) },
                { quote_header( "Latitude (°)" ), ptr_to_vector( true_data->latitude, true_data->length ) },
                { quote_header( "Longitude (°)" ), ptr_to_vector( true_data->longitude, true_data->length ) },
                { quote_header( "Height (m)" ), ptr_to_vector( true_data->height, true_data->length ) },
                { quote_header( "Velocity (m/s)" ), ptr_to_vector( true_data->velocity, true_data->length ) },
                { quote_header( "Direction (°)" ), ptr_to_vector( true_data->direction, true_data->length ) },
                { quote_header( "Horizontal Accuracy (m)" ), ptr_to_vector( true_data->horizontal_accuracy, true_data->length ) },
                { quote_header( "Vertical Accuracy (°)" ), ptr_to_vector( true_data->vertical_accuracy, true_data->length ) },
            };

            std::string gps_path = dir_path_name + "/Location.csv";
            append_to_csv( gps_path, gps_columns );
        }
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;
    }

    return ret;
}

// NOTE:以下函数在项目中并没有使用，也不对外提供接口，处于预留考虑暂时不删除
int fm_pdr_read_pdr_data( char* dir_path, PDRData* pdr_data )
{
    if ( ! dir_path || ! pdr_data )
        return PDRException::PARAMETER_ERROR;

    int ret = PDRException::SUCCESS;

    try
    {
        std::string dir_path_name( dir_path );

        // 检查目录是否存在
        struct stat info;
        if ( stat( dir_path_name.c_str(), &info ) != 0 )
            throw FileException( FileException::DIR_NOT_EXIST, dir_path_name.c_str() );

        if ( ! ( info.st_mode & S_IFDIR ) )
            throw FileException( FileException::NOT_DIRECTORY, dir_path_name.c_str() );

        PDRSensorData* sensor_data = &pdr_data->sensor_data;
        memset( sensor_data, 0x00, sizeof( PDRSensorData ) );

        // 读取加速度计数据
        std::string acc_path = dir_path_name + "/Accelerometer.csv";
        if ( file_exists( acc_path ) )
        {
            auto acc_data = read_csv_file( acc_path );
            if ( ! acc_data.empty() && acc_data.size() >= 4 )
            {
                sensor_data->length = acc_data[ 0 ].size();
                allocate_sensor_arrays( sensor_data, sensor_data->length );

                for ( size_t i = 0; i < sensor_data->length; ++i )
                {
                    sensor_data->acc_time[ i ] = acc_data[ 0 ][ i ];
                    sensor_data->acc_x[ i ]    = acc_data[ 1 ][ i ];
                    sensor_data->acc_y[ i ]    = acc_data[ 2 ][ i ];
                    sensor_data->acc_z[ i ]    = acc_data[ 3 ][ i ];
                }
            }
        }

        // 读取陀螺仪数据
        std::string gyr_path = dir_path_name + "/Gyroscope.csv";
        if ( file_exists( gyr_path ) )
        {
            auto gyr_data = read_csv_file( gyr_path );
            if ( ! gyr_data.empty() && gyr_data.size() >= 4 )
            {
                // 如果还没有分配数组，根据陀螺仪数据长度分配
                if ( sensor_data->length == 0 )
                {
                    sensor_data->length = gyr_data[ 0 ].size();
                    allocate_sensor_arrays( sensor_data, sensor_data->length );
                }

                for ( size_t i = 0; i < sensor_data->length && i < gyr_data[ 0 ].size(); ++i )
                {
                    sensor_data->gyr_time[ i ] = gyr_data[ 0 ][ i ];
                    sensor_data->gyr_x[ i ]    = gyr_data[ 1 ][ i ];
                    sensor_data->gyr_y[ i ]    = gyr_data[ 2 ][ i ];
                    sensor_data->gyr_z[ i ]    = gyr_data[ 3 ][ i ];
                }
            }
        }

        // 读取磁力计数据
        std::string mag_path = dir_path_name + "/Magnetometer.csv";
        if ( file_exists( mag_path ) )
        {
            auto mag_data = read_csv_file( mag_path );
            if ( ! mag_data.empty() && mag_data.size() >= 4 )
            {
                // 如果还没有分配数组，根据磁力计数据长度分配
                if ( sensor_data->length == 0 )
                {
                    sensor_data->length = mag_data[ 0 ].size();
                    allocate_sensor_arrays( sensor_data, sensor_data->length );
                }

                for ( size_t i = 0; i < sensor_data->length && i < mag_data[ 0 ].size(); ++i )
                {
                    sensor_data->mag_time[ i ] = mag_data[ 0 ][ i ];
                    sensor_data->mag_x[ i ]    = mag_data[ 1 ][ i ];
                    sensor_data->mag_y[ i ]    = mag_data[ 2 ][ i ];
                    sensor_data->mag_z[ i ]    = mag_data[ 3 ][ i ];
                }
            }
        }

        // 读取GPS数据
        PDRTrueData* true_data = &pdr_data->true_data;
        memset( true_data, 0x00, sizeof( PDRTrueData ) );

        std::string gps_path = dir_path_name + "/Location.csv";
        if ( file_exists( gps_path ) )
        {
            auto gps_data = read_csv_file( gps_path );
            if ( ! gps_data.empty() && gps_data.size() >= 8 )
            {
                true_data->length = gps_data[ 0 ].size();
                allocate_true_arrays( true_data, true_data->length );

                for ( size_t i = 0; i < true_data->length; ++i )
                {
                    true_data->time_location[ i ]       = gps_data[ 0 ][ i ];
                    true_data->latitude[ i ]            = gps_data[ 1 ][ i ];
                    true_data->longitude[ i ]           = gps_data[ 2 ][ i ];
                    true_data->height[ i ]              = gps_data[ 3 ][ i ];
                    true_data->velocity[ i ]            = gps_data[ 4 ][ i ];
                    true_data->direction[ i ]           = gps_data[ 5 ][ i ];
                    true_data->horizontal_accuracy[ i ] = gps_data[ 6 ][ i ];
                    true_data->vertical_accuracy[ i ]   = gps_data[ 7 ][ i ];
                }
            }
        }

        // 如果没有读取到任何数据，返回错误
        if ( sensor_data->length == 0 && true_data->length == 0 )
            ret = DataException::EMPTY_ERROR;
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();

        // 清理已分配的内存
        cleanup_pdr_data( pdr_data );
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDRException::GENERAL_ERROR;

        // 清理已分配的内存
        cleanup_pdr_data( pdr_data );
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDRException::UNKNOWN;

        // 清理已分配的内存
        cleanup_pdr_data( pdr_data );
    }

    return ret;
}

// NOTE:以下函数在项目中并没有使用，也不对外提供接口，处于预留考虑暂时不删除
void fm_pdr_free_pdr_data( PDRData* pdr_data )
{
    if ( ! pdr_data )
        return;
    cleanup_pdr_data( pdr_data );
}

int fm_pdr_generate_ubodt( const char* network_file, const char* output_file, double delta )
{
    if ( ! network_file || ! output_file )
        return PDRException::PARAMETER_ERROR;
    if ( delta <= 0 )
        return PDRException::PARAMETER_ERROR;

    try
    {
        SPDLOG_INFO( "Loading network from {}", network_file );
        FMM::NETWORK::Network network( network_file, "fid", "u", "v" );
        SPDLOG_INFO( "Network loaded: {} nodes, {} edges", network.get_node_count(), network.get_edge_count() );

        FMM::NETWORK::NetworkGraph graph( network );
        FMM::MM::UBODTGenAlgorithm ubodt_gen( network, graph );

        SPDLOG_INFO( "Generating UBODT to {} with delta={}", output_file, delta );
        std::string result = ubodt_gen.generate_ubodt( output_file, delta, false, true );
        SPDLOG_INFO( "{}", result );

        return PDRException::SUCCESS;
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        return PDRException::GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        return PDRException::UNKNOWN;
    }
}