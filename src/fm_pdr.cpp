#include "fm_pdr.h"
#include "data_buffer_loader.h"
#include "data_file_loader.h"
#include "data_manager.h"
#include "exception.h"
#include "fm_device_wrapper.h"
#include "json_operator.h"
#include "pdr.h"
#include <Eigen/src/Core/Matrix.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <moodycamel/concurrentqueue.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

typedef enum _FmPDRStatus
{
    PDR_STOPPED,
    PDR_RUNNING
} FmPDRStatus;

typedef struct _FmPDRHandler
{
    PDRConfig                                       m_config;            // 配置
    CFmPDR                                          m_pdr;               // PDR句柄
    StartInfo                                       m_si;                // 起点信息
    CFmDataManager*                                 m_data_loader;       // 数据加载器
    char*                                           m_sensor_data_path;  // PDR数据文件路径
    fm_device_handle_t                              m_device_handle;     // 设备操作句柄
    int                                             m_status;            // 0:停止,1:启动
    std::thread                                     m_worker;            // 子线程句柄
    moodycamel::ConcurrentQueue< Eigen::MatrixXd* > queue;               // 轨迹队列

    // 注意：创建PDR对象时，不能使用传入参数config，需要全局生命周期的m_config
    _FmPDRHandler( const PDRConfig& config, const CFmDataManager& train_data, Eigen::MatrixXd& train_position ) : m_config( config ), m_pdr( m_config, train_data, train_position ), m_data_loader( nullptr ), m_sensor_data_path( nullptr ), m_status( PDR_STOPPED )
    {
        memset( &m_device_handle, 0x00, sizeof( m_device_handle ) );
    }
    _FmPDRHandler( const PDRConfig& config ) : m_config( config ), m_pdr( m_config ), m_data_loader( nullptr ), m_sensor_data_path( nullptr ), m_status( PDR_STOPPED )
    {
        memset( &m_device_handle, 0x00, sizeof( m_device_handle ) );
    }
} FmPDRHandler;

static int eigenToPDRTrajectory( const Eigen::MatrixXd& predict_trajectories, PDRTrajectory** trajectories )
{
    const unsigned long n = predict_trajectories.rows();
    if ( n == 0 )
    {
        trajectories = NULL;
        return 0;
    }

    PDRTrajectory* new_traj = nullptr;
    try
    {
        // 预分配内存
        new_traj = new PDRTrajectory();

        // 直接将指针指向 Eigen 矩阵的列数据
        new_traj->time      = const_cast< double* >( predict_trajectories.col( 0 ).data() );
        new_traj->x         = const_cast< double* >( predict_trajectories.col( 1 ).data() );
        new_traj->y         = const_cast< double* >( predict_trajectories.col( 2 ).data() );
        new_traj->direction = const_cast< double* >( predict_trajectories.col( 3 ).data() );
        new_traj->length    = n;
        new_traj->ptr       = ( void* )&predict_trajectories;

        *trajectories = new_traj;
    }
    catch ( const std::bad_alloc& e )
    {
        // 内存分配失败
        delete new_traj;
        throw MemoryException( MemoryException::ALLOC_FAILED, "Convert PDRTrajectory error" + std::string( e.what() ) );
    }
    catch ( ... )
    {
        // 处理其他异常
        delete new_traj;
        throw;
    }
    return n;
}

// 配套的析构函数
static void free_trajectory( PDRTrajectory* trajectory ) noexcept
{
    if ( ! trajectory )
        return;
    trajectory->time      = NULL;
    trajectory->x         = NULL;
    trajectory->y         = NULL;
    trajectory->direction = NULL;
    trajectory->length    = 0;
    delete static_cast< Eigen::MatrixXd* >( trajectory->ptr );
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

    sensor_data->acc_time = new double[ length ];
    sensor_data->acc_x    = new double[ length ];
    sensor_data->acc_y    = new double[ length ];
    sensor_data->acc_z    = new double[ length ];

    sensor_data->gyr_time = new double[ length ];
    sensor_data->gyr_x    = new double[ length ];
    sensor_data->gyr_y    = new double[ length ];
    sensor_data->gyr_z    = new double[ length ];

    sensor_data->mag_time = new double[ length ];
    sensor_data->mag_x    = new double[ length ];
    sensor_data->mag_y    = new double[ length ];
    sensor_data->mag_z    = new double[ length ];

    sensor_data->length = length;
}

void allocate_true_arrays( PDRTrueData* true_data, int length )
{
    if ( length <= 0 )
        return;

    true_data->time_location       = new double[ length ];
    true_data->latitude            = new double[ length ];
    true_data->longitude           = new double[ length ];
    true_data->height              = new double[ length ];
    true_data->velocity            = new double[ length ];
    true_data->direction           = new double[ length ];
    true_data->horizontal_accuracy = new double[ length ];
    true_data->vertical_accuracy   = new double[ length ];

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

static void do_pdr( FmPDRHandler* hdl )
{
    PDRData    pdr_data;
    SensorData sensor_data;
    bool       is_first = true;
    int        ret;

    memset( &pdr_data, 0x00, sizeof( pdr_data ) );
    memset( &sensor_data, 0x00, sizeof( sensor_data ) );

    while ( hdl->m_status == PDR_RUNNING )
    {
        // 使用固定缓存模式读取传感器数据
        ret = fm_device_read( hdl->m_device_handle, is_first, hdl->m_config.sample_rate * hdl->m_config.pdr_duration, 1, &sensor_data );
        if ( ret != 0 )
        {
            std::cerr << "Sensor data reading failed." << std::endl;
            continue;
        }

        // 标记不是第一次读取数据，即不需要再次创建缓存
        is_first = false;

        // 转换为PDRData结构
        pdr_data.sensor_data = sensor_data.sensor_data;

        // 将sensor_data数据追加的形式保存到csv文件中，方便调试和验证
        if ( hdl->m_sensor_data_path )
        {
            int result = fm_pdr_save_pdr_data( ( char* )hdl->m_sensor_data_path, &pdr_data );
            if ( result != 0 )
            {
                std::cerr << "Failed to save data." << std::endl;
                continue;
            }
        }

        try
        {
            // 启动导航
            CFmDataBufferLoader data_loader( hdl->m_config, 0, pdr_data );
            hdl->m_si          = hdl->m_pdr.start( hdl->m_si.x0, hdl->m_si.y0, data_loader );
            Eigen::MatrixXd* t = new Eigen::MatrixXd( hdl->m_pdr.pdr( hdl->m_si, data_loader ) );

            // 导航结果写入无锁队列
            if ( t->rows() > 0 )
                hdl->queue.enqueue( t );
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

int fm_pdr_init_with_file( char* config_path, char* train_file_path, PDRHandler* handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! config_path || ! handler )
        return PDR_RESULT_PARAMETER_ERROR;
    if ( ! train_file_path && trajectories_array )
        return PDR_RESULT_PARAMETER_ERROR;

    int                       ret                 = PDR_RESULT_SUCCESS;
    FmPDRHandler*             h                   = nullptr;
    Eigen::MatrixXd*          train_trajectories  = nullptr;
    PDRTrajectory*            trajs               = nullptr;
    vector< PDRTrajectory* >* trajectories_vector = new std::vector< PDRTrajectory* >();

    try
    {
        PDRConfig config = CFmJSONOperator::readPDRConfigFromJson( config_path );

        if ( train_file_path )
        {
            CFmDataFileLoader data_loader( config, ( size_t )-1, train_file_path );
            train_trajectories = new Eigen::MatrixXd();
            h                  = new FmPDRHandler( config, data_loader, *train_trajectories );

            if ( trajectories_array )
            {
                ret = eigenToPDRTrajectory( *train_trajectories, &trajs );
                trajectories_vector->push_back( trajs );

                trajectories_array->array = trajectories_vector->data();
                trajectories_array->count = 1;
                trajectories_array->ptr   = trajectories_array->array;
            }
        }
        else
        {
            h = new FmPDRHandler( config );
        }

        *handler = static_cast< PDRHandler >( h );
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
        // 清理已分配资源
        if ( h )
            delete h;
        if ( train_trajectories )
            delete train_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDR_RESULT_GENERAL_ERROR;
        if ( h )
            delete h;
        if ( train_trajectories )
            delete train_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
        if ( h )
            delete h;
        if ( train_trajectories )
            delete train_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }

    return ret;
}

int fm_pdr_get_config( PDRHandler handler, PDRConfig* config )
{
    if ( ! handler )
        return PDR_RESULT_PARAMETER_ERROR;

    FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
    config            = &hdl->m_config;

    return PDR_RESULT_SUCCESS;
}

int fm_pdr_start( PDRHandler handler, PDRPoint *start_point, char* raw_data_path )
{
    if ( ! handler || ! start_point )
        return PDR_RESULT_PARAMETER_ERROR;

    int ret = PDR_RESULT_SUCCESS;

    try
    {
        FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
        if ( hdl->m_status )
            return PDR_RESULT_ALREADY_RUNNING;

        hdl->m_si.x0            = start_point->x;
        hdl->m_si.y0            = start_point->y;
        hdl->m_sensor_data_path = strdup( raw_data_path );
        hdl->m_status           = PDR_RUNNING;

        // 集成驱动
        ret = fm_device_init( hdl->m_config.sample_rate, &hdl->m_device_handle );
        if ( ret != 0 )
            return PDR_RESULT_DEVICE_INIT_ERROR;

        hdl->m_worker = std::thread( do_pdr, hdl );
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDR_RESULT_GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
    }
    return ret;
}

int fm_pdr_start_with_file( PDRHandler handler, char* sensor_file_path )
{
    if ( ! handler || ! sensor_file_path )
        return PDR_RESULT_PARAMETER_ERROR;

    int ret = PDR_RESULT_SUCCESS;
    try
    {
        FmPDRHandler* hdl       = reinterpret_cast< FmPDRHandler* >( handler );
        hdl->m_data_loader      = new CFmDataFileLoader( hdl->m_config, 0, sensor_file_path );
        VectorXd pos_x          = hdl->m_data_loader->get_true_data( TRUE_DATA_FIELD_LATITUDE );
        VectorXd pos_y          = hdl->m_data_loader->get_true_data( TRUE_DATA_FIELD_LONGITUDE );
        double   x0             = pos_x[ 0 ];
        double   y0             = pos_y[ 0 ];
        hdl->m_si               = hdl->m_pdr.start( x0, y0, *hdl->m_data_loader );
        hdl->m_sensor_data_path = strdup( sensor_file_path );
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
        ret = PDR_RESULT_GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
    }
    return ret;
}

int fm_pdr_predict( PDRHandler handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! handler || ! trajectories_array )
        return PDR_RESULT_PARAMETER_ERROR;

    int                       ret                  = PDR_RESULT_SUCCESS;
    Eigen::MatrixXd*          predict_trajectories = nullptr;
    PDRTrajectory*            trajs                = nullptr;
    vector< PDRTrajectory* >* trajectories_vector  = new std::vector< PDRTrajectory* >();

    try
    {
        FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
        // 停止时需要取残留数据
        // if ( hdl->m_status != PDR_RUNNING )
        //     return PDR_RESULT_CALL_ERROR;

        // 根据是否创建设备句柄判断PDR模式
        if ( ! hdl->m_device_handle.handler )
        {
            predict_trajectories = new Eigen::MatrixXd( hdl->m_pdr.pdr( hdl->m_si, *hdl->m_data_loader ) );
            ret                  = eigenToPDRTrajectory( *predict_trajectories, &trajs );
            trajectories_vector->push_back( trajs );

            trajectories_array->array = trajectories_vector->data();
            trajectories_array->count = 1;
            trajectories_array->ptr   = trajectories_array->array;
        }
        else
        {
            while ( true )
            {
                bool is_ok;

                // 从无锁队列中取得行人航迹数据
                is_ok = hdl->queue.try_dequeue( predict_trajectories );
                if ( ! is_ok )
                    break;

                ret += eigenToPDRTrajectory( *predict_trajectories, &trajs );
                trajectories_vector->push_back( trajs );
            }

            // 转换为C结构体传出
            trajectories_array->array = trajectories_vector->data();
            trajectories_array->count = trajectories_vector->size();
            trajectories_array->ptr   = trajectories_vector;
        }
    }
    catch ( const PDRException& e )
    {
        std::cerr << "[PDRError:" << e.code() << "] " << e.what() << std::endl;
        ret = e.code();
        if ( predict_trajectories )
            delete predict_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }
    catch ( const std::exception& e )
    {
        std::cerr << "[StdError] " << e.what() << std::endl;
        ret = PDR_RESULT_GENERAL_ERROR;
        if ( predict_trajectories )
            delete predict_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
        if ( predict_trajectories )
            delete predict_trajectories;
        if ( trajectories_array )
            fm_pdr_free_trajectory( trajectories_array );
    }
    return ret;
}

int fm_pdr_save_trajectory_data( char* file_path, PDRTrajectoryArray* trajectories_array )
{
    // 参数有效性校验
    if ( ! file_path || ! trajectories_array )
        return PDR_RESULT_PARAMETER_ERROR;

    int ret = PDR_RESULT_SUCCESS;

    try
    {
        for ( unsigned int i = 0; i < trajectories_array->count; ++i )
        {
            PDRTrajectory* trajectories = trajectories_array->array[ i ];
            if ( ! trajectories )
                return PDR_RESULT_NONE;
            
            // 数据指针完整性校验
            if ( ! trajectories->length || ! trajectories->time || ! trajectories->x || ! trajectories->y || ! trajectories->direction )
                return PDR_RESULT_EMPTY_ERROR;

            // 构建符合append_to_csv要求的列数据结构
            std::vector< std::pair< std::string, std::vector< double > > > columns = { { "Time (s)", { trajectories->time, trajectories->time + trajectories->length } },
                                                                                       { "Latitude (°)", { trajectories->x, trajectories->x + trajectories->length } },
                                                                                       { "Longitude (°)", { trajectories->y, trajectories->y + trajectories->length } },
                                                                                       { "Height (m)", {} },
                                                                                       { "Velocity (m/s)", {} },
                                                                                       { "Direction (°)", { trajectories->direction, trajectories->direction + trajectories->length } },
                                                                                       { "Horizontal Accuracy (m)", {} },
                                                                                       { "Vertical Accuracy (°)", {} } };

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
        ret = PDR_RESULT_GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
    }
    return ret;
}

void fm_pdr_free_trajectory( PDRTrajectoryArray* trajectories_array )
{
    if ( ! trajectories_array )
        return;

    for ( unsigned int i = 0; i < trajectories_array->count; ++i )
        free_trajectory( trajectories_array->array[ i ] );

    delete static_cast< vector< PDRTrajectory* >* >( trajectories_array->ptr );
}

int fm_pdr_stop( PDRHandler handler, PDRTrajectoryArray* trajectories_array )
{
    if ( ! handler || ! trajectories_array )
        return PDR_RESULT_PARAMETER_ERROR;

    try
    {
        FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( handler );
        if ( hdl->m_status == PDR_STOPPED )
            return PDR_RESULT_CALL_ERROR;

        hdl->m_status = PDR_STOPPED;
        if ( hdl->m_worker.joinable() )
            hdl->m_worker.join();
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
    return PDR_RESULT_SUCCESS;
}

void fm_pdr_uninit( PDRHandler* handler )
{
    if ( ! handler || ! ( *handler ) )
        return;

    FmPDRHandler* hdl = reinterpret_cast< FmPDRHandler* >( *handler );
    delete[] hdl->m_config.model_name;
    delete[] hdl->m_config.model_file_name;
    delete hdl->m_data_loader;
    free( hdl->m_sensor_data_path );
    delete hdl;
    hdl = NULL;
}

int fm_pdr_read_pdr_data( char* dir_path, PDRData* pdr_data )
{
    if ( ! dir_path || ! pdr_data )
        return PDR_RESULT_PARAMETER_ERROR;

    int ret = PDR_RESULT_SUCCESS;

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
            ret = PDR_RESULT_EMPTY_ERROR;
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
        ret = PDR_RESULT_GENERAL_ERROR;

        // 清理已分配的内存
        cleanup_pdr_data( pdr_data );
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;

        // 清理已分配的内存
        cleanup_pdr_data( pdr_data );
    }

    return ret;
}

int fm_pdr_save_pdr_data( char* dir_path, PDRData* pdr_data )
{
    if ( ! dir_path || ! pdr_data )
        return PDR_RESULT_PARAMETER_ERROR;

    int ret = PDR_RESULT_SUCCESS;

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
            std::vector< std::pair< std::string, std::vector< double > > > acc_columns = { { "Time (s)", ptr_to_vector( sensor_data->acc_time, sensor_data->length ) },
                                                                                           { "X (m/s^2)", ptr_to_vector( sensor_data->acc_x, sensor_data->length ) },
                                                                                           { "Y (m/s^2)", ptr_to_vector( sensor_data->acc_y, sensor_data->length ) },
                                                                                           { "Z (m/s^2)", ptr_to_vector( sensor_data->acc_z, sensor_data->length ) } };

            std::string acc_path = dir_path_name + "/Accelerometer.csv";
            append_to_csv( acc_path, acc_columns );
        }

        // // 处理线性加速度计数据
        // if ( validate_sensor_data( sensor_data->lacc_time, sensor_data->lacc_x, sensor_data->lacc_y, sensor_data->lacc_z, sensor_data->length ) )
        // {
        //     std::vector< std::pair< std::string, std::vector< double > > > lacc_columns = { { "Time (s)", ptr_to_vector( sensor_data->lacc_time, sensor_data->length ) },
        //                                                                                     { "X (m/s^2)", ptr_to_vector( sensor_data->lacc_x, sensor_data->length ) },
        //                                                                                     { "Y (m/s^2)", ptr_to_vector( sensor_data->lacc_y, sensor_data->length ) },
        //                                                                                     { "Z (m/s^2)", ptr_to_vector( sensor_data->lacc_z, sensor_data->length ) } };

        //     std::string lacc_path = dir_path_name + "/LinearAccelerometer.csv";
        //     append_to_csv( lacc_path, lacc_columns );
        // }

        // 处理陀螺仪数据
        if ( validate_sensor_data( sensor_data->gyr_time, sensor_data->gyr_x, sensor_data->gyr_y, sensor_data->gyr_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > gyr_columns = { { "Time (s)", ptr_to_vector( sensor_data->gyr_time, sensor_data->length ) },
                                                                                           { "X (rad/s)", ptr_to_vector( sensor_data->gyr_x, sensor_data->length ) },
                                                                                           { "Y (rad/s)", ptr_to_vector( sensor_data->gyr_y, sensor_data->length ) },
                                                                                           { "Z (rad/s)", ptr_to_vector( sensor_data->gyr_z, sensor_data->length ) } };

            std::string gyr_path = dir_path_name + "/Gyroscope.csv";
            append_to_csv( gyr_path, gyr_columns );
        }

        // 处理磁力计数据
        if ( validate_sensor_data( sensor_data->mag_time, sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > mag_columns = { { "Time (s)", ptr_to_vector( sensor_data->mag_time, sensor_data->length ) },
                                                                                           { "X (uT)", ptr_to_vector( sensor_data->mag_x, sensor_data->length ) },
                                                                                           { "Y (uT)", ptr_to_vector( sensor_data->mag_y, sensor_data->length ) },
                                                                                           { "Z (uT)", ptr_to_vector( sensor_data->mag_z, sensor_data->length ) } };

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
                { "Time (s)", ptr_to_vector( true_data->time_location, true_data->length ) },
                { "Latitude (°)", ptr_to_vector( true_data->latitude, true_data->length ) },
                { "Longitude (°)", ptr_to_vector( true_data->longitude, true_data->length ) },
                { "Height (m)", ptr_to_vector( true_data->height, true_data->length ) },
                { "Velocity (m/s)", ptr_to_vector( true_data->velocity, true_data->length ) },
                { "Direction (°)", ptr_to_vector( true_data->direction, true_data->length ) },
                { "Horizontal Accuracy (m)", ptr_to_vector( true_data->horizontal_accuracy, true_data->length ) },
                { "Vertical Accuracy (°)", ptr_to_vector( true_data->vertical_accuracy, true_data->length ) },
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
        ret = PDR_RESULT_GENERAL_ERROR;
    }
    catch ( ... )
    {
        std::cerr << "[Unknown Error]" << std::endl;
        ret = PDR_RESULT_UNKNOWN;
    }

    return ret;
}

void fm_pdr_free_pdr_data( PDRData* pdr_data )
{
    if ( ! pdr_data )
        return;
    cleanup_pdr_data( pdr_data );
}