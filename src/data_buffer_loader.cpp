#include "data_buffer_loader.h"
#include "data_manager.h"
#include <Eigen/src/Core/Matrix.h>
#include <ostream>

CFmDataBufferLoader::CFmDataBufferLoader() : CFmDataManager( DATA_TYPE_BUFFER ) {}

CFmDataBufferLoader::CFmDataBufferLoader( const PDRConfig& config, size_t train_data_size, const PDRData& data ) : CFmDataManager( config, DATA_TYPE_BUFFER, train_data_size )
{
    m_have_location_true        = ( data.true_data.length > 0 );
    m_have_line_accelererometer = ( data.sensor_data.lacc_x != nullptr && data.sensor_data.lacc_y != nullptr && data.sensor_data.lacc_z != nullptr );

    preprocess_data( data, false );
    generate_data();
    // debug_print_data(10);
}

CFmDataBufferLoader::~CFmDataBufferLoader() {}

void CFmDataBufferLoader::preprocess_data( const PDRData& data, bool is_save )
{
    m_slice_start = 0;
    m_slice_end   = data.sensor_data.length;

    // 如果包含训练数据，则先对齐真实位置时间戳，在进行训练和预测
    if ( m_train_data_size > 0 )
    {
        if ( m_have_location_true )
        {
            Map< const VectorXd > time_location_map( data.true_data.time_location, data.true_data.length );
            Eigen::Index          time_location_size = time_location_map.size();
            if ( m_train_data_size > ( size_t )time_location_size )
                throw std::invalid_argument( "Train data size exceeds available true location data." );

            m_time_location_true = time_location_map;
            m_time_location      = m_time_location_true.head( m_train_data_size ).eval();
            m_time               = VectorXd::Zero( time_location_size * m_config->sample_rate );

            for ( Eigen::Index i = 0; i < time_location_size - 1; ++i )
                m_time.segment( i * m_config->sample_rate, m_config->sample_rate ) = VectorXd::LinSpaced( m_config->sample_rate, m_time_location_true[ i ], m_time_location_true[ i + 1 ] - 1.0 / m_config->sample_rate );

            int last_index                                                              = time_location_size - 1;
            m_time.segment( last_index * m_config->sample_rate, m_config->sample_rate ) = VectorXd::LinSpaced( m_config->sample_rate, m_time_location_true[ last_index ], m_time_location_true[ last_index ] + ( 1 - 1.0 / m_config->sample_rate ) );

            // 获取 a, la, gs, m
            Map< const VectorXd > acc_time_map( data.sensor_data.acc_time, data.sensor_data.length );
            Map< const VectorXd > gyrp_time_map( data.sensor_data.gyr_time, data.sensor_data.length );
            Map< const VectorXd > mag_time_map( data.sensor_data.mag_time, data.sensor_data.length );

            // 根据 m_time 使用最近邻插值获取 a, la, gs, m
            m_a  = nearest_neighbor_interpolation( m_time, acc_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 1, 3, data.sensor_data.length ) );
            m_gs = nearest_neighbor_interpolation( m_time, gyrp_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 9, 11, data.sensor_data.length ) );
            m_m  = nearest_neighbor_interpolation( m_time, mag_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 13, 15, data.sensor_data.length ) );
            if ( m_have_line_accelererometer )
            {
                Map< const VectorXd > lacc_time_map( data.sensor_data.lacc_time, data.sensor_data.length );
                m_la = nearest_neighbor_interpolation( m_time, lacc_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 5, 7, data.sensor_data.length ) );

                // 通过 a - la 算出它自带的 g
                m_g = m_a - m_la;
                // m_g = get_gravity_with_ahrs( m_a, m_gs, m_m );

                // m_g = m_a - m_la;
                // for ( int i = 0; i < 100; ++i )
                //     std::cout << std::fixed << std::setprecision( 3 ) << "{" << m_g( i, 0 ) << "," << m_g( i, 1 ) << "," << m_g( i, 2 ) << "}" << std::endl;

                // std::cout << "----------------------------------" << std::endl;

                // Eigen::MatrixXd m_g2 = get_gravity_with_ahrs( m_a, m_gs, m_m );
                // for ( int i = 0; i < 100; ++i )
                //     std::cout << std::fixed << std::setprecision( 3 ) << "{" << m_g2( i, 0 ) << "," << m_g2( i, 1 ) << "," << m_g2( i, 2 ) << "}" << std::endl;
            }
            else
            {
                m_g = get_gravity_with_ahrs( m_a, m_gs, m_m );
            }
        }
        else
        {
            throw std::invalid_argument( "No true location data found, cannot determine time axis." );
        }
    }
    else
    {
        Map< const VectorXd > time_location_map( data.true_data.time_location, data.true_data.length );
        m_time_location_true = time_location_map;

        // 如果没有训练数据，则直接使用PDR数据的时间戳作为时间轴
        Map< const VectorXd > acc_time_map( data.sensor_data.acc_time, data.sensor_data.length );
        Map< const VectorXd > gyrp_time_map( data.sensor_data.gyr_time, data.sensor_data.length );
        Map< const VectorXd > mag_time_map( data.sensor_data.mag_time, data.sensor_data.length );
        m_time = acc_time_map;

        // 获取 a, la, gs, m
        m_a  = nearest_neighbor_interpolation( m_time, acc_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 1, 3, data.sensor_data.length ) );
        m_gs = nearest_neighbor_interpolation( m_time, gyrp_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 9, 11, data.sensor_data.length ) );
        m_m  = nearest_neighbor_interpolation( m_time, mag_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 13, 15, data.sensor_data.length ) );
        if ( m_have_line_accelererometer )
        {
            Map< const VectorXd > lacc_time_map( data.sensor_data.lacc_time, data.sensor_data.length );
            m_la = nearest_neighbor_interpolation( m_time, lacc_time_map, extract_eigen_matrix( ( void* )&data.sensor_data, 0, 5, 7, data.sensor_data.length ) );

            // 通过 a - la 算出它自带的 g
            m_g = m_a - m_la;
        }
        else
        {
            m_g = get_gravity_with_ahrs( m_a, m_gs, m_m );
        }
    }

    // 保存 preprocessed.csv, 每一列分别为 "t", "a", "la", "gs", "m"
    if ( is_save )
    {
        Index    tsiz = m_time.size();
        MatrixXd preprocessed_data;

        preprocessed_data.resize( tsiz, 1 + 4 * 3 );
        preprocessed_data.col( 0 )                = m_time;  // 时间列
        preprocessed_data.block( 0, 1, tsiz, 3 )  = m_a;     // a
        preprocessed_data.block( 0, 4, tsiz, 3 )  = m_la;    // la
        preprocessed_data.block( 0, 7, tsiz, 3 )  = m_gs;    // gs
        preprocessed_data.block( 0, 10, tsiz, 3 ) = m_m;     // m

        const vector< string > col_names = { "t", "a_x", "a_y", "a_z", "la_x", "la_y", "la_z", "gs_x", "gs_y", "gs_z", "m_x", "m_y", "m_z" };
        save_to_csv( preprocessed_data, "preprocessed.csv", col_names );
    }

    // 对 Location 进行相同的处理
    if ( m_have_location_true )
    {
        m_location      = extract_eigen_matrix( ( void* )&data.true_data, 1, -1, -1, m_train_data_size );
        m_location_true = extract_eigen_matrix( ( void* )&data.true_data, 1, -1, -1, -1 );
    }
}

void CFmDataBufferLoader::generate_data()
{
    // 提取各轴分量 (加速度)和模长
    m_a_x   = m_a.col( 0 );
    m_a_y   = m_a.col( 1 );
    m_a_z   = m_a.col( 2 );
    m_a_mag = magnitude( m_a );

    // 提取各轴分量 (线性加速度)和模长
    if ( m_have_line_accelererometer )
    {
        m_la_x   = m_la.col( 0 );
        m_la_y   = m_la.col( 1 );
        m_la_z   = m_la.col( 2 );
        m_la_mag = magnitude( m_la );
    }

    // 提取各轴分量(重力加速度)和模长
    m_g_x   = m_g.col( 0 );
    m_g_y   = m_g.col( 1 );
    m_g_z   = m_g.col( 2 );
    m_g_mag = magnitude( m_g );

    // cout << "acc: [" << m_a_x[0] << "," << m_a_y[0] << "," << m_a_z[0] << "], lacc: [" << m_la_x[0] << "," << m_la_y[0] << "," << m_la_z[0] << "]" << "], grv: [" << m_g_x[0] << "," << m_g_y[0] << "," << m_g_z[0] << "]" << endl;

    // 提取各轴分量 (陀螺仪)和模长
    m_gs_x   = m_gs.col( 0 );
    m_gs_y   = m_gs.col( 1 );
    m_gs_z   = m_gs.col( 2 );
    m_gs_mag = magnitude( m_gs );

    // 提取各轴分量 (磁力计)和模长
    m_m_x   = m_m.col( 0 );
    m_m_y   = m_m.col( 1 );
    m_m_z   = m_m.col( 2 );
    m_m_mag = magnitude( m_m );

    if ( m_have_location_true )
    {
        // 处理训练定位数据
        if ( m_train_data_size > 0 )
        {
            m_latitude            = m_location.col( 1 );
            m_longitude           = m_location.col( 2 );
            m_height              = m_location.col( 3 );
            m_velocity            = m_location.col( 4 );
            m_direction           = m_location.col( 5 );
            m_horizontal_accuracy = m_location.col( 6 );
            m_vertical_accuracy   = m_location.col( 7 );
            m_x                   = ( m_latitude.array() - m_latitude( m_train_data_size - 1 ) ) * kK;
            m_y                   = ( m_longitude.array() - m_longitude( m_train_data_size - 1 ) ) * kK;
        }

        // 处理真实定位数据
        m_latitude_true            = m_location_true.col( 1 );
        m_longitude_true           = m_location_true.col( 2 );
        m_height_true              = m_location_true.col( 3 );
        m_velocity_true            = m_location_true.col( 4 );
        m_direction_true           = m_location_true.col( 5 );
        m_horizontal_accuracy_true = m_location_true.col( 6 );
        m_vertical_accuracy_true   = m_location_true.col( 7 );
        m_x_true                   = ( m_latitude_true.array() - m_latitude_true( 0 ) ) * kK;
        m_y_true                   = ( m_longitude_true.array() - m_longitude_true( 0 ) ) * kK;

        // 如果包含训练数据，则以最后一个训练数据的经纬度作为原点；否则以第一条数据作为原点
        m_origin = make_pair( m_train_data_size > 0 ? m_latitude( m_train_data_size - 1 ) : m_latitude_true( 0 ), m_train_data_size > 0 ? m_longitude( m_train_data_size - 1 ) : m_longitude_true( 0 ) );
    }
    else
    {
        if ( m_train_data_size > 0 )
            throw std::invalid_argument( "No true location data found, cannot determine time axis." );
    }
}

// 切片方法 - 直接返回对象
CFmDataBufferLoader* slice( const CFmDataBufferLoader& buffer_loader, size_t start, size_t end )
{
    // 处理负索引
    if ( end == 0 )
        end = buffer_loader.m_time.size();

    // 边界检查
    if ( end > static_cast< size_t >( buffer_loader.m_time.size() ) || start >= end )
        throw out_of_range( "Invalid slice range: start=" + to_string( start ) + ", end=" + to_string( end ) + ", size=" + to_string( buffer_loader.m_time.size() ) );

    // 创建新对象
    CFmDataBufferLoader* new_buffer_loader = new CFmDataBufferLoader();  // 切片数据的训练数据大小始终为0
    new_buffer_loader->m_config            = buffer_loader.m_config;
    new_buffer_loader->m_slice_start       = start;
    new_buffer_loader->m_slice_end         = end;

    // 复制其它变量
    new_buffer_loader->m_have_line_accelererometer = buffer_loader.m_have_line_accelererometer;
    new_buffer_loader->m_have_location_true        = buffer_loader.m_have_location_true;

    // 切片原点
    new_buffer_loader->m_origin = buffer_loader.m_origin;

    // 计算时间切片索引
    int num_rows              = end - start;
    new_buffer_loader->m_time = buffer_loader.m_time.segment( start, num_rows );

    // 切片传感器数据
    new_buffer_loader->m_a  = buffer_loader.m_a.block( start, 0, num_rows, buffer_loader.m_a.cols() );
    new_buffer_loader->m_la = buffer_loader.m_la.block( start, 0, num_rows, buffer_loader.m_la.cols() );
    new_buffer_loader->m_gs = buffer_loader.m_gs.block( start, 0, num_rows, buffer_loader.m_gs.cols() );
    new_buffer_loader->m_m  = buffer_loader.m_m.block( start, 0, num_rows, buffer_loader.m_m.cols() );
    new_buffer_loader->m_g  = buffer_loader.m_g.block( start, 0, num_rows, buffer_loader.m_g.cols() );

    // 切片时间位置数据
    size_t _start = start / buffer_loader.m_config->sample_rate;
    size_t _end   = end / buffer_loader.m_config->sample_rate;

    // 处理位置输入切片和训练数据时间切片
    size_t start_input = ( _start < buffer_loader.m_train_data_size ) ? _start : buffer_loader.m_train_data_size;
    size_t end_input   = ( _end < buffer_loader.m_train_data_size ) ? _end : buffer_loader.m_train_data_size;

    if ( buffer_loader.m_train_data_size > 0 && end_input > start_input )
    {
        size_t loc_rows                    = end_input - start_input;
        new_buffer_loader->m_time_location = buffer_loader.m_time_location.segment( _start, loc_rows );
        new_buffer_loader->m_location      = buffer_loader.m_location.block( start_input, 0, loc_rows, buffer_loader.m_location.cols() );
    }
    else
    {
        // 创建空矩阵
        new_buffer_loader->m_time_location = Eigen::VectorXd( 0 );
        new_buffer_loader->m_location      = Eigen::MatrixXd( 0, buffer_loader.m_location.cols() );
    }
    new_buffer_loader->m_train_data_size = end_input - start_input;

    // 处理有效位置数据
    new_buffer_loader->m_have_location_true = buffer_loader.m_have_location_true;
    if ( buffer_loader.m_have_location_true && buffer_loader.m_location_true.rows() > 0 )
    {
        size_t true_rows                        = _end - _start;
        new_buffer_loader->m_time_location_true = buffer_loader.m_time_location_true.segment( _start, true_rows );
        new_buffer_loader->m_location_true      = buffer_loader.m_location_true.block( _start, 0, true_rows, buffer_loader.m_location_true.cols() );
    }

    // 重新生成数据
    new_buffer_loader->generate_data();

    return new_buffer_loader;
}

double* CFmDataBufferLoader::get_sensor_field_ptr( PDRSensorData* data, int col )
{
    switch ( col )
    {
        case 0:
            return data->acc_time;
        case 1:
            return data->acc_x;
        case 2:
            return data->acc_y;
        case 3:
            return data->acc_z;
        case 4:
            return data->lacc_time;
        case 5:
            return data->lacc_x;
        case 6:
            return data->lacc_y;
        case 7:
            return data->lacc_z;
        case 8:
            return data->gyr_time;
        case 9:
            return data->gyr_x;
        case 10:
            return data->gyr_y;
        case 11:
            return data->gyr_z;
        case 12:
            return data->mag_time;
        case 13:
            return data->mag_x;
        case 14:
            return data->mag_y;
        case 15:
            return data->mag_z;
        default:
            return nullptr;
    }
}

double* CFmDataBufferLoader::get_true_field_ptr( PDRTrueData* data, int col )
{
    switch ( col )
    {
        case 0:
            return data->time_location;
        case 1:
            return data->latitude;
        case 2:
            return data->longitude;
        case 3:
            return data->height;
        case 4:
            return data->velocity;
        case 5:
            return data->direction;
        case 6:
            return data->horizontal_accuracy;
        case 7:
            return data->vertical_accuracy;
        default:
            return nullptr;
    }
}

Eigen::MatrixXd CFmDataBufferLoader::extract_eigen_matrix( void* pointer, int type, int start_col, int end_col, unsigned long num_rows )
{
    // 步骤1：处理特殊情况（start_col < 0 或 end_col < 0）
    int actual_start_col = ( start_col < 0 ) ? 0 : start_col;
    int actual_end_col;

    // 步骤2：根据pointer类型确定总列数（需先判断类型才能获取列数）
    int  total_cols     = -1;
    bool is_sensor_data = false;
    bool is_true_data   = false;

    // 先判断pointer类型并获取总列数（避免重复类型转换）
    if ( type == 0 )
    {
        is_sensor_data = true;
        total_cols     = ( sizeof( PDRSensorData ) - sizeof( unsigned long ) ) / sizeof( double* );
    }
    else
    {
        if ( type == 1 )
        {
            is_true_data = true;
            total_cols   = ( sizeof( PDRTrueData ) - sizeof( unsigned long ) ) / sizeof( double* );
        }
        else
        {
            assert( false && "Unsupported struct type (before column calculation)" );
            return Eigen::MatrixXd();
        }
    }

    // 处理end_col < 0的情况（此时actual_end_col为总列数）
    actual_end_col = ( end_col < 0 ) ? total_cols - 1 : end_col;

    // 步骤3：参数校验（实际列范围需有效）
    assert( actual_start_col >= 0 && actual_end_col > actual_start_col && actual_end_col <= total_cols && "Invalid column range after adjusting start/end" );
    int num_cols = actual_end_col - actual_start_col + 1;

    // 步骤4：根据类型提取矩阵（复用之前的类型判断结果）
    if ( is_sensor_data )
    {
        PDRSensorData* sensor_data = static_cast< PDRSensorData* >( pointer );
        num_rows                   = ( num_rows == static_cast< unsigned long >( -1 ) ? sensor_data->length : num_rows );
        assert( num_rows <= sensor_data->length && "num_rows exceeds sensor data length" );
        Eigen::MatrixXd mat( num_rows, num_cols );
        for ( int col = 0; col < num_cols; ++col )
        {
            int     field_col = actual_start_col + col;
            double* field_ptr = get_sensor_field_ptr( sensor_data, field_col );
            assert( field_ptr != nullptr && "Invalid column index for PDRSensorData" );
            mat.col( col ) = Eigen::Map< Eigen::VectorXd >( field_ptr, num_rows );
        }

        return mat;
    }
    else if ( is_true_data )
    {
        PDRTrueData* true_data = static_cast< PDRTrueData* >( pointer );
        num_rows               = ( num_rows == static_cast< unsigned long >( -1 ) ? true_data->length : num_rows );
        assert( num_rows <= true_data->length && "num_rows exceeds true data length" );
        Eigen::MatrixXd mat( num_rows, num_cols );
        for ( int col = 0; col < num_cols; ++col )
        {
            int     field_col = actual_start_col + col;
            double* field_ptr = get_true_field_ptr( true_data, field_col );
            assert( field_ptr != nullptr && "Invalid column index for PDRTrueData" );
            mat.col( col ) = Eigen::Map< Eigen::VectorXd >( field_ptr, num_rows );
        }
        return mat;
    }

    //  unreachable（已在前面校验类型）
    assert( false && "Unsupported struct type (final check)" );
    return Eigen::MatrixXd();
}