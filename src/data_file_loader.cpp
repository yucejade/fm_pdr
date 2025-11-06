#include "data_file_loader.h"
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>

namespace fs = filesystem;

CFmDataFileLoader::CFmDataFileLoader() : CFmDataManager( DATA_TYPE_FILE ) {}

CFmDataFileLoader::CFmDataFileLoader( const PDRConfig& config, size_t train_data_size, const string& file_path ) : CFmDataManager( config, DATA_TYPE_FILE, train_data_size ), m_file_path( file_path )
{
    if ( file_path.empty() )
        throw std::invalid_argument( "File path cannot be empty." );

    load_data_from_file( file_path );
    preprocess_data( false );
    generate_data();
    // debug_print_data( 10 );
}

CFmDataFileLoader::~CFmDataFileLoader() {}

Document CFmDataFileLoader::load_csv( const string& filename )
{
    string full_path = m_file_path + "/" + filename;
    if ( ! fs::exists( full_path ) )
        throw runtime_error( "File not found: " + full_path );
    return Document( full_path, LabelParams( 0, -1 ) );
}

void CFmDataFileLoader::load_data_from_file( const string& file_path )
{
    // 读取加速度计数据
    m_doc_accelerometer = load_csv( "Accelerometer.csv" );

    // 读取陀螺仪数据
    m_doc_gyroscope = load_csv( "Gyroscope.csv" );

    // 读取磁力计数据
    m_doc_magnetometer = load_csv( "Magnetometer.csv" );

    // 读取位置输入数据
    // m_doc_location_input = load_csv( "Location_input.csv" );

    // 读取线性加速度计数据
    m_have_line_accelererometer = fs::exists( m_file_path + "/" + "Linear Accelerometer.csv" );
    if ( m_have_line_accelererometer )
        m_doc_linear_accelererometer = load_csv( "Linear Accelerometer.csv" );

    // 检查并读取真实位置数据，如果存在真实位置数据，则可以训练和评估，否则不需要读取真实位置数据（即：只能预测）
    m_have_location_true = fs::exists( m_file_path + "/" + "Location.csv" );
    if ( m_have_location_true )
    {
        m_doc_location = load_csv( "Location.csv" );
    }
    else
    {
        if ( m_train_data_size > 0 )
            throw std::invalid_argument( "No true location data found, cannot determine time axis." );
    }
}

void CFmDataFileLoader::preprocess_data( bool is_save )
{
    m_slice_start = 0;
    m_slice_end   = m_doc_accelerometer.GetRowCount();

    // 如果包含训练数据，则先对齐真实位置时间戳，在进行训练和预测
    if ( m_train_data_size > 0 )
    {
        if ( m_have_location_true )
        {
            const vector< double >& time_location_vec = m_doc_location.GetColumn< double >( 0 );
            Map< const VectorXd >   time_location_map( time_location_vec.data(), time_location_vec.size() );
            Eigen::Index            time_location_size = time_location_map.size();
            if ( m_train_data_size > ( size_t )time_location_size )
            {
                if (m_train_data_size == (size_t)-1)
                    m_train_data_size = time_location_size;
                else
                    throw std::invalid_argument( "Train data size exceeds available true location data." );
            }

            m_time_location_true = time_location_map;
            m_time_location      = m_time_location_true.head( m_train_data_size ).eval();
            m_time               = VectorXd::Zero( time_location_size * m_config->sample_rate );

            for ( Eigen::Index i = 0; i < time_location_size - 1; ++i )
                m_time.segment( i * m_config->sample_rate, m_config->sample_rate ) = VectorXd::LinSpaced( m_config->sample_rate, m_time_location_true[ i ], m_time_location_true[ i + 1 ] - 1.0 / m_config->sample_rate );

            int last_index                                                              = time_location_size - 1;
            m_time.segment( last_index * m_config->sample_rate, m_config->sample_rate ) = VectorXd::LinSpaced( m_config->sample_rate, m_time_location_true[ last_index ], m_time_location_true[ last_index ] + ( 1 - 1.0 / m_config->sample_rate ) );

            // 获取 a, la, gs, m
            const size_t&           acc_num_rows = m_doc_accelerometer.GetRowCount();
            const vector< double >& acc_time_vec = m_doc_accelerometer.GetColumn< double >( 0 );
            Map< const VectorXd >   acc_time_map( acc_time_vec.data(), acc_time_vec.size() );

            const size_t&           gyrp_num_rows = m_doc_gyroscope.GetRowCount();
            const vector< double >& gyrp_time_vec = m_doc_gyroscope.GetColumn< double >( 0 );
            Map< const VectorXd >   gyrp_time_map( gyrp_time_vec.data(), gyrp_time_vec.size() );

            const size_t&           mag_num_rows = m_doc_magnetometer.GetRowCount();
            const vector< double >& mag_time_vec = m_doc_magnetometer.GetColumn< double >( 0 );
            Map< const VectorXd >   mag_time_map( mag_time_vec.data(), mag_time_vec.size() );

            // 根据 m_time 使用最近邻插值获取 a, la, gs, m
            m_a  = nearest_neighbor_interpolation( m_time, acc_time_map, extract_eigen_matrix( m_doc_accelerometer, 1, 3, acc_num_rows ) );
            m_gs = nearest_neighbor_interpolation( m_time, gyrp_time_map, extract_eigen_matrix( m_doc_gyroscope, 1, 3, gyrp_num_rows ) );
            m_m  = nearest_neighbor_interpolation( m_time, mag_time_map, extract_eigen_matrix( m_doc_magnetometer, 1, 3, mag_num_rows ) );
            if ( m_have_line_accelererometer )
            {
                const size_t&           lacc_num_rows = m_doc_linear_accelererometer.GetRowCount();
                const vector< double >& lacc_time_vec = m_doc_linear_accelererometer.GetColumn< double >( 0 );
                Map< const VectorXd >   lacc_time_map( lacc_time_vec.data(), lacc_time_vec.size() );
                m_la = nearest_neighbor_interpolation( m_time, lacc_time_map, extract_eigen_matrix( m_doc_linear_accelererometer, 1, 3, lacc_num_rows ) );

                // 通过 a - la 算出它自带的 g
                m_g = m_a - m_la;
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
        const vector< double >& time_location_vec = m_doc_location.GetColumn< double >( 0 );
        Map< const VectorXd >   time_location_map( time_location_vec.data(), time_location_vec.size() );
        m_time_location_true = time_location_map;

        // 如果没有训练数据，则直接使用ACC数据的时间戳作为时间轴
        // 获取 a, la, gs, m
        const size_t&           acc_num_rows = m_doc_accelerometer.GetRowCount();
        const vector< double >& acc_time_vec = m_doc_accelerometer.GetColumn< double >( 0 );
        Map< const VectorXd >   acc_time_map( acc_time_vec.data(), acc_time_vec.size() );

        const size_t&           gyrp_num_rows = m_doc_gyroscope.GetRowCount();
        const vector< double >& gyrp_time_vec = m_doc_gyroscope.GetColumn< double >( 0 );
        Map< const VectorXd >   gyrp_time_map( gyrp_time_vec.data(), gyrp_time_vec.size() );

        const size_t&           mag_num_rows = m_doc_magnetometer.GetRowCount();
        const vector< double >& mag_time_vec = m_doc_magnetometer.GetColumn< double >( 0 );
        Map< const VectorXd >   mag_time_map( mag_time_vec.data(), mag_time_vec.size() );

        m_time    = acc_time_map;

        m_a  = nearest_neighbor_interpolation( m_time, acc_time_map, extract_eigen_matrix( m_doc_accelerometer, 1, 3, acc_num_rows ) );
        m_gs = nearest_neighbor_interpolation( m_time, gyrp_time_map, extract_eigen_matrix( m_doc_gyroscope, 1, 3, gyrp_num_rows ) );
        m_m  = nearest_neighbor_interpolation( m_time, mag_time_map, extract_eigen_matrix( m_doc_magnetometer, 1, 3, mag_num_rows ) );
        if ( m_have_line_accelererometer )
        {
            const size_t&           lacc_num_rows = m_doc_linear_accelererometer.GetRowCount();
            const vector< double >& lacc_time_vec = m_doc_linear_accelererometer.GetColumn< double >( 0 );
            Map< const VectorXd >   lacc_time_map( lacc_time_vec.data(), lacc_time_vec.size() );
            m_la = nearest_neighbor_interpolation( m_time, lacc_time_map, extract_eigen_matrix( m_doc_linear_accelererometer, 1, 3, lacc_num_rows ) );

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
        m_location      = extract_eigen_matrix( m_doc_location, -1, -1, m_train_data_size );
        m_location_true = extract_eigen_matrix( m_doc_location, -1, -1, -1 );
    }
}

void CFmDataFileLoader::generate_data()
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
CFmDataFileLoader* slice( const CFmDataFileLoader& file_loader, size_t start, size_t end )
{
    // 处理负索引
    if ( end == 0 )
        end = file_loader.m_time.size();

    // 边界检查
    if ( end > static_cast< size_t >( file_loader.m_time.size() ) || start >= end )
        throw out_of_range( "Invalid slice range: start=" + to_string( start ) + ", end=" + to_string( end ) + ", size=" + to_string( file_loader.m_time.size() ) );

    // 创建新对象
    CFmDataFileLoader* new_file_loader = new CFmDataFileLoader();  // 切片数据的训练数据大小始终为0
    new_file_loader->m_config          = file_loader.m_config;
    new_file_loader->m_slice_start     = start;
    new_file_loader->m_slice_end       = end;

    // 复制其它变量
    new_file_loader->m_file_path                 = file_loader.m_file_path;
    new_file_loader->m_have_line_accelererometer = file_loader.m_have_line_accelererometer;
    new_file_loader->m_have_location_true        = file_loader.m_have_location_true;

    // 切片原点
    new_file_loader->m_origin = file_loader.m_origin;

    // 计算时间切片索引
    int num_rows            = end - start;
    new_file_loader->m_time = file_loader.m_time.segment( start, num_rows );

    // 切片传感器数据
    new_file_loader->m_a  = file_loader.m_a.block( start, 0, num_rows, file_loader.m_a.cols() );
    new_file_loader->m_la = file_loader.m_la.block( start, 0, num_rows, file_loader.m_la.cols() );
    new_file_loader->m_gs = file_loader.m_gs.block( start, 0, num_rows, file_loader.m_gs.cols() );
    new_file_loader->m_m  = file_loader.m_m.block( start, 0, num_rows, file_loader.m_m.cols() );
    new_file_loader->m_g  = file_loader.m_g.block( start, 0, num_rows, file_loader.m_g.cols() );

    // 切片时间位置数据
    size_t _start = start / file_loader.m_config->sample_rate;
    size_t _end   = end / file_loader.m_config->sample_rate;

    // 处理位置输入切片和训练数据时间切片
    size_t start_input = ( _start < file_loader.m_train_data_size ) ? _start : file_loader.m_train_data_size;
    size_t end_input   = ( _end < file_loader.m_train_data_size ) ? _end : file_loader.m_train_data_size;

    if ( file_loader.m_train_data_size > 0 && end_input > start_input )
    {
        size_t loc_rows                  = end_input - start_input;
        new_file_loader->m_time_location = file_loader.m_time_location.segment( _start, loc_rows );
        new_file_loader->m_location      = file_loader.m_location.block( start_input, 0, loc_rows, file_loader.m_location.cols() );
    }
    else
    {
        // 创建空矩阵
        new_file_loader->m_time_location = Eigen::VectorXd( 0 );
        new_file_loader->m_location      = Eigen::MatrixXd( 0, file_loader.m_location.cols() );
    }
    new_file_loader->m_train_data_size = end_input - start_input;

    // 处理有效位置数据
    new_file_loader->m_have_location_true = file_loader.m_have_location_true;
    if ( file_loader.m_have_location_true && file_loader.m_location_true.rows() > 0 )
    {
        size_t true_rows                      = _end - _start;
        new_file_loader->m_time_location_true = file_loader.m_time_location_true.segment( _start, true_rows );
        new_file_loader->m_location_true      = file_loader.m_location_true.block( _start, 0, true_rows, file_loader.m_location_true.cols() );
    }

    // 重新生成数据
    new_file_loader->generate_data();

    return new_file_loader;
}

Eigen::MatrixXd CFmDataFileLoader::extract_eigen_matrix( Document& data, int start_col, int end_col, long num_rows )
{
    // 获取文档的实际尺寸
    const long total_rows = static_cast< long >( data.GetRowCount() );
    const long total_cols = static_cast< long >( data.GetColumnCount() );

    // 处理 num_rows 参数
    if ( num_rows < 0 )
        num_rows = total_rows;  // -1 表示获取所有行
    else if ( num_rows > total_rows )
        throw out_of_range( "请求的行数超过文档总行数" );

    // 处理列范围参数
    long actual_start_col = 0;
    long actual_end_col   = total_cols - 1;

    if ( start_col != -1 && end_col != -1 )
    {
        // 使用指定的列范围
        actual_start_col = start_col;
        actual_end_col   = end_col;
    }
    else if ( start_col == -1 || end_col == -1 )
    {
        // 如果任一列为-1，则获取所有列
        actual_start_col = 0;
        actual_end_col   = total_cols - 1;
    }

    // 验证列范围有效性
    if ( actual_start_col < 0 || actual_end_col >= total_cols || actual_start_col > actual_end_col )
        throw invalid_argument( "无效的列范围" );

    // 计算实际列数
    const long num_cols = actual_end_col - actual_start_col + 1;

    // 创建结果矩阵
    Eigen::MatrixXd mat( num_rows, num_cols );

    // 填充矩阵数据
    for ( long row = 0; row < num_rows; ++row )
    {
        // 先获取字符串格式的行数据
        std::vector< std::string > strRow = data.GetRow< std::string >( row );

        for ( long col = 0; col < num_cols; ++col )
        {
            if ( strRow[ actual_start_col + col ].empty() )
            {
                // 空值处理：使用特殊值或NaN
                mat( row, col ) = std::numeric_limits< double >::quiet_NaN();
            }
            else
            {
                try
                {
                    // 处理科学计数法
                    std::stringstream ss( strRow[ actual_start_col + col ] );
                    double            val;
                    ss >> val;
                    mat( row, col ) = val;
                }
                catch ( ... )
                {
                    // 处理转换失败
                    mat( row, col ) = std::numeric_limits< double >::quiet_NaN();
                }
            }
        }
    }

    return mat;
}