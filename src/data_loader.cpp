#include "data_loader.h"
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <filesystem>
#include <vector>

namespace fs = filesystem;

CFmDataLoader::CFmDataLoader( const string& test_case_path ) : m_test_case_path( test_case_path )
{
    if ( ! m_test_case_path.empty() )
    {
        load_data_from_csv();
        preprocess_data();
        generate_data();
    }
}

CFmDataLoader::~CFmDataLoader() {}

Document CFmDataLoader::load_csv( const string& filename )
{
    string full_path = m_test_case_path + "/" + filename;
    if ( ! fs::exists( full_path ) )
        throw runtime_error( "File not found: " + full_path );
    return Document( full_path, LabelParams( 0, -1 ) );
}

void CFmDataLoader::load_data_from_csv()
{
    // 读取加速度计数据
    m_doc_accelerometer = load_csv( "Accelerometer.csv" );

    // 读取线性加速度计数据（两种可能的文件名）
    if ( fs::exists( m_test_case_path + "/" + "Linear Accelerometer.csv" ) )
        m_doc_linear_accelererometer = load_csv( "Linear Accelerometer.csv" );
    else
        m_doc_linear_accelererometer = load_csv( "Linear Acceleration.csv" );

    // 读取陀螺仪数据
    m_doc_gyroscope = load_csv( "Gyroscope.csv" );

    // 读取磁力计数据
    m_doc_magnetometer = load_csv( "Magnetometer.csv" );

    // 读取位置输入数据（两种可能的文件名）
    if ( fs::exists( m_test_case_path + "/" + "Location_input.csv" ) )
        m_doc_location_input = load_csv( "Location_input.csv" );
    else
        m_doc_location_input = load_csv( "Location.csv" );

    // 检查并读取位置输出数据
    m_have_location_output = fs::exists( m_test_case_path + "/" + "Location_output.csv" );
    if ( m_have_location_output )
        m_doc_location_output = load_csv( "Location_output.csv" );

    // 检查并读取位置数据
    m_have_location_valid = fs::exists( m_test_case_path + "/" + "Location.csv" );
    if ( m_have_location_valid )
        m_doc_location = load_csv( "Location.csv" );
}

MatrixXd CFmDataLoader::nearest_neighbor_interpolation( const VectorXd& time_query, const VectorXd& time_data, const MatrixXd& data ) const
{
    // 结果矩阵：行数 = 查询时间点数，列数 = 数据维度数
    MatrixXd data_interp( time_query.size(), data.cols() );

    // 边界检查
    if ( time_data.size() == 0 || data.rows() == 0 )
        return data_interp;  // 返回空矩阵

    size_t idx = 0;  // 当前数据索引
    for ( int i = 0; i < time_query.size(); ++i )
    {
        const double t = time_query( i );

        // 推进到包含当前时间点的区间
        while ( idx < ( size_t )time_data.size() - 1 && t >= time_data( idx + 1 ) )
            ++idx;

        // 整行复制（处理所有维度）
        data_interp.row( i ) = data.row( idx );
    }

    return data_interp;
}

Eigen::MatrixXd CFmDataLoader::extract_eigen_matrix( Document& data, int start_col, int end_col, long num_rows )
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

bool CFmDataLoader::save_to_csv( const MatrixXd& matrix, const string& filename, const vector< string >& col_names )
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

void CFmDataLoader::preprocess_data( bool is_save )
{
    // 0. Location_input 对应的 time_location
    m_time_location = m_doc_location_input.GetColumn< double >( 0 );
    m_slice_start   = 0;
    m_slice_end     = m_doc_location_input.GetRowCount();
    MatrixXd preprocessed_data;

    // 1. 如果不存在 preprocessed.csv 文件，则进行预处理
    if ( ! fs::exists( m_test_case_path + "/" + "preprocessed.csv" ) )
    {
        // 2. 通过 time_location 进行 1 : 50 的插值获取 m_time
        m_time = VectorXd::Zero( m_time_location.size() * 50 );

        for ( size_t i = 0; i < m_time_location.size() - 1; ++i )
            m_time.segment( i * 50, 50 ) = VectorXd::LinSpaced( 50, m_time_location[ i ], m_time_location[ i + 1 ] - 0.02 );

        int last_index                        = m_time_location.size() - 1;
        m_time.segment( last_index * 50, 50 ) = VectorXd::LinSpaced( 50, m_time_location[ last_index ], m_time_location[ last_index ] + 0.98 );

        // 3. 根据 m_time 使用最近邻插值获取 a, la, gs, m
        const size_t&           acc_num_rows = m_doc_accelerometer.GetRowCount();
        const vector< double >& acc_time_vec = m_doc_accelerometer.GetColumn< double >( 0 );
        Map< const VectorXd >   acc_time_map( acc_time_vec.data(), acc_time_vec.size() );
        m_a = nearest_neighbor_interpolation( m_time, acc_time_map, extract_eigen_matrix( m_doc_accelerometer, 1, 3, acc_num_rows ) );

        const size_t&           lacc_num_rows = m_doc_linear_accelererometer.GetRowCount();
        const vector< double >& lacc_time_vec = m_doc_linear_accelererometer.GetColumn< double >( 0 );
        Map< const VectorXd >   lacc_time_map( lacc_time_vec.data(), lacc_time_vec.size() );
        m_la = nearest_neighbor_interpolation( m_time, lacc_time_map, extract_eigen_matrix( m_doc_linear_accelererometer, 1, 3, lacc_num_rows ) );

        const size_t&           gyrp_num_rows = m_doc_gyroscope.GetRowCount();
        const vector< double >& gyrp_time_vec = m_doc_gyroscope.GetColumn< double >( 0 );
        Map< const VectorXd >   gyrp_time_map( gyrp_time_vec.data(), gyrp_time_vec.size() );
        m_gs = nearest_neighbor_interpolation( m_time, gyrp_time_map, extract_eigen_matrix( m_doc_gyroscope, 1, 3, gyrp_num_rows ) );

        const size_t&           mag_num_rows = m_doc_magnetometer.GetRowCount();
        const vector< double >& mag_time_vec = m_doc_magnetometer.GetColumn< double >( 0 );
        Map< const VectorXd >   mag_time_map( mag_time_vec.data(), mag_time_vec.size() );
        m_m = nearest_neighbor_interpolation( m_time, mag_time_map, extract_eigen_matrix( m_doc_magnetometer, 1, 3, mag_num_rows ) );

        // 4. 保存 preprocessed.csv, 每一列分别为 "t", "a", "la", "gs", "m"
        if ( is_save )
        {
            Index tsiz = m_time.size();
            preprocessed_data.resize( tsiz, 1 + 4 * 3 );
            preprocessed_data.col( 0 )                = m_time;  // 时间列
            preprocessed_data.block( 0, 1, tsiz, 3 )  = m_a;     // a
            preprocessed_data.block( 0, 4, tsiz, 3 )  = m_la;    // la
            preprocessed_data.block( 0, 7, tsiz, 3 )  = m_gs;    // gs
            preprocessed_data.block( 0, 10, tsiz, 3 ) = m_m;     // m

            const vector< string > col_names = { "t", "a_x", "a_y", "a_z", "la_x", "la_y", "la_z", "gs_x", "gs_y", "gs_z", "m_x", "m_y", "m_z" };
            save_to_csv( preprocessed_data, "preprocessed.csv", col_names );
        }
    }
    // 5. 如果存在 preprocessed.csv 文件，则直接读取
    else
    {
        Document      preprocessed_handler = load_csv( "preprocessed.csv" );
        const size_t& num_rows             = preprocessed_handler.GetRowCount();

        preprocessed_data = extract_eigen_matrix( preprocessed_handler, -1, -1, -1 );
        m_time            = Map< VectorXd >( preprocessed_handler.GetColumn< double >( 0 ).data(), num_rows );
        m_a               = extract_eigen_matrix( preprocessed_handler, 4, 6, num_rows );
        m_gs              = extract_eigen_matrix( preprocessed_handler, 7, 9, num_rows );
        m_m               = extract_eigen_matrix( preprocessed_handler, 10, 12, num_rows );
    }

    // 8. 计算前10%的长度并获取数据
    m_len_input = static_cast< size_t >( m_time_location.size() * 0.1 );
    m_location  = extract_eigen_matrix( m_doc_location_input, -1, -1, m_len_input );

    m_latitude  = m_location.col( 1 );
    m_longitude = m_location.col( 2 );

    // # 9. 选取前 10% 中最后一个数据作为经纬度原点
    m_origin = make_pair( m_latitude( m_latitude.size() - 1 ), m_longitude( m_longitude.size() - 1 ) );

    // 10. 对 Location 进行相同的处理
    if ( m_have_location_valid )
        m_location_valid = extract_eigen_matrix( m_doc_location, -1, -1, -1 );

    // 11. 对 Location_output 进行相同的处理
    if ( m_have_location_output )
        m_location_output = extract_eigen_matrix( m_doc_location_output, -1, -1, -1 );
}

// 计算向量模长的重载函数
VectorXd CFmDataLoader::magnitude( const MatrixXd& matrix )
{
    // 验证输入矩阵的列数 (应为 3 列)
    if ( matrix.cols() != 3 )
        throw invalid_argument( "Input matrix must have 3 columns" );

    // 高效向量化计算 (避免循环)
    return ( matrix.array().square().rowwise().sum() ).sqrt();
}

void CFmDataLoader::generate_data()
{
    // 提取各轴分量 (加速度)
    m_a_x = m_a.col( 0 );
    m_a_y = m_a.col( 1 );
    m_a_z = m_a.col( 2 );

    // 提取各轴分量 (线性加速度)
    m_la_x = m_la.col( 0 );
    m_la_y = m_la.col( 1 );
    m_la_z = m_la.col( 2 );

    // 提取各轴分量 (陀螺仪)
    m_gs_x = m_gs.col( 0 );
    m_gs_y = m_gs.col( 1 );
    m_gs_z = m_gs.col( 2 );

    // 提取各轴分量 (磁力计)
    m_m_x = m_m.col( 0 );
    m_m_y = m_m.col( 1 );
    m_m_z = m_m.col( 2 );

    // 计算模长
    m_a_mag  = magnitude( m_a );
    m_la_mag = magnitude( m_la );
    m_gs_mag = magnitude( m_gs );
    m_m_mag  = magnitude( m_m );

    // 7. 通过 a - la 算出它自带的 g
    m_g     = m_a - m_la;
    m_g_x   = m_g.col( 0 );
    m_g_y   = m_g.col( 1 );
    m_g_z   = m_g.col( 2 );
    m_g_mag = magnitude( m_g );

    // 处理 Location
    m_latitude            = m_location.col( 1 );
    m_longitude           = m_location.col( 2 );
    m_height              = m_location.col( 3 );
    m_velocity            = m_location.col( 4 );
    m_direction           = m_location.col( 5 );
    m_horizontal_accuracy = m_location.col( 6 );
    m_vertical_accuracy   = m_location.col( 7 );
    // 对经纬度进行处理: 减去原点后乘以 K
    m_x = ( m_latitude.array() - m_origin.first ) * kK;
    m_y = ( m_longitude.array() - m_origin.second ) * kK;

    if ( m_have_location_valid )
    {
        m_latitude_valid            = m_location_valid.col( 1 );
        m_longitude_valid           = m_location_valid.col( 2 );
        m_height_valid              = m_location_valid.col( 3 );
        m_velocity_valid            = m_location_valid.col( 4 );
        m_direction_valid           = m_location_valid.col( 5 );
        m_horizontal_accuracy_valid = m_location_valid.col( 6 );
        m_vertical_accuracy_valid   = m_location_valid.col( 7 );
        m_x_valid                   = ( m_latitude_valid.array() - m_origin.first ) * kK;
        m_y_valid                   = ( m_longitude_valid.array() - m_origin.second ) * kK;
    }

    if ( m_have_location_output )
    {
        m_latitude_output            = m_location_output.col( 1 );
        m_longitude_output           = m_location_output.col( 2 );
        m_height_output              = m_location_output.col( 3 );
        m_velocity_output            = m_location_output.col( 4 );
        m_direction_output           = m_location_output.col( 5 );
        m_horizontal_accuracy_output = m_location_output.col( 6 );
        m_vertical_accuracy_output   = m_location_output.col( 7 );
        m_x_output                   = ( m_latitude_output.array() - m_origin.first ) * kK;
        m_y_output                   = ( m_longitude_output.array() - m_origin.second ) * kK;
    }
}

// 切片方法 - 直接返回对象
CFmDataLoader slice( const CFmDataLoader& loader, size_t start, size_t end )
{
    // 1. 处理负索引
    if ( end == 0 )
        end = loader.m_time_location.size() + end;

    // 2. 边界检查
    if ( end > static_cast< size_t >( loader.m_time_location.size() ) || start >= end )
        throw out_of_range( "Invalid slice range: start=" + to_string( start ) + ", end=" + to_string( end ) + ", size=" + to_string( loader.m_time_location.size() ) );

    // 3. 创建新对象
    CFmDataLoader new_test_case( loader.m_test_case_path );  // TODO:能否不重新加载文件？
    new_test_case.m_slice_start = start;
    new_test_case.m_slice_end   = end;

    // 4. 计算时间切片索引
    int _start = 50 * start;
    int _end   = 50 * end;

    // 6. 切片时间位置数据
    new_test_case.m_time_location = vector< double >( loader.m_time_location.begin() + start, loader.m_time_location.begin() + end );

    // 7. 切片时间序列数据
    int num_rows         = _end - _start;
    new_test_case.m_time = loader.m_time.segment( _start, num_rows );

    // 8. 切片传感器数据 (Eigen 矩阵)
    new_test_case.m_a  = loader.m_a.block( _start, 0, num_rows, loader.m_a.cols() );
    new_test_case.m_la = loader.m_la.block( _start, 0, num_rows, loader.m_la.cols() );
    new_test_case.m_gs = loader.m_gs.block( _start, 0, num_rows, loader.m_gs.cols() );
    new_test_case.m_m  = loader.m_m.block( _start, 0, num_rows, loader.m_m.cols() );

    // 9. 处理位置输入切片
    size_t start_input     = ( start < loader.m_len_input ) ? start : loader.m_len_input;
    size_t end_input       = ( end < loader.m_len_input ) ? end : loader.m_len_input;
    new_test_case.m_origin = loader.m_origin;

    if ( end_input > start_input )
    {
        int loc_rows             = end_input - start_input;
        new_test_case.m_location = loader.m_location.block( start_input, 0, loc_rows, loader.m_location.cols() );
    }
    else
    {
        // 创建空矩阵
        new_test_case.m_location = Eigen::MatrixXd( 0, loader.m_location.cols() );
    }
    new_test_case.m_len_input = end_input - start_input;

    // 10. 处理有效位置数据
    new_test_case.m_have_location_valid = loader.m_have_location_valid;
    if ( loader.m_have_location_valid && loader.m_location_valid.rows() > 0 )
    {
        int valid_rows                 = end - start;
        new_test_case.m_location_valid = loader.m_location_valid.block( start, 0, valid_rows, loader.m_location_valid.cols() );
    }

    // 11. 处理输出位置数据
    new_test_case.m_have_location_output = loader.m_have_location_output;
    if ( loader.m_have_location_output && loader.m_location_output.rows() > 0 )
    {
        int output_rows                 = end - start;
        new_test_case.m_location_output = loader.m_location_output.block( start, 0, output_rows, loader.m_location_output.cols() );
    }

    // 12. 重新生成数据
    new_test_case.generate_data();

    return new_test_case;
}

void CFmDataLoader::set_location_output( const std::vector< PDRPosition >& positions, const int from )
{
    m_have_location_output = true;
    m_location_output      = extract_eigen_matrix( m_doc_location_input, -1, -1, -1 );

    const int n = positions.size();

    // 填充矩阵数据
    for ( int i = 0; i < n; ++i )
    {
        const int idx               = i + from;
        m_location_output( idx, 0 ) = positions[ i ].time;                      // 时间戳
        m_location_output( idx, 1 ) = positions[ i ].x / kK + m_origin.first;   // 经度→纬度
        m_location_output( idx, 2 ) = positions[ i ].y / kK + m_origin.second;  // 维度→经度
        m_location_output( idx, 5 ) = positions[ i ].direction;                 // 运动方向
    }

    // 重新处理
    generate_data();

    // 保存成 CSV 文件
    const vector< string > col_names = m_doc_location_input.GetColumnNames();
    save_to_csv( m_location_output, "Location_output.csv", col_names );
}

void CFmDataLoader::set_location_output( VectorXd x, VectorXd y, VectorXd direction )
{
    // 生成经纬度
    m_latitude_output  = x.array() / kK + m_origin.first;
    m_longitude_output = y.array() / kK + m_origin.second;

    // 设置 location_output
    m_have_location_output = true;
    m_location_output      = extract_eigen_matrix( m_doc_location_input, -1, -1, -1 );

    // 放上 x, y, direction
    m_location_output.col( 1 ) = m_latitude_output;
    m_location_output.col( 2 ) = m_longitude_output;
    m_location_output.col( 5 ) = direction;

    // 覆盖上前 10% 的数据
    m_location_output.block( 0, 0, m_location.rows(), m_location.cols() ) = m_location;

    // 重新处理
    generate_data();

    // 保存成 CSV 文件
    const vector< string > col_names = m_doc_location_input.GetColumnNames();
    save_to_csv( m_location_output, "Location_output.csv", col_names );
}

void CFmDataLoader::eval_model() const
{
    // 检查是否有位置输出数据
    if ( ! m_have_location_output )
    {
        cout << "No location output" << endl;
        return;
    }

    // 检查是否有有效位置数据
    if ( ! m_have_location_valid )
    {
        cout << "No location valid" << endl;
        return;
    }

    // 计算并获取各项评估指标
    double dist_error = get_dist_error();
    double dir_error  = get_dir_error();
    double dir_ratio  = get_dir_ratio();

    // 输出评估结果
    cout << "Distances error: " << dist_error << endl;
    cout << "Direction error: " << dir_error << endl;
    cout << "Direction ratio: " << dir_ratio << endl;
}

double CFmDataLoader::get_dir_error() const
{
    // 1. 检查数据可用性
    if ( ! m_have_location_output )
    {
        cout << "No location output" << endl;
        return -1.0;  // 返回错误值
    }
    if ( ! m_have_location_valid )
    {
        cout << "No location valid" << endl;
        return -1.0;  // 返回错误值
    }

    // 2. 初始化方向误差列表
    vector< double > dir_list;

    // 3. 获取有效数据范围
    int start_index = m_len_input;
    int end_index   = m_time_location.size();

    // 4. 循环计算每个点的方向误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // 4.1 计算原始差值
        double raw_diff = abs( m_direction_valid[ i ] - m_direction_output[ i ] );

        // 4.2 计算循环差值（考虑角度周期性）
        double cyclic_diff = 360.0 - raw_diff;

        // 4.3 取两者中的较小值（实际角度差）
        double dir_error = min( raw_diff, cyclic_diff );

        // 4.4 将当前点的误差添加到列表
        dir_list.push_back( dir_error );
    }

    // 5. 检查是否有有效数据点
    if ( dir_list.empty() )
    {
        cout << "No valid direction data points for error calculation" << endl;
        return -1.0;  // 返回错误值
    }

    // 6. 计算平均误差
    double sum = 0.0;
    for ( double error : dir_list )
        sum += error;
    double average_error = sum / dir_list.size();

    // 7. 返回平均方向误差
    return average_error;
}

double CFmDataLoader::get_dir_ratio( double diff ) const
{
    // 1. 检查数据可用性
    if ( ! m_have_location_output )
    {
        cout << "No location output" << endl;
        return -1.0;  // 返回错误值
    }
    if ( ! m_have_location_valid )
    {
        cout << "No location valid" << endl;
        return -1.0;  // 返回错误值
    }

    // 2. 初始化方向误差列表
    vector< double > dir_list;

    // 3. 获取有效数据范围
    int start_index = m_len_input;
    int end_index   = m_time_location.size();

    // 4. 循环计算每个点的方向误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // 4.1 计算原始差值
        double raw_diff = abs( m_direction_valid[ i ] - m_direction_output[ i ] );

        // 4.2 计算循环差值（考虑角度周期性）
        double cyclic_diff = 360.0 - raw_diff;

        // 4.3 取两者中的较小值（实际角度差）
        double dir_error = min( raw_diff, cyclic_diff );

        // 4.4 将当前点的误差添加到列表
        dir_list.push_back( dir_error );
    }

    // 5. 检查是否有有效数据点
    if ( dir_list.empty() )
    {
        cout << "No valid direction data points for ratio calculation" << endl;
        return -1.0;  // 返回错误值
    }

    // 6. 计算满足条件的比例
    int count = count_if( dir_list.begin(), dir_list.end(),
                          [ diff ]( double error )
                          {
                              return error <= diff;
                          } );

    double ratio = static_cast< double >( count ) / dir_list.size();

    // 7. 返回比例值
    return ratio;
}

double CFmDataLoader::get_dist_error() const
{
    // 1. 检查数据可用性
    if ( ! m_have_location_output )
    {
        cout << "No location output" << endl;
        return -1.0;  // 返回错误值
    }
    if ( ! m_have_location_valid )
    {
        cout << "No location valid" << endl;
        return -1.0;  // 返回错误值
    }

    // 2. 获取Geodesic实例（使用WGS84椭球体）
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // 3. 初始化距离列表
    vector< double > dist_list;

    // 4. 获取有效数据范围
    int start_index = m_len_input;
    int end_index   = m_time_location.size();

    // 5. 循环计算每个点的距离误差
    for ( int i = start_index; i < end_index; ++i )
    {
        // 5.1 获取有效点坐标
        double lat_valid = m_latitude_valid[ i ];
        double lon_valid = m_longitude_valid[ i ];

        // 5.2 获取输出点坐标
        double lat_output = m_latitude_output[ i ];
        double lon_output = m_longitude_output[ i ];

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