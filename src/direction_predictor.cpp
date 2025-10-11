#include "direction_predictor.h"
#include "fm_pdr.h"
#include <algorithm>

CFmDirectionPredictor::CFmDirectionPredictor( const PDRConfig& config ) : m_config(config)
{
    // f.setup(sampling_rate, cutoff_freq);
    m_f.setupN( config.butter_wn );
}

CFmDirectionPredictor::~CFmDirectionPredictor() {}

// 实现零相位滤波 (Eigen版本)
Eigen::VectorXd CFmDirectionPredictor::filtfilt( Iir::Butterworth::LowPass< 2, Iir::DirectFormII >& filter, const Eigen::VectorXd& input )
{
    const int N = input.size();
    if ( N < 3 )
        return input;

    // 1. 镜像填充 (使用Eigen块操作)
    const int       pad_len = std::min( 100, N / 2 );
    Eigen::VectorXd padded( 2 * pad_len + N );

    // 前端镜像填充 (前pad_len个元素的反向)
    padded.head( pad_len ) = input.head( pad_len ).reverse();

    // 原始数据
    padded.segment( pad_len, N ) = input;

    // 后端镜像填充 (后pad_len个元素的反向)
    padded.tail( pad_len ) = input.tail( pad_len ).reverse();

    // 2. 预热滤波器建立稳态
    filter.reset();
    double init_val = padded[ 0 ];
    for ( int i = 0; i < 1000; i++ )
        filter.filter( init_val );

    // 3. 正向滤波
    Eigen::VectorXd forward( padded.size() );
    for ( int i = 0; i < padded.size(); i++ )
        forward[ i ] = filter.filter( padded[ i ] );

    // 4. 反向滤波
    filter.reset();
    Eigen::VectorXd reversed = forward.reverse();
    Eigen::VectorXd backward( reversed.size() );
    for ( int i = 0; i < reversed.size(); i++ )
        backward[ i ] = filter.filter( reversed[ i ] );

    // 5. 反转并裁剪结果
    Eigen::VectorXd full_result = backward.reverse();
    return full_result.segment( pad_len, N );
}

void CFmDirectionPredictor::butterworth_filter( const CFmDataManager& data, MatrixXd& mag, MatrixXd& grv )
{
    m_f.reset();

    // const VectorXd &mag_x = data.get_pdr_data( PDR_DATA_FIELD_MAG_X );
    // const VectorXd &mag_y = data.get_pdr_data( PDR_DATA_FIELD_MAG_Y );
    // const VectorXd &mag_z = data.get_pdr_data( PDR_DATA_FIELD_MAG_Z );

    // const VectorXd &grv_x = data.get_pdr_data( PDR_DATA_FIELD_GRV_X );
    // const VectorXd &grv_y = data.get_pdr_data( PDR_DATA_FIELD_GRV_Y );
    // const VectorXd &grv_z = data.get_pdr_data( PDR_DATA_FIELD_GRV_Z );

    // cout << "mag: [" << mag_x[0] << "," << mag_y[0] << "," << mag_z[0] << "], grv: [" << grv_x[0] << "," << grv_y[0] << "," << grv_z[0] << "]" << endl;

    // 低通滤波
    VectorXd x = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_MAG_X ) );
    VectorXd y = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_MAG_Y ) );
    VectorXd z = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_MAG_Z ) );

    mag.col( 0 ) = x;
    mag.col( 1 ) = y;
    mag.col( 2 ) = z;

    // 对 a 低通滤波得到重力加速度
    VectorXd g_x = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_GRV_X ) );
    VectorXd g_y = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_GRV_Y ) );
    VectorXd g_z = filtfilt( m_f, data.get_pdr_data( PDR_DATA_FIELD_GRV_Z ) );

    grv.col( 0 ) = g_x;
    grv.col( 1 ) = g_y;
    grv.col( 2 ) = g_z;
}

Eigen::MatrixXd CFmDirectionPredictor::calc_east_vector( const MatrixXd& mag, const MatrixXd& grv, const int& rows )
{
    const int       k_cols = 3;
    Eigen::MatrixXd e( rows, k_cols );

    for ( int i = 0; i < rows; ++i )
    {
        Eigen::Vector3d m_vec = mag.row( i );                      // 提取磁场向量
        Eigen::Vector3d g_vec = grv.row( i );                      // 提取重力向量
        e.row( i )            = g_vec.cross( m_vec ).transpose();  // 叉乘得到东向量
        // cout << "m_vec: [" << m_vec[ 0 ] << "," << m_vec[ 1 ]  << "," << m_vec[ 2 ] << "], g_vec: [" << g_vec[ 0 ]  << "," << g_vec[ 1 ] << "," << g_vec[ 2 ] << "], e: [" << e.row( i )[0] << "," << e.row( i )[1] << "," << e.row( i )[2] << "]" << endl;
    }

    return e;
}

StartInfo CFmDirectionPredictor::start( const CFmDataManager& start_data, const int least_point )
{
    // 必须有两个及以上点才能计算方向
    const size_t mag_rows = start_data.get_pdr_data_size();
    if ( ( int )mag_rows <= least_point )
        throw std::invalid_argument( "Input data length must be greater than " + std::to_string( least_point ) );

    const int    k_rows    = m_config.default_east_point;
    const int    k_cols    = 3;
    Eigen::Index data_rows = mag_rows;
    MatrixXd     mag( data_rows, k_cols );
    MatrixXd     grv( data_rows, k_cols );
    butterworth_filter( start_data, mag, grv );

    // 计算前m_config.default_east_point行东向量
    int             number_of_point = std::min( k_rows, ( int )data_rows );
    Eigen::MatrixXd e               = calc_east_vector( mag, grv, number_of_point );

    // 东向量平均值作为初始东向量
    Vector3d no_opt_e0 = e.colwise().mean();

    // 计算前least_point个点平均方向作为计算初始direction
    // 计算与北方向的角度（0°=北，90°=东），角度规范化到 [0, 360) 范围
    const int       sample_count        = std::min( least_point, ( int )( data_rows - 1 ) );  // 取前least_point段位移
    const VectorXd& magnetometer_data_x = start_data.get_pdr_data( PDR_DATA_FIELD_MAG_X );
    const VectorXd& magnetometer_data_y = start_data.get_pdr_data( PDR_DATA_FIELD_MAG_Y );

    Eigen::Vector2d avg_delta( 0, 0 );

    for ( int i = 0; i < sample_count; ++i )
    {
        avg_delta.x() += magnetometer_data_x[ i + 1 ] - magnetometer_data_x[ i ];
        avg_delta.y() += magnetometer_data_y[ i + 1 ] - magnetometer_data_y[ i ];
    }
    avg_delta /= sample_count;  // 平均位移向量

    double heading_rad = std::atan2( avg_delta.x(), avg_delta.y() );
    double heading_deg = heading_rad * 180.0 / M_PI;
    if ( heading_deg < 0 )
        heading_deg += 360.0;
    double no_opt_direction0 = heading_deg;

    return { no_opt_e0.x(), no_opt_e0.y(), no_opt_e0.z(), no_opt_direction0 };
}

Eigen::VectorXd CFmDirectionPredictor::predict_direction( const StartInfo& start_info, const CFmDataManager& process_data )
{
    // 必须有两个及以上点才能计算方向
    const size_t mag_rows = process_data.get_pdr_data_size();
    if ( mag_rows <= 0 )
        throw std::invalid_argument( "Input data length must be greater than 0" );

    const int    k_cols = 3;
    Eigen::Index rows   = mag_rows;
    MatrixXd     mag( rows, k_cols );
    MatrixXd     grv( rows, k_cols );
    butterworth_filter( process_data, mag, grv );  // TODO: 第一次送过来的数据进行了2次巴特沃斯滤波

    // 计算所有行东向量
    Eigen::MatrixXd e = calc_east_vector( mag, grv, rows );

    // 求出所有东向量和初始东向量的角度
    Vector3d no_opt_e0( start_info.e0_x, start_info.e0_y, start_info.e0_z );
    double   norm_e0 = no_opt_e0.norm();
    VectorXd no_opt_angles( rows );
    VectorXd no_opt_signs( rows );

    for ( int i = 0; i < rows; ++i )
    {
        // 显式创建固定大小向量
        Vector3d current_e( e.row( i )[ 0 ], e.row( i )[ 1 ], e.row( i )[ 2 ] );
        Vector3d current_g( grv.row( i )[ 0 ], grv.row( i )[ 1 ], grv.row( i )[ 2 ] );

        // 角度计算
        double dot_val     = current_e.dot( no_opt_e0 );
        double norm_ei     = current_e.norm();
        double cos_angle   = dot_val / ( norm_ei * norm_e0 );
        cos_angle          = std::max( -1.0, std::min( 1.0, cos_angle ) );
        no_opt_angles[ i ] = std::acos( cos_angle ) * 180.0 / M_PI;

        // 叉积和点积计算
        Vector3d cross_vec = current_e.cross( no_opt_e0 );
        double   dot_cg    = cross_vec.dot( current_g );

        no_opt_signs[ i ] = ( ( dot_cg > 0 ) ? -1.0 : ( dot_cg < 0 ) ? 1.0 : 0.0 );

        // cout << "no_opt_angles: " << no_opt_angles[ i ] << ", no_opt_signs: " << no_opt_signs[ i ] << endl;
    }

    // 计算预测方向并取模
    VectorXd no_opt_direction_pred = no_opt_signs.cwiseProduct( no_opt_angles ).array() + start_info.direction0;

    // 取模360并处理负值
    no_opt_direction_pred = no_opt_direction_pred.unaryExpr(
        []( double x )
        {
            x = std::fmod( x, 360.0 );
            return ( x < 0 ) ? x + 360.0 : x;
        } );

    return no_opt_direction_pred;
}