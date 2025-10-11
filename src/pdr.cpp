#include "pdr.h"

// bool compare_time( double t_val, const PDRPosition& pos );

CFmPDR::CFmPDR( const PDRConfig& config, const CFmDataManager& train_data, Eigen::MatrixXd& train_position ) : m_merge_direction_step( config, train_data, train_position ) {}

CFmPDR::CFmPDR( const PDRConfig& config ) : m_merge_direction_step( config ) {}

CFmPDR::~CFmPDR() {}

StartInfo CFmPDR::start( double x0, double y0, const CFmDataManager& start_data )
{
    StartInfo si;

    si    = m_merge_direction_step.start( start_data );
    si.x0 = x0;
    si.y0 = y0;

    return si;
}

size_t CFmPDR::find_interval( double t, const MatrixXd& trajectory ) const
{
    const size_t n = trajectory.rows();

    if ( n < 2 )
        return 0;

    size_t low  = 0;
    size_t high = n - 2;  // 最大有效索引是 n-2

    while ( low <= high )
    {
        size_t mid    = low + ( high - low ) / 2;
        double t_mid  = trajectory( mid, 0 );
        double t_next = trajectory( mid + 1, 0 );

        if ( t >= t_mid && t < t_next )
        {
            return mid;
        }
        else if ( t < t_mid )
        {
            high = mid - 1;
        }
        else
        {
            low = mid + 1;
        }
    }

    // 如果没找到，返回最后一个区间
    return n - 2;
}

// 核心插值函数（返回Eigen矩阵）
MatrixXd CFmPDR::linear_interpolation( const VectorXd& target_times, const MatrixXd& trajectory )
{
    // 0. 边界处理
    const size_t traj_rows   = trajectory.rows();
    const size_t num_targets = target_times.size();
    if ( traj_rows == 0 || num_targets == 0 )
        return MatrixXd();

    // 1. 检查列数
    if ( trajectory.cols() < 4 )
        throw std::invalid_argument( "Trajectory matrix must have 4 columns (time, x, y, direction)." );

    // 2. 准备结果矩阵
    MatrixXd result( num_targets, 4 );

    // 3. 单点轨迹处理
    if ( traj_rows == 1 )
    {
        result.col( 0 ) = target_times;
        result.col( 1 ).fill( trajectory( 0, 1 ) );
        result.col( 2 ).fill( trajectory( 0, 2 ) );
        result.col( 3 ).fill( trajectory( 0, 3 ) );
        return result;
    }

    // 4. 获取时间范围
    const double first_time = trajectory( 0, 0 );
    const double last_time  = trajectory( traj_rows - 1, 0 );

    for ( size_t i = 0; i < num_targets; ++i )
    {
        const double t = target_times( i );
        RowVector4d  interp_row;
        interp_row( 0 ) = t;

        // 5. 统一使用线性插值（包括边界情况）
        size_t idx = ( t <= first_time ) ? 0 : ( t >= last_time ) ? traj_rows - 2 : find_interval( t, trajectory );

        const RowVector4d& p0 = trajectory.row( idx );
        const RowVector4d& p1 = trajectory.row( idx + 1 );

        // 6. 计算精确的插值比例
        double time_diff = p1( 0 ) - p0( 0 );
        double ratio     = ( time_diff > 1e-10 ) ? ( t - p0( 0 ) ) / time_diff : 0.0;

        // 7. 线性插值x和y
        interp_row( 1 ) = p0( 1 ) + ratio * ( p1( 1 ) - p0( 1 ) );
        interp_row( 2 ) = p0( 2 ) + ratio * ( p1( 2 ) - p0( 2 ) );

        // 8. 严格的角度插值（确保总是执行）
        double v0   = p0( 3 );
        double v1   = p1( 3 );
        double diff = v1 - v0;

        // 处理角度环绕
        if ( diff > 180.0 )
            diff -= 360.0;
        else if ( diff < -180.0 )
            diff += 360.0;

        double interpolated_dir = v0 + diff * ratio;

        // 标准化到[0,360)
        interpolated_dir = fmod( interpolated_dir, 360.0 );
        if ( interpolated_dir < 0.0 )
            interpolated_dir += 360.0;

        interp_row( 3 ) = interpolated_dir;

        result.row( i ) = interp_row;
    }

    return result;
}

MatrixXd CFmPDR::pdr( StartInfo& start_info, const CFmDataManager& process_data )
{
    Eigen::MatrixXd trajectory = m_merge_direction_step.merge_dir_step( start_info, process_data );
    MatrixXd        t;

    // for ( Eigen::Index i = 0; i < trajectory.rows(); i++ )
    //     cout << "time:" << trajectory( i, 0 ) << ", x:" << trajectory( i, 1 ) << ", y:" << trajectory( i, 2 ) << ", direction:" << trajectory( i, 3 ) << endl;

    if ( process_data.have_location_true() )
    {
        size_t          true_data_size = process_data.get_true_data_size();
        const VectorXd& true_data_time = process_data.get_true_data( TRUE_DATA_FIELD_TIME );
        Eigen::VectorXd time_location  = Eigen::Map< const Eigen::VectorXd >( true_data_time.data(), true_data_size );
        t                              = linear_interpolation( time_location, trajectory );
    }
    else
    {
        size_t          data_size     = process_data.get_pdr_data_size();
        const VectorXd& data_time     = process_data.get_pdr_data( PDR_DATA_FIELD_TIME );
        Eigen::VectorXd time_location = Eigen::Map< const Eigen::VectorXd >( data_time.data(), data_size );
        t                             = linear_interpolation( time_location, trajectory );
    }

    // cout << "==========================================================================================" << endl;
    // for ( Eigen::Index i = 0; i < t.rows(); i++ )
    //     cout << "time:" << t( i, 0 ) << ", x:" << t( i, 1 ) << ", y:" << t( i, 2 ) << ", direction:" << t( i, 3 ) << endl;

    constexpr double kK = 1e5;

    t.col( 1 ) = t.col( 1 ).array() / kK + start_info.x0;  // 第1列（x）整体缩放+偏移
    t.col( 2 ) = t.col( 2 ).array() / kK + start_info.y0;  // 第2列（y）整体缩放+偏移

    return t;
}