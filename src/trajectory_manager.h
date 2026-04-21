#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

class TrajectoryManager
{
public:
    // cols: 矩阵列数
    // initialRows: 初始开辟的行数（预留空间）
    TrajectoryManager( int cols, int initialRows = 500 ) : m_cols( cols ), m_usedRows( 0 ), m_capacityRows( initialRows )
    {
        m_data = std::make_unique< Eigen::MatrixXd >( m_capacityRows, m_cols );
    }

    // 追加新数据（支持任意行数）
    void append( const Eigen::MatrixXd& newData )
    {
        if ( newData.cols() != m_cols )
        {
            throw std::runtime_error( "Column count mismatch!" );
        }

        int incomingRows = newData.rows();

        // 检查剩余空间是否足够
        if ( m_usedRows + incomingRows > m_capacityRows )
        {
            // 模仿 vector 扩容策略：至少翻倍，且确保能放下新数据
            int minRequired = m_usedRows + incomingRows;
            m_capacityRows  = std::max( m_capacityRows * 2, minRequired );

            m_data->conservativeResize( m_capacityRows, Eigen::NoChange );
        }

        // 将新数据拷贝到当前可用位置的起始处
        m_data->block( m_usedRows, 0, incomingRows, m_cols ) = newData;
        m_usedRows += incomingRows;
    }

    // 获取当前有效数据的视图（零拷贝）
    auto view() const
    {
        return m_data->topRows( m_usedRows );
    }

    // 重置轨迹（不释放内存，仅重置计数器，方便复用空间）
    void clear()
    {
        m_usedRows = 0;
    }

    int rows() const
    {
        return m_usedRows;
    }
    int cols() const
    {
        return m_cols;
    }

    Eigen::MatrixXd process_trajectory( int match_duration )
    {
        // 确保轨迹有效且包含时间列
        if ( m_usedRows == 0 || m_data->cols() < 4 )
        {
            return Eigen::MatrixXd();
        }

        // 只使用有效行数据，排除预分配但未写入的垃圾行
        Eigen::MatrixXd valid_data = m_data->topRows( m_usedRows );

        // 1. 只传入 trajectory 的时间戳列作为参数（提取第 0 列）
        Eigen::VectorXd original_times = valid_data.col( 0 );

        // 2. 获取经过新逻辑处理过的时间戳序列
        Eigen::VectorXd target_times = generate_target_times( original_times, match_duration );

        // 3. 调用 linear_interpolation 函数，获取新时间序列上的插值结果
        return linear_interpolation( target_times, valid_data );
    }
private:
    size_t find_interval( double t, const Eigen::MatrixXd& trajectory ) const
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
    /**
     * 根据原有时间戳列生成新的采样时间戳序列
     * @param original_times 原始trajectory的时间戳列 (VectorXd)
     * @return 重新采样后的时间戳序列 (VectorXd)
     */
    Eigen::VectorXd generate_target_times( const Eigen::VectorXd& original_times, int match_duration )
    {
        if ( original_times.size() == 0 )
            return Eigen::VectorXd();
        if ( original_times.size() == 1 )
            return original_times;
        if ( match_duration <= 0 )
            return original_times;

        std::vector< double > new_times;
        int                   n = original_times.size();

        new_times.push_back( original_times( 0 ) );

        int pos = 0;
        while ( pos < n )
        {
            double        threshold = original_times( pos ) + match_duration;
            const double* begin     = original_times.data() + pos + 1;
            const double* end       = original_times.data() + n;
            const double* found     = std::upper_bound( begin, end, threshold );
            if ( found == end )
                break;

            pos = static_cast< int >( found - original_times.data() );
            new_times.push_back( original_times( pos ) );
        }

        if ( new_times.back() != original_times( n - 1 ) )
            new_times.push_back( original_times( n - 1 ) );

        return Eigen::Map< Eigen::VectorXd >( new_times.data(), new_times.size() );
    }

    // 核心插值函数（返回Eigen矩阵）
    Eigen::MatrixXd linear_interpolation( const Eigen::VectorXd& target_times, const Eigen::MatrixXd& trajectory )
    {
        // 0. 边界处理
        const size_t traj_rows   = trajectory.rows();
        const size_t num_targets = target_times.size();
        if ( traj_rows == 0 || num_targets == 0 )
            return Eigen::MatrixXd();

        // 1. 检查列数
        if ( trajectory.cols() < 4 )
            throw std::invalid_argument( "Trajectory matrix must have 4 columns (time, x, y, direction)." );

        // 2. 准备结果矩阵
        Eigen::MatrixXd result( num_targets, 4 );

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
private:
    std::unique_ptr< Eigen::MatrixXd > m_data;
    int                                m_cols;
    int                                m_usedRows;      // 当前已填入的总行数
    int                                m_capacityRows;  // 当前内存块的总行数
};
