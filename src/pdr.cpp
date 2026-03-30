#include "pdr.h"
#include <Eigen/Dense>
#include <fstream>
#include <rapidcsv.h>
#include <string>
#include <iomanip>

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

/**
 * 根据原有时间戳列生成新的采样时间戳序列
 * @param original_times 原始trajectory的时间戳列 (VectorXd)
 * @return 重新采样后的时间戳序列 (VectorXd)
 */
VectorXd CFmPDR::generate_target_times(const VectorXd& original_times)
{
    // 如果为空或只有一个点，直接返回
    if (original_times.size() == 0) return VectorXd();
    if (original_times.size() == 1) return original_times;

    double t_start = original_times(0);
    double t_end = original_times(original_times.size() - 1);
    double duration = t_end - t_start;

    // 如果时间跨度为0（防止除0或死循环），直接返回
    if (duration <= 1e-6) return original_times;

    std::vector<double> new_times;
    new_times.reserve(100); // 预分配内存

    // a. trajectory的时间戳（即总时长）如果小于或等于 5 * 100 秒 (500秒)
    if (duration <= 500.0) 
    {
        new_times.push_back(t_start); // c. 保留第一个时间戳
        double current_t = t_start + 5.0;
        
        // 每5秒转换一个，最多保留100个点。
        // 为了给最后一个时间戳(t_end)预留1个位置，循环添加的点数不得超过99个。
        // current_t < t_end - 1e-3 防止浮点精度导致加入一个极度接近 t_end 的点。
        while (current_t < t_end - 1e-3 && new_times.size() < 99) 
        {
            new_times.push_back(current_t);
            current_t += 5.0;
        }
        
        new_times.push_back(t_end); // c. 保留最后一个时间戳
    }
    // b. trajectory的时间戳如果大于 5 * 100 秒
    else 
    {
        int num_points = 100; // 固定转换100个时间戳
        double step = duration / (num_points - 1); // 计算步长（99段间隔）
        
        for (int i = 0; i < num_points - 1; ++i) 
        {
            new_times.push_back(t_start + i * step); // c. 第一个点自然是t_start
        }
        // c. 强制把最后一个点设为t_end，避免累加浮点误差带来的微小偏差
        new_times.push_back(t_end); 
    }

    // 将 std::vector 映射/转换为 Eigen::VectorXd 返回
    return Map<VectorXd>(new_times.data(), new_times.size());
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

MatrixXd CFmPDR::process_trajectory(const MatrixXd& original_trajectory)
{
    // 确保轨迹有效且包含时间列
    if (original_trajectory.rows() == 0 || original_trajectory.cols() < 4) {
        return MatrixXd(); 
    }

    // 1. 只传入 trajectory 的时间戳列作为参数（提取第 0 列）
    const VectorXd &original_times = original_trajectory.col(0);

    // 2. 获取经过新逻辑处理过的时间戳序列
    VectorXd target_times = generate_target_times(original_times);

    // 3. 调用你原有的 linear_interpolation 函数，获取新时间序列上的插值结果
    return linear_interpolation(target_times, original_trajectory);

}

bool CFmPDR::appendEigenMatrixToCsv(const Eigen::MatrixXd& matrix, const std::string& filename, 
                            const std::vector<std::string>& headers) {
    // 修复：有符号/无符号类型比较
    if (matrix.rows() == 0 || (headers.size() > 0 && headers.size() != static_cast<size_t>(matrix.cols()))) {
        std::cerr << "[Error] 矩阵为空或表头列数不匹配！" << std::endl;
        return false;
    }

    try {
        bool fileExists = std::ifstream(filename).good();
        rapidcsv::Document doc;

        if (fileExists) {
            doc.Load(filename);
        }

        if (!fileExists && !headers.empty()) {
            for (size_t i = 0; i < headers.size(); ++i) {
                doc.SetColumnName(i, headers[i]);
            }
        }

        Eigen::Index startRow = doc.GetRowCount();
        for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
            std::vector<std::string> rowData;
            rowData.reserve(matrix.cols());
            for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
                // 核心修改：用 stringstream 控制小数位数（保留8位）
                std::stringstream ss;
                ss << std::fixed << std::setprecision(8) << matrix(row, col);
                rowData.emplace_back(ss.str());
            }
            doc.InsertRow(startRow + row, rowData);
        }

        doc.Save(filename);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Error] 追加失败: " << e.what() << std::endl;
        return false;
    }
}

MatrixXd CFmPDR::pdr( StartInfo& start_info, const CFmDataManager& process_data )
{
    return m_merge_direction_step.merge_dir_step( start_info, process_data );
    // Eigen::MatrixXd trajectory = m_merge_direction_step.merge_dir_step( start_info, process_data );
    // if ( 0 == trajectory.rows() )
    //     return Eigen::MatrixXd();

    //std::vector< std::string > customHeaders = { "timestamp", "pos_x", "pos_y", "angle" };
    //appendEigenMatrixToCsv( trajectory, "./t1.csv", customHeaders );
    // for ( Eigen::Index i = 0; i < trajectory.rows(); i++ )
    //     cout << "time:" << trajectory( i, 0 ) << ", x:" << trajectory( i, 1 ) << ", y:" << trajectory( i, 2 ) << ", direction:" << trajectory( i, 3 ) << endl;

    // MatrixXd t;

    // if ( process_data.have_location_true() )
    // {
    //     size_t          true_data_size = process_data.get_true_data_size();
    //     const VectorXd& true_data_time = process_data.get_true_data( TRUE_DATA_FIELD_TIME );
    //     Eigen::VectorXd time_location  = Eigen::Map< const Eigen::VectorXd >( true_data_time.data(), true_data_size );
    //     t                              = linear_interpolation( time_location, trajectory );
    // }
    // else
    // {
    //     size_t          data_size     = process_data.get_pdr_data_size();
    //     const VectorXd& data_time     = process_data.get_pdr_data( PDR_DATA_FIELD_TIME );
    //     Eigen::VectorXd time_location = Eigen::Map< const Eigen::VectorXd >( data_time.data(), data_size );
    //     t                             = linear_interpolation( time_location, trajectory );
    // }

    // t = CFmPDR::process_trajectory(trajectory)
    // //appendEigenMatrixToCsv( t, "./t2.csv", customHeaders );
    // // cout << "==========================================================================================" << endl;
    // // for ( Eigen::Index i = 0; i < t.rows(); i++ )
    // //     cout << "time:" << t( i, 0 ) << ", x:" << t( i, 1 ) << ", y:" << t( i, 2 ) << ", direction:" << t( i, 3 ) << endl;

    // constexpr double kK = 111319.49079;  // 地球半径 * π / 180，单位：米/度

    // t.col( 1 ) = t.col( 1 ).array() / kK + start_info.x0;  // 第1列（x）整体缩放+偏移
    // t.col( 2 ) = t.col( 2 ).array() / kK + start_info.y0;  // 第2列（y）整体缩放+偏移

    // //appendEigenMatrixToCsv( t, "./t3.csv", customHeaders );
    // return t;
}