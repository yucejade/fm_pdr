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