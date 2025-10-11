#pragma once
#include "data_manager.h"
#include <rapidcsv.h>

using namespace rapidcsv;

class CFmDataFileLoader : public CFmDataManager
{
public:
    CFmDataFileLoader( );
    CFmDataFileLoader( const PDRConfig& config, size_t train_data_size, const string& file_path );
    ~CFmDataFileLoader();

    friend CFmDataFileLoader slice( const CFmDataFileLoader& data_manager, size_t start, size_t end );
private:
    string m_file_path;

    Document m_doc_accelerometer;
    Document m_doc_linear_accelererometer;
    Document m_doc_gyroscope;
    Document m_doc_magnetometer;
    Document m_doc_location;

    MatrixXd               m_a;
    MatrixXd               m_la;
    MatrixXd               m_gs;
    MatrixXd               m_m;
    MatrixXd               m_g;
    MatrixXd               m_location;       // 没有训练数据时，为空
    MatrixXd               m_location_true;  // 没有真实定位数据时，为空
private:
    Document        load_csv( const string& filename );
    Eigen::MatrixXd extract_eigen_matrix( Document& data, int start_col, int end_col, long num_rows );
    MatrixXd        nearest_neighbor_interpolation( const VectorXd& time_query, const VectorXd& time_data, const MatrixXd& data ) const;
    VectorXd        magnitude( const MatrixXd& matrix );

    void load_data_from_file( const string& file_path );
    void preprocess_data( bool is_save );
    void generate_data();
};