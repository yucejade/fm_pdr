#pragma once
#include "data_manager.h"
#include <rapidcsv.h>

using namespace rapidcsv;

class CFmDataFileLoader : public CFmDataManager
{
public:
    CFmDataFileLoader();
    CFmDataFileLoader( const PDRConfig& config, size_t train_data_size, const string& file_path );
    ~CFmDataFileLoader();

    friend CFmDataFileLoader *slice( const CFmDataFileLoader& data_manager, size_t start, size_t end );
private:
    string m_file_path;

    Document m_doc_accelerometer;
    Document m_doc_linear_accelererometer;
    Document m_doc_gyroscope;
    Document m_doc_magnetometer;
    Document m_doc_location;
private:
    Document        load_csv( const string& filename );
    Eigen::MatrixXd extract_eigen_matrix( Document& data, int start_col, int end_col, long num_rows );

    void load_data_from_file( const string& file_path );
    void preprocess_data( bool is_save );
    void generate_data();
};