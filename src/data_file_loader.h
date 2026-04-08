#pragma once
#include "data_manager.h"
#include <rapidcsv.h>


class CFmDataFileLoader : public CFmDataManager
{
public:
    CFmDataFileLoader();
    CFmDataFileLoader( const PDRConfig& config, size_t train_data_size, const std::string& file_path );
    ~CFmDataFileLoader();

    friend CFmDataFileLoader *slice( const CFmDataFileLoader& data_manager, size_t start, size_t end );
private:
    std::string m_file_path;

    rapidcsv::Document m_doc_accelerometer;
    rapidcsv::Document m_doc_linear_accelererometer;
    rapidcsv::Document m_doc_gyroscope;
    rapidcsv::Document m_doc_magnetometer;
    rapidcsv::Document m_doc_location;
private:
    rapidcsv::Document load_csv( const std::string& filename );
    Eigen::MatrixXd extract_eigen_matrix( rapidcsv::Document& data, int start_col, int end_col, long num_rows );

    void load_data_from_file( const std::string& file_path );
    void preprocess_data( bool is_save );
    void generate_data();
};