#pragma once
#include "data_manager.h"

class CFmDataBufferLoader : public CFmDataManager
{
public:
    CFmDataBufferLoader( );
    CFmDataBufferLoader( const PDRConfig& config, size_t train_data_size, const PDRData& data );
    ~CFmDataBufferLoader();

    friend CFmDataBufferLoader *slice( const CFmDataBufferLoader& buffer_loader, size_t start, size_t end );
private:

private:
    void preprocess_data( const PDRData& data, bool is_save );
    void generate_data();
    
    double* get_sensor_field_ptr( PDRSensorData* data, int col );
    double* get_true_field_ptr( PDRTrueData* data, int col );
    Eigen::MatrixXd extract_eigen_matrix( void* pointer, int type, int start_col, int end_col, unsigned long num_rows );
};