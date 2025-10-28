#include "fm_pdr.h"
#include "rapidcsv.h"
#include "data_buffer_loader.h"
#include "data_file_loader.h"
#include "json_operator.h"
#include "pdr.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <Eigen/src/Core/Matrix.h>

int fm_pdr_init( char* config_path, char* train_file_path, PDRConfig* config, PDRTrajectory** trajectories )
{
    Eigen::MatrixXd    train_position;
    *config = CFmJSONOperator::readPDRConfigFromJson( config_path );
    CFmDataFileLoader  data( *config, 0, train_file_path );
    CFmPDR             pdr( *config, data, train_position );
}

void fm_pdr_set_start_point( PDRPoint start_point ) {}

int fm_pdr_predict( PDRSensorData sensor_data, PDRTrajectory** trajectories )
{
    return 0;
}

void fm_pdr_free_trajectory() {}

void fm_pdr_uninit() {}

// 辅助函数：将指针数据转换为vector
static std::vector< double > ptr_to_vector( double* ptr, unsigned long len )
{
    if ( ! ptr || len <= 0 )
        return {};
    return std::vector< double >( ptr, ptr + len );
}

// 辅助函数：验证传感器数据有效性
static bool validate_sensor_data( double* time, double* x, double* y, double* z, unsigned long len )
{
    return ( len > 0 ) && time && x && y && z;
}

// 追加数据到CSV文件的函数
static int append_to_csv( const std::string& full_path, const std::vector< std::pair< std::string, std::vector< double > > >& columns )
{

    // 检查数据有效性
    for ( const auto& col : columns )
    {
        if ( col.second.empty() )
        {
            std::cerr << "警告: 空数据列: " << col.first << std::endl;
            return -1;
        }
    }

    // 检查所有列数据长度是否一致
    size_t expected_size = columns[ 0 ].second.size();
    for ( const auto& col : columns )
    {
        if ( col.second.size() != expected_size )
        {
            std::cerr << "错误: 数据列长度不一致" << std::endl;
            return -1;
        }
    }

    // 检查文件是否存在
    std::ifstream file_test( full_path );
    bool          file_exists = file_test.is_open();
    file_test.close();

    if ( file_exists )
    {
        // 文件已存在，追加模式
        std::ofstream outfile;
        outfile.open( full_path, std::ios_base::app );

        if ( ! outfile.is_open() )
        {
            std::cerr << "错误: 无法以追加模式打开文件: " << full_path << std::endl;
            return -1;
        }

        // 获取数据行数
        size_t row_count = columns[ 0 ].second.size();

        // 追加数据
        for ( size_t i = 0; i < row_count; ++i )
        {
            for ( size_t j = 0; j < columns.size(); ++j )
            {
                outfile << columns[ j ].second[ i ];
                if ( j < columns.size() - 1 )
                    outfile << ",";
            }
            outfile << "\n";
        }

        outfile.close();
    }
    else
    {
        // 文件不存在，创建新文件
        std::ofstream outfile( full_path );
        if ( ! outfile.is_open() )
        {
            std::cerr << "错误: 无法创建文件: " << full_path << std::endl;
            return -1;
        }

        // 写入列头
        for ( size_t i = 0; i < columns.size(); ++i )
        {
            outfile << columns[ i ].first;
            if ( i < columns.size() - 1 )
                outfile << ",";
        }
        outfile << "\n";

        // 写入数据
        size_t row_count = columns[ 0 ].second.size();
        for ( size_t i = 0; i < row_count; ++i )
        {
            for ( size_t j = 0; j < columns.size(); ++j )
            {
                outfile << columns[ j ].second[ i ];
                if ( j < columns.size() - 1 )
                    outfile << ",";
            }
            outfile << "\n";
        }

        outfile.close();
    }

    return 0;
}

// 保存传感器数据函数
int fm_pdr_save_sensor_data( char* dir_path, PDRSensorData* sensor_data )
{
    if ( ! dir_path || ! sensor_data )
    {
        std::cerr << "错误: 目录路径或传感器数据为空" << std::endl;
        return -1;
    }

    try
    {
        std::string dir_path_name( dir_path );

        // 创建目录（如果不存在）
        int ret = mkdir( dir_path_name.c_str(), 0755 );
        if ( ret != 0 && errno != EEXIST )
        {
            std::cerr << "无法创建目录: " << dir_path_name << ", 错误: " << strerror( errno ) << std::endl;
            return -2;
        }

        // 处理加速度计数据
        if ( validate_sensor_data( sensor_data->acc_time, sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > acc_columns = { { "Time (s)", ptr_to_vector( sensor_data->acc_time, sensor_data->length ) },
                                                                                           { "X (m/s^2)", ptr_to_vector( sensor_data->acc_x, sensor_data->length ) },
                                                                                           { "Y (m/s^2)", ptr_to_vector( sensor_data->acc_y, sensor_data->length ) },
                                                                                           { "Z (m/s^2)", ptr_to_vector( sensor_data->acc_z, sensor_data->length ) } };

            std::string acc_path = dir_path_name + "/Accelerometer.csv";
            if ( append_to_csv( acc_path, acc_columns ) != 0 )
            {
                std::cerr << "警告: 加速度计数据保存失败" << std::endl;
            }
        }

        // // 处理线性加速度计数据
        // if ( validate_sensor_data( sensor_data->lacc_time, sensor_data->lacc_x, sensor_data->lacc_y, sensor_data->lacc_z, sensor_data->length ) )
        // {
        //     std::vector< std::pair< std::string, std::vector< double > > > lacc_columns = { { "Time (s)", ptr_to_vector( sensor_data->lacc_time, sensor_data->length ) },
        //                                                                                     { "X (m/s^2)", ptr_to_vector( sensor_data->lacc_x, sensor_data->length ) },
        //                                                                                     { "Y (m/s^2)", ptr_to_vector( sensor_data->lacc_y, sensor_data->length ) },
        //                                                                                     { "Z (m/s^2)", ptr_to_vector( sensor_data->lacc_z, sensor_data->length ) } };

        //     std::string lacc_path = dir_path_name + "/LinearAccelerometer.csv";
        //     if ( append_to_csv( lacc_path, lacc_columns ) != 0 )
        //     {
        //         std::cerr << "警告: 线性加速度计数据保存失败" << std::endl;
        //     }
        // }

        // 处理陀螺仪数据
        if ( validate_sensor_data( sensor_data->gyr_time, sensor_data->gyr_x, sensor_data->gyr_y, sensor_data->gyr_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > gyr_columns = { { "Time (s)", ptr_to_vector( sensor_data->gyr_time, sensor_data->length ) },
                                                                                           { "X (rad/s)", ptr_to_vector( sensor_data->gyr_x, sensor_data->length ) },
                                                                                           { "Y (rad/s)", ptr_to_vector( sensor_data->gyr_y, sensor_data->length ) },
                                                                                           { "Z (rad/s)", ptr_to_vector( sensor_data->gyr_z, sensor_data->length ) } };

            std::string gyr_path = dir_path_name + "/Gyroscope.csv";
            if ( append_to_csv( gyr_path, gyr_columns ) != 0 )
            {
                std::cerr << "警告: 陀螺仪数据保存失败" << std::endl;
            }
        }

        // 处理磁力计数据
        if ( validate_sensor_data( sensor_data->mag_time, sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, sensor_data->length ) )
        {
            std::vector< std::pair< std::string, std::vector< double > > > mag_columns = { { "Time (s)", ptr_to_vector( sensor_data->mag_time, sensor_data->length ) },
                                                                                           { "X (uT)", ptr_to_vector( sensor_data->mag_x, sensor_data->length ) },
                                                                                           { "Y (uT)", ptr_to_vector( sensor_data->mag_y, sensor_data->length ) },
                                                                                           { "Z (uT)", ptr_to_vector( sensor_data->mag_z, sensor_data->length ) } };

            std::string mag_path = dir_path_name + "/Magnetometer.csv";
            if ( append_to_csv( mag_path, mag_columns ) != 0 )
            {
                std::cerr << "警告: 磁力计数据保存失败" << std::endl;
            }
        }

        return 0;
    }
    catch ( const std::exception& e )
    {
        std::cerr << "传感器数据保存失败: " << e.what() << std::endl;
        return -3;
    }
}
