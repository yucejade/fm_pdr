#include "data_buffer_loader.h"
#include "data_file_loader.h"
#include "data_manager.h"
#include "fm_device_wrapper.h"
#include "fm_pdr.h"
#include "json_operator.h"
#include "pdr.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = filesystem;

// 简单的命令行参数解析类
class ArgParser
{
public:
    ArgParser( int argc, char* argv[] )
    {
        for ( int i = 1; i < argc; ++i )
        {
            tokens_.push_back( std::string( argv[ i ] ) );
        }
    }

    // 获取选项值
    std::string getOption( const std::string& option ) const
    {
        auto it = std::find( tokens_.begin(), tokens_.end(), option );
        if ( it != tokens_.end() && ++it != tokens_.end() )
        {
            return *it;
        }
        return "";
    }

    // 检查选项是否存在
    bool hasOption( const std::string& option ) const
    {
        return std::find( tokens_.begin(), tokens_.end(), option ) != tokens_.end();
    }

    // 显示帮助信息
    void showHelp() const
    {
        std::cout << "Usage: pdr [options]\n"
                  << "Options:\n"
                  << "  -f, --file\t\t\t\t使用读数据文件方式进行行人航线推算，默认使用此模式；如果该选项为false，则直接使用传感器数据\n"
                  << "  -e, --evaluation\t\t\t是否打印指标评估\n"
                  << "  -c, --config <配置文件路径>\t\t指定PDR配置文件路径，默认使用../conf/config.json\n"
                  << "  -t, --train <样本数据路径>\t\t在--file选项生效情况下，表示需要加载的<样本数据路径>；如果--file选项为false，则表示传感器数据的保存路径，训练模型输出到model_file_name配置项设置的路径下\n"
                  << "  -d, --dataset <PDR数据路径>\t\t在--file选项生效情况下，表示需要加载的<PDR测试数据路径>；如果--file选项为false，则表示传感器数据保存路径，使用model_file_name配置项设置路径下的模型文件进行推算\n"
                  << "  -h, --help\t\t\t\t帮助信息\n";
    }
private:
    std::vector< std::string > tokens_;
};

std::vector< double > g_acc_time;
std::vector< double > g_acc_x;
std::vector< double > g_acc_y;
std::vector< double > g_acc_z;
std::vector< double > g_lacc_time;
std::vector< double > g_lacc_x;
std::vector< double > g_lacc_y;
std::vector< double > g_lacc_z;
std::vector< double > g_gyr_time;
std::vector< double > g_gyr_x;
std::vector< double > g_gyr_y;
std::vector< double > g_gyr_z;
std::vector< double > g_mag_time;
std::vector< double > g_mag_x;
std::vector< double > g_mag_y;
std::vector< double > g_mag_z;

std::vector< double > g_time_location;
std::vector< double > g_latitude;
std::vector< double > g_longitude;
std::vector< double > g_height;
std::vector< double > g_velocity;
std::vector< double > g_direction;
std::vector< double > g_horizontal_accuracy;
std::vector< double > g_vertical_accuracy;

PDRData LoadPDRData( const string& file_path )
{
    PDRData data_buffer;
    string  full_name;

    // 读取加速度计数据
    full_name = file_path + "/" + "Accelerometer.csv";
    if ( ! fs::exists( full_name ) )
        throw runtime_error( "File not found: " + full_name );
    Document accelerometer = Document( full_name, LabelParams( 0, -1 ) );
    g_acc_time             = accelerometer.GetColumn< double >( 0 );
    g_acc_x                = accelerometer.GetColumn< double >( 1 );
    g_acc_y                = accelerometer.GetColumn< double >( 2 );
    g_acc_z                = accelerometer.GetColumn< double >( 3 );

    // 读取陀螺仪数据
    full_name = file_path + "/" + "Gyroscope.csv";
    if ( ! fs::exists( full_name ) )
        throw runtime_error( "File not found: " + full_name );
    Document gyroscope = Document( full_name, LabelParams( 0, -1 ) );
    g_gyr_time         = gyroscope.GetColumn< double >( 0 );
    g_gyr_x            = gyroscope.GetColumn< double >( 1 );
    g_gyr_y            = gyroscope.GetColumn< double >( 2 );
    g_gyr_z            = gyroscope.GetColumn< double >( 3 );

    // 读取磁力计数据
    full_name = file_path + "/" + "Magnetometer.csv";
    if ( ! fs::exists( full_name ) )
        throw runtime_error( "File not found: " + full_name );
    Document magnetometer = Document( full_name, LabelParams( 0, -1 ) );
    g_mag_time            = magnetometer.GetColumn< double >( 0 );
    g_mag_x               = magnetometer.GetColumn< double >( 1 );
    g_mag_y               = magnetometer.GetColumn< double >( 2 );
    g_mag_z               = magnetometer.GetColumn< double >( 3 );

    // 读取线性加速度计数据
    full_name = file_path + "/" + "Linear Accelerometer.csv";
    if ( fs::exists( full_name ) )
    {
        Document linear_accelerometer = Document( full_name, LabelParams( 0, -1 ) );
        g_lacc_time                   = linear_accelerometer.GetColumn< double >( 0 );
        g_lacc_x                      = linear_accelerometer.GetColumn< double >( 1 );
        g_lacc_y                      = linear_accelerometer.GetColumn< double >( 2 );
        g_lacc_z                      = linear_accelerometer.GetColumn< double >( 3 );
    }

    data_buffer.sensor_data.acc_time  = g_acc_time.data();
    data_buffer.sensor_data.acc_x     = g_acc_x.data();
    data_buffer.sensor_data.acc_y     = g_acc_y.data();
    data_buffer.sensor_data.acc_z     = g_acc_z.data();
    data_buffer.sensor_data.lacc_time = g_lacc_time.empty() ? nullptr : g_lacc_time.data();
    data_buffer.sensor_data.lacc_x    = g_lacc_x.empty() ? nullptr : g_lacc_x.data();
    data_buffer.sensor_data.lacc_y    = g_lacc_y.empty() ? nullptr : g_lacc_y.data();
    data_buffer.sensor_data.lacc_z    = g_lacc_z.empty() ? nullptr : g_lacc_z.data();
    data_buffer.sensor_data.gyr_time  = g_gyr_time.data();
    data_buffer.sensor_data.gyr_x     = g_gyr_x.data();
    data_buffer.sensor_data.gyr_y     = g_gyr_y.data();
    data_buffer.sensor_data.gyr_z     = g_gyr_z.data();
    data_buffer.sensor_data.mag_time  = g_mag_time.data();
    data_buffer.sensor_data.mag_x     = g_mag_x.data();
    data_buffer.sensor_data.mag_y     = g_mag_y.data();
    data_buffer.sensor_data.mag_z     = g_mag_z.data();
    data_buffer.sensor_data.length    = g_acc_time.size();

    // 检查并读取真实位置数据，如果存在真实位置数据，则可以训练和评估，否则不需要读取真实位置数据（即：只能预测）
    full_name = file_path + "/" + "Location.csv";
    if ( ! fs::exists( full_name ) )
        throw runtime_error( "File not found: " + full_name );
    Document location     = Document( full_name, LabelParams( 0, -1 ) );
    g_time_location       = location.GetColumn< double >( 0 );
    g_latitude            = location.GetColumn< double >( 1 );
    g_longitude           = location.GetColumn< double >( 2 );
    g_height              = location.GetColumn< double >( 3 );
    g_velocity            = location.GetColumn< double >( 4 );
    g_direction           = location.GetColumn< double >( 5 );
    g_horizontal_accuracy = location.GetColumn< double >( 6 );
    g_vertical_accuracy   = location.GetColumn< double >( 7 );

    data_buffer.true_data.time_location       = g_time_location.empty() ? nullptr : g_time_location.data();
    data_buffer.true_data.latitude            = g_latitude.empty() ? nullptr : g_latitude.data();
    data_buffer.true_data.longitude           = g_longitude.empty() ? nullptr : g_longitude.data();
    data_buffer.true_data.height              = g_height.empty() ? nullptr : g_height.data();
    data_buffer.true_data.velocity            = g_velocity.empty() ? nullptr : g_velocity.data();
    data_buffer.true_data.direction           = g_direction.empty() ? nullptr : g_direction.data();
    data_buffer.true_data.horizontal_accuracy = g_horizontal_accuracy.empty() ? nullptr : g_horizontal_accuracy.data();
    data_buffer.true_data.vertical_accuracy   = g_vertical_accuracy.empty() ? nullptr : g_vertical_accuracy.data();
    data_buffer.true_data.length              = g_time_location.size();

    return data_buffer;
}

void convert_sensor_data_to_pdr_data( SensorData* sensor_data, PDRData* pdr_data )
{
    memset( pdr_data, 0x00, sizeof( PDRData ) );

    pdr_data->sensor_data = sensor_data->sensor_data;
}

int main( int argc, char* argv[] )
{
    // 解析命令行参数
    ArgParser parser( argc, argv );

    // 检查帮助选项
    if ( parser.hasOption( "-h" ) || parser.hasOption( "--help" ) )
    {
        parser.showHelp();
        return 0;
    }

    // 获取数据集路径
    std::string train_dataset_path = parser.getOption( "-t" );
    if ( train_dataset_path.empty() )
        train_dataset_path = parser.getOption( "--train" );

    std::string pdr_dataset_path = parser.getOption( "-d" );
    if ( pdr_dataset_path.empty() )
        pdr_dataset_path = parser.getOption( "--dataset" );

    std::string pdr_config_path = parser.getOption( "-c" );
    if ( pdr_config_path.empty() )
        pdr_config_path = parser.getOption( "--config" );

    // 验证必要参数
    if ( train_dataset_path.empty() && pdr_dataset_path.empty() )
    {
        std::cerr << "Argument error.\n";
        parser.showHelp();
        return 1;
    }

    // 默认配置文件路径
    if ( pdr_config_path.empty() )
        pdr_config_path = "../conf/config.json";

    // 检查是否以file方式运行
    bool file = parser.hasOption( "-f" ) || parser.hasOption( "--file" );

    // 检查是否输出评估结果
    bool eval = parser.hasOption( "-e" ) || parser.hasOption( "--evaluation" );

    try
    {
        PDRConfig config = CFmJSONOperator::readPDRConfigFromJson( pdr_config_path.c_str() );

        // 创建测试用例
        if ( ! train_dataset_path.empty() )
        {
            if ( file )
            {
                constexpr size_t   test_case0_input_length = 60;
                Eigen::MatrixXd    train_position;
                CFmDataFileLoader  data( config, test_case0_input_length, train_dataset_path );
                CFmDataFileLoader* train_data = slice( data, 0, data.get_train_data_size() * config.sample_rate );
                CFmPDR             pdr( config, *train_data, train_position );
                delete train_data;
            }
            else
            {
                // TODO: 因为现有设备没有GPS模块，暂时无法进行训练数据的采集和模型训练，请使用手机生成相关数据文件进行训练
            }
        }

        if ( ! pdr_dataset_path.empty() )
        {
            if ( file )
            {
                constexpr size_t  test_case0_input_length = 60;
                CFmDataFileLoader data( config, test_case0_input_length, pdr_dataset_path );
                CFmDataManager*   pdr_data   = slice( data, data.get_train_data_size() * config.sample_rate, 0 );
                CFmDataManager*   train_data = slice( data, 0, data.get_train_data_size() * config.sample_rate );
                VectorXd          pos_x      = data.get_true_data( TRUE_DATA_FIELD_LATITUDE );
                VectorXd          pos_y      = data.get_true_data( TRUE_DATA_FIELD_LONGITUDE );
                double            x0         = pos_x[ test_case0_input_length - 1 ];
                double            y0         = pos_y[ test_case0_input_length - 1 ];
                CFmPDR            pdr( config );
                size_t            i                      = 0;
                size_t            slice_interval_seconds = 2 * config.sample_rate;
                bool              is_stop                = false;
                Eigen::MatrixXd   trajectory;

                // 执行PDR算法，这里假定手动设置的初始位置为真实定位数据中的第一个真实位置点
                // for ( size_t idx = 0; idx < data.get_true_data_size(); ++idx )
                //     std::cout << "True Data Point " << idx << ": (" << std::fixed << std::setprecision( 10 )  // 设置固定10位小数格式
                //               << pos_x[ idx ] << ", " << pos_y[ idx ] << ")\n";
                StartInfo si = pdr.start( x0, y0, *pdr_data );

                while ( true )
                {
                    size_t pdr_size = pdr_data->get_true_data_size() * config.sample_rate;
                    size_t s        = i * slice_interval_seconds;
                    size_t e        = std::min( ( i + 1 ) * slice_interval_seconds, pdr_size );

                    // 计时开始，测试PDR处理时间
                    // auto start_time = std::chrono::steady_clock::now();
                    CFmDataManager* segment = slice( dynamic_cast< CFmDataFileLoader& >( *pdr_data ), s, e );
                    Eigen::MatrixXd t       = pdr.pdr( si, *segment );
                    delete segment;

                    size_t rows = t.rows();
                    size_t cols = t.cols();
                    if ( rows == 0 )
                    {
                        if ( ! is_stop )
                            cout << "A stop event has been detected." << endl;
                        is_stop = true;
                    }
                    else
                    {
                        if ( is_stop )
                            cout << "Resuming from stop event." << endl;
                        is_stop = false;

                        size_t old_rows = trajectory.rows();
                        trajectory.conservativeResize( old_rows + rows, cols );
                        trajectory.block( old_rows, 0, rows, cols ) = t;
                    }

                    // auto   end_time = std::chrono::steady_clock::now();
                    // double time     = std::chrono::duration< double, std::micro >( end_time - start_time ).count();
                    // cout << "Segment " << i << ": " << s << " to " << e << ", Time taken: " << time << " microseconds" << endl;

                    if ( e >= pdr_size )
                        break;
                    i++;
                }

                // 拼接训练数据结果和PDR数据结果
                Eigen::MatrixXd all_trajectory;
                Eigen::MatrixXd train_position;
                CFmPDR          train_pdr( config, *train_data, train_position );
                all_trajectory.resize( train_position.rows() + trajectory.rows(), train_position.cols() );
                all_trajectory.topRows( train_position.rows() ) = train_position;
                all_trajectory.bottomRows( trajectory.rows() )  = trajectory;

                // 保存结果
                pdr_data->set_location_output( all_trajectory );

                if ( eval )
                    pdr_data->eval_model( all_trajectory );

                delete pdr_data;
                delete train_data;
            }
            else
            {
                PDRData            pdr_data;
                SensorData         sensor_data;
                fm_device_handle_t device_handle;
                int                is_first = 1;

                int ret = fm_device_init( config.sample_rate, &device_handle );
                if ( ret != 0 )
                {
                    std::cerr << "Failed to initialize device wrapper, error code: " << ret << std::endl;
                    return 1;
                }

                while ( true )
                {
                    // 采集传感器数据
                    ret = fm_device_read( device_handle, is_first, config.sample_rate * 2, 1, &sensor_data );
                    if ( ret != 0 )
                    {
                        std::cerr << "Failed to read sensor data, error code: " << ret << std::endl;
                        fm_device_uninit( device_handle );
                        return 1;
                    }

                    is_first = 0;

                    // 处理采集到的传感器数据
                    convert_sensor_data_to_pdr_data( &sensor_data, &pdr_data );
                    CFmDataBufferLoader data_loader( config, 0, pdr_data );

                    // 将sensor_data数据追加的形式保存到csv文件中，方便调试和验证
                    int result = fm_pdr_save_sensor_data( ( char* )pdr_dataset_path.c_str(), &pdr_data.sensor_data );
                    if ( result != 0 )
                        std::cout << "保存失败，错误码: " << result << std::endl;
                }

                fm_device_free_sensor_data( sensor_data );
                fm_device_uninit( device_handle );
            }
        }
    }
    catch ( const std::exception& e )
    {
        // 处理其他异常
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
