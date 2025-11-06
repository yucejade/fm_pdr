#include "data_file_loader.h"
#include "data_manager.h"
#include "fm_device_wrapper.h"
#include "fm_pdr.h"
#include "json_operator.h"
#include "pdr.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <algorithm>
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
                  << "  -e, --evaluation\t\t\t是否打印指标评估\n"
                  << "  -c, --config <配置文件路径>\t\t指定PDR配置文件路径，默认使用../conf/config.json\n"
                  << "  -t, --train <样本数据路径>\t\t表示需要加载的<样本数据路径>，训练模型输出到model_file_name配置项设置的路径下\n"
                  << "  -d, --dataset <PDR数据路径>\t\t表示需要加载的<PDR测试数据路径>，使用model_file_name配置项设置路径下的模型文件进行推算\n"
                  << "  -h, --help\t\t\t\t帮助信息\n";
    }
private:
    std::vector< std::string > tokens_;
};

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
        return -1;
    }

    // 默认配置文件路径
    if ( pdr_config_path.empty() )
        pdr_config_path = "../conf/config.json";

    // 检查是否输出评估结果
    bool eval = parser.hasOption( "-e" ) || parser.hasOption( "--evaluation" );

    try
    {
        PDRConfig config = CFmJSONOperator::readPDRConfigFromJson( pdr_config_path.c_str() );

        // 创建测试用例
        if ( ! train_dataset_path.empty() )
        {
            constexpr size_t   test_case0_input_length = 60;
            Eigen::MatrixXd    train_position;
            CFmDataFileLoader  data( config, test_case0_input_length, train_dataset_path );
            CFmDataFileLoader* train_data = slice( data, 0, data.get_train_data_size() * config.sample_rate );
            CFmPDR             pdr( config, *train_data, train_position );
            delete train_data;
        }

        if ( ! pdr_dataset_path.empty() )
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
    }
    catch ( const std::exception& e )
    {
        // 处理其他异常
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
