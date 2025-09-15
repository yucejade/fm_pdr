#include "pdr.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

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
                  << "  -t, --train <样本数据路径>	                                  在配置文件中的model_file_name路径下输出模型\n"
                  << "  -d, --dataset <PDR测试数据路径>	                              使用配置文件中的model_file_name路径下的模型文件对测试数据进行PDR检测\n"
                  << "  -t, --train <样本数据路径> -d, --dataset <PDR测试数据路径>     直接使用训练样本训练结果对测试数据进行PDR检测（当配置文件中的model_file_name路径为空时，不输出模型）\n"
                  << "  -o, --original-logic  使用原始开源工程的逻辑进行PDR检测\n"
                  << "  -e, --evaluation      是否打印指标评估\n"
                  << "  -h, --help            帮助信息\n";
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

    std::string org_dataset_path = parser.getOption( "-o" );
    if ( org_dataset_path.empty() )
        org_dataset_path = parser.getOption( "--original-logic" );

    // 验证必要参数
    if ( ( ( ! train_dataset_path.empty() || ! pdr_dataset_path.empty() ) && ! org_dataset_path.empty() ) || ( train_dataset_path.empty() && pdr_dataset_path.empty() && org_dataset_path.empty() ) )
    {
        std::cerr << "Argument error.\n";
        parser.showHelp();
        return 1;
    }

    // 检查是否静默模式
    bool eval = parser.hasOption( "-e" ) || parser.hasOption( "--evaluation" );

    try
    {
        // 创建测试用例
        if ( ! train_dataset_path.empty() )
        {
            PDRConfig                  config = { ( char* )"Mean", ( char* )"model.dat", 4, 10, 20, 4.0, 0.95, 0.0035, 150 };
            CFmDataLoader              data( train_dataset_path );
            CFmDataLoader              train_data = slice( data, 0, data.m_len_input );
            std::vector< PDRPosition > train_position;
            CFmPDR                     pdr( config, train_data, train_position );
        }
        else if ( ! pdr_dataset_path.empty() )
        {
            PDRConfig                  config = { ( char* )"Mean", ( char* )"model.dat", 4, 10, 20, 4.0, 0.95, 0.0035, 150 };
            CFmDataLoader              data( pdr_dataset_path );
            CFmDataLoader              pdr_data = slice( data, data.m_len_input, 0 );
            CFmPDR                     pdr( config );
            size_t                     i = 0, step = 2;
            bool                       is_stop = false;
            std::vector< PDRPosition > trajectory;

            // 执行PDR算法
            StartInfo si = pdr.start( pdr_data );
            while ( true )
            {
                size_t        pdr_size = pdr_data.m_time_location.size();
                size_t        s        = i * step;
                size_t        e        = std::min( ( i + 1 ) * step, pdr_size );
                CFmDataLoader segment  = slice( pdr_data, s, e );

                auto                       start_time = std::chrono::steady_clock::now();
                std::vector< PDRPosition > t          = pdr.pdr( si, segment );
                if ( t.empty() )
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
                    for ( const auto& pos : t )
                        trajectory.push_back( pos );
                }
                auto   end_time = std::chrono::steady_clock::now();
                double time     = std::chrono::duration< double, std::micro >( end_time - start_time ).count();
                cout << "Segment " << i << ": " << s << " to " << e << ", Time taken: " << time << " microseconds" << endl;

                if ( e >= pdr_size )
                    break;
                i++;
            }

            // 保存结果
            pdr_data.set_location_output( trajectory, data.m_len_input );

            if ( ! eval )
                pdr.eval_model( pdr_data );
        }
        else if ( ! train_dataset_path.empty() && ! pdr_dataset_path.empty() )
        {
            PDRConfig                  config = { ( char* )"Linear", ( char* )"", 4, 10, 20, 4.0, 0.95, 0.0035, 50 };
            CFmDataLoader              data( pdr_dataset_path );
            CFmDataLoader              train_data = slice( data, 0, data.m_len_input );
            CFmDataLoader              pdr_data   = slice( data, data.m_len_input, 0 );
            std::vector< PDRPosition > train_position;
            CFmPDR                     pdr( config, train_data, train_position );

            // 执行PDR算法
            StartInfo si = pdr.start( pdr_data );

            std::vector< PDRPosition > trajectory = pdr.pdr( si, pdr_data );
            if ( trajectory.empty() )
                throw std::runtime_error( "PDR process returned empty trajectory." );

            // 保存结果
            pdr_data.set_location_output( trajectory, data.m_len_input );

            if ( ! eval )
                pdr.eval_model( pdr_data );
        }
        else if ( ! org_dataset_path.empty() )
        {
            PDRConfig                  config = { ( char* )"Mean", ( char* )"model.dat", 4, 10, 20, 4.0, 0.95, 0.0035, 10 };
            CFmDataLoader              data( org_dataset_path );
            std::vector< PDRPosition > train_position;
            CFmPDR                     pdr( config, data, train_position );

            // 执行PDR算法
            pdr.pdr( data, "Mean" );
            if ( ! eval )
                pdr.eval_model( data );
            return 0;
        }
        else
        {
            std::cerr << "Argument error.\n";
            parser.showHelp();
            return 1;
        }

        // CFmDataLoader train_data = slice(data, config.clean_data, data.m_len_input);

        // // 获取启动信息
        // StartInfo start_info = pdr.start(train_data);
        // CFmDataLoader process_data = slice(data, data.m_len_input, 0);

        // if (!silent)
        // {
        //     // 检查是否有有效位置数据
        //     if (data.m_have_location_valid)
        //     {
        //         // 获取评估指标
        //         double dist_error = data.get_dist_error();
        //         double dir_error = data.get_dir_error();
        //         double dir_ratio = data.get_dir_ratio();

        //         // 输出评估结果
        //         std::cout << "Distances error: " << dist_error << std::endl;
        //         std::cout << "Direction error: " << dir_error << std::endl;
        //         std::cout << "Direction ratio: " << dir_ratio << std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "Location.csv not found, cannot calculate distance and direction error" << std::endl;
        //     }
        // }
    }
    catch ( const std::exception& e )
    {
        // 处理其他异常
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
