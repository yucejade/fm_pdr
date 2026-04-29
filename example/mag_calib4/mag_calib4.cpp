#include "SixParametersCorrector.h"
#include "SoftAndHardIronCalibration.h"
#include "fm_device_wrapper.h"
#include <algorithm>
#include <iostream>
#include <rapidcsv.h>
#include <string>
#include <vector>

using namespace Boardcore;
using namespace Eigen;

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

    int getIntOption( const std::string& option, int defaultValue = 180 ) const
    {
        std::string strValue = getOption( option );
        if ( strValue.empty() )
            return defaultValue;

        try
        {
            return std::stoi( strValue );
        }
        catch ( const std::invalid_argument& )
        {
            // 字符串不是整数（如"-i abc"），返回默认值
            return defaultValue;
        }
        catch ( const std::out_of_range& )
        {
            // 整数超出int范围（如"-i 1234567890123"），返回默认值
            return defaultValue;
        }
    }

    float getFloatOption( const std::string& option, float defaultValue = 0.0f ) const
    {
        std::string strValue = getOption( option );
        if ( strValue.empty() )
            return defaultValue;

        try
        {
            return std::stof( strValue );
        }
        catch ( const std::invalid_argument& )
        {
            return defaultValue;
        }
        catch ( const std::out_of_range& )
        {
            return defaultValue;
        }
    }

    // 检查选项是否存在
    bool hasOption( const std::string& option ) const
    {
        return std::find( tokens_.begin(), tokens_.end(), option ) != tokens_.end();
    }

    // 显示帮助信息
    void showHelp() const
    {
        std::cout << "Usage: mag_calib4 [options]\n"
                  << "Options:\n"
                  << "  -t, --type <type>\t\t操作类型\n"
                  << "    0: 实时采集磁力计数据\n"
                  << "    1: 从数据文件生成校准参数\n"
                  << "    2: 实时校准验证\n"
                  << "    3: 验证校准文件有效性\n"
                  << "  -d, --mag-data-path <path>\t磁力计数据文件路径\n"
                  << "  -c, --calibration-path <path>\t校准文件路径\n"
                  << "  -o, --output-path <path>\t输出校准结果文件路径\n"
                  << "  -i, --interval <seconds>\t实时操作时间间隔\n"
                  << "  --max-avg-error <float>\t模长变异系数阈值（-t 3），默认 0.05\n"
                  << "  --mag-range <min,max>\t\t校正后模长范围µT（-t 3），默认 40,60\n"
                  << "  -h, --help\t\t\t帮助信息\n";
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

    int type;
    bool has_type = parser.hasOption( "-t" ) || parser.hasOption( "--type" );
    if ( ! has_type )
    {
        std::cerr << "参数错误!!!" << std::endl;
        parser.showHelp();
        return -1;
    }
    if ( parser.hasOption( "-t" ) )
        type = parser.getIntOption( "-t" );
    else if ( parser.hasOption( "--type" ) )
        type = parser.getIntOption( "--type" );
    else
        type = 0;  // 默认类型

    if ( type == 0 )  // 实时链接磁力计显示校准前后数值
    {
        std::string mag_data_path = parser.getOption( "-d" );
        if ( mag_data_path.empty() ) {
            mag_data_path = parser.getOption( "--mag-data-path" );
            if (mag_data_path.empty())
                mag_data_path = "./mag_calib_data";
        }

        int interval;
        if ( parser.hasOption( "-i" ) )
            interval = parser.getIntOption( "-i" );
        else if ( parser.hasOption( "--interval" ) )
            interval = parser.getIntOption( "--interval" );
        else
            interval = 180;  // 默认时间间隔

        try
        {
            fm_device_handle_t     device_handler;
            SensorData             sensor_data;
            bool                   is_first = true;
            int                    count    = 0;
            SixParametersCorrector loaded_corrector;
            int                    ret = PDR_RESULT_SUCCESS;

            ret = fm_device_init( 50, &device_handler );
            if ( ret != 0 )
                return PDR_RESULT_DEVICE_INIT_ERROR;

            memset( &sensor_data, 0x00, sizeof( sensor_data ) );

            if ( interval > 0 )
                interval = ( interval % 2 != 0 ? interval + 1 : interval ) / 2;
            while ( interval <= 0 || ++count < interval )
            {
                const int length = 50 * 2;

                // 使用固定缓存模式读取传感器数据
                ret = fm_device_read( device_handler, is_first, length, 1, &sensor_data );
                if ( ret != 0 )
                {
                    std::cerr << "Sensor data reading failed." << std::endl;
                    continue;
                }

                // 标记不是第一次读取数据，即不需要再次创建缓存
                is_first = false;

                // 转换为PDRData结构
                PDRData pdr_data;
                memset( &pdr_data, 0x00, sizeof( pdr_data ) );
                pdr_data.sensor_data = sensor_data.sensor_data;

                int result = fm_pdr_save_pdr_data( ( char* )mag_data_path.c_str(), &pdr_data );
                if ( result != 0 )
                {
                    std::cerr << "Failed to save data." << std::endl;
                    continue;
                }
            }

            // 释放设备读取缓存
            fm_device_free_sensor_data( sensor_data );
            fm_device_uninit( device_handler );
        }
        catch ( ... )
        {
            std::cerr << "磁力计校准失败" << std::endl;
        }
    }
    else if ( type == 1 ) // 输入磁力计数据生成校准文件
    {
        std::string mag_data_path = parser.getOption( "-d" );
        if ( mag_data_path.empty() )
        {
            mag_data_path = parser.getOption( "--mag-data-path" );
            if ( mag_data_path.empty() )
                mag_data_path = "./mag_calib_data/Magnetometer.csv";
        }

        std::string output_path = parser.getOption( "-o" );
        if ( output_path.empty() )
        {
            output_path = parser.getOption( "--output-path" );
            if ( output_path.empty() )
                output_path = "./mag_calib.csv";
        }

        try
        {
            SoftAndHardIronCalibration calib;
            rapidcsv::Document         doc( mag_data_path, rapidcsv::LabelParams( 0 ) );
            std::vector< double >      mag_x = doc.GetColumn< double >( "X (µT)" );  // 提取"X (µT)"列的所有数据
            std::vector< double >      mag_y = doc.GetColumn< double >( "Y (µT)" );  // 提取"Y (µT)"列的所有数据
            std::vector< double >      mag_z = doc.GetColumn< double >( "Z (µT)" );  // 提取"Z (µT)"列的所有数据

            for ( size_t i = 0; i < mag_x.size(); ++i )
            {
                MagnetometerData data;

                // 复制文件中的数据
                data.magneticFieldX = mag_x[ i ];
                data.magneticFieldY = mag_y[ i ];
                data.magneticFieldZ = mag_z[ i ];

                // 喂入校准器（累积数据）
                calib.feed( data );
            }

            SixParametersCorrector result = calib.computeResult();
            std::cout << "校准完成！" << std::endl;

            const std::string param_file = output_path;
            if ( result.toFile( param_file ) )
            {
                std::cout << "校准参数已保存到：" << param_file << std::endl;
            }
            else
            {
                std::cerr << "保存校准参数失败！" << std::endl;
                return 1;
            }
        }
        catch ( ... )
        {
            std::cerr << "磁力计校准失败" << std::endl;
        }
    }
    else if (type == 2)
    {
        std::string calibration_path = parser.getOption( "-c" );
        if ( calibration_path.empty() )
        {
            calibration_path = parser.getOption( "--calibration-path" );
            if (calibration_path.empty())
                calibration_path = "./mag_calib.csv";
        }

        int interval;
        if ( parser.hasOption( "-i" ) )
            interval = parser.getIntOption( "-i" );
        else if ( parser.hasOption( "--interval" ) )
            interval = parser.getIntOption( "--interval" );
        else
            interval = 180;  // 默认时间间隔

        try
        {
            fm_device_handle_t     device_handler;
            SensorData             sensor_data;
            bool                   is_first = true;
            int                    count    = 0;
            SixParametersCorrector loaded_corrector;
            int                    ret = PDR_RESULT_SUCCESS;

            if ( loaded_corrector.fromFile( calibration_path ) )
            {
                std::cout << "成功加载校准参数！" << std::endl;
                // 打印加载的参数（可选，用于验证）
                std::cout << "加载的硬铁偏移（b）: " << loaded_corrector.getb().transpose() << " μT" << std::endl;
                std::cout << "加载的软铁增益（A）: " << loaded_corrector.getA().transpose() << "（无单位）" << std::endl;
            }
            else
            {
                std::cerr << "加载校准参数失败！" << std::endl;
                return 1;
            }

            ret = fm_device_init( 50, &device_handler );
            if ( ret != 0 )
                return PDR_RESULT_DEVICE_INIT_ERROR;

            memset( &sensor_data, 0x00, sizeof( sensor_data ) );

            if ( interval > 0 )
                interval *= 50;
            while ( interval <= 0 || ++count < interval )
            {
                // 使用固定缓存模式读取传感器数据
                ret = fm_device_read( device_handler, is_first, 1, 1, &sensor_data );
                if ( ret != 0 )
                {
                    std::cerr << "Sensor data reading failed." << std::endl;
                    continue;
                }

                // 标记不是第一次读取数据，即不需要再次创建缓存
                is_first = false;

                const double&    timestamp = sensor_data.sensor_data.acc_time[ 0 ];
                const double&    mag_x     = sensor_data.sensor_data.mag_x[ 0 ];
                const double&    mag_y     = sensor_data.sensor_data.mag_y[ 0 ];
                const double&    mag_z     = sensor_data.sensor_data.mag_z[ 0 ];
                MagnetometerData raw_data( timestamp, mag_x, mag_y, mag_z );
                Vector3f         raw_vec( raw_data.magneticFieldX, raw_data.magneticFieldY, raw_data.magneticFieldZ );

                // 调用校正方法（公式：校正后 = (原始数据 - 偏移) × 增益）
                Vector3f corrected_vec = loaded_corrector.correct( raw_vec );

                // 输出校正结果
                std::cout << "\n=== 数据校正示例 ===" << std::endl;
                std::cout << "原始数据: " << raw_vec.transpose() << " μT" << std::endl;
                std::cout << "校正后数据: " << corrected_vec.transpose() << ", " << corrected_vec.norm() << " μT" << std::endl;
            }

            // 释放设备读取缓存
            fm_device_free_sensor_data( sensor_data );
            fm_device_uninit( device_handler );
        }
        catch ( ... )
        {
            std::cerr << "磁力计校准失败" << std::endl;
        }
    }
    else
    {
        std::cerr << "参数错误!!!" << std::endl;
        parser.showHelp();
    }
    return 0;
}