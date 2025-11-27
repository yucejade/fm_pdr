#include "fm_device_wrapper.h"
#include "magnetometer-calibration.h"
#include <motion_mc.h>
#include <algorithm>
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
                  << "  -t, --type\t\t包含type选项，实时链接磁力计显示校准前后数值，否则输入磁力计数据生成校准文件\n"
                  << "  -c, --calibration-path <校准文件>\t\t实时链接磁力计时生成的校准文件，默认使用./mag_calib.json\n"
                  << "  -d, --data-path <数据文件>\t\t实时链接磁力计时生成的数据文件，默认使用./mag_data.csv\n"
                  << "  -m, --mag-data-path <磁力计数据文件>\t\t默认使用./Magnetometer.csv\n"
                  << "  -o, --output-path <输出校准结果文件>\t\t默认使用./mag_calib.json\n"
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

    bool type = parser.hasOption( "-t" ) || parser.hasOption( "--type" );

    if ( type )  // 实时链接磁力计显示校准前后数值
    {
        std::string calibration_path = parser.getOption( "-c" );
        if ( calibration_path.empty() )
        {
            calibration_path = parser.getOption( "--calibration-path" );
            if ( calibration_path.empty() )
                calibration_path = "./mag_calib.json";
        }
        std::string data_path = parser.getOption( "-d" );
        if ( data_path.empty() )
        {
            data_path = parser.getOption( "--data-path" );
            if ( data_path.empty() )
                data_path = "./mag_data.csv";
        }

        try
        {
            fm_device_handle_t         device_handler;
            PDRData                    pdr_data;
            SensorData                 sensor_data;
            bool                       is_first = true;
            int                        count    = 0;
            CFmMagnetometerCalibration calib( calibration_path );
            int                        ret = PDR_RESULT_SUCCESS;

            ret = fm_device_init( 50, &device_handler );
            if ( ret != 0 )
                return PDR_RESULT_DEVICE_INIT_ERROR;

            memset( &pdr_data, 0x00, sizeof( pdr_data ) );
            memset( &sensor_data, 0x00, sizeof( sensor_data ) );

            while ( ++count <= 30 )
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

                for (int i = 0; i < length; ++i)
                {
                    double mag_x = sensor_data.sensor_data.mag_x[i];
                    double mag_y = sensor_data.sensor_data.mag_y[i];
                    double mag_z = sensor_data.sensor_data.mag_z[i];

                    // 计算模长
                    double magnitude_before = std::sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
                    std::cout << "校准前数据：(" << mag_x << "," << mag_y << "," << mag_z << "," << magnitude_before << ")" << std::endl;

                    calib.Calibration( mag_x, mag_y, mag_z );

                    mag_x *= sensor_data.sensor_data.mag_x[i];
                    mag_y *= sensor_data.sensor_data.mag_y[i];
                    mag_z *= sensor_data.sensor_data.mag_z[i];
                    double magnitude_after = std::sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
                    std::cout << "校准后数据：(" << mag_x << "," << mag_y << "," << mag_z << "," << magnitude_after << ")" << std::endl;
                }

                // 转换为PDRData结构
                pdr_data.sensor_data = sensor_data.sensor_data;

                // 将sensor_data数据追加的形式保存到csv文件中，方便调试和验证
                int result = fm_pdr_save_pdr_data( ( char* )data_path.c_str(), &pdr_data );
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
    else  // 输入磁力计数据生成校准文件
    {
        std::string mag_data_path = parser.getOption( "-m" );
        if ( mag_data_path.empty() )
        {
            mag_data_path = parser.getOption( "--mag-data-path" );
            if ( mag_data_path.empty() )
                mag_data_path = "./Magnetometer.csv";
        }

        std::string output_path = parser.getOption( "-o" );
        if ( output_path.empty() )
        {
            output_path = parser.getOption( "--output-path" );
            if ( output_path.empty() )
                output_path = "./mag_calib.json";
        }

        try
        {
            CFmMagnetometerCalibration calib( mag_data_path, output_path );
        }
        catch ( ... )
        {
            std::cerr << "磁力计校准失败" << std::endl;
        }
    }
    return 0;
}