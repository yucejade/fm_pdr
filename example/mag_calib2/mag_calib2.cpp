#include "fm_device_wrapper.h"
#include "realtime_mag_calibration.h"
#include <iomanip>
#include <thread>
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace chrono;

int main( int argc, char* argv[] )
{
    fm_device_handle_t    device_handler;
    SensorData            sensor_data;
    RealTimeMagCalibrator calibrator( 40, 1.0 );  // 窗口40个点，每1秒校准1次
    int                   ret = PDR_RESULT_SUCCESS;

    ret = fm_device_init( 50, &device_handler );
    if ( ret != 0 )
        return PDR_RESULT_DEVICE_INIT_ERROR;

    cout << "Real-time Magnetometer Calibration Demo\n";
    cout << "----------------------------------------\n";
    cout << "Raw Mag (x,y,z) | Calibrated Mag (x,y,z) | Hard Iron (x,y,z)\n";
    cout << "----------------------------------------\n";

    while ( true )
    {
        memset( &sensor_data, 0x00, sizeof( sensor_data ) );

        // 1. 生成并喂入原始数据
        ret = fm_device_read( device_handler, false, 1, 1, &sensor_data );
        if ( ret != 0 )
        {
            std::cerr << "Sensor data reading failed." << std::endl;
            continue;
        }
        Vector3d raw(sensor_data.sensor_data.mag_x[0], sensor_data.sensor_data.mag_y[0], sensor_data.sensor_data.mag_z[0]);
        calibrator.feed( raw );

        // 2. 检查是否需要校准（每1秒一次）
        if ( calibrator.needCalibrate() )
        {
            if ( calibrator.calibrate() )
            {
                // 输出校准参数变化
                Vector3d hi = calibrator.getHardIron();
                // cout << "Calibrated at " << fixed << setprecision( 1 ) << "Hard Iron: (" << hi.x() << "," << hi.y() << "," << hi.z() << ")\n";
            }
        }

        // 3. 应用校准并输出结果
        Vector3d calibrated = calibrator.apply( raw );
        cout << fixed << setprecision( 1 ) << "(" << raw.x() << "," << raw.y() << "," << raw.z() << ") | (" << calibrated.x() << "," << calibrated.y() << "," << calibrated.z() << ")\n";

        // 控制输出频率（与数据生成频率一致）
        this_thread::sleep_for( milliseconds( 50 ) );
    }

    // 释放设备读取缓存
    fm_device_free_sensor_data( sensor_data );
    fm_device_uninit( device_handler );

    return 0;
}