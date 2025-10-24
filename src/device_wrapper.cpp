#include "device_wrapper.h"
#include <chrono>
#include <thread>

const std::string i2cDevice         = "/dev/i2c-1";
uint8_t           deviceAddress_mmc = 0x30;
uint8_t           deviceAddress_imu = 0x69;

CFmDeviceWrapper::CFmDeviceWrapper(int sample_rate)
{
    // Initializing the MMC56x3
    if ( ! sensor_mmc.begin( deviceAddress_mmc, i2cDevice.c_str() ) )
        throw std::invalid_argument( "Failed to initialize MMC56x3 sensor" );

    // Initializing the ICM42670
    if (0 != sensor_imu.begin( false, deviceAddress_imu, i2cDevice.c_str() ))
        throw std::invalid_argument( "Failed to initialize ICM42670 sensor" );

    // Accel ODR = sample_rate(Hz) and Full Scale Range = 16G
    sensor_imu.startAccel( sample_rate, 16 );
    // Gyro ODR = sample_rate(Hz) and Full Scale Range = 2000 dps
    sensor_imu.startGyro( sample_rate, 2000 );

    // Wait IMU to start
    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
}

CFmDeviceWrapper::~CFmDeviceWrapper()
{
}

int64_t CFmDeviceWrapper::GetMicrosecondTimestamp()
{
    auto now      = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast< std::chrono::microseconds >( duration ).count();
}

bool CFmDeviceWrapper::ReadData( SensorData& sensor_data, unsigned long index )
{
    double timestamp = (double)GetMicrosecondTimestamp() / 1e6;

    // TDK42607
    inv_imu_sensor_event_t imu_event;
    sensor_imu.getDataFromRegisters( imu_event );

    sensor_data.sensor_data.acc_time[index] = timestamp;
    sensor_data.sensor_data.acc_x[index] = imu_event.accel[ 0 ] / 2048.0;
    sensor_data.sensor_data.acc_y[index] = imu_event.accel[ 1 ] / 2048.0;
    sensor_data.sensor_data.acc_z[index] = imu_event.accel[ 2 ] / 2048.0;
    sensor_data.sensor_data.gyr_time[index] = timestamp;
    sensor_data.sensor_data.gyr_x[index] = imu_event.gyro[ 0 ] / 16.4;
    sensor_data.sensor_data.gyr_y[index] = imu_event.gyro[ 1 ] / 16.4;
    sensor_data.sensor_data.gyr_z[index] = imu_event.gyro[ 2 ] / 16.4;

    // MMC56x3
    float x, y, z;
    if ( ! sensor_mmc.getEvent( x, y, z ) )
        return false;

    sensor_data.sensor_data.mag_time[index] = timestamp;
    sensor_data.sensor_data.mag_x[index] = x;
    sensor_data.sensor_data.mag_y[index] = y;
    sensor_data.sensor_data.mag_z[index] = z;

    return true;
}