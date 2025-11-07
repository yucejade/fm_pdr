#include "fm_device_wrapper.h"
#include "device_wrapper.h"
#include <cstdint>
#include <memory.h>
#include <stdexcept>
#include <chrono>
#include <thread>

int fm_device_init( int sample_rate, fm_device_handle_t* device_handle )
{
    if ( ! device_handle )
        return -1;

    try
    {
        device_handle->handler     = new CFmDeviceWrapper( sample_rate );
        device_handle->sample_rate = sample_rate;
        return 0;
    }
    catch ( const std::invalid_argument& e )
    {
        return -1;
    }
}

int fm_device_read( fm_device_handle_t device_handle, int is_first, int count, int rewrite, SensorData* data )
{
    if ( ! device_handle.handler || ! data )
        return -1;

    memset( data, 0x00, sizeof( SensorData ) );

    if ( rewrite )
        fm_device_free_sensor_data( *data );

    data->real_length = static_cast< unsigned long >( count > 0 ? count : 1 );
    if ( ! data->sensor_data.acc_time )
        data->sensor_data.acc_time = new double[ data->real_length ]();
    if ( ! data->sensor_data.acc_x )
        data->sensor_data.acc_x = new double[ data->real_length ]();
    if ( ! data->sensor_data.acc_y )
        data->sensor_data.acc_y = new double[ data->real_length ]();
    if ( ! data->sensor_data.acc_z )
        data->sensor_data.acc_z = new double[ data->real_length ]();
    if ( ! data->sensor_data.lacc_time )
        data->sensor_data.lacc_time = new double[ data->real_length ]();
    if ( ! data->sensor_data.lacc_x )
        data->sensor_data.lacc_x = new double[ data->real_length ]();
    if ( ! data->sensor_data.lacc_y )
        data->sensor_data.lacc_y = new double[ data->real_length ]();
    if ( ! data->sensor_data.lacc_z )
        data->sensor_data.lacc_z = new double[ data->real_length ]();
    if ( ! data->sensor_data.gyr_time )
        data->sensor_data.gyr_time = new double[ data->real_length ]();
    if ( ! data->sensor_data.gyr_x )
        data->sensor_data.gyr_x = new double[ data->real_length ]();
    if ( ! data->sensor_data.gyr_y )
        data->sensor_data.gyr_y = new double[ data->real_length ]();
    if ( ! data->sensor_data.gyr_z )
        data->sensor_data.gyr_z = new double[ data->real_length ]();
    if ( ! data->sensor_data.mag_time )
        data->sensor_data.mag_time = new double[ data->real_length ]();
    if ( ! data->sensor_data.mag_x )
        data->sensor_data.mag_x = new double[ data->real_length ]();
    if ( ! data->sensor_data.mag_y )
        data->sensor_data.mag_y = new double[ data->real_length ]();
    if ( ! data->sensor_data.mag_z )
        data->sensor_data.mag_z = new double[ data->real_length ]();

    CFmDeviceWrapper* wrapper = static_cast< CFmDeviceWrapper* >( device_handle.handler );
    for (unsigned long i = 0; i < data->real_length; ++i)
    {
        int64_t timestamp = 0;
        wrapper->ReadData( *data, i, (bool)is_first, timestamp );
        data->sensor_data.length = i + 1;
        is_first = 0;
        
        int64_t time_consuming = wrapper->GetMicrosecondTimestamp() - timestamp;
        int64_t target_time = 1000000 / device_handle.sample_rate;
        std::this_thread::sleep_for(std::chrono::microseconds(device_handle.sample_rate > 0 ? (target_time - time_consuming) : 0));
    }

    return 0;
}

void fm_device_free_sensor_data( SensorData data )
{
    if (data.sensor_data.acc_time)
    {
        delete[] data.sensor_data.acc_time;
        data.sensor_data.acc_time = nullptr;
    }
    if (data.sensor_data.acc_x)
    {
        delete[] data.sensor_data.acc_x;
        data.sensor_data.acc_x = nullptr;
    }
    if (data.sensor_data.acc_y)
    {
        delete[] data.sensor_data.acc_y;
        data.sensor_data.acc_y = nullptr;
    }
    if (data.sensor_data.acc_z)
    {
        delete[] data.sensor_data.acc_z;
        data.sensor_data.acc_z = nullptr;
    }
    if (data.sensor_data.lacc_time)
    {
        delete[] data.sensor_data.lacc_time;
        data.sensor_data.lacc_time = nullptr;
    }
    if (data.sensor_data.lacc_x)
    {
        delete[] data.sensor_data.lacc_x;
        data.sensor_data.lacc_x = nullptr;
    }
    if (data.sensor_data.lacc_y)
    {
        delete[] data.sensor_data.lacc_y;
        data.sensor_data.lacc_y = nullptr;
    }
    if (data.sensor_data.lacc_z)
    {
        delete[] data.sensor_data.lacc_z;
        data.sensor_data.lacc_z = nullptr;
    }
    if (data.sensor_data.gyr_time)
    {
        delete[] data.sensor_data.gyr_time;
        data.sensor_data.gyr_time = nullptr;
    }
    if (data.sensor_data.gyr_x)
    {
        delete[] data.sensor_data.gyr_x;
        data.sensor_data.gyr_x = nullptr;
    }
    if (data.sensor_data.gyr_y)
    {
        delete[] data.sensor_data.gyr_y;
        data.sensor_data.gyr_y = nullptr;
    }
    if (data.sensor_data.gyr_z)
    {
        delete[] data.sensor_data.gyr_z;
        data.sensor_data.gyr_z = nullptr;
    }
    if (data.sensor_data.mag_time)
    {
        delete[] data.sensor_data.mag_time;
        data.sensor_data.mag_time = nullptr;
    }
    if (data.sensor_data.mag_x)
    {
        delete[] data.sensor_data.mag_x;
        data.sensor_data.mag_x = nullptr;
    }
    if (data.sensor_data.mag_y)
    {
        delete[] data.sensor_data.mag_y;
        data.sensor_data.mag_y = nullptr;
    }
    if (data.sensor_data.mag_z)
    {
        delete[] data.sensor_data.mag_z;
        data.sensor_data.mag_z = nullptr;
    }
    data.real_length = 0;
}

void fm_device_uninit( fm_device_handle_t device_handle )
{
    if ( device_handle.handler )
    {
        delete static_cast< CFmDeviceWrapper* >( device_handle.handler );
        device_handle.handler = nullptr;
    }
}