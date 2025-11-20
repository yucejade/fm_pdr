#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "fm_device_wrapper.h"
#include <cstddef>

class CFmDeviceWrapper
{
public:
    CFmDeviceWrapper( int sample_rate );
    ~CFmDeviceWrapper();

    int64_t GetMicrosecondTimestamp();
    bool    ReadData( SensorData& sensor_data, unsigned long index, bool is_first, int64_t& timestamp );
private:
    MMC56x3  m_sensor_mmc;
    ICM42670 m_sensor_imu;

    int64_t         m_start_time_ms;
};