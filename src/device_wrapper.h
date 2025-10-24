#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "fm_device_wrapper.h"

class CFmDeviceWrapper
{
public:
    CFmDeviceWrapper(int sample_rate);
    ~CFmDeviceWrapper();

    bool ReadData( SensorData& sensor_data, unsigned long index );
private:
    MMC56x3 sensor_mmc;
    ICM42670 sensor_imu;
private:
    int64_t GetMicrosecondTimestamp();
};