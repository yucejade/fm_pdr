#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "fm_pdr.h"

class CFmDeviceWrapper
{
public:
    CFmDeviceWrapper();
    ~CFmDeviceWrapper();

    bool ReadData( PDRSensorData& sensor_data, int index );
private:
    MMC56x3 sensor_mmc;
    ICM42670 sensor_imu;
private:
    int64_t GetMicrosecondTimestamp();
};