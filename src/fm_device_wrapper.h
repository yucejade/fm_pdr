/// @file fm_device_wrapper.h
/// @brief 设备访问接口
/// @ingroup localization_algs
#ifndef __FM_DEVICE_WRAPPER__
#define __FM_DEVICE_WRAPPER__

#ifdef __cplusplus
extern "C" {
#endif

#include "fm_pdr.h"

typedef struct _SensorData
{
    PDRSensorData sensor_data;  ///< 传感器数据
    unsigned long real_length;  ///< 表示实际缓存大小
} SensorData;

typedef struct _fm_device_handle_t
{
    void* handler;      ///< 设备句柄
    int   sample_rate;  ///< 采样率
} fm_device_handle_t;

/// @fn int fm_device_init( int sample_rate, fm_device_handle_t *device_handle )
/// @brief 初始化设备
/// @param sample_rate [in] 传感器采样率
/// @param device_handle [out] 设备句柄
/// @return 0: 初始化成功
///         <0: 错误码
int fm_device_init( int sample_rate, fm_device_handle_t* device_handle );

/// @fn int fm_device_read(void *device_handle, int count, int rewrite, SensorData data);
/// @brief 读取传感器数据
/// @param count [in] 传感器采样时长(微秒)，大于0时表示采集数据个数，小于等于0时表示采集一次数据
/// @param data [in] 是否覆盖读取数据，!=0表示覆盖，0表示重新创建内存
/// @param data [in] 读取的传感器数据
/// @return 0: 读取成功
///         <0: 错误码
int fm_device_read( fm_device_handle_t device_handle, int count, int rewrite, SensorData* data );

/// @fn int fm_device_free_sensor_data(void *device_handle, SensorData data);
/// @brief 释放传感器数据内存
/// @param data [in] 读取的传感器数据
/// @return 无
void fm_device_free_sensor_data( SensorData data );

/// @fn void fm_device_uninit( fm_device_handle_t device_handle )
/// @brief 反初始化设备
/// @param device_handle [in] 设备句柄
/// @return 0: 反初始化成功
///         <0: 错误码
void fm_device_uninit( fm_device_handle_t device_handle );

#ifdef __cplusplus
}
#endif

#endif  // __FM_DEVICE_WRAPPER__