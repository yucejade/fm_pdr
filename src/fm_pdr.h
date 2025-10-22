/// @file fm_pdr.h
/// @brief 行人航位推算(PDR)算法C接口
/// @ingroup localization_algs
#ifndef __FM_PDR__
#define __FM_PDR__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PDRConfig
{
    int    sample_rate;           ///< 采样率
    char*  model_name;            ///< 模型名
    char*  model_file_name;       ///< 保存/加载模型文件名
    int    clean_start;           ///< 训练时去除的头部不稳定数据
    int    clean_end;             ///< 训练时去除的尾部不稳定数据
    int    default_east_point;    ///< 确定初始东向量需要的点数
    int    move_average;          ///< 移动平均数N
    int    min_distance;          ///< 假定包含峰值的最小点数间隔
    double distance_frac_step;    ///< 表示每隔几步计算一次步长sigma、f
    double optimized_mode_ratio;  ///< 计算初始方向时两种方案所占百分比(单点方向和使用最小化平均)
    double butter_wn;             ///< 巴特沃斯滤波归一化频率
    int    least_start_point;     ///< 传给start函数的最少点数
} PDRConfig;

/// @struct PDRPoint
/// @brief 行人航位推算(PDR)的起点数据结构
typedef struct _PDRPoint
{
    double x;  ///< X轴经度
    double y;  ///< Y轴维度
} PDRPoint;

typedef struct _PDRSensorData
{
    double* acc_time;   ///< 时间戳（单位：秒）
    double* acc_x;      ///< 加速度计X轴
    double* acc_y;      ///< 加速度计Y轴
    double* acc_z;      ///< 加速度计Z轴
    double* lacc_time;  ///< 时间戳（单位：秒）
    double* lacc_x;     ///< 线性加速度计X轴
    double* lacc_y;     ///< 线性加速度计Y轴
    double* lacc_z;     ///< 线性加速度计Z轴
    double* gyr_time;   ///< 时间戳（单位：秒）
    double* gyr_x;      ///< 陀螺仪X轴
    double* gyr_y;      ///< 陀螺仪Y轴
    double* gyr_z;      ///< 陀螺仪Z轴
    double* mag_time;   ///< 时间戳（单位：秒）
    double* mag_x;      ///< 磁力计X轴
    double* mag_y;      ///< 磁力计Y轴
    double* mag_z;      ///< 磁力计Z轴

    unsigned long length;  ///< 数组长度
} PDRSensorData;

typedef struct _PDRTrueData
{
    double* time_location;        ///< 定位时间戳（单位：秒）
    double* latitude;             ///< 纬度
    double* longitude;            ///< 经度
    double* height;               ///< 高度
    double* velocity;             ///< 速度
    double* direction;            ///< 方向
    double* horizontal_accuracy;  ///< 水平精度
    double* vertical_accuracy;    ///< 垂直精度

    unsigned long length;  ///< 数组长度
} PDRTrueData;

typedef struct _PDRData
{
    PDRSensorData sensor_data;  ///< 传感器数据
    PDRTrueData   true_data;    ///< 真实定位数据
} PDRData;

/// @struct PDRTrajectory
/// @brief 行人航位推算(PDR)的定位数据结构
typedef struct _PDRTrajectory
{
    double*       time;       ///< 时间戳（单位：秒）
    double*       x;          ///< X轴经度
    double*       y;          ///< Y轴维度
    double*       direction;  ///< 运动方向（单位：度，TODO：0表示方向待定）
    unsigned long length;     ///< 数组长度
} PDRTrajectory;

/// @fn int fm_pdr_init(char* config_path, PDRData data, PDRTrajectory** trajectories)
/// @brief 初始化PDR算法
/// @param config_path [in] 配置文件路径
/// @param data [in] PDR训练数据
/// @param trajectories [out] 结果位置数组，需预分配内存（可为NULL）
/// @return >0: 训练生成位置点数量
///          0: trajectories传递NULL值并且初始化成功
///         <0: 错误码
int fm_pdr_init( char* config_path, PDRData data, PDRTrajectory** trajectories );

/// @fn void fm_pdr_set_start_point(PDRPoint start_point);
/// @brief 设置导航起点
/// @param start_point [in] 起点数据指针
void fm_pdr_set_start_point( PDRPoint start_point );

/// @fn int fm_pdr_predict(PDRSensorData sensor_data, PDRTrajectory** trajectories)
/// @brief 执行行人航位推算预测
/// @param sensor_data [in] 待处理数据指针
/// @param trajectories [out] 预测结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量
///          0: trajectories传递NULL值并且推算成功
///         <0: 错误码
int fm_pdr_predict( PDRSensorData sensor_data, PDRTrajectory** trajectories );

/// @fn void fm_pdr_free_trajectory()
/// @brief 释放PDR算法资源
void fm_pdr_free_trajectory();

/// @fn void fm_pdr_uninit()
/// @brief 释放PDR算法资源
void fm_pdr_uninit();

#ifdef __cplusplus
}
#endif

#endif  // __FM_PDR__