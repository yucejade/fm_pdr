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

typedef enum
{
    PDR_SUCCESS       = 0,
    PDR_INVALID_INPUT = -1,
    PDR_SENSOR_ERROR  = -2,
    PDR_CALIB_FAILED  = -3
} PDRStatus;

/// @fn int fm_pdr_init(void* train_data, PDRTrajectory** trajectories)
/// @brief 初始化PDR算法
/// @param config_path [in] 配置文件路径
/// @param train_data [in] PDR训练数据
/// @param trajectories [out] 结果位置数组，需预分配内存（可为NULL）
/// @return >0: 成功
///          0: trajectories为NULL时的成功返回
///         <0: 错误码（如锚点无效）
int fm_pdr_init( char* config_path, void* train_data, PDRTrajectory** trajectories );

/// @fn int fm_pdr_predict(void* process_data, PDRTrajectory** trajectories)
/// @brief 执行行人航位推算预测
/// @param process_data [in] 待处理数据指针（TODO:类型待定）
/// @param trajectories [out] 预测结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量
///          0: trajectories为NULL时的成功返回
///         <0: 错误码（如锚点无效）
int fm_pdr_predict( PDRPoint start_point, void* process_data, PDRTrajectory** trajectories );

/// @fn int fm_pdr_calibration(void *anchor_point, PDRTrajectory** trajectories);
/// @brief 根据矫正点重新对行人航位路线进行校准
/// @param process_data [in] 待处理数据指针（TODO:类型待定）
/// @param trajectories [out] 预测结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量
///          0: trajectories为NULL时的成功返回
///         <0: 错误码（如锚点无效）
int fm_pdr_calibration( void* anchor_point, PDRTrajectory** trajectories );

/// @fn void fm_pdr_uninit()
/// @brief 释放PDR算法资源
void fm_pdr_uninit();

#ifdef __cplusplus
}
#endif

#endif  // __FM_PDR__