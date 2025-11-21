/// @file fm_pdr.h
/// @brief 行人航迹推算(PDR)算法C接口
/// @ingroup localization_algs
#ifndef __FM_PDR__
#define __FM_PDR__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PDRConfig
{
    int    sample_rate;           ///< 采样率
    int    pdr_duration;          ///< 送给PDR算法的时间间隔
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
/// @brief 行人航迹推算(PDR)的起点数据结构
typedef struct _PDRPoint
{
    double x;  ///< X轴经度
    double y;  ///< Y轴维度
} PDRPoint;

/// @struct PDRData
/// @brief 行人航迹推算(PDR)使用的传感器数据
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

    unsigned long length;  ///< 数组有效长度
} PDRSensorData;

/// @struct PDRData
/// @brief 行人航迹推算(PDR)使用的真实(GPS)数据
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

    unsigned long length;  ///< 数组有效长度
} PDRTrueData;

/// @struct PDRData
/// @brief 行人航迹推算(PDR)的传感器数据汇总
typedef struct _PDRData
{
    PDRSensorData sensor_data;  ///< 传感器数据
    PDRTrueData   true_data;    ///< 真实定位数据
} PDRData;

/// @struct PDRTrajectory
/// @brief 行人航迹推算(PDR)的定位数据
typedef struct _PDRTrajectory
{
    double*       time;       ///< 时间戳（单位：秒）
    double*       x;          ///< X轴经度
    double*       y;          ///< Y轴维度
    double*       direction;  ///< 运动方向（单位：度，TODO：0表示方向待定）
    unsigned long length;     ///< 数组长度
    void*         ptr;        ///< 对象指针
} PDRTrajectory;

/// @struct PDRTrajectoryArray
/// @brief 行人航迹推算(PDR)的定位数据数组
typedef struct _PDRTrajectoryArray
{
    PDRTrajectory** array;  ///< 数组指针
    unsigned int    count;  ///< 数组长度
    void*           ptr;    ///< 对象指针
} PDRTrajectoryArray;

/// @enum PDRResult
/// @brief PDR接口返回值定义
typedef enum _PDRResult
{
    PDR_RESULT_ALLOC_FAILED           = -5000,  ///< 内存分配错误
    PDR_RESULT_CALIBRATION_LOAD_ERROR = -4002,  ///< 加载校准文件失败
    PDR_RESULT_CALIBRATION_SAVE_ERROR = -4001,  ///< 保存校准文件失败
    PDR_RESULT_CALIBRATION_ERROR      = -4000,  ///< 传感器校准错误
    PDR_RESULT_COLUMN_INCONSISTENT    = -3001,  ///< 列不一致
    PDR_RESULT_EMPTY_ERROR            = -3000,  ///< 数据为空
    PDR_RESULT_TYPE_MISMATCH          = -2003,  ///< JSON字段类型错误
    PDR_RESULT_MISSING_FIELD          = -2002,  ///< JSON必要字段缺失
    PDR_RESULT_INVALID_ROOT           = -2001,  ///< JSON根对象错误，必须为"{}"包含的JSON对象
    PDR_RESULT_PARSE_ERROR            = -2000,  ///< JSON解析错误，如：非JSON格式
    PDR_RESULT_NOT_DIRECTORY          = -1005,  ///< 指定目录文件不是目录
    PDR_RESULT_DIR_NOT_EXIST          = -1004,  ///< 目录不存在
    PDR_RESULT_READ_FAILED            = -1003,  ///< 读文件失败
    PDR_RESULT_WRITE_FAILED           = -1002,  ///< 写文件失败
    PDR_RESULT_OPEN_FAILED            = -1001,  ///< 打开文件失败
    PDR_RESULT_CREATE_FAILED          = -1000,  ///< 创建文件失败
    PDR_RESULT_UNKNOWN                = -7,     ///< 未知错误
    PDR_RESULT_GENERAL_ERROR          = -6,     ///< 系统错误，如：crt错误
    PDR_RESULT_DEVICE_INIT_ERROR      = -5,     ///< 设备驱动初始化错误
    PDR_RESULT_CALL_ERROR             = -4,     ///< 错误的函数调用
    PDR_RESULT_ALREADY_RUNNING        = -3,     ///< PDR已经启动
    PDR_RESULT_PARAMETER_ERROR        = -2,     ///< 参数错误
    PDR_RESULT_NONE                   = -1,     ///< 未检测到行进
    PDR_RESULT_SUCCESS                = 0,      ///< 成功
} PDRResult;

/// @typedef PDRHandler
/// @brief PDR算法句柄类型
/// @details 该类型为PDR算法实例的不透明指针
typedef void* PDRHandler;

/// @fn int fm_pdr_init_with_file( char* config_dir, char* train_file_path, PDRHandler* handler, PDRTrajectoryArray *trajectories_array )
/// @brief 初始化PDR算法
/// @param config_dir [in] 配置文件路径
/// @param train_file_path [in] PDR训练数据路径，传递NULL表示不进行训练，只进行预测（使用配置项中的模型路径）和读取配置
/// @param handler [out] PDR句柄
/// @param trajectories_array [out] 预测的行人航迹，内部分配多个数据块构成的列表，每个数据块有多条数据，每条数据表示每步的位置信息
/// @return >0: 训练生成位置点数量
///         =0: trajectories传递NULL值并且初始化成功
///         <0: 错误码
int fm_pdr_init_with_file( char* config_dir, char* train_file_path, PDRHandler* handler, PDRTrajectoryArray* trajectories_array );

/// @fn int fm_pdr_start( PDRHandler handler, PDRPoint *start_point, char* raw_data_path )
/// @brief 基于传感器数据，开始导航
/// @param handler [in] PDR句柄
/// @param start_point [in] 起点经纬度数据，不可为空
/// @param pdr_data [in] 传感器数据，用于确定初始行进方向，建议设置3秒以上传感器数据，pdr_data中的真实数据(true_data)可以为NULL
/// @return 无
int fm_pdr_start( PDRHandler handler, PDRPoint* start_point, char* raw_data_path );

/// @fn void fm_pdr_start_with_file( PDRHandler* handler, char *sensor_file_dpath )
/// @brief 基于记录在文件中的传感器数据，开始导航
/// @param handler [in] PDR句柄
/// @param sensor_file_path [in] 记录行进数据目录的路径
/// @return 无
int fm_pdr_start_with_file( PDRHandler handler, char* sensor_file_path );

/// @fn int fm_pdr_predict( PDRHandler handler, PDRTrajectoryArray *trajectories_array )
/// @brief 基于传感器数据，执行行人航迹推算预测
/// @param handler [in] PDR句柄
/// @param trajectories_array [out] 预测的行人航迹，内部分配多个数据块构成的列表，每个数据块有多条数据，每条数据表示每步的位置信息
/// @return >0: 校正后位置点数量
///         =0: trajectories传递NULL值并且推算成功
///         <0: 错误码
int fm_pdr_predict( PDRHandler handler, PDRTrajectoryArray* trajectories_array );

/// @fn void fm_pdr_free_trajectory( PDRTrajectoryArray* trajectories_array )
/// @brief 释放航迹数据
/// @param trajectories_array [in] 预测的行人航迹，内部分配多个数据块构成的列表，每个数据块有多条数据，每条数据表示每步的位置信息
/// @return 无
void fm_pdr_free_trajectory( PDRTrajectoryArray* trajectories_array );

/// @fn int fm_pdr_stop( PDRHandler handler, PDRTrajectoryArray *trajectories_array )
/// @brief 停止导航
/// @param handler [in] PDR句柄
/// @param trajectories_array [in] 预测的行人航迹，内部分配多个数据块构成的列表，每个数据块有多条数据，每条数据表示每步的位置信息
/// @return >0: 校正后位置点数量
///         =0: trajectories传递NULL值并且推算成功
///         <0: 错误码
int fm_pdr_stop( PDRHandler handler, PDRTrajectoryArray* trajectories_array );

/// @fn void fm_pdr_uninit(PDRHandler *handler)
/// @brief 释放PDR算法资源
/// @param handler [in] 句柄指针
/// @return 无
void fm_pdr_uninit( PDRHandler* handler );

/////////////////////////////////////////////////////////////////////////////////////////
// 下面是调试PDR程序可能用到的函数
/////////////////////////////////////////////////////////////////////////////////////////

/// @fn int fm_pdr_get_config(PDRHandler* handler, PDRConfig *config)
/// @brief 开始导航
/// @param handler [in] PDR句柄
/// @param config [out] 传感器数据，用于确定初始行进方向，建议设置3秒以上传感器数据
/// @return 无
int fm_pdr_get_config( PDRHandler handler, PDRConfig* config );

/// @fn int fm_pdr_save_trajectory_data( char* file_path, PDRTrajectoryArray *trajectories_array )
/// @brief 保存行人航迹数据，函数可以重复调用，每次追加写入数据
/// @param file_path [in] 保存文件路径
/// @param trajectories_array [in] 预测的行人航迹，内部分配多个数据块构成的列表，每个数据块有多条数据，每条数据表示每步的位置信息
/// @return 0: 保存成功；!=0: 保存失败
int fm_pdr_save_trajectory_data( char* file_path, PDRTrajectoryArray* trajectories_array );

// TODO:改为私有函数
/// @fn int fm_pdr_read_pdr_data( char* dir_path, PDRData* pdr_data )
/// @brief 读取dir_path目录传感器数据到PDRData结构体中
/// @param dir_path [in] 保存文件路径
/// @param pdr_data [out] PDR数据指针
/// @return 0: 读取成功；!=0: 读取失败
int fm_pdr_read_pdr_data( char* dir_path, PDRData* pdr_data );

// TODO:改为私有函数
/// @fn void fm_pdr_free_pdr_data( PDRData* pdr_data )
/// @brief 释放PDRData结构体中的数据
/// @param pdr_data [in] PDR数据指针
/// @return 无
void fm_pdr_free_pdr_data( PDRData* pdr_data );

// TODO:改为私有函数
/// @fn int fm_pdr_save_pdr_data( char* dir_path, PDRData* pdr_data )
/// @brief 保存传感器数据，函数可以重复调用，每次追加写入数据
/// @param dir_path [in] 保存文件路径
/// @param pdr_data [in] PDR数据指针
/// @return 0: 保存成功；!=0: 保存失败
int fm_pdr_save_pdr_data( char* dir_path, PDRData* pdr_data );

#ifdef __cplusplus
}
#endif

#endif  // __FM_PDR__