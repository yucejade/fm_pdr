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
    char *model_name;             ///< 模型名
    char *model_file_name;        ///< 保存/加载模型文件名
    int clean_data;               ///< 训练时去除的头部不稳定数据
    int move_average;             ///< 移动平均数N
    int min_distance;             ///< 假定包含峰值的最小点数间隔
    double distance_frac_step;    ///< 表示每隔几步计算一次步长sigma、f
    double optimized_mode_ratio;  ///< 计算初始方向时两种方案所占百分比(单点方向和使用最小化平均)
    double butter_wn;             ///< 巴特沃斯滤波归一化频率
    int least_start_point;        ///< 传给start函数的最少点数
} PDRConfig;

typedef struct _StartInfo
{
    double e0_x;
    double e0_y;
    double e0_z;
    double direction0;
    double last_x;
    double last_y;
} StartInfo;

/// @struct PDRPosition
/// @brief 行人航位推算(PDR)的定位数据结构
typedef struct _PDRPosition
{
    double time;        ///< 时间戳（单位：秒）
    double x;           ///< X轴经度
    double y;           ///< Y轴维度
    double direction;   ///< 运动方向（单位：度，TODO：0表示方向待定）
} PDRPosition;

typedef enum {
    PDR_SUCCESS = 0,
    PDR_INVALID_INPUT = -1,
    PDR_SENSOR_ERROR = -2,
    PDR_CALIB_FAILED = -3
} PDRStatus;

/// @fn int fm_pdr_init(void* train_data, PDRPosition** positions)
/// @brief 初始化PDR算法
/// @param true_data [in] 真实数据指针（如GPS轨迹，TODO:类型待定）
/// @param train_data [in] 训练数据指针（传感器数据集，TODO:类型待定）
/// @param positions [in] 结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量\n
///          0: positions为NULL时的成功返回\n
///         <0: 错误码（如锚点无效）
int fm_pdr_init(void *true_data, void* train_data, PDRPosition** positions); //TODO: true_data和train_data区别待定

/// @fn int fm_pdr_predict(void* process_data, PDRPosition** positions)
/// @brief 执行行人航位推算预测
/// @param process_data [in] 待处理数据指针（TODO:类型待定）
/// @param positions [out] 预测结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量\n
///          0: positions为NULL时的成功返回\n
///         <0: 错误码（如锚点无效）
int fm_pdr_predict(void* process_data, PDRPosition** positions); //TODO:应修改为事件回调形式，以每一步为一个事件

/// @fn int fm_pdr_calibration(void *anchor_point, PDRPosition** positions);
/// @brief 根据矫正点重新对行人航位路线进行校准
/// @param process_data [in] 待处理数据指针（TODO:类型待定）
/// @param positions [out] 预测结果位置数组，需预分配内存（可为NULL）
/// @return >0: 校正后位置点数量\n
///          0: positions为NULL时的成功返回\n
///         <0: 错误码（如锚点无效）
int fm_pdr_calibration(void *anchor_point, PDRPosition** positions);

/// @fn void fm_pdr_uninit()
/// @brief 释放PDR算法资源
void fm_pdr_uninit();

#ifdef __cplusplus
}
#endif

#endif // __FM_PDR__