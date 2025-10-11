#pragma once
#include "Fusion.h"
#include "fm_pdr.h"
#include <eigen3/Eigen/Dense>
#include <rapidcsv.h>

using namespace std;
using namespace Eigen;

typedef enum _DataType
{
    DATA_TYPE_BUFFER,  ///< 训练数据
    DATA_TYPE_FILE,    ///< PDR数据
} DataType;

typedef enum _PDRDataField
{
    PDR_DATA_FIELD_TIME = 0,
    PDR_DATA_FIELD_ACC_X,
    PDR_DATA_FIELD_ACC_Y,
    PDR_DATA_FIELD_ACC_Z,
    PDR_DATA_FIELD_ACC_MAG,
    PDR_DATA_FIELD_LACC_X,
    PDR_DATA_FIELD_LACC_Y,
    PDR_DATA_FIELD_LACC_Z,
    PDR_DATA_FIELD_LACC_MAG,
    PDR_DATA_FIELD_GYR_X,
    PDR_DATA_FIELD_GYR_Y,
    PDR_DATA_FIELD_GYR_Z,
    PDR_DATA_FIELD_GYR_MAG,
    PDR_DATA_FIELD_MAG_X,
    PDR_DATA_FIELD_MAG_Y,
    PDR_DATA_FIELD_MAG_Z,
    PDR_DATA_FIELD_MAG_MAG,
    PDR_DATA_FIELD_GRV_X,
    PDR_DATA_FIELD_GRV_Y,
    PDR_DATA_FIELD_GRV_Z,
    PDR_DATA_FIELD_GRV_MAG,
    PDR_DATA_FIELD_MAX
} PDRDataField;

typedef enum _TrueDataField
{
    TRUE_DATA_FIELD_TIME,
    TRUE_DATA_FIELD_LATITUDE,
    TRUE_DATA_FIELD_LONGITUDE,
    TRUE_DATA_FIELD_HEIGHT,
    TRUE_DATA_FIELD_VELOCITY,
    TRUE_DATA_FIELD_DIRECTION,
    TRUE_DATA_FIELD_HORIZONTAL_ACCURACY,
    TRUE_DATA_FIELD_VERTICAL_ACCURACY,
    TRUE_DATA_FIELD_X,
    TRUE_DATA_FIELD_Y,
    TRUE_DATA_FIELD_MAX
} TrueDataField;

class CFmDataManager
{
public:
    CFmDataManager();
    CFmDataManager( const PDRConfig& config, DataType type, size_t train_data_size );
    virtual ~CFmDataManager();

    void set_location_output( const Eigen::MatrixXd& trajectory );
    void eval_model( const Eigen::MatrixXd& trajectory ) const;

    inline DataType get_data_type() const
    {
        return m_data_type;
    }

    inline bool have_train_data() const
    {
        return m_train_data_size > 0;
    }

    inline bool have_location_true() const
    {
        return m_have_location_true;
    }

    inline bool have_line_accelererometer_data() const
    {
        return m_have_line_accelererometer;
    }

    inline size_t get_pdr_data_size() const
    {
        return m_time.size();
    }

    inline const VectorXd& get_pdr_data( PDRDataField field ) const
    {
        switch ( field )
        {
            case PDR_DATA_FIELD_TIME:
                return m_time;
            case PDR_DATA_FIELD_ACC_X:
                return m_a_x;
            case PDR_DATA_FIELD_ACC_Y:
                return m_a_y;
            case PDR_DATA_FIELD_ACC_Z:
                return m_a_z;
            case PDR_DATA_FIELD_ACC_MAG:
                return m_a_mag;
            case PDR_DATA_FIELD_LACC_X:
                return m_la_x;
            case PDR_DATA_FIELD_LACC_Y:
                return m_la_y;
            case PDR_DATA_FIELD_LACC_Z:
                return m_la_z;
            case PDR_DATA_FIELD_LACC_MAG:
                return m_la_mag;
            case PDR_DATA_FIELD_GYR_X:
                return m_gs_x;
            case PDR_DATA_FIELD_GYR_Y:
                return m_gs_y;
            case PDR_DATA_FIELD_GYR_Z:
                return m_gs_z;
            case PDR_DATA_FIELD_GYR_MAG:
                return m_gs_mag;
            case PDR_DATA_FIELD_MAG_X:
                return m_m_x;
            case PDR_DATA_FIELD_MAG_Y:
                return m_m_y;
            case PDR_DATA_FIELD_MAG_Z:
                return m_m_z;
            case PDR_DATA_FIELD_MAG_MAG:
                return m_m_mag;
            case PDR_DATA_FIELD_GRV_X:
                return m_g_x;
            case PDR_DATA_FIELD_GRV_Y:
                return m_g_y;
            case PDR_DATA_FIELD_GRV_Z:
                return m_g_z;
            case PDR_DATA_FIELD_GRV_MAG:
                return m_g_mag;
            default:
                throw std::out_of_range( "PDRDataField out of range" );
        }
    }

    inline size_t get_true_data_size() const
    {
        return m_time_location_true.size();
    }

    inline const VectorXd& get_true_data( TrueDataField field ) const
    {
        switch ( field )
        {
            case TRUE_DATA_FIELD_TIME:
                return m_time_location_true;
            case TRUE_DATA_FIELD_LATITUDE:
                return m_latitude_true;
            case TRUE_DATA_FIELD_LONGITUDE:
                return m_longitude_true;
            case TRUE_DATA_FIELD_HEIGHT:
                return m_height_true;
            case TRUE_DATA_FIELD_VELOCITY:
                return m_velocity_true;
            case TRUE_DATA_FIELD_DIRECTION:
                return m_direction_true;
            case TRUE_DATA_FIELD_HORIZONTAL_ACCURACY:
                return m_horizontal_accuracy_true;
            case TRUE_DATA_FIELD_VERTICAL_ACCURACY:
                return m_vertical_accuracy_true;
            case TRUE_DATA_FIELD_X:
                return m_x_true;
            case TRUE_DATA_FIELD_Y:
                return m_y_true;
            default:
                throw std::out_of_range( "TrueDataField out of range" );
        }
    }

    inline size_t get_train_data_size() const
    {
        return m_train_data_size;
    }

    inline const VectorXd& get_train_data( TrueDataField field ) const
    {
        if ( m_train_data_size == 0 )
            throw std::invalid_argument( "No training data available" );

        switch ( field )
        {
            case TRUE_DATA_FIELD_TIME:
                return m_time_location;
            case TRUE_DATA_FIELD_LATITUDE:
                return m_latitude;
            case TRUE_DATA_FIELD_LONGITUDE:
                return m_longitude;
            case TRUE_DATA_FIELD_HEIGHT:
                return m_height;
            case TRUE_DATA_FIELD_VELOCITY:
                return m_velocity;
            case TRUE_DATA_FIELD_DIRECTION:
                return m_direction;
            case TRUE_DATA_FIELD_HORIZONTAL_ACCURACY:
                return m_horizontal_accuracy;
            case TRUE_DATA_FIELD_VERTICAL_ACCURACY:
                return m_vertical_accuracy;
            case TRUE_DATA_FIELD_X:
                return m_x;
            case TRUE_DATA_FIELD_Y:
                return m_y;
            default:
                throw std::out_of_range( "TrueDataField out of range" );
        }
    }
protected:
    static constexpr double kK = 1e5;
    const PDRConfig*        m_config;
    DataType                m_data_type;
    size_t                  m_train_data_size;  // 对于切片后得到对象，该数据为0，表示包含训练数据的对象不允许切片

    // Define calibration (replace with actual calibration data if available)
    FusionMatrix m_gyroscopeMisalignment     = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector m_gyroscopeSensitivity      = { 1.0f, 1.0f, 1.0f };
    FusionVector m_gyroscopeOffset           = { 0.0f, 0.0f, 0.0f };
    FusionMatrix m_accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector m_accelerometerSensitivity  = { 1.0f, 1.0f, 1.0f };
    FusionVector m_accelerometerOffset       = { 0.0f, 0.0f, 0.0f };
    FusionMatrix m_softIronMatrix            = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector m_hardIronOffset            = { 0.0f, 0.0f, 0.0f };

    // Initialise algorithms
    FusionOffset m_offset;
    FusionAhrs   m_ahrs;

    bool m_have_location_true        = false;  // 如果存在真实位置数据，则可以训练和评估，否则只能预测
    bool m_have_line_accelererometer = false;  // 如果存在线性加速度计数据，则使用线性加速度计计算重力加速度，否则，使用加速度计来计算重力加速度

    pair< double, double > m_origin;  // 当存在训练数据时，以最后一条训练数据作为原点；当不存在训练数据时，以第一个点作为原点

    double m_slice_start = 0.0;  // private
    double m_slice_end   = 0.0;  // private

    VectorXd m_time;    // public
    VectorXd m_a_x;     // private
    VectorXd m_a_y;     // private
    VectorXd m_a_z;     // private
    VectorXd m_a_mag;   // public 计算步长
    VectorXd m_la_x;    // private
    VectorXd m_la_y;    // private
    VectorXd m_la_z;    // private
    VectorXd m_la_mag;  // private and delete
    VectorXd m_gs_x;    // private
    VectorXd m_gs_y;    // private
    VectorXd m_gs_z;    // private
    VectorXd m_gs_mag;  // private
    VectorXd m_m_x;     // public 获取水平方向
    VectorXd m_m_y;     // public 获取水平方向
    VectorXd m_m_z;     // public 获取东向
    VectorXd m_m_mag;   // private
    VectorXd m_g_x;     // public 获取东向
    VectorXd m_g_y;     // public 获取东向
    VectorXd m_g_z;     // public 获取东向
    VectorXd m_g_mag;   // private

    VectorXd m_time_location;        // public
    VectorXd m_latitude;             // private
    VectorXd m_longitude;            // private
    VectorXd m_height;               // private
    VectorXd m_velocity;             // private
    VectorXd m_direction;            // public 原始代码需要使用，做方向判断
    VectorXd m_horizontal_accuracy;  // private
    VectorXd m_vertical_accuracy;    // private
    VectorXd m_x;                    // public 训练结果用
    VectorXd m_y;                    // public 训练结果用

    VectorXd m_time_location_true;        // public
    VectorXd m_latitude_true;             // private
    VectorXd m_longitude_true;            // private
    VectorXd m_height_true;               // private
    VectorXd m_velocity_true;             // private
    VectorXd m_direction_true;            // private
    VectorXd m_horizontal_accuracy_true;  // private
    VectorXd m_vertical_accuracy_true;    // private
    VectorXd m_x_true;                    // private
    VectorXd m_y_true;                    // private
protected:
    bool            save_to_csv( const MatrixXd& matrix, const string& filename, const vector< string >& col_names );
    Eigen::MatrixXd get_gravity_with_ahrs( Eigen::MatrixXd& accelerometer, Eigen::MatrixXd& gyroscope, Eigen::MatrixXd& magnetometer );
private:
    double get_dir_error( const Eigen::MatrixXd& trajectory ) const;
    double get_dir_ratio( const Eigen::MatrixXd& trajectory, double diff = 15.0 ) const;
    double get_dist_error( const Eigen::MatrixXd& trajectory ) const;
};