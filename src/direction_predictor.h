#include "data_file_loader.h"
#include "fm_pdr.h"
#include "Iir.h"

typedef struct _StartInfo
{
    double e0_x;        ///< 初始东向量x
    double e0_y;        ///< 初始东向量y
    double e0_z;        ///< 初始东向量z
    double direction0;  ///< 初始行进方向
    double x0;          ///< 手动设置的初始x(经度)位置
    double y0;          ///< 手动设置的初始y(维度)位置
    double last_x;      ///< 每批次数据的最后x(经度)位置
    double last_y;      ///< 每批次数据的最后y(纬度)位置
} StartInfo;

class CFmDirectionPredictor
{
public:
    CFmDirectionPredictor( const PDRConfig& config );
    ~CFmDirectionPredictor();

    StartInfo       start( const CFmDataManager& start_data, const int least_point );
    Eigen::VectorXd predict_direction( const StartInfo& start_info, const CFmDataManager& process_data );
private:
    const PDRConfig& m_config;
    Iir::Butterworth::LowPass< 2, Iir::DirectFormII > m_f;

    VectorXd        filtfilt( Iir::Butterworth::LowPass< 2, Iir::DirectFormII >& filter, const VectorXd& input );
    void            butterworth_filter( const CFmDataManager& data, MatrixXd& mag, MatrixXd& grv );
    Eigen::MatrixXd calc_east_vector( const MatrixXd& mag, const MatrixXd& grv, const int& rows );
};