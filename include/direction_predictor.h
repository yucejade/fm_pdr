#include "data_loader.h"
#include "DspFilters/Butterworth.h"
#include "iir/Iir.h"
#include "fm_pdr.h"

class CFmDirectionPredictor
{
public:
    CFmDirectionPredictor(double butter_Wn);
    ~CFmDirectionPredictor();

    StartInfo start(const CFmDataLoader &start_data, const int least_point);
    Eigen::VectorXd predict_direction(const StartInfo &start_info, const CFmDataLoader &process_data);

private:
    Iir::Butterworth::LowPass<2, Iir::DirectFormII> m_f;

    VectorXd filtfilt(Iir::Butterworth::LowPass<2, Iir::DirectFormII> &filter, const VectorXd &input);
    void butterworth_filter(const CFmDataLoader &data, MatrixXd &mag, MatrixXd &grv);
    Eigen::MatrixXd calc_east_vector(const MatrixXd &mag, const MatrixXd &grv, const int &rows);

//TODO:OLD
public:
    double direction_diff(double a, double b);
    VectorXd direction_diff(const VectorXd &a, const VectorXd &b);
    VectorXd direction_mix(const VectorXd &a, const VectorXd &b, double ratio);

    VectorXd predict_direction(const CFmDataLoader &data_loader,
                               double optimized_mode_ratio = 0.4,
                               const int butter_N = 2,
                               double butter_Wn = 0.005);
};