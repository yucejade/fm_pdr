#include <eigen3/Eigen/Dense>
#include <dlib/mlp.h>
#include <dlib/svm.h>
#include <dlib/statistics.h>
#include "data_loader.h"

using FeatureMatrix = dlib::matrix<double, 2, 1>;
using LinearModel = dlib::decision_function<dlib::linear_kernel<FeatureMatrix>>;

using AnyModel = dlib::any_decision_function<FeatureMatrix>;

class CFmStepPredictor
{
public:
    CFmStepPredictor(const CFmDataLoader &train_data, int clean_start = 0, int clean_end = 0);
    CFmStepPredictor();
    ~CFmStepPredictor();

    Eigen::VectorXi find_real_peak_indices(const Eigen::VectorXd &data,
                                           int range,
                                           int min_distance,
                                           Eigen::VectorXd &filtered_accel_data,
                                           double &valid_peak_value,
                                           bool is_train = false);
    FeatureMatrix calculate_features(const CFmDataLoader &data,
                                     const Eigen::VectorXi &real_peak_indices,
                                     const Eigen::VectorXd &filtered_accel_data,
                                     int start_step_index,
                                     int end_step_index);
    double step_process_mean(int move_average,
                             int min_distance,
                             const std::string &save_model_name,
                             double &valid_peak_value);
    LinearModel step_process_regression(const std::string &model_str,
                                        int move_average,
                                        int min_distance,
                                        size_t distance_frac_step,
                                        const std::string &save_model_name,
                                        double &valid_peak_value,
                                        bool write_log = false);

    void save_model(const LinearModel &model, const double &valid_peak_value, const std::string &filename); // TODO:使用AnyModel会引起序列化错误
    void save_model(const double &mean_model, const double &valid_peak_value, const std::string &filename);
    void load_model(const std::string &filename, LinearModel &model, double &valid_peak_value); // TODO:使用AnyModel会引起序列化错误
    void load_model(const std::string &filename, double &mean_model, double &valid_peak_value);

private:
    Eigen::VectorXd filter(int range, const Eigen::VectorXd &data);
    Eigen::VectorXi find_peaks(const Eigen::VectorXd &data, int min_distance);
    double compute_variance(const Eigen::VectorXd &data, int start_idx, int end_idx);
    FeatureMatrix calculate_features(const Eigen::VectorXi &real_peak_indices,
                                     const Eigen::VectorXd &filtered_accel_data,
                                     int start_step_index,
                                     int end_step_index);
    LinearModel select_model(const std::string &model_str,
                             std::vector<FeatureMatrix> &x,
                             std::vector<double> &y);

private:
    CFmDataLoader m_train_data;

// TODO:OLD
public:
};