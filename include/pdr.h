#include "merge_direction_step.h"

class CFmPDR
{
public:
    CFmPDR(const PDRConfig &config, const CFmDataLoader &train_data, std::vector<PDRPosition> &train_position);
    CFmPDR(const PDRConfig &config);
    ~CFmPDR();

    StartInfo start(const CFmDataLoader &start_data);
    std::vector<PDRPosition> pdr(StartInfo &start_info, const CFmDataLoader &process_data);
    void eval_model(const CFmDataLoader &data);

private:
    CFmMergeDirectionStep m_merge_direction_step;

    size_t find_interval(double t, const std::vector<PDRPosition> &trajectory) const;
    const std::vector<PDRPosition> linear_interpolation(const Eigen::VectorXd &target_times,
                                                        const std::vector<PDRPosition> &trajectory);

    // TODO:OLD
public:
    void pdr(CFmDataLoader &test_case,
             const std::string &model_name = "SVR",
             double distance_frac_step = 4.0,
             int clean_data = 4,
             double optimized_mode_ratio = 0.95,
             double butter_Wn = 0.0035);

private:
    Eigen::VectorXd linear_interpolation(const Eigen::VectorXd &time,
                                         const Eigen::VectorXd &time_data,
                                         const Eigen::VectorXd &data);
};