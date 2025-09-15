#include "data_loader.h"
#include "direction_predictor.h"
#include "fm_pdr.h"
#include "step_predictor.h"

struct MergeResult
{
    Eigen::MatrixXd trajectory;
    Eigen::VectorXd direction_pred;
};

class CFmMergeDirectionStep
{
public:
    CFmMergeDirectionStep( const PDRConfig& config, const CFmDataLoader& train_data, std::vector< PDRPosition >& train_position );
    CFmMergeDirectionStep( const PDRConfig& config );
    ~CFmMergeDirectionStep();

    StartInfo                  start( const CFmDataLoader& start_data );
    std::vector< PDRPosition > merge_dir_step( StartInfo& start_info, const CFmDataLoader& process_data );
private:
    const PDRConfig& m_config;
    double           m_valid_peak_value;
    LinearModel      m_model;
    double           m_mean_step;

    CFmStepPredictor      m_step_predictor;
    CFmDirectionPredictor m_direction_predictor;

    // TODO:OLD
public:
    MergeResult merge_dir_step( const CFmDataLoader& test_case, const std::string& model_name = "SVR", double distance_frac_step = 5.0, int clean_data = 5, double optimized_mode_ratio = 0.1, double butter_Wn = 0.005 );
};