#include "data_file_loader.h"
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
    CFmMergeDirectionStep( const PDRConfig& config, const CFmDataManager& train_data, Eigen::MatrixXd& train_position );
    CFmMergeDirectionStep( const PDRConfig& config );
    ~CFmMergeDirectionStep();

    StartInfo       start( const CFmDataManager& start_data );
    Eigen::MatrixXd merge_dir_step( StartInfo& start_info, const CFmDataManager& process_data );
private:
    const PDRConfig& m_config;
    double           m_valid_peak_value;
    LinearModel      m_model;
    double           m_mean_step;

    CFmStepPredictor      m_step_predictor;
    CFmDirectionPredictor m_direction_predictor;
};