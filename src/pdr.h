#include "merge_direction_step.h"

class CFmPDR
{
public:
    CFmPDR( const PDRConfig& config, const CFmDataManager& train_data, Eigen::MatrixXd& train_position );
    CFmPDR( const PDRConfig& config );
    ~CFmPDR();

    StartInfo                  start( double x0, double y0, const CFmDataManager& start_data );
    MatrixXd pdr( StartInfo& start_info, const CFmDataManager& process_data );
private:
    CFmMergeDirectionStep m_merge_direction_step;

    size_t find_interval( double t, const Eigen::MatrixXd& trajectory ) const;
    MatrixXd linear_interpolation( const VectorXd& target_times, const MatrixXd& trajectory );
};