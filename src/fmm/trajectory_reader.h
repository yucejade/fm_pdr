#ifndef FMM_TRAJECTORY_READER_HPP
#define FMM_TRAJECTORY_READER_HPP

#include "fmm/fmm-api.hpp"
#include <Eigen/Dense>
#include <memory>


/**
 * Reader for Eigen matrix trajectory data
 *
 * Matrix format:
 * col0: timestamp
 * col1: x
 * col2: y
 * col3: direction (optional, not used currently)
 */
class EigenMatrixReader : public FMM::IO::ITrajectoryReader
{
public:
    EigenMatrixReader( const Eigen::MatrixXd& traj_matrix, int traj_id = 0 );

    FMM::CORE::Trajectory read_next_trajectory() override;
    bool                  has_next_trajectory() override;
    bool                  has_timestamp() override;
    void                  close() override;

    void reset_cursor();
private:
    Eigen::MatrixXd matrix;
    int             cursor = 0;
    int             trid   = 0;
};

/**
 * Unified trajectory reader wrapper
 * Supports both file-based and Eigen-based input
 */
class TrajectoryReader
{
public:
    // Eigen-based
    TrajectoryReader( const Eigen::MatrixXd& matrix );

    std::shared_ptr< FMM::IO::ITrajectoryReader > get_reader();
private:
    std::shared_ptr< FMM::IO::ITrajectoryReader > reader;
};

#endif  // FMM_TRAJECTORY_READER_HPP