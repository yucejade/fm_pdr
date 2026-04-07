//
// Created by Can Yang on 2020/4/1.
//

#include "fmm_app.h"
#include "io/gps_reader.hpp"
#include "io/mm_writer.hpp"
#include "trajectory_reader.h"

using namespace FMM;
using namespace FMM::CORE;
using namespace FMM::NETWORK;
using namespace FMM::MM;

Eigen::MatrixXd FMMApp::match( const Eigen::MatrixXd& traj_matrix )
{
    auto                      start_time = UTIL::get_current_time();
    FastMapMatch              mm_model( network_, ng_, ubodt_ );
    const FastMapMatchConfig& fmm_config = config_.fmm_config;
    TrajectoryReader          reader( traj_matrix );
    // IO::GPSReader             reader( config_.gps_config );
    // IO::CSVMatchResultWriter  writer( config_.result_config.file, config_.result_config.output_config );
    // Start map matching
    int points_matched = 0;
    int total_points   = 0;

    while ( reader.get_reader()->has_next_trajectory() )
    {
        FMM::CORE::Trajectory trajectory   = reader.get_reader()->read_next_trajectory();
        int                   points_in_tr = trajectory.geom.get_num_points();
        MM::MatchResult       result       = mm_model.match_traj( trajectory, fmm_config );
        // writer.write_result( trajectory, result );
        if ( ! result.cpath.empty() )
        {
            points_matched += points_in_tr;
        }
        total_points += points_in_tr;
    }

    //TODO:build a matrix to return, currently just return an empty matrix
    return Eigen::Matrix();
};
