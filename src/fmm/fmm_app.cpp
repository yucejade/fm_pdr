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

    while ( reader.get_reader()->has_next_trajectory() )
    {
        FMM::CORE::Trajectory trajectory   = reader.get_reader()->read_next_trajectory();
        MM::MatchResult       result       = mm_model.match_traj( trajectory, fmm_config );

        if ( ! result.cpath.empty() && ! result.opt_candidate_path.empty() )
        {
            // 取最后一个点的匹配结果
            const auto& last_matched = result.opt_candidate_path.back();
            double      matched_x    = boost::geometry::get< 0 >( last_matched.c.point );
            double      matched_y    = boost::geometry::get< 1 >( last_matched.c.point );

            // 时间戳和方向使用 traj_matrix 中最后一个点的原始数据
            int    last_row   = traj_matrix.rows() - 1;
            double timestamp  = traj_matrix( last_row, 0 );
            double direction  = traj_matrix.cols() > 3 ? traj_matrix( last_row, 3 ) : 0.0;

            // 输出格式: [timestamp, matched_x, matched_y, direction]
            Eigen::MatrixXd out( 1, 4 );
            out << timestamp, matched_x, matched_y, direction;
            return out;
        }
    }

    // 匹配失败，返回空矩阵
    return Eigen::MatrixXd();
};
