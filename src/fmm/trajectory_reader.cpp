#include "trajectory_reader.h"

using namespace FMM;
using namespace FMM::IO;
using namespace FMM::CORE;

//
// ==========================
// EigenMatrixReader
// ==========================
//

EigenMatrixReader::EigenMatrixReader( const Eigen::MatrixXd& traj_matrix, int traj_id ) : matrix( traj_matrix ), cursor( 0 ), trid( traj_id )
{
    SPDLOG_INFO( "EigenMatrixReader initialized with {} points", matrix.rows() );
}

bool EigenMatrixReader::has_next_trajectory()
{
    return cursor == 0;
}

bool EigenMatrixReader::has_timestamp()
{
    return true;
}

Trajectory EigenMatrixReader::read_next_trajectory()
{
    LineString            geom;
    std::vector< double > timestamps;

    for ( int i = 0; i < matrix.rows(); ++i )
    {
        double t = matrix( i, 0 );
        double x = matrix( i, 1 );
        double y = matrix( i, 2 );

        geom.add_point( x, y );
        timestamps.push_back( t );
    }

    cursor = 1;

    return Trajectory{ trid, geom, timestamps };
}

void EigenMatrixReader::reset_cursor()
{
    cursor = 0;
}

void EigenMatrixReader::close()
{
    // no-op
}

//
// ==========================
// TrajectoryReader
// ==========================
//

TrajectoryReader::TrajectoryReader( const Eigen::MatrixXd& matrix )
{
    reader = std::make_shared< EigenMatrixReader >( matrix );
}

std::shared_ptr< ITrajectoryReader > TrajectoryReader::get_reader()
{
    return reader;
}