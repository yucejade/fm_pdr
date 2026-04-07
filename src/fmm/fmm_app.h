/**
 * Fast map matching.
 *
 * fmm command line program
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FMM_FMM_APP_H_
#define FMM_FMM_APP_H_

#include "fmm_config.h"
#include <Eigen/Dense>

/**
 * Class of fmm command line program
 */
class FMMApp
{
public:
    /**
     * Create FMMApp from configuration data
     * @param config Configuration of the FMMApp defining network, graph
     * and UBODT.
     */
    FMMApp( const FMMConfig& config ) : config_( config ), network_( config_.network_config ), ng_( network_ ), ubodt_( UBODT::read_ubodt_file( config_.ubodt_file ) ) {};
    /**
     * Match the fmm program
     */
    Eigen::MatrixXd  match( const Eigen::MatrixXd& traj_matrix );
private:
    const FMMConfig&         config_;
    NETWORK::Network         network_;
    NETWORK::NetworkGraph    ng_;
    std::shared_ptr< UBODT > ubodt_;
};

#endif
