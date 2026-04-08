/**
 * Fast map matching.
 *
 * fmm command line program configuration
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FMM_FMM_APP_CONFIG_HPP_
#define FMM_FMM_APP_CONFIG_HPP_

#include "fmm/fmm-api.hpp"
#include "fmm_algorithm.h"


/**
 * Configuration class of fmm command line program
 */
class FMMConfig
{
public:
    /**
     * Constructor of the configuration from xml file name.
     *
     * @param file xml file name
     */
    FMMConfig( const std::string& file );

    /**
     * Load configuration from an XML file
     * @param file xml file name
     */
    void load_xml( const std::string& file );
    /**
     * Validate the configuration
     * @return true if valid
     */
    bool validate() const;
    /**
     * Print configuration data
     */
    void print() const;
    /**
     * Print help information
     */
    static void print_help();

    FMM::CONFIG::NetworkConfig network_config; /**< Network data configuraiton */
    FMM::CONFIG::GPSConfig     gps_config;     /**< GPS data configuraiton */
    FMM::CONFIG::ResultConfig  result_config;  /**< Result configuraiton */
    FastMapMatchConfig    fmm_config;     /**< Map matching configuraiton */
    std::string           ubodt_file;     /**< UBODT file name */
};

#endif  // FMM_SRC_MM_FMM_FMM_APP_CONFIG_HPP_
