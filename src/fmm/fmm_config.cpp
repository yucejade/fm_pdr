//
// Created by Can Yang on 2020/4/1.
//

#include "fmm_config.h"

using namespace FMM::CORE;
using namespace FMM::NETWORK;
using namespace FMM::CONFIG;
using namespace FMM::MM;

FMMConfig::FMMConfig( const std::string& file )
{
    std::string configfile( file );
    if ( UTIL::check_file_extension( configfile, "xml,XML" ) )
        load_xml( configfile );
};

void FMMConfig::load_xml( const std::string& file )
{
    // Create empty property tree object
    boost::property_tree::ptree tree;
    boost::property_tree::read_xml( file, tree );
    network_config = NetworkConfig::load_from_xml( tree );
    // gps_config = GPSConfig::load_from_xml(tree);
    // result_config = CONFIG::ResultConfig::load_from_xml(tree);
    fmm_config = FastMapMatchConfig::load_from_xml( tree );
    // UBODT
    ubodt_file = tree.get< std::string >( "config.input.ubodt.file" );
};

bool FMMConfig::validate() const
{
    // if (!gps_config.validate()) {
    //   return false;
    // }
    // if (!result_config.validate()) {
    //   return false;
    // }
    if ( ! network_config.validate() )
    {
        return false;
    }
    if ( ! fmm_config.validate() )
    {
        return false;
    }
    if ( ! UTIL::file_exists( ubodt_file ) )
    {
        return false;
    }
    return true;
};
