#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H
#include <comma/base/types.h>
#include <boost/concept_check.hpp>
#include <vector>
#include "units.h"

namespace snark { namespace ur { namespace robotic_arm { 
    
struct config {
    config() : home_position( 6 ) {} // position of six joints
    // vector of plane_angle_degrees_t does not work with boost::ptree
    std::vector< double > home_position;	

    bool operator==( const config& rhs ) const { return home_position == rhs.home_position; }
};
    
} }  } //namespace snark { namespace ur { namespace robotic_arm { 

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H