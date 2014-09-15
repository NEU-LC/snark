#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H
#include <comma/base/types.h>
#include <boost/concept_check.hpp>
#include <vector>
#include "units.h"

namespace snark { namespace ur { namespace robotic_arm { 
    
static const unsigned char joints_num = 6;

struct continuum_t 
{
    struct scan_type
    {
    	plane_angle_degrees_t min;
    	plane_angle_degrees_t max;
    };

    typedef boost::array< plane_angle_t, joints_num > arm_position_t;
    // vector of plane_angle_degrees_t does not work with boost::ptree
    arm_position_t home_position; // position of six joints
    arm_position_t giraffe_position; // position of six joints
    std::string work_directory;
    scan_type scan;

    bool operator==( const continuum_t& rhs ) const {
    	return ( home_position == rhs.home_position && work_directory == rhs.work_directory );
    }


};
    
struct config {
	continuum_t continuum;
};
    
} }  } //namespace snark { namespace ur { namespace robotic_arm { 

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_CONFIG_H