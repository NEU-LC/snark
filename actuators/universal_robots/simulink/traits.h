#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_SIMULINK_TRAITS_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_SIMULINK_TRAITS_H
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <comma/visiting/traits.h>
#include <comma/base/types.h>
#include <comma/base/exception.h>
extern "C" {
    #include "Arm_controller_v2.h"
}

namespace snark { namespace ur {

struct current_positions : public ExtY_Arm_controller_v2_T {};    
    
} } // namespace snark { namespace ur { 

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_SIMULINK_TRAITS_H