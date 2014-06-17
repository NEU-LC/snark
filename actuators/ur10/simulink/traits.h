#ifndef SNARK_ACTUATORS_ROBOT_ARM_SIMULINK_TRAITS_H
#define SNARK_ACTUATORS_ROBOT_ARM_SIMULINK_TRAITS_H
#include <string>
#include <boost/lexical_cast.hpp>
#include <comma/visiting/traits.h>
#include <comma/base/types.h>
#include <comma/base/exception.h>
extern "C" {
    #include "Arm_Controller.h"
}

namespace comma { namespace visiting  {
    
template <> struct traits< ExtY_Arm_Controller_T >
{
    template< typename K, typename V > static void visit( const K& k, ExtY_Arm_Controller_T& t, V& v )
    {
        v.apply( "joint1", t.joint_angle_vector[0] );
        v.apply( "joint2", t.joint_angle_vector[1] );
        v.apply( "joint3", t.joint_angle_vector[2] );
        v.apply( "joint4", t.joint_angle_vector[3] );
        v.apply( "joint5", t.joint_angle_vector[4] );
        v.apply( "joint6", t.joint_angle_vector[5] );
    }
    template< typename K, typename V > static void visit( const K& k, const ExtY_Arm_Controller_T& t, V& v )
    {
        v.apply( "joint1", t.joint_angle_vector[0] );
        v.apply( "joint2", t.joint_angle_vector[1] );
        v.apply( "joint3", t.joint_angle_vector[2] );
        v.apply( "joint4", t.joint_angle_vector[3] );
        v.apply( "joint5", t.joint_angle_vector[4] );
        v.apply( "joint6", t.joint_angle_vector[5] );
    }
};
    
} } // namespace comma { namespace visiting  {
    
    


#endif // SNARK_ACTUATORS_ROBOT_ARM_SIMULINK_TRAITS_H