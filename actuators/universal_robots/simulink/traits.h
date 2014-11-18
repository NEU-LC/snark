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

namespace snark { namespace ur { namespace robotic_arm {

struct current_positions : public ExtY_Arm_controller_v2_T {};    
    
} } } // namespace snark { namespace ur { namespace robotic_arm {

namespace comma { namespace visiting  {
    
// template <> struct traits< ExtY_Arm_controller_v2_T >
// {
//     template< typename K, typename V > static void visit( const K& k, ExtY_Arm_controller_v2_T& t, V& v )
//     {
//         v.apply( "joint1", t.WP1[0] );
//         v.apply( "joint2", t.WP1[1] );
//         v.apply( "joint3", t.WP1[2] );
//         v.apply( "joint4", t.WP1[3] );
//         v.apply( "joint5", t.WP1[4] );
//         v.apply( "joint6", t.WP1[5] );
//     }
//     template< typename K, typename V > static void visit( const K& k, const ExtY_Arm_controller_v2_T& t, V& v )
//     {
//         v.apply( "joint1", t.WP1[0] );
//         v.apply( "joint2", t.WP1[1] );
//         v.apply( "joint3", t.WP1[2] );
//         v.apply( "joint4", t.WP1[3] );
//         v.apply( "joint5", t.WP1[4] );
//         v.apply( "joint6", t.WP1[5] );
//     }
// };
    
// template <> struct traits< snark::ur::robotic_arm::current_positions >
// {
//     // output in degrees
//     template< typename K, typename V > static void visit( const K& k, const snark::ur::robotic_arm::current_positions& t, V& v )
//     {
//         typedef boost::units::quantity< boost::units::degree::plane_angle > plane_angle_degrees_t;
//         v.apply( "status_code", char(t.arm_status) );
//         v.apply( "joint1", static_cast< plane_angle_degrees_t >( t.arm_position[0] * boost::units::si::radian ).value() );
//         v.apply( "joint2", static_cast< plane_angle_degrees_t >( t.arm_position[1] * boost::units::si::radian ).value() );
//         v.apply( "joint3", static_cast< plane_angle_degrees_t >( t.arm_position[2] * boost::units::si::radian ).value() );
//         v.apply( "joint4", static_cast< plane_angle_degrees_t >( t.arm_position[3] * boost::units::si::radian ).value() );
//         v.apply( "joint5", static_cast< plane_angle_degrees_t >( t.arm_position[4] * boost::units::si::radian ).value() );
//         v.apply( "joint6", static_cast< plane_angle_degrees_t >( t.arm_position[5] * boost::units::si::radian ).value() );
//     }
// };
//     
} }  // namespace comma { namespace visiting  {
    
    


#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_SIMULINK_TRAITS_H