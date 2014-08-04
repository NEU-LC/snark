#include "transforms.h"

namespace snark { namespace ur { namespace robotic_arm { namespace ur5 {
    
    
void tcp_transform( const boost::array< plane_angle_t, 6 >& joint_angles, boost::array< double, 6 >& position )
{
    tcp_transform_initialize();
    tcp_transform_U.Joint1 = joint_angles[0].value();
    tcp_transform_U.Joint2 = joint_angles[1].value();
    tcp_transform_U.Joint3 = joint_angles[2].value();
    tcp_transform_U.Joint4 = joint_angles[3].value();
    tcp_transform_U.Joint5 = joint_angles[4].value();
    tcp_transform_U.Joint6 = joint_angles[5].value();
    
    tcp_transform_step();
    
    position[0] = tcp_transform_Y.x;
    position[1] = tcp_transform_Y.y;
    position[2] = tcp_transform_Y.z;
    position[3] = tcp_transform_Y.Pan;
    position[4] = tcp_transform_Y.Tilt;
    position[5] = tcp_transform_Y.Roll;
    
    tcp_transform_terminate();
}
    

} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace ur5 {

