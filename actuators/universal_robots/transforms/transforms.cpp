#include "transforms.h"

namespace snark { namespace ur { namespace ur5 {

void tcp_transform( const boost::array< plane_angle_t, 6 >& joint_angles, snark::applications::position& position )
{
    tcp_transform_initialize();
    tcp_transform_U.Joint1 = joint_angles[0].value();
    tcp_transform_U.Joint2 = joint_angles[1].value();
    tcp_transform_U.Joint3 = joint_angles[2].value();
    tcp_transform_U.Joint4 = joint_angles[3].value();
    tcp_transform_U.Joint5 = joint_angles[4].value();
    tcp_transform_U.Joint6 = joint_angles[5].value();
    
    tcp_transform_step();
    
    position.coordinates = Eigen::Vector3d( tcp_transform_Y.x, tcp_transform_Y.y, tcp_transform_Y.z );
    position.orientation = Eigen::Vector3d( tcp_transform_Y.Roll, tcp_transform_Y.Tilt, tcp_transform_Y.Pan );
    
    tcp_transform_terminate();
}    

} } } // namespace snark { namespace ur { namespace ur5 {

