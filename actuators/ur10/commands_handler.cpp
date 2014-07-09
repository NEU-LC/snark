#include "commands_handler.h"

namespace snark { namespace robot_arm { namespace handlers {

static const char* name() {
    return "robot-arm-daemon: ";
}

void commands_handler::handle( arm::power& p )
{
    std::cerr << name() << "running power" << std::endl;
    os << "power " << ( p.is_on ? "on" : "off" ) << std::endl;
    os.flush();
    ret = result();
}

void commands_handler::handle( arm::brakes& b )
{
    std::cerr << name() << "running brakes: " << b.enable  << std::endl;
    os << "stopj([0,0,0,0,0,0])" << std::endl;
    os.flush();
    if( b.enable ) {
        os << "set robotmode run" <<std::endl;
        os.flush();
    }
    ret = result();
}

void commands_handler::handle( arm::auto_init& a )
{
    std::cerr << name() << "dummy auto init" << std::endl;
    ret = result();
}

void commands_handler::handle( arm::move_cam& cam )
{
    std::cerr << name() << " running move_cam" << std::endl; 
    static const plane_angle_degrees_t max_pan = 45.0 * degree;
    static const plane_angle_degrees_t min_pan = -45.0 * degree;
    static const plane_angle_degrees_t max_tilt = 90.0 * degree;
    static const plane_angle_degrees_t min_tilt = -90.0 * degree;
    static const length_t min_height = 0.1 * meter;
    static const length_t max_height = 0.8 * meter;
    
    
    if( cam.pan < min_pan ) { ret = result( "pan angle is below minimum limit of -45.0", result::error::invalid_input ); return; }
    if( cam.pan > max_pan ) { ret = result( "pan angle is above minimum limit of 45.0", result::error::invalid_input ); return; }
    if( cam.tilt < min_tilt ) { ret = result( "tilt angle is below minimum limit of -90.0", result::error::invalid_input ); return; }
    if( cam.tilt > max_tilt ) { ret = result( "tilt angle is above minimum limit of 90.0", result::error::invalid_input ); return; }
    if( cam.height < min_height ) { ret = result( "height value is below minimum limit of 0.1m", result::error::invalid_input ); return; }
    if( cam.height > max_height ) { ret = result( "height value is above minimum limit of 0.5m", result::error::invalid_input ); return; }
    
    static double zero_tilt = 90.0;
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = cam.pan.value();
    inputs_.Input_2 = zero_tilt - cam.tilt.value();
    inputs_.Input_3 = cam.height.value();
    
    ret = result();
}

void commands_handler::handle( arm::move_joints& joints )
{
    std::cerr << name() << " running move joints"  << std::endl; 
    static const plane_angle_degrees_t min = 0.0 * degree;
    static const plane_angle_degrees_t max = 360.0 * degree;
    for( std::size_t i=0; i<joints.joints.size(); ++i )
    {
        if( joints.joints[i] < min || joints.joints[0] > max ) { 
        	ret = result( "joint angle must be 0-360 degrees", result::error::invalid_input ); 
        	return;
        }
    }
    
    inputs_.motion_primitive = real_T( input_primitive::movej );
    inputs_.Input_1 = joints.joints[0].value();
    inputs_.Input_2 = joints.joints[1].value();
    inputs_.Input_3 = joints.joints[2].value();
    inputs_.Input_4 = joints.joints[3].value();
    inputs_.Input_5 = joints.joints[4].value();
    inputs_.Input_6 = joints.joints[5].value();
    ret = result();
}

void commands_handler::handle( arm::joint_move& joint )
{
    static const unsigned char min_id = 0;
    static const unsigned char max_id = 5;
    std::cerr << name() << "dummy move joint: " << int(joint.joint_id) << " dir: " << joint.dir << std::endl;
    static const angular_velocity_t velocity = 0.1 * rad_per_sec;
    static const angular_acceleration_t acceleration = 0.05 * rad_per_s2;
    static const boost::posix_time::time_duration duration = boost::posix_time::milliseconds( 20 );
    
    if( joint.joint_id < min_id || joint.joint_id > max_id ) {
        ret = result( "joint id must be 0-5", result::error::invalid_input );
        return;
    }
    
    double vel = ( joint.dir ? velocity.value() : -velocity.value() );
    
    std::ostringstream ss;
    ss << "speedj_init([";
    for( std::size_t i=min_id; i<=max_id; ++i )
    {
        ss << (i == joint.joint_id ? vel : 0);
        if( i != max_id ) { ss << ','; };
    }
    ss << "],"  << acceleration.value() << ',' << (duration.total_milliseconds()/1000.0) << ')' << std::endl;
    os << ss.str();
    os.flush();
    
    ret = result();
}

void commands_handler::handle( arm::set_home& h )
{
    inputs_.motion_primitive = input_primitive::set_home;
	ret = result();
}

void commands_handler::handle( arm::set_position& pos )
{
    inputs_.motion_primitive = input_primitive::set_position;
    
	ret = result();

    if( pos.position == "giraffe" ) { inputs_.Input_1 = set_position::giraffe; }
    else if( pos.position == "home" ) { inputs_.Input_1 = set_position::home; }
    else { ret = result("unknown position type", int(result::error::invalid_input) ); }
}

void commands_handler::handle( arm::move_effector& e )
{
}


} } } // namespace snark { namespace robot_arm { namespace handlers {
