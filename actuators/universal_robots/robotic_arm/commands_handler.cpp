#include "commands_handler.h"

namespace snark { namespace ur { namespace robotic_arm {

const char* robotmode_str( robotmode::mode mode )
{
	switch( mode )
	{
		case robotmode::running:
			return "running_mode";
		case robotmode::freedrive:
			return "freedrive_mode";
		case robotmode::ready:
			return "ready_mode";
		case robotmode::initializing:
			return "initializing_mode";
		case robotmode::security_stopped:
			return "security_stopped_mode";
		case robotmode::estopped:
			return "estopped_mode";
		case robotmode::fatal_error:
			return "fatal_error_mode";
		case robotmode::no_power:
			return "no_power_mode";
		case robotmode::not_connected:
			return "not_connected_mode";
		case robotmode::shutdown:
			return "shutdown_mode";
		case robotmode::safeguard_stop:
			return "safeguard_stop_mode";
		default:
			std::cerr << "unknown robot mode: " << int(mode) << std::endl;
			COMMA_THROW( comma::exception, "unknown robot mode" );
			return "";
	}
}

const char* jointmode_str( jointmode::mode mode )
{
	switch( mode )
	{
		case jointmode::power_off:
			return "power_off_mode";
		case jointmode::error:
			return "error_mode";
		case jointmode::freedrive:
			return "freedrive_mode";
		case jointmode::calibration:
			return "calibration_mode";
		case jointmode::stopped:
			return "stopped_mode";
		case jointmode::running:
			return "running_mode";
		case jointmode::initializing:
			return "initializing_mode";
		case jointmode::idle:
			return "idle_mode";
		default:
			return "other_mode"; // some other mode not converted to string
	}
}
} } } // namespace snark { namespace ur { namespace robotic_arm {

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

static const char* name() {
    return "robot-arm-daemon: ";
}

void commands_handler::handle( arm::power& p )
{
    std::cerr << name() << "powering robot arm " << ( p.is_on ? "on" : "off" ) << std::endl;
    os << "power " << ( p.is_on ? "on" : "off" ) << std::endl;
    os.flush();
    ret = result();
}

void commands_handler::handle( arm::brakes& b )
{
    std::cerr << name() << "running brakes: " << b.enable  << std::endl;
    if( !b.enable ) {
        os << "set robotmode run" <<std::endl;
    }
    else {
    	os << "stopj([0,0,0,0,0,0])" << std::endl;
    }
    os.flush();
    ret = result();
}

void commands_handler::handle( arm::auto_init& a )
{
    std::cerr << name() << "dummy auto init" << std::endl;
    ret = result();
}

static const plane_angle_degrees_t max_pan = 45.0 * degree;
static const plane_angle_degrees_t min_pan = -45.0 * degree;
static const plane_angle_degrees_t max_tilt = 90.0 * degree;
static const plane_angle_degrees_t min_tilt = -90.0 * degree;

void commands_handler::handle( arm::move_cam& cam )
{
    std::cerr << name() << " running move_cam" << std::endl; 

    if( !is_running() ) {
    	ret = result( "cannot move (camera) as rover is not in running mode", result::error::invalid_robot_state );
    	return;
    }

    static const length_t min_height = 0.1 * meter;
    static const length_t max_height = 1.0 * meter;
    
    
    if( cam.pan < min_pan ) { ret = result( "pan angle is below minimum limit of -45.0", result::error::invalid_input ); return; }
    if( cam.pan > max_pan ) { ret = result( "pan angle is above minimum limit of 45.0", result::error::invalid_input ); return; }
    if( cam.tilt < min_tilt ) { ret = result( "tilt angle is below minimum limit of -90.0", result::error::invalid_input ); return; }
    if( cam.tilt > max_tilt ) { ret = result( "tilt angle is above minimum limit of 90.0", result::error::invalid_input ); return; }
    if( cam.height < min_height ) { ret = result( "height value is below minimum limit of 0.1m", result::error::invalid_input ); return; }
    if( cam.height > max_height ) { ret = result( "height value is above minimum limit of 1.0m", result::error::invalid_input ); return; }
    
    static double zero_tilt = 90.0;
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = cam.pan.value();
    inputs_.Input_2 = zero_tilt - cam.tilt.value();
    inputs_.Input_3 = cam.height.value();
    
    ret = result();
}

void commands_handler::handle( arm::move_joints& joints )
{
    if( !is_running() ) {
    	ret = result( "cannot move (joints) as rover is not in running mode", result::error::invalid_robot_state );
    	return;
    }

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
    if( !is_initialising() ) {
    	ret = result( "cannot initialise joint as rover is not in initialisation mode", result::error::invalid_robot_state );
    	return;
    }
    /// command can be use if in running or initialising mode
    int index = joint.joint_id;
    if( status_.robot_mode() != robotmode::initializing && 
        status_.joint_modes[index]() != jointmode::initializing )
    { 
        std::ostringstream ss;
        ss << "robot and  joint (" << index << ") must be initializing state. However current robot mode is '" 
           << arm::robotmode_str( (arm::robotmode::mode)int(status_.robot_mode()) ) 
           << "' and joint mode is '" << arm::jointmode_str( (arm::jointmode::mode)int(status_.joint_modes[index]()) ) << '\'' << std::endl;
        ret = result( ss.str(), result::error::invalid_robot_state );
        return; 
    }

    static const unsigned char min_id = 0;
    static const unsigned char max_id = 5;
    std::cerr << name() << "move joint: " << int(joint.joint_id) << " dir: " << joint.dir << std::endl;
    static const angular_velocity_t velocity = 0.1 * rad_per_sec;
    static const angular_acceleration_t acceleration = 0.05 * rad_per_s2;
    static const boost::posix_time::time_duration duration = boost::posix_time::milliseconds( 20 );
    
    if( joint.joint_id < min_id || joint.joint_id > max_id ) {
        ret = result( "joint id must be 0-5", result::error::invalid_input );
        return;
    }
    
    double vel = ( joint.dir ? velocity.value() : -velocity.value() );
    if( joint.joint_id == 2 ) { vel /= 2; }
    else if( joint.joint_id < 1 ) { vel /= 3; }
    
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
    if( !is_running() ) {
    	ret = result( "cannot set position as rover is not in running mode", result::error::invalid_robot_state );
    	return;
    }

    inputs_.motion_primitive = input_primitive::set_position;

    if( pos.position == "giraffe" ) { inputs_.Input_1 = set_position::giraffe; }
    else if( pos.position == "home" ) { inputs_.Input_1 = set_position::home; }
    else { ret = result("unknown position type", int(result::error::invalid_input) ); return; }

    inputs_.Input_2 = 0;    // zero pan for giraffe
    inputs_.Input_3 = 0;    // zero tilt for giraffe
    
    ret = result();
}
void commands_handler::handle( arm::set_position_giraffe& giraffe )
{
    if( !is_running() ) {
        ret = result( "cannot set giraffe position as rover is not in running mode", result::error::invalid_robot_state );
        return;
    }

    if( giraffe.position != "giraffe" ) { 
        ret = result("unknown position type: expected 'giraffe' when spcifying pan and tilt angles", 
                     int(result::error::invalid_input) );
        return; 
    }
    if( giraffe.pan < min_pan ) { ret = result( "pan angle is below minimum limit of -45.0", result::error::invalid_input ); return; }
    if( giraffe.pan > max_pan ) { ret = result( "pan angle is above minimum limit of 45.0", result::error::invalid_input ); return; }
    if( giraffe.tilt < min_tilt ) { ret = result( "tilt angle is below minimum limit of -90.0", result::error::invalid_input ); return; }
    if( giraffe.tilt > max_tilt ) { ret = result( "tilt angle is above minimum limit of 90.0", result::error::invalid_input ); return; }

    std::cerr << "giraffe pan tilt" << std::endl;

    static double zero_tilt = 90.0;
    inputs_.motion_primitive = input_primitive::move_cam;
    inputs_.Input_1 = giraffe.pan.value();    // zero pan for giraffe
    inputs_.Input_2 = zero_tilt - giraffe.tilt.value();    // zero tilt for giraffe
    inputs_.Input_3 = 1.0; // height of 1m
    
    ret = result();
}


void commands_handler::handle( arm::move_effector& e )
{
}

bool commands_handler::is_powered() const {
	return (status_.robot_mode() != robotmode::no_power);
}

bool commands_handler::is_running() const 
{
	if(status_.robot_mode() != robotmode::running) { 
		std::cerr << "robot mode " << status_.robot_mode() << " expected: " << robotmode::running << std::endl;
		return false; 
	}

	for( std::size_t i=0; i<joints_num; ++i ) {
		std::cerr << "joint " << i << " mode " << status_.joint_modes[i]() << " expected: " << jointmode::running << std::endl;
		if( status_.joint_modes[i]() != jointmode::running ) { return false; }
	}

	return true;
}
bool commands_handler::is_initialising() const 
{
	if( status_.robot_mode() != robotmode::initializing ) { 
		// std::cerr << "robot mode " << status_.robot_mode() << " expected: " << robotmode::initializing << std::endl;
		return false; 
	}

	for( std::size_t i=0; i<joints_num; ++i ) 
	{
		// std::cerr << "joint " << i << " mode " << status_.joint_modes[i]() << " expected: " << jointmode::running << std::endl;
		if( status_.joint_modes[i]() != jointmode::initializing && 
			status_.joint_modes[i]() != jointmode::running ) { 
			return false; 
		}
	}

	return true;
}


} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
