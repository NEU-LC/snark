#ifndef SNARK_ACTUATORS_UR10_APPLICATIONS_ACTION_H
#define SNARK_ACTUATORS_UR10_APPLICATIONS_ACTION_H
#include "../traits.h"
#include "../commands.h"
#include "../inputs.h"
#include "../units.h"
extern "C" {
    #include "../simulink/Arm_Controller.h"
}
#include "../simulink/traits.h"

namespace snark { namespace robot_arm {
    
static std::ostream& ostream = std::cout;
    
static const char* name() {
    return "robot-arm-daemon: ";
}
    
struct input_primitive
{
    enum {
        no_action = 0,
        move_cam = 1,
        set_position = 2,
        set_home=3,      // define home position, internal usage
        movej=4
    };  
};

struct result
{
    struct error { enum { success=0, invalid_input=1 }; };
    int code;
    std::string message;
    
    result( const std::string& msg, int code_ ) : code( code_ ), message( msg ) {}
    result() : code( error::success ), message( "success" ) {}
    
    std::string get_message() const 
    {
        std::ostringstream ss;
        ss << code << ',' << '"' << message << '"';
        return ss.str();
    }
    bool is_success() const { return code == error::success; }
};

template < typename T > struct action;

template < > struct action< move_cam > {
    static result run( const move_cam& cam, std::ostream& os )
    {
#ifdef MAMMOTH_VERBOSE
        std::cerr << name() << " running " << cam.serialise() << std::endl; 
#endif        
        static const plane_angle_degrees_t max_pan = 45.0 * degree;
        static const plane_angle_degrees_t min_pan = -45.0 * degree;
        static const plane_angle_degrees_t max_tilt = 90.0 * degree;
        static const plane_angle_degrees_t min_tilt = -90.0 * degree;
        static const length_t min_height = 0.1 * meter;
        static const length_t max_height = 0.8 * meter;
        
        
        if( cam.pan < min_pan ) { return result( "pan angle is below minimum limit of -45.0", result::error::invalid_input ); }
        if( cam.pan > max_pan ) { return result( "pan angle is above minimum limit of 45.0", result::error::invalid_input ); }
        if( cam.tilt < min_tilt ) { return result( "tilt angle is below minimum limit of -90.0", result::error::invalid_input ); }
        if( cam.tilt > max_tilt ) { return result( "tilt angle is above minimum limit of 90.0", result::error::invalid_input ); }
        if( cam.height < min_height ) { return result( "height value is below minimum limit of 0.1m", result::error::invalid_input ); }
        if( cam.height > max_height ) { return result( "height value is above minimum limit of 0.5m", result::error::invalid_input ); }
        
        static double zero_tilt = 90.0;
        Arm_Controller_U.motion_primitive = real_T( input_primitive::move_cam );
        Arm_Controller_U.Input_1 = cam.pan.value();
        Arm_Controller_U.Input_2 = zero_tilt - cam.tilt.value();
        Arm_Controller_U.Input_3 = cam.height.value();
        
        return result();
    }  
};

template < > struct action< move_joints > {
    static result run( const move_joints& joints, std::ostream& os )
    {
#ifdef MAMMOTH_VERBOSE
        std::cerr << name() << " running " << joints.serialise() << std::endl; 
#endif
        static const plane_angle_degrees_t min = 0.0 * degree;
        static const plane_angle_degrees_t max = 360.0 * degree;
        for( std::size_t i=0; i<joints.joints.size(); ++i )
        {
            if( joints.joints[i] < min || joints.joints[0] > max ) { return result( "joint angle must be 0-360 degrees", result::error::invalid_input ); }
        }
        
        Arm_Controller_U.motion_primitive = real_T( input_primitive::movej );
        Arm_Controller_U.Input_1 = joints.joints[0].value();
        Arm_Controller_U.Input_2 = joints.joints[1].value();
        Arm_Controller_U.Input_3 = joints.joints[2].value();
        Arm_Controller_U.Input_4 = joints.joints[3].value();
        Arm_Controller_U.Input_5 = joints.joints[4].value();
        Arm_Controller_U.Input_6 = joints.joints[5].value();
        return result();
    }  
};

template < > struct action< set_position > {
    static result run( const set_position& pos, std::ostream& os )
    {
        Arm_Controller_U.motion_primitive = input_primitive::set_position;
        
        if( pos.position == "giraffe" ) { Arm_Controller_U.Input_1 = set_position::giraffe; }
        else if( pos.position == "home" ) { Arm_Controller_U.Input_1 = set_position::home; }
        else { return result("unknown position type", int(result::error::invalid_input) ); }
//         std::cerr << name() << " running " << pos.serialise()  << " pos_input: " << Arm_Controller_U.Input_1 
//             << " tag: " << pos.position << std::endl; 
        return result();
    }  
};

template < > struct action< set_home > {
    static result run( const set_home& h, std::ostream& os )
    {
        Arm_Controller_U.motion_primitive = input_primitive::set_home;
        return result();
    }  
};
    
template < > struct action< enable > {
    static result run( const enable& h, std::ostream& os )
    {
        std::cerr << name() << "dummy enable" << std::endl;
        return result();
    }  
};
    
template < > struct action< release_brakes > {
    static result run( const release_brakes& h, std::ostream& os )
    {
        std::cerr << name() << "dummy release brakes" << std::endl;
        return result();
    }  
};
    
template < > struct action< auto_init > {
    static result run( const auto_init& h, std::ostream& os )
    {
        std::cerr << name() << "dummy auto init" << std::endl;
        return result();
    }  
};

template < > struct action< robot_arm::joint_move > {
    
    static const unsigned char min_id = 0;
    static const unsigned char max_id = 5;
    static result run( const robot_arm::joint_move& j, std::ostream& os )
    {
        std::cerr << name() << "dummy move joint: " << int(j.joint_id) << " dir: " << j.dir << std::endl;
        static const angular_velocity_t velocity = 0.1 * rad_per_sec;
        static const angular_acceleration_t acceleration = 0.05 * rad_per_s2;
        static const boost::posix_time::time_duration duration = boost::posix_time::milliseconds( 20 );
        
        if( j.joint_id < min_id || j.joint_id > max_id ) {
            return result( "joint id must be 0-5", result::error::invalid_input );
        }
        
        double vel = ( j.dir ? velocity.value() : -velocity.value() );
        
        std::ostringstream ss;
        ss << "speedj_init([";
        for( std::size_t i=min_id; i<=max_id; ++i )
        {
            ss << (i == j.joint_id ? vel : 0);
            if( i != max_id ) { ss << ','; };
        }
        ss << "],"  << acceleration.value() << ',' << (duration.total_milliseconds()/1000.0) << ')' << std::endl;
        os << ss.str();
        os.flush();
        
        return result();
    }  
};
    
} } // namespace snark { namespace robot_arm {

#endif // SNARK_ACTUATORS_UR10_APPLICATIONS_ACTION_H