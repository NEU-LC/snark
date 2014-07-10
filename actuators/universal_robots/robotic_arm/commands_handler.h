#ifndef SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#define SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <comma/base/types.h>
#include <comma/dispatch/dispatched.h>
#include <comma/csv/stream.h>
#include <comma/base/exception.h>
#include <boost/optional.hpp>
#include "data.h"
#include "commands.h"
extern "C" {
    #include "simulink/Arm_Controller.h"
}


namespace snark { namespace ur { namespace robotic_arm {

struct robotmode {
    enum mode { running, freedrive, ready, initializing, security_stopped, estopped, fatal_error, no_power, not_connected, shutdown, safeguard_stop };
};
struct jointmode {
    enum mode { power_off=239, error=242, freedrive=243, calibration=250, stopped=251, running=253, initializing=254, idle=255 };
};

const char* robotmode_str( robotmode::mode mode );

const char* jointmode_str( jointmode::mode mode );

} } }  // namespace snark { namespace ur { namespace robotic_arm {

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

namespace arm = robotic_arm;

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
    struct error { enum { success=0, invalid_input=1, invalid_robot_state }; };
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


class commands_handler : public comma::dispatch::handler_of< power >,
                                  public comma::dispatch::handler_of< brakes >,
                                  public comma::dispatch::handler_of< set_home >,
                                  public comma::dispatch::handler_of< auto_init >,
                                  public comma::dispatch::handler_of< move_cam >,
                                  public comma::dispatch::handler_of< move_joints >,
                                  public comma::dispatch::handler_of< joint_move >,
                                  public comma::dispatch::handler_of< set_position >,
                                  public comma::dispatch::handler_of< move_effector >
{
public:
    void handle( power& p );
    void handle( brakes& b );
    void handle( auto_init& a );
    void handle( move_cam& c );
    void handle( move_effector& e );
    void handle( move_joints& js );
    void handle( set_home& h );
    void handle( set_position& p );
    void handle( joint_move& j );
    
    commands_handler( ExtU_Arm_Controller_T& simulink_inputs, 
    				  const arm::fixed_status& status, std::ostream& robot ) : 
    	inputs_(simulink_inputs), status_( status ), os( robot ) {}
    
    result ret;  /// Indicate if command succeed
private:
	ExtU_Arm_Controller_T& inputs_; /// inputs into simulink engine 
	const fixed_status& status_;
	std::ostream& os;		/// output stream to robot arm

	bool is_powered() const;
	bool is_initialising() const;
	bool is_running() const; 
};

} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {


#endif // SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H