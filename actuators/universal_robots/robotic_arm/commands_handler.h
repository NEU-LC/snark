#ifndef SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#define SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <comma/base/types.h>
#include <comma/dispatch/dispatched.h>
#include <comma/io/stream.h>
#include <comma/io/select.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <boost/optional.hpp>
#include "data.h"
#include "commands.h"
#include "auto_initialization.h"
#include <boost/filesystem.hpp>
extern "C" {
    #include "simulink/Arm_Controller.h"
}

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

namespace arm = robotic_arm;
namespace fs = boost::filesystem;

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



class commands_handler : public comma::dispatch::handler_of< power >,
                                  public comma::dispatch::handler_of< brakes >,
                                  public comma::dispatch::handler_of< set_home >,
                                  public comma::dispatch::handler_of< auto_init >,
                                  public comma::dispatch::handler_of< move_cam >,
                                  public comma::dispatch::handler_of< move_joints >,
                                  public comma::dispatch::handler_of< joint_move >,
                                  public comma::dispatch::handler_of< set_position >,
                                  public comma::dispatch::handler_of< auto_init_force >,
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
    void handle( auto_init_force& p );
    void handle( joint_move& j );
    
    commands_handler( ExtU_Arm_Controller_T& simulink_inputs, 
                      arm::fixed_status& status, std::ostream& robot, auto_initialization& init ) : 
        inputs_(simulink_inputs), status_( status ), os( robot ), init_(init),
        home_filepath_( init_.home_filepath() ) {}
        
    bool is_powered() const;
    bool is_initialising() const;
    bool is_running() const; 
    
    result ret;  /// Indicate if command succeed
private:
    ExtU_Arm_Controller_T& inputs_; /// inputs into simulink engine 
    fixed_status& status_;
    std::ostream& os;
    auto_initialization init_;
    fs::path home_filepath_;
};

} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {


#endif // SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H