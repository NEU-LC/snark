#ifndef SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H
#define SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H
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
#include "inputs.h"

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    
struct result
{
    struct error { enum { success=0, invalid_input=1, invalid_robot_state, failure }; };
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

namespace arm = robotic_arm;

/// This class takes over control from the main loop and do auto init,
/// because we dont want to use a second thread (simpler) and auto init is a long running action.
/// The code is put into this class, with the run member.
class auto_initialization
{
    /// Status to check if initialized 
    arm::fixed_status& status_;
    std::ostream& os;           /// output to the rover
    comma::io::istream& iss_;   /// for reading new statuses
    comma::io::select& select_; /// select for above stream
    comma::signal_flag& signaled_;  /// Check if signal received
    arm::inputs& inputs_;   // to check if new command/s are received
    std::string name_;  // name of the executable running this
    /// This is the value of force limit on arm before failing auto initialisation.
    /// Should not be 0 as there is a laszer mount?
    double force_max_; // newtons
    
    const std::string& name() const { return name_; }
    /// Get the latest status from the arm
    void read_status(); 
    
public:
    auto_initialization( arm::fixed_status& status, std::ostream& robot, 
                         comma::io::istream& status_iss, comma::io::select& select,
                         comma::signal_flag& signaled, arm::inputs& inputs ) : 
        status_( status ), os( robot ),
        iss_(status_iss), select_( select ), signaled_( signaled ),
        inputs_( inputs ), force_max_( 0.0 ) {}
    
    void set_app_name( const char* name ) { name_ = name; }
    /// Performs auto initialisation but also listens for new commands.
    /// If a new command arrives or a signal is received run() returns immediately.
    /// result: shows wether success or failure.
    result run();
};


} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

#endif // SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H