#ifndef SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H
#define SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <comma/base/types.h>
#include <comma/dispatch/dispatched.h>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <boost/optional.hpp>
#include <boost/function.hpp>
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
public:
    typedef comma::csv::binary_input_stream< arm::status_t > binary_stream_t;   /// for reading new statuses
private:
    /// Status to check if initialized 
    arm::status_t& status_;
    std::ostream& os;           /// output to the rover
    comma::csv::binary_input_stream< arm::status_t >& iss_;   /// for reading new statuses
    boost::function< void ( binary_stream_t& ) > update_status_;
    comma::signal_flag& signaled_;  /// Check if signal received
    arm::inputs& inputs_;   // to check if new command/s are received
    std::string name_;  // name of the executable running this
    /// This is the value of force limit on arm before failing auto initialisation.
    /// Should not be 0 as there is a laszer mount?
    double force_max_; // newtons
    std::string home_filepath_;
    
    const std::string& name() const { return name_; }
    /// Get the latest status from the arm
    void read_status(); 
    
    static const char* filename;
    
public:
    auto_initialization( arm::status_t& status, std::ostream& robot, 
                         comma::csv::binary_input_stream< arm::status_t >& status_iss, 
			             // comma::io::select& select, comma::io::file_descriptor fd,
                         boost::function< void (binary_stream_t&) > f,
                         comma::signal_flag& signaled, 
                         arm::inputs& inputs, const std::string& work_dir ) : 
        status_( status ), os( robot ), 
        iss_(status_iss), 
        update_status_(f),
        // select_( select ), fd_( fd ), 
        signaled_( signaled ),
        inputs_( inputs ), force_max_( 13.0 ), home_filepath_( work_dir + '/' + filename ) {}
    
    void set_app_name( const char* name ) { name_ = name; }
    void set_force_limit( double newtons ){ force_max_ = newtons; }
    
    const std::string& home_filepath() const { return home_filepath_; }
    /// Performs auto initialisation but also listens for new commands.
    /// If a new command arrives or a signal is received run() returns immediately.
    /// result: shows wether success or failure.
    result run( bool force );
};


} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

#endif // SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H