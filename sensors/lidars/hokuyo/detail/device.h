#pragma once
// common interface and functionality for hokuyo laser devices

#include <comma/application/command_line_options.h>
#include "../streams.h"

namespace snark { namespace hokuyo {

struct laser_device
{
    //get args
    virtual void init(comma::command_line_options options)=0;
    //setup communication
    // The caller is responsible for deleting this stream_base object - on the heap.
    virtual stream_base* connect()=0;
    //e.g. switch to scip2
    virtual void setup(stream_base& ios)=0;
    virtual void reboot(stream_base& ios)=0;
};

//scoped class to turn on/off laser in constructor/destructor
struct turn_laser_on
{
    stream_base& ios;
    turn_laser_on(stream_base& ios, laser_device& device, bool reboot_on_error);
    ~turn_laser_on();
};

struct information_commands
{
    void version();
    void sensor_parameters();
    void sensor_state();
};

} }  // namespace snark { namespace hokuyo {
