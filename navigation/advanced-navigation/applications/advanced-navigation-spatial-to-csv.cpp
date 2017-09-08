// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author Navid Pirmarzdashti

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <Eigen/Core>
#include <boost/date_time/posix_time/ptime.hpp>
#include "../../../math/roll_pitch_yaw.h"
#include "../../../visiting/traits.h"
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include "../device.h"
#include "../messages.h"
#include <comma/application/signal_flag.h>
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../traits.h"

using namespace snark::navigation::advanced_navigation;

comma::signal_flag signaled;
const unsigned default_baud_rate=115200;
const unsigned default_sleep=10000;

void usage(bool detail)
{
    std::cerr<<"    connect to Advanced Navigation Spatial device and output GPS data" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <port> [<what>] [<options>]" << std::endl;
    std::cerr<< "    <port>: serial port" << std::endl;
    std::cerr<< "    <what>: select data packet to output, default: navigation"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "what: " << std::endl;
    std::cerr<< "    navigation: navigation data from system state packet" << std::endl;
    std::cerr<< "    system-state: full system state packet"<< std::endl;
    std::cerr<< "    raw-sensors" << std::endl;
    std::cerr<< "    satellites" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h:       show help" << std::endl;
    std::cerr << "    --verbose,-v:    show detailed messages" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << "    --baud-rate=<n>: baud rate for connection, default "<< default_baud_rate << std::endl;
    std::cerr << "    --sleep=<n>: microsecond sleep between reading, default "<< default_sleep << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" \"/dev/usb/ttyUSB0\" " << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" \"/dev/usb/ttyUSB0\" --raw-sensors" << std::endl;
    std::cerr << std::endl;
}

struct output
{
    boost::posix_time::ptime t;
    snark::spherical::coordinates coordinates;
    double height;
    snark::roll_pitch_yaw orientation;
};

namespace comma { namespace visiting {

template <>
struct traits< output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
    }

    template < typename Key, class Visitor > static void visit( const Key&, output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
    }
};
    
} } // namespace comma { namespace visiting {

struct app_i
{
    virtual void run()=0;
    virtual void output_fields()=0;
};

template<typename T>
struct app_t : protected device
{
    comma::csv::output_stream<T> os;
    unsigned us;
    bool output_sensors;
    app_t(const std::string& port,const comma::command_line_options& options) : 
        device(port,options.value<unsigned>("--baud-rate",default_baud_rate)),
        os(std::cout,comma::csv::options(options,"",true)),
        us(options.value<unsigned>("--sleep",default_sleep)),
        output_sensors()
    {
        
    }
    void process()
    {
        while( !signaled && std::cout.good() )
        {
            device::process();
            usleep(us);
        }
    }
    void handle(const messages::raw_sensors* msg)
    {
        if(output_sensors)
        {
            
        }
    }
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<T>(true), ',' ) << std::endl; 
    }
    static void output_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<T>() << std::endl;
    }
};

struct app_nav : public app_t<output>
{
    app_nav(const std::string& port,const comma::command_line_options& options) : app_t(port,options) { }
    //message handlers
    void handle(const messages::system_state* msg)
    {
        if(output_sensors)
            return;
        output o;
        o.t=msg->t();
        o.coordinates.latitude=msg->latitude();
        o.coordinates.longitude=msg->longitude();
        o.height=msg->height();
        o.orientation=snark::roll_pitch_yaw(msg->orientation[0](),msg->orientation[1](),msg->orientation[2]());
        os.write(o);
    }
};

template<typename T>
struct app_packet : public app_t<T>
{
    app_packet(const std::string& port,const comma::command_line_options& options) : app_t<T>(port,options) { }
    void handle(const T* msg)
    {
        app_t<T>::os.write(*msg);
    }
};

struct factory_i
{
    virtual void output_fields()=0;
    virtual void output_format()=0;
    virtual void run(const std::vector<std::string>& unnamed,const comma::command_line_options& options)=0;
};

template<typename T>
struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run(const std::vector<std::string>& unnamed,const comma::command_line_options& options)
    {

        T app(unnamed[0],options);
        app.process();
    }
};

static void bash_completion( int argc, char** argv )
{
    std::cout << "--help --verbose" <<
        " navigation raw-sensors system-state satellites" <<
        " --output-fields --output-format"<< 
        std::endl;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        
        if(options.exists("--bash-completion")) { bash_completion( argc, argv ); return 0; }
        
        std::vector<std::string> unnamed=options.unnamed( comma::csv::options::valueless_options()+ ",--verbose,-v,--output-fields,--output-format", "-.*" );
        
        std::unique_ptr<factory_i> factory;
        if(unnamed.size()<2 || unnamed[1]=="navigation") { factory.reset(new factory_t<app_nav>()); }
        else if(unnamed[1]=="raw-sensors") { factory.reset(new factory_t<app_packet<messages::raw_sensors>>()); }
        else if(unnamed[1]=="system-state") { factory.reset(new factory_t<app_packet<messages::system_state>>()); }
        else if(unnamed[1]=="satellites") { factory.reset(new factory_t<app_packet<messages::satellites>>()); }
        else { COMMA_THROW( comma::exception,"expected <what>: navigation | raw-sensors | system-state; got "<<unnamed[1]);}
        
        if(options.exists("--output-fields")) { factory->output_fields(); return 0; }
        if(options.exists("--output-format")) { factory->output_format(); return 0; }

        if(unnamed.size()<1) { COMMA_THROW( comma::exception, "expected at least one unnamed option for port name, got "<<unnamed.size()); }
        factory->run(unnamed,options);
        
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
