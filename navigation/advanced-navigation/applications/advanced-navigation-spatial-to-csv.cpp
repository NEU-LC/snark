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

using namespace snark::navigation::advanced_navigation;

comma::signal_flag signaled;
const unsigned default_baud_rate=115200;
const unsigned default_sleep=10000;

void usage(bool detail)
{
    std::cerr<<"    connect to Advanced Navigation Spatial device and output GPS data" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <port> [<options>]" << std::endl;
    std::cerr<< "    <port>: serial port" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h:       show help" << std::endl;
    std::cerr << "    --verbose,-v:    show detailed messages" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << "    --baud-rate=<n>: baud rate for connection, default "<< default_baud_rate << std::endl;
    std::cerr << "    --sleep=<n>: microsecond sleep between reading, default "<< default_sleep << std::endl;
    std::cerr << "    --binary: output data in binary (default is csv)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" \"/dev/usb/ttyUSB0\" " << std::endl;
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

struct app
{
    device dev;
    unsigned us;
    app(const std::string& port,const comma::command_line_options& options) : 
        dev(port,options.value<unsigned>("--baud-rate",default_baud_rate)),
        us(options.value<unsigned>("--sleep",default_sleep))
    {
        
        
    }
    void process()
    {
        while( !signaled && std::cout.good() )
        {
            dev.process();
            usleep(us);
        }
    }
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<output>(true), ',' ) << std::endl; 
    }
    static void output_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<output>() << std::endl;
    }
};
    
int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        
        if(options.exists("--output-fields")) { app::output_fields(); return 0; }
        if(options.exists("--output-format")) { app::output_format(); return 0; }

        std::vector<std::string> unnamed=options.unnamed( "--verbose,-v,--output-fields,--output-format", "-*" );
        if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected one unnamed option for port name, got "<<unnamed.size()); }

        app app_(unnamed[0],options);
        app_.process();
        
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
