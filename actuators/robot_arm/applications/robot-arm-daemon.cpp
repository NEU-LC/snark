// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#include <stdint.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include "../traits.h"
#include "../commands.h"
#include "../inputs.h"

static const char* name() {
    return "robot-arm-daemon: ";
}

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
} // namespace impl_ {


void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: " << name() << " " << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << std::endl;
    exit ( code );
}

namespace arm = snark::robot_arm;

void output( const std::string& msg, std::ostream& os=std::cout )
{
    os << msg << std::endl;
}

template < typename C >
std::string handle( const std::vector< std::string >& line )
{
    C c;
    try
    {
        c = C::ascii().get( line );
    }
    catch( boost::bad_lexical_cast& le ) {
        std::ostringstream ss;
        ss << '<' << comma::join( line, ',' ) << ',' << impl_::str( arm::errors::format_error )
           << ",\"command format error, wrong field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( comma::exception& ce ) {
        std::ostringstream ss;
        ss << '<' << comma::join( line, ',' ) << ',' << impl_::str( arm::errors::format_error )
           << ",\"command format error, wrong field/s or field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( ... ) { COMMA_THROW( comma::exception, "unknown error is parsing: " + comma::join( line , ',' ) ); }
       
    
    return "error!!!!!";
    // basics::result ret = action< C >::run( c, rover );
    // std::ostringstream ss;
    // ss << '<' << c.serialise() << ',' << ret.code << ",\"" << ret.message << "\";";
    // return ss.str();
}

void process_command( const std::vector< std::string >& v )
{
    if( v[2] == "WAYPOINT" ) { output( handle< arm::move_cam >( v ) ); }
    if( v[2] == "STOP" )     { output( handle< arm::move_joints >( v ) ); }
    else { output( comma::join( v, v.size(), ',' ) + ',' + 
        impl_::str( arm::errors::unknown_command ) + ",\"unknown command found: '" + v[2] + "'\"" ); return; }
}

int main( int ac, char** av )
{
    
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    
    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;

    char batch_size = 10; // number of commands to process

    try
    {
        std::cerr << name() << "started" << std::endl;
        comma::uint16 rover_id = options.value< comma::uint16 >( "--rover-id" );
        double sleep = options.value< double >( "--sleep" );

        arm::inputs inputs( rover_id );

        typedef std::vector< std::string > command_vector;

        const comma::uint32 usec( sleep * 1000000u );
        while( std::cin.good() )
        {
            inputs.read();
            for( std::size_t i=0; !inputs.is_empty() && i<batch_size; ++i )
            {
                const command_vector& v = inputs.front();
                std::cerr << name() << " got " << comma::join( v, ',' ) << std::endl;

                inputs.pop();
            }

            usleep( usec );
        }


        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
