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

#include <boost/array.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/csv/traits.h>
#include "base.h"

static const char* name() { return "ur-arm-command"; }

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take a command code and six values (e.g., joint angles) and output a ur-script-formatted command to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message; --help" << std::endl; 
    std::cerr << "    --verbose,-v: more help" << std::endl;
    std::cerr << "    --fields: input stream fields" << std::endl;
    std::cerr << "        default: command,values" << std::endl;
    std::cerr << "        optional: acceleration,speed,time,radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "command codes:" << std::endl;
    std::cerr << "    0: stop currently executed motion (requires acceleration; values are ignored)" << std::endl;
    std::cerr << "    1: move joints to given joint angles (requires joint angles)" << std::endl;
    std::cerr << "    2: move joints at given joint speeds (requires joint speeds, acceleration and time)" << std::endl;
    std::cerr << "    3: move tool to given pose (requires tool pose)" << std::endl;
    std::cerr << "    4: move tool at given pose velocity (requires pose velocity, acceleration and time)" << std::endl;
    std::cerr << "    5: move joints so that the tool reaches given pose (requires tool pose)" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << "csv options" << std::endl << comma::csv::options::usage( "command,values" ) << std::endl; }
    else { std::cerr << "csv options... use --help --verbose for more" << std::endl << std::endl; }
    std::cerr << "default values:" << std::endl;
    std::cerr << "    joints: speed = 0.75 rad/s, acceleration = 3 rad/s^2" << std::endl;
    std::cerr << "    tool: speed = 0.3 m/s, acceleration = 1.2 m/s^2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "maximum values (approximate):" << std::endl;
    std::cerr << "    joints: speed = 3.14 rad/s" << std::endl;
    std::cerr << "    tool: speed = 1 m/s" << std::endl;    
    std::cerr << std::endl;
    std::cerr << "examples (assuming robot.arm is the arm's IP address):" << std::endl;
    std::cerr << "    move joints at a speed of 0.02 rad/sec to new joint angles 0,1,2,3,4,5 (in radians):" << std::endl;
    std::cerr << "        echo \"1,0,1,2,3,4,5,0.02\" | ur-arm-command --fields=command,values,speed | nc robot.arm 30002" << std::endl;
    std::cerr << "    same as above but with a timestamp in the input stream (timestamp is ignored):" << std::endl;
    std::cerr << "        echo \"20140101T000000,1,0,1,2,3,4,5,0.02\" | ur-arm-command move --fields=t,command,values,speed | nc robot.arm 30002" << std::endl;        
    std::cerr << "    move joints to new joint angles 0,1,2,3,4,5 (complete the motion in 10 seconds):" << std::endl;
    std::cerr << "        echo \"1,0,1,2,3,4,5,10\" | ur-arm-command --fields=command,values,time | nc robot.arm 30002" << std::endl;
    std::cerr << "    move joints so that the tool reaches new pose 0,1,2,0,1,2 with default joint speed:" << std::endl;
    std::cerr << "        echo \"5,0,1,2,0,1,2\" | ur-arm-command | nc robot.arm 30002" << std::endl;
    std::cerr << "    move joints at speeds 0,1,2,3,4,5 rad/sec for 0.5 seconds:" << std::endl;
    std::cerr << "        echo \"2,0,1,2,3,4,5,0.5\" | ur-arm-command --fields=command,values,time | nc robot.arm 30002" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    move tool to new pose 0,1,2,0,1,2 at a speed of 0.1 m/sec:" << std::endl;
    std::cerr << "        echo \"3,0,1,2,0,1,2,0.1\" | ur-arm-command --fields=command,values,speed | nc robot.arm 30002" << std::endl;
    std::cerr << "    move tool at pose velocity 0,1,2,0,1,2 for 10 seconds with acceleration 0.1 m/sec2:" << std::endl;
    std::cerr << "        echo \"4,0,1,2,0,1,2,10,0.1\" | ur-arm-command --fields=command,values,time,acceleration | nc robot.arm 30002" << std::endl;    
    std::cerr << std::endl;
    std::cerr << "    stop current motion with acceleration of 0.5 rad/sec2 (values and time are ignored):" << std::endl;
    std::cerr << "        echo \"0,0,1,2,3,4,5,0.5,10\" | ur-arm-command --fields=command,values,acceleration,time | nc robot.arm 30002" << std::endl;
    std::cerr << std::endl;
    exit ( -1 );
}

enum command_t { stop=0, move_joints=1, speed_joints=2, move_tool=3, speed_tool=4, move_joints_to_pose=5 };

struct input_t 
{ 
    typedef double type;
    typedef boost::array< type, 6 > values_t;
    command_t command;
    values_t values;
    type acceleration;
    type speed;
    type time;
    type radius;
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K&, input_t& t, V& v )
    {
        int command;
        v.apply( "command", command );
        t.command = static_cast< command_t >( command );
        v.apply( "values", t.values );
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }
    template< typename K, typename V > static void visit( const K&, const input_t& t, V& v )
    {
        v.apply( "command", static_cast< int >( t.command ) );
        v.apply( "values", t.values );
        v.apply( "acceleration", t.acceleration );
        v.apply( "speed", t.speed );
        v.apply( "time", t.time );
        v.apply( "radius", t.radius );
    }
};
    
} } // namespace comma { namespace visiting {

class optional_parameters_t
{
public:
    optional_parameters_t( const comma::csv::options& csv ) : input_stream_has_( csv ) {}
    std::string operator()( const input_t* input )
    {
        std::stringstream ss;
        if( input_stream_has_.acceleration ) { ss << ",a=" << input->acceleration; }
        if( input_stream_has_.speed ) { ss << ",v=" << input->speed; }
        if( input_stream_has_.time ) { ss << ",t=" << input->time; }
        if( input_stream_has_.radius ) { ss << ",r=" << input->radius; }
        return ss.str();
    }

private:
    struct input_stream_has_t
    {
        input_stream_has_t( const comma::csv::options& csv )
            : acceleration( csv.has_field( "acceleration" ) )
            , speed( csv.has_field( "speed" ) )
            , time( csv.has_field( "time" ) )
            , radius( csv.has_field( "radius" ) ) {}
        bool acceleration;
        bool speed;
        bool time;
        bool radius;
    };
    input_stream_has_t input_stream_has_;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv = comma::csv::options( options, "command,values" );
        csv.full_xpath = false;
        if( !csv.has_field( "command" ) ) { std::cerr << name() << ": failed to find command in the input fields" << std::endl; }
        if( !csv.has_field( "values" ) && !csv.has_field( "values[0],values[1],values[2],values[3],values[4],values[5]" ) ) { std::cerr << name() << ": failed to find command in the input fields" << std::endl; }
        comma::csv::input_stream< input_t > istream( std::cin, csv );
        optional_parameters_t optional_parameters( csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const input_t* input = istream.read();
            if( !input ) { break; }
            switch( input->command )
            {
                case stop:
                    if( !csv.has_field( "acceleration" ) ) { std::cerr << name() << ": command " << input->command << " requires acceleration" << std::endl; return 1; }
                    std::cout << "stopj(" << input->acceleration << ")" << std::endl; 
                    break;
                case move_joints: 
                    std::cout << "movej([" << comma::join( input->values, ',' ) << "]" << optional_parameters( input ) << ")" << std::endl; 
                    break; 
                case speed_joints: 
                    if( !csv.has_field( "acceleration,time" ) ) { std::cerr << name() << ": command " << input->command << " requires acceleration and time" << std::endl; return 1; }
                    std::cout << "speedj([" << comma::join( input->values, ',' ) << "]" << "," << input->acceleration << "," << input->time << ")" << std::endl; 
                    break;
                case move_tool: 
                    std::cout << "movel(p[" << comma::join( input->values, ',' ) << "]" << optional_parameters( input ) << ")" << std::endl;
                    break;
                case speed_tool: 
                    if( !csv.has_field( "acceleration,time" ) ) { std::cerr << name() << ": command " << input->command << " requires acceleration and time" << std::endl; return 1; }
                    std::cout << "speedl([" << comma::join( input->values, ',' ) << "]" << "," << input->acceleration << "," << input->time << ")" << std::endl; 
                    break;
                case move_joints_to_pose:
                    std::cout << "movej(p[" << comma::join( input->values, ',' ) << "]" << optional_parameters( input ) << ")" << std::endl; 
                    break;
                default:
                    std::cerr << name() << ": expected a command code, got" << input->command << std::endl; return 1;
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name() << ": " << ex.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception" << std::endl; return 1; }
}
