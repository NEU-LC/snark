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

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/options.h>
#include <comma/math/compare.h>
#include "../../visiting/eigen.h"
#include "../rotation_matrix.h"

static std::string name() { return "math-rotation-convert"; }

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert between different representations of rotation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "DEPRECATED; WILL BE REMOVED SOON; please use math-eigen rotation instead" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --from: input type" << std::endl;
    std::cerr << "    --to: desired output type" << std::endl;
    std::cerr << "    --fields: input fields (for other csv options run '"<< name() << " -h -v')" << std::endl;
    std::cerr << "    --output-fields: desired output fields" << std::endl;
    std::cerr << "    --output-format: show binary output format" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-h: show csv options" << std::endl;
    std::cerr << std::endl;
    std::cerr << "types:" << std::endl;
    std::cerr << "    axis-angle: coordinates of an axis-angle vector whose norm is the angle of rotation in [0,pi]" << std::endl;
    std::cerr << "    euler: Euler angles using z-y'-x'' (yaw-pitch-roll) sequence of intrinsic rotations with roll and yaw in [-pi,pi] and pitch in [-pi/2,pi/2]" << std::endl;
    std::cerr << "    quaternion: coordiantes of a normalized quaternion (a general quaternion in input will be normalized if possible)" << std::endl;
    std::cerr << std::endl; 
    std::cerr << "default fields:" << std::endl;
    std::cerr << "    axis-angle: x,y,z" << std::endl;
    std::cerr << "    euler: roll,pitch,yaw" << std::endl;
    std::cerr << "    quaternion: x,y,z,w" << std::endl;
    std::cerr << std::endl;     
    std::cerr << "examples:" << std::endl;
    std::cerr << "    echo 0.1,0.2,0.3 | " << name() << " --from euler --to axis-angle" << std::endl;
    std::cerr << "    echo 0.1,0.2,0.3 | " << name() << " --from euler --fields=roll,pitch,yaw --to axis-angle --output-fields=x,y" << std::endl;
    std::cerr << "    echo 0.1,0.2,0.3 | " << name() << " --from euler --to quaternion" << std::endl;
    std::cerr << "    cat axis-angle.bin | " << name() << " --from axis-angle --to euler --binary=3d" << std::endl;
    std::cerr << "    " << name() << " --from axis-angle --to euler --output-fields=roll,pitch,yaw --output-format" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << "csv options:" << std::endl << comma::csv::options::usage() << std::endl; }
    exit( -1 );
}

typedef Eigen::Vector3d axis_angle_t;
typedef Eigen::Quaterniond quaternion_t;

template< typename T > T cast_to( const Eigen::Vector3d& );
template<> Eigen::AngleAxisd cast_to< Eigen::AngleAxisd >( const Eigen::Vector3d& axis_angle )
{
    double angle = axis_angle.norm();
    if( comma::math::equal( angle, 0) ) 
    { 
        return Eigen::AngleAxisd(0, Eigen::Vector3d(0,0,1) ); 
    }
    else
    {
        Eigen::Vector3d axis( axis_angle );
        axis.normalize();
        return Eigen::AngleAxisd( angle, axis );
    }
}

struct euler_t
{
    euler_t( double roll_ = 0, double pitch_ = 0, double yaw_ = 0 ) : roll( roll_ ), pitch( pitch_ ), yaw( yaw_ ) {}
    euler_t( const Eigen::Vector3d& v ) : roll( v.x() ), pitch( v.y() ), yaw( v.z() ) {}
    Eigen::Vector3d as_vector() const { return Eigen::Vector3d( roll, pitch, yaw ); };
    double roll;
    double pitch;
    double yaw;
};

namespace comma { namespace visiting {

template <> struct traits< euler_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, euler_t& e, Visitor& v )
    {
        v.apply( "roll", e.roll );
        v.apply( "pitch", e.pitch );
        v.apply( "yaw", e.yaw );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const euler_t& e, Visitor& v )
    {
        v.apply( "roll", e.roll );
        v.apply( "pitch", e.pitch );
        v.apply( "yaw", e.yaw );
    }
};

} } // namespace comma { namespace visiting {

template< typename from_t, typename to_t > to_t convert( const from_t& from );
template<> euler_t convert< quaternion_t, euler_t >( const quaternion_t& q ) { return euler_t( snark::rotation_matrix( q ).roll_pitch_yaw() ); }
template<> euler_t convert< axis_angle_t, euler_t >( const axis_angle_t& axis_angle ) { return euler_t( snark::rotation_matrix( cast_to< Eigen::AngleAxisd >( axis_angle ) ).roll_pitch_yaw() ); }
template<> quaternion_t convert< euler_t, quaternion_t >( const euler_t& e ) { return snark::rotation_matrix( e.as_vector() ).quaternion(); }
template<> axis_angle_t convert< euler_t, axis_angle_t >( const euler_t& e ) { return snark::rotation_matrix( e.as_vector() ).angle_axis(); }
template<> axis_angle_t convert< quaternion_t, axis_angle_t >( const quaternion_t& q ) { return snark::rotation_matrix( q ).angle_axis(); }
template<> quaternion_t convert< axis_angle_t, quaternion_t >( const axis_angle_t& axis_angle ) {  return snark::rotation_matrix( cast_to< Eigen::AngleAxisd >( axis_angle ) ).quaternion(); }

template< typename from_t, typename to_t >
struct pipeline_t
{
    comma::command_line_options options;
    comma::csv::options csv_in;
    comma::csv::options csv_out;
    std::string output_format;
    template< typename T > std::string names() { return comma::join( comma::csv::names< T >(), comma::csv::options().delimiter ); }
    pipeline_t( const comma::command_line_options& options_ ) : options( options_ ), csv_in( options, names< from_t >() ), csv_out( csv_in )
    {
        csv_out.fields = options.value< std::string >( "--output-fields", names< to_t >() );
        output_format = comma::csv::format::value< to_t >( csv_out.fields, false );
        if( csv_out.binary() ) { csv_out.format( output_format ); }
    }
    void run()
    {
        comma::csv::input_stream< from_t > istream( std::cin, csv_in );
        comma::csv::output_stream< to_t > ostream( std::cout, csv_out );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const from_t* from = istream.read();
            if( !from ) break;
            to_t to = convert< from_t, to_t >( *from );
            ostream.write( to );
        }
    }
};

template< typename T >
void process( T pipeline )
{
    if( pipeline.options.exists( "--output-format" ) ) { std::cout << pipeline.output_format << std::endl; return; }
    pipeline.run();
}

int main( int ac, char** av )
{    
    try
    {
        comma::command_line_options options( ac, av, usage );
        options.unnamed( "--help,-h,--verbose,-v,--output-format", "-.*,--.*" );
        if( !options.exists( "--from" ) || !options.exists( "--to" ) ) { std::cerr << name() << ": please specify --from and --to, e.g. --from=euler --to=axis-angle" << std::endl; return 1;}
        std::string to = options.value< std::string >( "--to" );
        std::string from = options.value< std::string >( "--from" );
        if( to == from ) { std::cerr << name() << ": expected different --from and --to, got --from=" << from << " and --to=" << to << std::endl; return 1; }
        if( from == "axis-angle" && to == "euler" ) { process( pipeline_t< axis_angle_t, euler_t >( options ) ); }
        else if( from == "euler" && to == "axis-angle" ) { process( pipeline_t< euler_t, axis_angle_t >( options ) ); }
        else if( from == "quaternion" && to == "euler" ) { process( pipeline_t< quaternion_t, euler_t >( options ) ); }
        else if( from == "euler" && to == "quaternion" ) { process( pipeline_t< euler_t, quaternion_t >( options ) ); }
        else if( from == "quaternion" && to == "axis-angle" ) { process( pipeline_t< quaternion_t, axis_angle_t >( options ) ); }
        else if( from == "axis-angle" && to == "quaternion" ) { process( pipeline_t< axis_angle_t, quaternion_t >( options ) ); }
        else { std::cerr << name() << ": expected --to and --from to be axis-angle, euler, or quaternion, got --to=" << to << " and --from=" << from << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "axis-angle: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "axis-angle: unknown exception" << std::endl; }
}
