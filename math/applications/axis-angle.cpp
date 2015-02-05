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
#include <snark/visiting/eigen.h>

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert between axis angle representation of rotaion and the Euler angles" << std::endl;
    std::cerr << std::endl;
    std::cerr << "conventions:" << std::endl;
    std::cerr << "    axis angle vector is a 3d vector that defines the axis of rotation and whose norm gives the angle of rotation " << std::endl;
    std::cerr << "    Euler angles follow the z-y-x convention with yaw indicating rotation around z, pitch around y, and roll around x" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input/output fields:" << std::endl;
    std::cerr << "    coordinates of the axis angle vector (default: x,y,z)" << std::endl;
    std::cerr << "    Euler angles (default: roll,pitch,yaw)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --to-euler: convert axis angle vector to Euler angles" << std::endl;
    std::cerr << "    --from-euler: convert Euler angles to axis angle vector" << std::endl;
    std::cerr << "    --output-format: show binary output format" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-h: show more help" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    echo 0.1,0.2,0.3 | axis-angle --from-euler --fields=roll,pitch,yaw --output-fields=x,y,z" << std::endl;
    std::cerr << "    cat axis-angle.bin | axis-angle --to-euler --binary=3d --fields=x,y,z --output-fields=roll,pitch,yaw | csv-from-bin 3d" << std::endl;
    std::cerr << "    axis-angle --to-euler --output-fields=roll,pitch,yaw --output-format" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << "csv options:" << std::endl << comma::csv::options::usage( "x,y,z" ) << std::endl; }
    exit( -1 );
}

struct euler_t
{
    euler_t( double roll_ = 0, double pitch_ = 0, double yaw_ = 0 ) : roll( roll_ ), pitch( pitch_ ), yaw( yaw_ ) {}
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

euler_t transform( const Eigen::Vector3d& axis_angle_vector )
{
    double angle = axis_angle_vector.norm();
    if( comma::math::equal( angle, 0) ) { return euler_t(); }
    else
    {
        Eigen::Vector3d axis_angle_vector_normalised( axis_angle_vector );
        axis_angle_vector_normalised.normalize();
        Eigen::Vector3d v = Eigen::AngleAxisd( angle, axis_angle_vector_normalised ).toRotationMatrix().eulerAngles(2, 1, 0);
        return euler_t( v[2], v[1], v[0] );
    }    
}

Eigen::Vector3d transform( const euler_t& e )
{
    Eigen::AngleAxisd yaw = Eigen::AngleAxisd( e.yaw, Eigen::Vector3d::UnitZ() );
    Eigen::AngleAxisd pitch = Eigen::AngleAxisd( e.pitch, Eigen::Vector3d::UnitY() );
    Eigen::AngleAxisd roll = Eigen::AngleAxisd( e.roll, Eigen::Vector3d::UnitX() );
    Eigen::AngleAxisd v( yaw * pitch * roll );
    return v.axis() * std::abs( v.angle() );
}

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
            to_t to = transform( *from );
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
        options.unnamed( "--help,-h,--verbose,-v,--to-euler,--from-euler,--output-format", "-.*,--.*" );
        options.assert_mutually_exclusive( "--to-euler,--from-euler" );
        bool to_euler = options.exists( "--to-euler" );
        bool from_euler = options.exists( "--from-euler" );
        if( !to_euler && !from_euler ) { COMMA_THROW( comma::exception, "axis-angle: expected either --to-euler or --from-euler" ); }
        if( to_euler ) { process( pipeline_t< Eigen::Vector3d, euler_t >( options ) ); }
        if( from_euler ) { process( pipeline_t< euler_t, Eigen::Vector3d >( options ) ); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "axis-angle: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "axis-angle: unknown exception" << std::endl; }
}

