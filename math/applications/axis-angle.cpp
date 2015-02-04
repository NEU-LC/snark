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
    std::cerr << "convert axis angle representation of rotaion to Euler angles" << std::endl;
    std::cerr << std::endl;
    std::cerr << "conventions:" << std::endl;
    std::cerr << "    axis angle vector is a 3d vector that defines the axis of rotation and whose norm gives the angle of rotation " << std::endl;
    std::cerr << "    Euler angles follow the z-y-x convention with yaw indicating rotation around z, pitch around y, and roll around x" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields:" << std::endl;
    std::cerr << "    input: coordinates of the axis angle vector (default: x,y,z)" << std::endl;
    std::cerr << "    output: Euler angles (default: roll,pitch,yaw)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-h: show more help" << std::endl;
    std::cerr << "    --output-format: show binary output format" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    echo 1,2,3 | axis-angle --fields=x,y,z --output-fields=roll,pitch,yaw" << std::endl;
    std::cerr << "    cat axis-angle.bin | axis-angle --binary=3d --fields=x,y,z --output-fields=roll,pitch,yaw | csv-from-bin 3d" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << "csv options:" << std::endl << comma::csv::options::usage( "x,y,z" ) << std::endl; }
    exit( -1 );
}

struct euler_t
{
    euler_t() : roll( 0 ), pitch( 0 ), yaw( 0 ) {}
    euler_t( Eigen::Vector3d p ) 
    {
        double angle = p.norm();
        if( comma::math::equal( angle, 0) ) 
        {
            roll = 0;
            pitch = 0;
            yaw = 0;
        }
        else
        {
            p.normalize();
            Eigen::Vector3d v = Eigen::AngleAxisd( angle, p ).toRotationMatrix().eulerAngles(2, 1, 0);
            roll = v[2];
            pitch = v[1];
            yaw = v[0];
        }
    }
    double roll;
    double pitch;
    double yaw;
};

struct axis_angle_t
{
    axis_angle_t() : coordinates( Eigen::Vector3d::Zero() ) {}
    axis_angle_t( euler_t e )
    {
        Eigen::AngleAxisd yaw = Eigen::AngleAxisd( e.yaw, Eigen::Vector3d::UnitZ() );
        Eigen::AngleAxisd pitch = Eigen::AngleAxisd( e.pitch, Eigen::Vector3d::UnitY() );
        Eigen::AngleAxisd roll = Eigen::AngleAxisd( e.roll, Eigen::Vector3d::UnitX() );
        Eigen::AngleAxisd v( yaw * pitch * roll );
        coordinates = v.axis() * std::abs( v.angle() );
    }
    Eigen::Vector3d coordinates;
};

namespace comma { namespace visiting {

template <> struct traits< euler_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, euler_t& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const euler_t& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};

template <> struct traits< axis_angle_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, axis_angle_t& p, Visitor& v )
    {
        v.apply( "cordinates", p.coordinates );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const axis_angle_t& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{    
    try
    {
        comma::command_line_options options( ac, av, usage );
        options.assert_mutually_exclusive( "--to-euler,--from-euler" );
        bool to_euler = options.exists( "--to-euler" );
        bool from_euler = options.exists( "--from-euler" );
        std::string default_input_fields;
        std::string default_output_fields;
        if( to_euler )
        {
            default_input_fields = "x,y,z";
            default_output_fields = "roll,pitch,yaw";
        }
        else if ( from_euler )
        {
            default_input_fields = "roll,pitch,yaw";
            default_output_fields = "x,y,z";
        }
        else
        {
            COMMA_THROW( comma::exception, "axis-angle: expected either --to-euler or --from-euler" );
        }
        comma::csv::options csv_in( options, default_input_fields );
        csv_in.full_xpath = false;
        comma::csv::options csv_out( csv_in );
        csv_out.full_xpath = false;
        csv_out.fields = options.value< std::string >( "--output-fields", default_output_fields );
        std::string output_format =  comma::csv::format::value< euler_t >( csv_out.fields, true );
        if( csv_out.binary() ) { csv_out.format( output_format ); }
        if( options.exists( "--output-format" ) ) { std::cout << output_format << std::endl; return 0; }
        if( to_euler ) 
        {
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv_in );
            comma::csv::output_stream< euler_t > ostream( std::cout, csv_out );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) break;
                ostream.write( euler_t( *p ) );
            }
        }
        if( from_euler )
        {
            comma::csv::input_stream< euler_t > istream( std::cin, csv_in );
            comma::csv::output_stream< axis_angle_t > ostream( std::cout, csv_out );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const euler_t* p = istream.read();
                if( !p ) break;
                ostream.write( axis_angle_t( *p ) );
            }            
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "axis-angle: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "axis-angle: unknown exception" << std::endl; }
}

