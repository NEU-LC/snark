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


#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/ascii.h>
#include <comma/csv/binary.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include "frame.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "convert timestamped Cartesian points from a reference frame to a target coordinate frame[s]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-frame <options> > points.converted.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --from <frame> : convert points from frames to the reference frame," << std::endl;
    std::cerr << "    --to <frame> : convert points from the reference frame to frame" << std::endl;
    std::cerr << "        <frame> ::= <frame>[+<frames>]" << std::endl;
    std::cerr << "        <frame> : name of file with timestamped nav data" << std::endl;
    std::cerr << "                  or <x>,<y>,<z>[,<roll>,<pitch>,<yaw>]" << std::endl;
    std::cerr << "                  nav data in file: <t>,<x>,<y>,<z>,<roll>,<pitch>,<yaw>" << std::endl;
    std::cerr << "    --discard-out-of-order,--discard : if present, discard out of order points silently" << std::endl;
    std::cerr << "    --max-gap <seconds> : max valid time gap between two successive nav solutions;" << std::endl;
    std::cerr << "                          if exceeded, input points between those two timestamps" << std::endl;
    std::cerr << "                          will be discarded, thus use --discard, too; default: infinity" << std::endl;
    std::cerr << "    --no-interpolate: don't interpolate, use nearest point instead" << std::endl;
    std::cerr << "    --output-frame : output each frame for each point" << std::endl;
    std::cerr << "                     can be individually specified for a frame, e.g.:" << std::endl;
    std::cerr << "                     --from \"novatel.csv;output-frame\"" << std::endl;
    std::cerr << "    --position=[<x>,<y>,<z>,<roll>,<pitch>,<yaw>]: default position, if 'frame' fields are present" << std::endl;
    std::cerr << "        example scenario: you have a gps unit trajectory and gps unit offset from the centre of the vehicle" << std::endl;
    std::cerr << "                          now, you want to find the trajectory of the centre of the vehicle" << std::endl;
    std::cerr << "                          in this case, the gps unit trajectory is the frame, which is variable," << std::endl;
    std::cerr << "                          while the point to georeference (the vehicle centre) is fixed" << std::endl;
    std::cerr << "                          then you could run:" << std::endl;
    std::cerr << "                              cat gps.unit.trajectory.csv | points-frame --fields frame --position <gps unit offset relative to vehicle centre>" << std::endl;
    std::cerr << "                          this is equivalent to a much longer version:" << std::endl;
    std::cerr << "                              cat gps.unit.trajectory.csv \\" << std::endl;
    std::cerr << "                                  | csv-paste - value=<gps unit offset relative to vehicle centre> \\" << std::endl;
    std::cerr << "                                  | points-frame --fields frame,position \\" << std::endl;
    std::cerr << "                                  | csv-shuffle --fields ,,,,,,,,,,,,x,y,z,roll,pitch,yaw --output-fields x,y,z,roll,pitch,yaw \\" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    fields" << std::endl;
    std::cerr << "        t: timestamp" << std::endl;
    std::cerr << "        x,y,z: coordinates" << std::endl;
    std::cerr << "        roll,pitch,yaw: rotation" << std::endl;
    std::cerr << "        frame/coordinates/x,frame/coordinates/y,frame/coordinates/z,frame/orientation/roll,frame/orientation/pitch,frame/orientation/yaw: frame" << std::endl;
    std::cerr << "            if frame fields are present, get frame from them, apply to" << std::endl;
    std::cerr << "            coordinates (x,y,z) and output input line with converted" << std::endl;
    std::cerr << "            coordinates and rotation appended as x,y,z,roll,pitch,yaw" << std::endl;
    std::cerr << "            --from: if present, conversion is from reference frame, default" << std::endl;
    std::cerr << "            --to: if present, conversion is to reference frame" << std::endl;
    std::cerr << "        default: t,x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    IMPORTANT: <frame> is the transformation from reference frame to frame" << std::endl;
    std::cerr << std::endl;
    std::cerr << "               If still confused, try simple coordinate transformations," << std::endl;
    std::cerr << "               just like examples below." << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples (try them):" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"100,100,0\"" << std::endl;
    std::cerr << "20101010T101010,101,100,0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"100,100,0,0,0,$(math-deg2rad 90)\"" << std::endl;
    std::cerr << "20101010T101010,100,101,0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"0,-3,2,$(math-deg2rad 90),0,0\"" << std::endl;
    std::cerr << "20101010T101010,1,-3,2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"3,2,0,0,$( math-deg2rad 90 ),0\"" << std::endl;
    std::cerr << "20101010T101010,3,2,-1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"-2,1,3,0,0,$(math-deg2rad 90)\"" << std::endl;
    std::cerr << "20101010T101010,-2,2,3" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 1,2,3 | points-frame --from \"3,2,1\" --fields=x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "cat log.csv | points-frame --from nav.csv > log.world.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "frame is passed on stdin" << std::endl;
    std::cerr << "    translation and rotation" << std::endl;
    std::cerr << "        echo 1,2,3,10,20,30,0,0,0 | points-frame --fields x,y,z,frame" << std::endl;
    std::cerr << "        1,2,3,10,20,30,0,0,0,11,22,33,0,-0,0" << std::endl;
    std::cerr << "    translation only" << std::endl;
    std::cerr << "        echo 1,2,3,10,20,30 | points-frame --fields x,y,z,frame/x,frame/y,frame/z" << std::endl;
    std::cerr << "        1,2,3,10,20,30,11,22,33,0,-0,0" << std::endl;
    std::cerr << "    rotation only" << std::endl;
    std::cerr << "        echo 1,2,3,0,$( math-deg2rad 90 ),0 | points-frame --fields x,y,z,frame/roll,frame/pitch,frame/yaw" << std::endl;
    std::cerr << "        1,2,3,0,1.57079632679,0,3,2,-1,0,1.57079632679,0" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

bool timestamp_required = false; // quick and dirty
boost::optional< boost::posix_time::time_duration > max_gap;
uint64_t discarded_counter=0;
boost::posix_time::time_duration discarded_time_diff_max;

std::vector< boost::shared_ptr< snark::applications::frame > > parse_frames( const std::vector< std::string >& values
                                                    , const comma::csv::options& options
                                                    , bool discard_out_of_order
                                                    , bool outputframe
                                                    , std::vector< bool > to
                                                    , bool interpolate
                                                    , bool rotation_present )
{
    std::vector< boost::shared_ptr< snark::applications::frame > > frames;
    for( std::size_t i = 0; i < values.size(); ++i )
    {
        std::vector< std::string > s = comma::split( values[i], '+' );
        for( std::size_t j = 0; j < s.size(); ++j )
        {
            std::string stripped = comma::strip( s[j], ' ' );
            std::vector< std::string > t = comma::split( stripped, ',' );
            boost::optional< snark::applications::position > position;
            try
            {
                switch( t.size() )
                {
                    case 3:
                        position = snark::applications::position( comma::csv::ascii< Eigen::Vector3d >().get( t ) );
                        break;
                    case 6:
                        position = comma::csv::ascii< snark::applications::position >().get( t );
                        break;
                    default:
                        break;
                }
            }
            catch( ... ) {}
            if( position )
            {
                frames.push_back( boost::shared_ptr< snark::applications::frame >( new snark::applications::frame( *position, to[i], interpolate, rotation_present ) ) );
            }
            else
            {
                comma::csv::options csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( stripped );
                csv.full_xpath = false;
                if( csv.fields == "" )
                {
                    csv.fields = "t,x,y,z,roll,pitch,yaw";
                    if( options.binary() ) { csv.format( "t,6d" ); }
                }
                outputframe = outputframe || comma::name_value::map( stripped, "filename" ).exists( "output-frame" );
                timestamp_required = true;
                frames.push_back( boost::shared_ptr< snark::applications::frame >( new snark::applications::frame( csv, discard_out_of_order, max_gap, outputframe, to[i], interpolate, rotation_present ) ) );
            }
        }
    }
    return frames;
}

void run( const std::vector< boost::shared_ptr< snark::applications::frame > >& frames, const comma::csv::options& csv )
{
    comma::signal_flag is_shutdown;
    comma::csv::input_stream< snark::applications::frame::point_type > istream( std::cin, csv );
    comma::csv::output_stream< snark::applications::frame::point_type > ostream( std::cout, csv );
    // ---------------------------------------------------
    // outputting frame: quick and dirty, uglier than britney spears! brush up later
    unsigned int output_frame_count = 0;
    for( std::size_t i = 0; i < frames.size(); ++i ) { if( frames[i]->outputframe ) { ++output_frame_count; } }
    boost::scoped_ptr< comma::csv::binary< snark::applications::frame::point_type > > binary_point;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::frame::point_type > > ascii_point;
    boost::scoped_ptr< comma::csv::binary< snark::applications::position > > binary_frame;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::position > > ascii_frame;
    if( output_frame_count > 0 )
    {
        if( csv.binary() )
        {
            binary_point.reset( new comma::csv::binary< snark::applications::frame::point_type >( csv ) );
            binary_frame.reset( new comma::csv::binary< snark::applications::position >() );
        }
        else
        {
            ascii_point.reset( new comma::csv::ascii< snark::applications::frame::point_type >( csv ) );
            ascii_frame.reset( new comma::csv::ascii< snark::applications::position >() );
        }
    }
    // ---------------------------------------------------

    std::string output_buf;
    if( csv.binary() ) { output_buf.resize( csv.format().size() ); }

    while( !is_shutdown )
    {
        const snark::applications::frame::point_type* p = istream.read();
        if( p == NULL ) { return; }
        snark::applications::frame::point_type converted = *p;
        const snark::applications::frame::point_type* c = NULL;
        bool discarded = false;
        for( std::size_t i = 0; i < frames.size(); ++i )
        {
            c = frames[i]->converted( converted );
            discarded = frames[i]->discarded();
            if( discarded ) 
            {
                if(discarded_time_diff_max < frames[i]->discarded_time_diff())
                    discarded_time_diff_max = frames[i]->discarded_time_diff();
                discarded_counter++;
                break; 
            }
            if( c == NULL ) { return; }
            converted = *c;
        }
        if( discarded ) { continue; }
        if( output_frame_count > 0 ) // quick and dirty, and probably slow; brush up later
        {
            // ---------------------------------------------------
            // outputting frame: quick and dirty, uglier than britney spears! brush up later
            if( csv.binary() )
            {
                static const std::size_t point_size = csv.format().size();
                static const std::size_t position_size = comma::csv::format( comma::csv::format::value< snark::applications::position >() ).size();
                static const std::size_t size = point_size + output_frame_count * position_size;
                std::vector< char > buf( size );
                ::memset( &buf[0], 0, size );
                ::memcpy( &buf[0], istream.binary().last(), point_size );
                binary_point->put( converted, &buf[0] );
                unsigned int count = 0;
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    binary_frame->put( frames[i]->last(), &buf[0] + point_size + count * position_size );
                    ++count;
                }
                std::cout.write( &buf[0], size );
                if( csv.flush ) { std::cout.flush(); }
            }
            else
            {
                std::string s = comma::join( istream.ascii().last(), csv.delimiter );
                ascii_point->put( converted, s );
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    std::vector< std::string > f;
                    ascii_frame->put( frames[i]->last(), f );
                    s += ( csv.delimiter + comma::join( f, csv.delimiter ) );
                }
                std::cout << s << std::endl;
            }
            // ---------------------------------------------------
        }
        else
        {
            if( csv.binary() )
            {
                ::memcpy( &output_buf[0], istream.binary().last(), csv.format().size() );
                ostream.write( converted, output_buf );
            }
            else
            {
                ostream.write( converted, istream.ascii().last() );
            }
        }
    }
}

struct position_and_frame
{
    snark::applications::position position;
    snark::applications::position frame;
    position_and_frame( const snark::applications::position& position = snark::applications::position(), const snark::applications::position& frame = snark::applications::position() ): position( position ), frame( frame ) {}
};

namespace comma { namespace visiting {

template <> struct traits< position_and_frame >
{
    template < typename Key, class Visitor > static void visit( const Key&, position_and_frame& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "frame", p.frame );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const position_and_frame& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "frame", p.frame );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = false;
        if( csv.fields == "" ) { csv.fields="t,x,y,z"; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        bool stdin_has_frame = false;
        for( unsigned int i = 0; i < v.size() && !stdin_has_frame; ++i ) { stdin_has_frame = v[i] == "frame" || v[i] == "frame/x" || v[i] == "frame/y" || v[i] == "frame/z" || v[i] == "frame/roll" || v[i] == "frame/pitch" || v[i] == "frame/yaw"; }
        bool rotation_present = false;
        for( unsigned int i = 0; i < v.size() && !rotation_present; ++i ) { rotation_present = v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw"; }
        if( stdin_has_frame ) // quick and dirty
        {
            for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "x" || v[i] == "y" || v[i] == "z" || v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "position/" + v[i]; } }
            csv.fields = comma::join( v, ',' );
            csv.full_xpath = true;
            comma::csv::input_stream< position_and_frame > is( std::cin, csv, position_and_frame( comma::csv::ascii< snark::applications::position >().get( options.value< std::string >( "--position", "0,0,0,0,0,0" ) ) ) );
            comma::csv::options output_csv;
            output_csv.flush = csv.flush;
            if( csv.binary() ) { output_csv.format( comma::csv::format::value< snark::applications::position >() ); }
            comma::csv::output_stream< snark::applications::position > os( std::cout, output_csv );
            comma::csv::tied< position_and_frame, snark::applications::position > tied( is, os );
            bool from = !options.exists( "--to" );
            while( is.ready() || std::cin.good() )
            {
                const position_and_frame* p = is.read();
                if( !p ) { break; }
                Eigen::Translation3d translation( p->frame.coordinates );
                Eigen::Matrix3d rotation = snark::rotation_matrix::rotation( p->frame.orientation );
                Eigen::Affine3d transform = from ? ( translation * rotation ) : ( rotation.transpose() * translation.inverse() );
                Eigen::Matrix3d m = snark::rotation_matrix::rotation( p->position.orientation );
                if( from ) { m = rotation * m; } else { m = rotation.transpose() * m; }
                tied.append( snark::applications::position( transform * p->position.coordinates, snark::rotation_matrix::roll_pitch_yaw( m ) ) );
            }
            return 0;
        }
        if( options.exists( "--position" ) ) { std::cerr << "points-frame: --position given, but no frame fields on stdin: not supported, yet" << std::endl; return 1; }
        bool discard_out_of_order = options.exists( "--discard-out-of-order,--discard" );
        bool from = options.exists( "--from" );
        bool to = options.exists( "--to" );
        if( !to && !from ) { COMMA_THROW( comma::exception, "please specify either --to or --from" ); }
        bool interpolate = !options.exists( "--no-interpolate" );
        std::vector< std::string > names = options.names();
        std::vector< bool > to_vector;
        for( unsigned int i = 0u; i < names.size(); i++ )
        {
            if( names[i] == "--from" ) { to_vector.push_back( false ); }
            else if( names[i] == "--to" ) { to_vector.push_back( true ); }
        }
        if( options.exists( "--max-gap" ) )
        {
            double d = options.value< double >( "--max-gap" );
            if( !comma::math::less( 0, d ) ) { std::cerr << "points-frame: expected --max-gap in seconds, got " << d << std::endl; usage(); }
            max_gap = boost::posix_time::seconds( int( d ) ) + boost::posix_time::microseconds( int( 1000000.0 * ( d - int( d ) ) ) );
        }
        std::vector< boost::shared_ptr< snark::applications::frame > > frames = parse_frames( options.values< std::string >( "--from,--to" )
                                                                                            , csv
                                                                                            , discard_out_of_order
                                                                                            , options.exists( "--output-frame" )
                                                                                            , to_vector
                                                                                            , interpolate
                                                                                            , rotation_present );
        //if( timestamp_required ) { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "t,x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields t,x,y,z; got " << csv.fields ); } }
        //else { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields x,y,z; got " << csv.fields ); } }
        if( timestamp_required ) { if( csv.fields != "" && !comma::csv::fields_exist( csv.fields, "t" ) ) { COMMA_THROW( comma::exception, "expected mandatory field t; got " << csv.fields ); } }
        run( frames, csv );
        if( discarded_counter ) { std::cerr << "points-frame: discarded " << discarded_counter << " points; max time diff: " << discarded_time_diff_max.total_microseconds() <<" microseconds" << std::endl; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-frame: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-frame: unknown exception" << std::endl; }
}
