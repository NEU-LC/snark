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
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <snark/visiting/eigen.h>
#include "./frame.h"

static void usage()
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
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"100,100,0,$(math-deg2rad 0,0,90)\"" << std::endl;
    std::cerr << "20101010T101010,100,101,0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"0,-3,2,$(math-deg2rad 90,0,0)\"" << std::endl;
    std::cerr << "20101010T101010,1,-3,2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"3,2,0,$(math-deg2rad 0,90,0)\"" << std::endl;
    std::cerr << "20101010T101010,3,2,-1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 20101010T101010,1,0,0 | points-frame --from \"-2,1,3,$(math-deg2rad 0,0,90)\"" << std::endl;
    std::cerr << "20101010T101010,-2,1,3" << std::endl;
    std::cerr << std::endl;
    std::cerr << "echo 1,2,3 | points-frame --from \"3,2,1\" --fields=x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "cat log.csv | points-frame --from nav.csv > log.world.csv" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

bool timestampRequired = false; // quick and dirty
boost::optional< boost::posix_time::time_duration > maxGap;

std::vector< boost::shared_ptr< snark::applications::frame > > parseframes( const std::vector< std::string >& values
                                                    , const comma::csv::options& options
                                                    , bool discardOutOfOrder
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
                if( csv.fields == "" )
                {
                    csv.fields = "t,x,y,z,roll,pitch,yaw";
                    if( options.binary() ) { csv.format( "t,6d" ); }
                }
                csv.full_xpath = false;
                outputframe = outputframe || comma::name_value::map( stripped, "filename" ).exists( "output-frame" );
                timestampRequired = true;
                frames.push_back( boost::shared_ptr< snark::applications::frame >( new snark::applications::frame( csv, discardOutOfOrder, maxGap, outputframe, to[i], interpolate, rotation_present ) ) );
            }
        }
    }
    return frames;
}

void run( const std::vector< boost::shared_ptr< snark::applications::frame > >& frames, const comma::csv::options& csv )
{
    comma::signal_flag shutdownFlag;
    comma::csv::input_stream< snark::applications::frame::point_type > istream( std::cin, csv );
    comma::csv::output_stream< snark::applications::frame::point_type > ostream( std::cout, csv );
    if( !ostream.is_binary() ) { ostream.ascii().precision( 12 ); } // quick and dirty, brush up later
    // ---------------------------------------------------
    // outputting frame: quick and dirty, uglier than britney spears! brush up later
    unsigned int outputframeCount = 0;
    for( std::size_t i = 0; i < frames.size(); ++i ) { if( frames[i]->outputframe ) { ++outputframeCount; } }
    boost::scoped_ptr< comma::csv::binary< snark::applications::frame::point_type > > binaryPoint;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::frame::point_type > > asciiPoint;
    boost::scoped_ptr< comma::csv::binary< snark::applications::position > > binaryframe;
    boost::scoped_ptr< comma::csv::ascii< snark::applications::position > > asciiframe;
    if( outputframeCount > 0 )
    {
        if( csv.binary() )
        {
            binaryPoint.reset( new comma::csv::binary< snark::applications::frame::point_type >( csv ) );
            binaryframe.reset( new comma::csv::binary< snark::applications::position >() );
        }
        else
        {
            asciiPoint.reset( new comma::csv::ascii< snark::applications::frame::point_type >( csv ) );
            asciiframe.reset( new comma::csv::ascii< snark::applications::position >() );
        }
    }
    // ---------------------------------------------------

    std::string outputBuf;
    if( csv.binary() ) { outputBuf.resize( csv.format().size() ); }

    while( !shutdownFlag )
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
            if( discarded ) { break; }
            if( c == NULL ) { return; }
            converted = *c;
        }
        if( discarded ) { continue; }
        if( outputframeCount > 0 ) // quick and dirty, and probably slow; brush up later
        {
            // ---------------------------------------------------
            // outputting frame: quick and dirty, uglier than britney spears! brush up later
            if( csv.binary() )
            {
                static const std::size_t pointSize = csv.format().size();
                static const std::size_t positionSize = comma::csv::format( comma::csv::format::value< snark::applications::position >() ).size();
                static const std::size_t size = pointSize + outputframeCount * positionSize;
                std::vector< char > buf( size );
                ::memset( &buf[0], 0, size );
                ::memcpy( &buf[0], istream.binary().last(), pointSize );
                binaryPoint->put( converted, &buf[0] );
                unsigned int count = 0;
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    binaryframe->put( frames[i]->last(), &buf[0] + pointSize + count * positionSize );
                    ++count;
                }
                std::cout.write( &buf[0], size );
            }
            else
            {
                std::string s = comma::join( istream.ascii().last(), csv.delimiter );
                asciiPoint->put( converted, s );
                for( std::size_t i = 0; i < frames.size(); ++i )
                {
                    if( !frames[i]->outputframe ) { continue; }
                    std::vector< std::string > f;
                    asciiframe->put( frames[i]->last(), f );
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
                ::memcpy( &outputBuf[0], istream.binary().last(), csv.format().size() );
                ostream.write( converted, outputBuf );
            }
            else
            {
                ostream.write( converted, istream.ascii().last() );
            }
        }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) || ac == 1 ) { usage(); }
        bool discardOutOfOrder = options.exists( "--discard-out-of-order,--discard" );
        bool from = options.exists( "--from" );
        bool to = options.exists( "--to" );
        if( !to && !from ) { COMMA_THROW( comma::exception, "please specify either --to or --from" ); }
        bool interpolate = !options.exists( "--no-interpolate" );

        std::vector< std::string > names = options.names();
        std::vector< bool > toVector;
        for( unsigned int i = 0u; i < names.size(); i++ )
        {
            if( names[i] == "--from" )
            {
                toVector.push_back( false );
            }
            else if( names[i] == "--to" )
            {
                toVector.push_back( true );
            }
        }

        if( options.exists( "--max-gap" ) )
        {
            double d = options.value< double >( "--max-gap" );
            if( !comma::math::less( 0, d ) ) { std::cerr << "points-frame: expected --max-gap in seconds, got " << maxGap << std::endl; usage(); }
            maxGap = boost::posix_time::seconds( int( d ) ) + boost::posix_time::microseconds( int( 1000000.0 * ( d - int( d ) ) ) );
        }
        comma::csv::options csv( options );
        if( csv.fields == "" ) { csv.fields="t,x,y,z"; }
        bool rotation_present = comma::csv::fields_exist( csv.fields, "roll,pitch,yaw" );
        std::vector< boost::shared_ptr< snark::applications::frame > > frames = parseframes( options.values< std::string >( "--from,--to" )
                                                                      , csv
                                                                      , discardOutOfOrder
                                                                      , options.exists( "--output-frame" )
                                                                      , toVector
                                                                      , interpolate
                                                                      , rotation_present );
        //if( timestampRequired ) { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "t,x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields t,x,y,z; got " << csv.fields ); } }
        //else { if( csv.fields != "" && !comma::csv::namesValid( comma::split( csv.fields, ',' ), comma::split( "x,y,z", ',' ) ) ) { COMMA_THROW( comma::exception, "expected mandatory fields x,y,z; got " << csv.fields ); } }
        if( timestampRequired ) { if( csv.fields != "" && !comma::csv::fields_exist( csv.fields, "t" ) ) { COMMA_THROW( comma::exception, "expected mandatory field t; got " << csv.fields ); } }
        csv.precision = 12;
        run( frames, csv );
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-frame: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-frame: unknown exception" << std::endl; }
}
