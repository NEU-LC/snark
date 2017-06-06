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

/// @author vsevolod vlaskine

#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <boost/optional.hpp>
#include <boost/unordered_set.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../voxel_grid.h"
#include "../voxel_map.h"
#include "../../visiting/eigen.h"
#include "points-calc/plane_intersection.h"
#include "points-calc/plane_intersection_with_trajectory.h"
#include "points-calc/project.h"
#include "points-calc/vector_calc.h"
#include "points-calc/remove_outliers.h"

static comma::csv::options csv;
static bool verbose;
std::string app_name="points-calc";
typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;

static comma::csv::ascii< Eigen::Vector3d > ascii;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates from stdin, perform calculations, and output result to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    points-calc <operation> [<options>]" << std::endl;
    std::cerr << "    points-calc --help" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat points.csv | points-calc distance > results.csv" << std::endl;
    std::cerr << "    echo -e \"0\\n1\\n3\\n6\" | points-calc distance --fields x --next" << std::endl;
    std::cerr << "    cat points.csv | points-calc cumulative-distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc thin --resolution <resolution> > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc discretise --step <step> > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    angle-axis" << std::endl;
    std::cerr << "    cumulative-distance" << std::endl;
    std::cerr << "    cumulative-discretise,cumulative-discretize,sample" << std::endl;
    std::cerr << "    distance" << std::endl;
    std::cerr << "    discretise,discretize" << std::endl;
    std::cerr << "    find-outliers" << std::endl;
    std::cerr << "    local-min" << std::endl;
    std::cerr << "    local-max" << std::endl;
    std::cerr << "    nearest-min" << std::endl;
    std::cerr << "    nearest-max" << std::endl;
    std::cerr << "    nearest-point,nearest-any" << std::endl;
    std::cerr << "    nearest" << std::endl;
    std::cerr << "    plane-intersection" << std::endl;
    std::cerr << "    plane-intersection-with-trajectory" << std::endl;
    std::cerr << "    project-onto-line" << std::endl;
    std::cerr << "    project-onto-plane" << std::endl;
    std::cerr << "    thin" << std::endl;
    vector_calc::usage_list_operations();
    std::cerr << std::endl;
    std::cerr << "general options" << std::endl;
    std::cerr << "    usage: points-calc <option>" << std::endl;
    std::cerr << "    --help,-h: show help, --verbose for more detailed help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << std::endl;
    std::cerr << "common operation options" << std::endl;
    std::cerr << "    usage: points-calc <operation> <option>" << std::endl;
    std::cerr << "    --output-fields: print output field names and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << "operation details" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    angle-axis" << std::endl;
    std::cerr << "        angle-axis between subsequent points or, if input is pairs, between" << std::endl;
    std::cerr << "        the points of the same record. For subsequent points the first value" << std::endl;
    std::cerr << "        (or last if --next is given) is invalid as we don't yet have a pair of" << std::endl;
    std::cerr << "        points." << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options: " << std::endl;
    std::cerr << "            --next: subsequent points only, angle-axis to next point is appended" << std::endl;
    std::cerr << "                    (default: angle-axis to previous point is appended)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-distance" << std::endl;
    std::cerr << "        cumulative distance between subsequent points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-discretise, cumulative-discretize, sample" << std::endl;
    std::cerr << "        read input data and discretise intervals with --step along the whole" << std::endl;
    std::cerr << "        trajectory" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --step=<step>: linear step of discretisation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance" << std::endl;
    std::cerr << "        distance between subsequent points or, if input is pairs, between the" << std::endl;
    std::cerr << "        points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options: " << std::endl;
    std::cerr << "            --next: for subsequent points only, distance to next point" << std::endl;
    std::cerr << "                    is appended" << std::endl;
    std::cerr << "                    (default: distance to previous point is appended)" << std::endl;
    std::cerr << "                    fake zero is appended to the final point (since there is" << std::endl;
    std::cerr << "                    no next point)" << std::endl;
    std::cerr << "            --propagate: if the current point is the same as previous," << std::endl;
    std::cerr << "                    output distance from one step before" << std::endl;
    std::cerr << "                    e.g: ( echo 0,0,0 ; echo 0,0,1 ; echo 0,0,1 ) \\" << std::endl;
    std::cerr << "                             | points-calc distance --propagate" << std::endl;
    std::cerr << "                         0,0,0,0" << std::endl;
    std::cerr << "                         0,0,1,1" << std::endl;
    std::cerr << "                         0,0,1,1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    discretise, discretize" << std::endl;
    std::cerr << "        read input data and discretise intervals between adjacent points" << std::endl;
    std::cerr << "        with --step; skip discretised points that are closer to the end of" << std::endl;
    std::cerr << "        the interval than --tolerance" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --step=<step>: linear step of discretisation" << std::endl;
    std::cerr << "            --tolerance=<tolerance>: tolerance; default: " << std::numeric_limits< double >::min() << std::endl;
    std::cerr << std::endl;
    std::cerr << "    find-outliers" << std::endl;
    std::cerr << "        find points in low density areas currently quick and dirty, may have" << std::endl;
    std::cerr << "        aliasing problems unless --no-antialiasing defined, will not remove" << std::endl;
    std::cerr << "        points on the border of a denser cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< remove_outliers::point >( false ), ',' ) << "; default: x,y,z"<<std::endl;
    std::cerr << std::endl;
    std::cerr << "        output: input line with appended 0 for outliers, 1 for non-outliers" << std::endl;
    std::cerr << "                (unless --discard is specified)" << std::endl;
    std::cerr << "                binary output: flag as unsigned byte (ub)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --resolution=<resolution>: size of the voxel to remove outliers" << std::endl;
    std::cerr << "            --min-number-of-points-per-voxel,--size=<number>:" << std::endl;
    std::cerr << "                    min number of points for a voxel to keep" << std::endl;
    std::cerr << "            --no-antialiasing: don't check neighbour voxels, which is faster," << std::endl;
    std::cerr << "                    but may remove points in borderline voxels" << std::endl;
    std::cerr << "            --discard: discards records that are outliers; output: same as input" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    local-min, local-max: deprecated, use nearest-min, nearest-max;" << std::endl;
    std::cerr << "        output local minimums or maximums inside of given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,id,scalar; default x,y,z,scalar" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                where reference_id is nearest point id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << "            --trace: if a points's reference point is not local extremum," << std::endl;
    std::cerr << "                     replace it with its reference (todo: debug)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest-point, nearest-any" << std::endl;
    std::cerr << "        for each point, output the single nearest point id (if any) inside the" << std::endl;
    std::cerr << "        given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,id; default x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                       where reference_id is nearest point id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --output-full-record,--full-record,--full:" << std::endl;
    std::cerr << "                    instead of reference_id and distance, output full extremum" << std::endl;
    std::cerr << "                    record, discard points with no extremum nearby" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest-min, nearest-max" << std::endl;
    std::cerr << "        for each point, output nearest minimums or maximums inside of given" << std::endl;
    std::cerr << "        radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,scalar,id; default: x,y,z,id" << std::endl;
//     std::cerr << "            if mask field is specified it only considers points with a mask value of 1, ignoring points with mask value of 0"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "        output fields: <input line>,reference_id,distance," << std::endl;
    std::cerr << "                       where reference_id is nearest min or max id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --output-extremums-only,--output-extremums,--extremums:" << std::endl;
    std::cerr << "                    output only the extremum records associated with points" << std::endl;
    std::cerr << "            --output-full-record,--full-record,--full:" << std::endl;
    std::cerr << "                    instead of reference_id and distance, output full extremum" << std::endl;
    std::cerr << "                    record, discard points with no extremum nearby" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        example: get local height maxima in the radius of 5 metres:" << std::endl;
    std::cerr << "            cat xyz.csv | points-calc local-max --fields=x,y,scalar --radius=5" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest" << std::endl;
    std::cerr << "        find point nearest to a given point" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --point,--to=<x>,<y>,<z>" << std::endl;
    std::cerr << std::endl;
    std::cerr << snark::points_calc::project::onto_line::traits::usage() << std::endl;
    std::cerr << snark::points_calc::project::onto_plane::traits::usage() << std::endl;
    std::cerr << snark::points_calc::plane_intersection::traits::usage() << std::endl;
    std::cerr << snark::points_calc::plane_intersection_with_trajectory::traits::usage() << std::endl;
    std::cerr << "    thin" << std::endl;
    std::cerr << "        read input data and thin them down by the given --resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --resolution=<distance>: minimum distance from one point to the next" << std::endl;
    std::cerr << std::endl;
    vector_calc::usage();
    exit( 0 );
}

static void calculate_distance( bool cumulative, bool propagate = false )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double distance = 0;
    double previous_norm = 0;
    if( cumulative && propagate ) { std::cerr << "points-calc: cumulative distance or propagate are mutually exclusive" << std::endl; exit( 1 ); }
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        double norm = last ? ( *p - *last ).norm() : 0;
        if( propagate ) { if( comma::math::equal( norm, 0 ) ) { norm = previous_norm; } else { previous_norm = norm; } }
        distance = cumulative ? distance + norm : norm;
        last = *p;
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
        }
    }
}

static void calculate_distance_next( bool propagate )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double previous_norm = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        if( last )
        {
            double distance = ( *p - *last ).norm();
            if( propagate ) { if( comma::math::equal( distance, 0 ) ) { distance = previous_norm; } else { previous_norm = distance; } }
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
                if( csv.flush ) { std::cout.flush(); }
            }
            else { std::cout << csv.delimiter << distance << std::endl; }
        }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ); }
        last = *p;
    }
    if( last )
    {
        double fake_final_distance = 0;
        if( csv.binary() )
        {
            std::cout.write( reinterpret_cast< const char* >( &fake_final_distance ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << csv.delimiter << fake_final_distance << std::endl; }
    }
}

static void calculate_distance_for_pairs( bool propagate )
{
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv );
    double previous_norm = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        double norm = ( p->first - p->second ).norm();
        if( propagate ) { if( comma::math::equal( norm, 0 ) ) { norm = previous_norm; } else { previous_norm = norm; } }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &norm ), sizeof( double ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << norm << std::endl;
        }
    }
}

static void angle_axis()
{
    comma::csv::options output_csv( csv );
    output_csv.fields = comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' );
    if( output_csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::AngleAxis< double > >() ); }
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    comma::csv::output_stream< Eigen::AngleAxis< double > > ostream( std::cout, output_csv );
    comma::csv::tied< Eigen::Vector3d, Eigen::AngleAxis< double > > tied( istream, ostream );

    boost::optional< Eigen::Vector3d > last;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        Eigen::AngleAxis< double > angle_axis;
        if( last ) { angle_axis = Eigen::Quaternion< double >::FromTwoVectors( *last, *p ); }
        else { angle_axis = Eigen::AngleAxis< double >( 0, Eigen::Vector3d( 0, 0, 0 )); }
        last = *p;

        tied.append( angle_axis );
    }
}

static void angle_axis_next()
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        if( last )
        {
            Eigen::AngleAxis< double > angle_axis( Eigen::Quaternion< double >::FromTwoVectors( *last, *p ));
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &angle_axis.angle() ), sizeof( double ));
                std::cout.write( reinterpret_cast< const char* >( &angle_axis.axis() ), sizeof( double ) * 3 );
                if( csv.flush ) { std::cout.flush(); }
            }
            else
            {
                std::cout << csv.delimiter << angle_axis.angle()
                          << csv.delimiter << comma::join( angle_axis.axis(), csv.delimiter )
                          << std::endl;
            }
        }
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ); }
        last = *p;
    }
    if( last )
    {
        Eigen::AngleAxis< double > angle_axis( 0, Eigen::Vector3d( 0, 0, 0 ));
        if( csv.binary() )
        {
            std::cout.write( reinterpret_cast< const char* >( &angle_axis.angle() ), sizeof( double ));
            std::cout.write( reinterpret_cast< const char* >( &angle_axis.axis() ), sizeof( double ) * 3 );
            if( csv.flush ) { std::cout.flush(); }
        }
        else
        {
            std::cout << csv.delimiter << angle_axis.angle()
                      << csv.delimiter << comma::join( angle_axis.axis(), csv.delimiter )
                      << std::endl;
        }
    }
}

static void angle_axis_for_pairs()
{
    comma::csv::options output_csv( csv );
    output_csv.fields = comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' );
    if( output_csv.binary() ) { output_csv.format( comma::csv::format::value< Eigen::AngleAxis< double > >() ); }
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv );
    comma::csv::output_stream< Eigen::AngleAxis< double > > ostream( std::cout, output_csv );
    comma::csv::tied< point_pair_t, Eigen::AngleAxis< double > > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        Eigen::AngleAxis< double > angle_axis( Eigen::Quaternion< double >::FromTwoVectors( p->first, p->second ));
        tied.append( angle_axis );
    }
}

static void thin( double resolution )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double distance = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        distance += last ? ( *p - *last ).norm() : 0;
        if( !last || distance >= resolution )
        {
            distance = 0;
            if( csv.binary() )
            {
                std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
                if( csv.flush ) { std::cout.flush(); }
            }
            else
            {
                std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << std::endl;
            }
        }
        last = *p;
    }
}

void output_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    if( csv.binary() )
    {
        std::cout.write( reinterpret_cast< const char* >( &p1 ), sizeof( double ) * 3 );
        std::cout.write( reinterpret_cast< const char* >( &p2 ), sizeof( double ) * 3 );
        if( csv.flush ) { std::cout.flush(); }
    }
    else 
    {
        std::cout << ascii.put( p1 ) << csv.delimiter << ascii.put( p2 ) << std::endl; 
    }
}

static void discretise( double step, double tolerance )
{
    if( csv.has_some_of_fields( "first,first/x,first/y,first/z,second,second/x,second/y,second/z" ) )
    {
        comma::csv::input_stream< std::pair< Eigen::Vector3d, Eigen::Vector3d > > istream( std::cin, csv );
        comma::csv::options output_csv;
        if( csv.binary() ) { output_csv.format( "3d" ); }
        output_csv.flush = csv.flush;
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const std::pair< Eigen::Vector3d, Eigen::Vector3d >* p = istream.read();
            if( !p ) { break; }
            Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( p->first, p->second );
            double norm = ( p->first - p->second ).norm();
            unsigned int size = norm / step;
            if( ( line.pointAt( step * size ) - p->second ).norm() > tolerance ) { ++size; }
            for( unsigned int i = 0; i < size; ++i ) { comma::csv::append( istream, ostream, line.pointAt( step * i ) ); }
            comma::csv::append( istream, ostream, p->second );
        }
    }
    else
    {
        BOOST_STATIC_ASSERT( sizeof( Eigen::Vector3d ) == sizeof( double ) * 3 );
        comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
        boost::optional< Eigen::Vector3d > previous_point;
        comma::csv::options output_csv;
        if( csv.binary() ) { output_csv.format( "3d" ); }
        output_csv.flush = csv.flush;
        comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const Eigen::Vector3d* current_point = istream.read();
            if( !current_point ) { break; }
            if( previous_point )
            {
                comma::csv::append( istream, ostream, *previous_point );
                double distance = ( *previous_point - *current_point ).norm();
                if( comma::math::less( step, distance ) )
                {
                    Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( *previous_point, *current_point );
                    for( double t = step; comma::math::less( t + tolerance, distance ); t += step )
                    {
                        Eigen::Vector3d point = line.pointAt( t );
                        comma::csv::append( istream, ostream, point );
                    }
                }
            }
            previous_point.reset( *current_point );
        }
        comma::csv::append( istream, ostream, *previous_point );
    }
}

static int cumulative_discretise( const comma::command_line_options& options )
{
    double step = options.value< double >( "--step" );
    if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    comma::csv::options output_csv;
    if( csv.binary() ) { output_csv.format( "3d" ); }
    output_csv.flush = csv.flush;
    comma::csv::output_stream< Eigen::Vector3d > ostream( std::cout, output_csv );
    comma::signal_flag is_shutdown;
    const Eigen::Vector3d* p = istream.read();
    if( !p ) { return 0; }
    Eigen::Vector3d previous_point = *p;
    comma::csv::append( istream, ostream, previous_point );
    double length = 0;
    double stepped = step;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* current_point = istream.read();
        if( !current_point ) { break; }
        double end = length + ( previous_point - *current_point ).norm();
        Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( previous_point, *current_point );
        for( ; !is_shutdown && comma::math::less( stepped, end ); stepped += step ) { comma::csv::append( istream, ostream, line.pointAt( stepped - length ) ); }
        previous_point = *current_point;
        length = end;
    }
    if( !comma::math::equal( length, stepped ) ) { comma::csv::append( istream, ostream, previous_point ); }
    return 0;
}

namespace local_operation {

struct point
{
    Eigen::Vector3d coordinates;
    double scalar;
    comma::uint32 id;
    comma::uint32 block;
//     uint8_t mask;
    
    point() : coordinates( 0, 0, 0 ), scalar( 0 ), id( 0 ), block( 0 ) {}
    point( const Eigen::Vector3d& coordinates, double scalar ) : coordinates( coordinates ), scalar( scalar ), id( 0 ), block( 0 ) {}
};

struct output // quick and dirty
{
    comma::uint32 id;
    double distance;
    output() : id( 0 ), distance( 0 ) {}
    output( comma::uint32 id, double distance ) : id( id ), distance( distance ) {}
};

struct record // todo: it's a mess; remove code duplication: all information can be extracted from reference_record (see nearest for usage)
{
    local_operation::point point;
    std::string line;
    bool is_extremum;
    comma::uint32 reference_id;
    double distance;
    record* reference_record;
    
    static comma::uint32 invalid_id;
    
    record() : is_extremum( true ), reference_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    record( const local_operation::point& p, const std::string& line ) : point( p ), line( line ), is_extremum( true ), reference_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    local_operation::output output( bool force = true ) const { return !force && reference_id == invalid_id ? local_operation::output( invalid_id, 0 ) : local_operation::output( reference_record->point.id, ( reference_record->point.coordinates - point.coordinates ).norm() ); }
};

comma::uint32 record::invalid_id = std::numeric_limits< comma::uint32 >::max();

static void evaluate_local_extremum( record* i, record* j, double radius, double sign )
{
    if( i == j || !i->is_extremum ) { return; }
    if( ( i->point.coordinates - j->point.coordinates ).squaredNorm() > radius * radius ) { return; }
    i->is_extremum = !comma::math::less( ( i->point.scalar - j->point.scalar ) * sign, 0 );
    if( i->is_extremum ) { j->is_extremum = false; }
}

static void update_nearest_extremum( record* i, record* j, double radius, bool trace )
{
    if( i->is_extremum )
    {
        if( i->reference_id != record::invalid_id ) { return; }
        i->reference_id = i->point.id;
        i->distance = 0;
        i->reference_record = i; // todo: dump reference_id, reference_record is enough
        return;
    }
    record* k = j;
    for( ; trace && k->reference_record != k && !k->reference_record->is_extremum; k = k->reference_record );
    if( !trace && !k->is_extremum ) { return; }
    double norm = ( i->point.coordinates - k->point.coordinates ).norm();
    if( norm > radius ) { return; }
    if( i->reference_id != record::invalid_id && i->distance < norm ) { return; }
    i->reference_id = k->point.id;
    i->distance = norm;
    i->reference_record = k; // todo: dump reference_id, reference_record is enough
}

static void update_nearest( record* i, record* j, double radius, double sign, bool any )
{
    if( i == j ) { return; }
    if( any ) // quick and dirty
    {
        double norm = ( i->point.coordinates - j->point.coordinates ).norm();
        if( norm > radius ) { return; }
        if( i != i->reference_record && ( i->reference_record->point.coordinates - i->point.coordinates ).norm() < norm ) { return; }
        i->reference_record = j;
    }
    else
    {
        if( comma::math::less( ( j->point.scalar - i->reference_record->point.scalar ) * sign, 0 ) ) { return; }
        double norm = ( i->point.coordinates - j->point.coordinates ).norm();
        if( norm > radius ) { return; }
        if(    comma::math::equal( ( i->reference_record->point.scalar - j->point.scalar ) * sign, 0 )
            && ( i->reference_record->point.coordinates - i->point.coordinates ).norm() < norm ) { return; }
        i->reference_record = j;
    }
}

static void process_nearest_extremum_block( std::deque< local_operation::record >& records
                                          , snark::math::closed_interval< double, 3 > const&  extents
                                          , Eigen::Vector3d const&  resolution
                                          , double const sign
                                          , double const radius
                                          , bool const any )
{
    if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
    typedef std::vector< local_operation::record* > voxel_t; // todo: is vector a good container? use deque
    typedef snark::voxel_map< voxel_t, 3 > grid_t;

    grid_t grid( extents.min(), resolution );
    for( std::size_t i = 0; i < records.size(); ++i )
    { 
//         if(records[i].point.mask )
        ( grid.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] );
    }
    for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
    {
        grid_t::index_type i;
        for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
        {
            for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
            {
                for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                {
                    grid_t::iterator git = grid.find( i );
                    if( git == grid.end() ) { continue; }
                    for( std::size_t n = 0; n < it->second.size(); ++n )
                    {
                        for( std::size_t k = 0; k < git->second.size(); ++k )
                        {
                            local_operation::update_nearest( it->second[n], git->second[k], radius, sign, any );
                        }
                    }
                }
            }
        }
    }
}

static void output_nearest_extremum_block( const std::deque< local_operation::record >& records
                                         , comma::csv::options const& output_csv
                                         , std::string const& endl
                                         , std::string const& delimiter
                                         , bool output_full_record
                                         , bool output_extremums_only )
{
    if( output_full_record )
    {
        unsigned int discarded = 0;
        for( std::size_t i = 0; i < records.size(); ++i )
        {
            if( !records[i].reference_record ) { ++discarded; continue; }
            std::cout.write( &records[i].line[0], records[i].line.size() );
            std::cout.write( &delimiter[0], delimiter.size() );
            std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
            std::cout.write( &endl[0], endl.size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        if( verbose ) { std::cerr << "points-calc: discarded " << discarded << " point(s) of " << records.size() << std::endl; }
        return;
    }
    if( output_extremums_only )
    {
        boost::unordered_set< comma::uint32 > ids;
        for( std::size_t i = 0; i < records.size(); ++i )
        {
            if( !records[i].reference_record ) { continue; }
            if( ids.find( records[i].reference_record->point.id ) != ids.end() ) { continue; }
            ids.insert( records[i].reference_record->point.id );
            std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
            std::cout.write( &endl[0], endl.size() );
            if( csv.flush ) { std::cout.flush(); }
        }
        return;
    }
    comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
    for( std::size_t i = 0; i < records.size(); ++i )
    {
        std::cout.write( &records[i].line[0], records[i].line.size() );
        std::cout.write( &delimiter[0], delimiter.size() );
        ostream.write( records[i].output() );
    }
}

} // namespace local_operation {

namespace comma { namespace visiting {

template <> struct traits< local_operation::point >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
//         v.apply( "mask", t.mask );
    }
    
    template< typename K, typename V > static void visit( const K&, local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
//         v.apply( "mask", t.mask );
    }
};

template <> struct traits< local_operation::output >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::output& t, V& v )
    {
        v.apply( "id", t.id );
        v.apply( "distance", t.distance );
    }
};

} } // namespace comma { namespace visiting {

template < typename Traits > static int run( const comma::command_line_options& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << Traits::input_fields() << std::endl; return 0; }
    if( options.exists( "--input-format" ) ) { std::cout << Traits::input_format() << std::endl; return 0; }
    if( options.exists( "--output-fields" ) ) { std::cout << Traits::output_fields() << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << Traits::output_format() << std::endl; return 0; }
    return Traits::run( options );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        csv = comma::csv::options( options );
        csv.full_xpath = true;
        ascii = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", csv.delimiter );
        const std::vector< std::string >& operations = options.unnamed( "--verbose,-v,--trace,--no-antialiasing,--next,--unit,--output-full-record,--full-record,--full,--flush,--with-trajectory,--trajectory", "-.*" );
        if( operations.size() != 1 ) { std::cerr << "points-calc: expected one operation, got " << operations.size() << ": " << comma::join( operations, ' ' ) << std::endl; return 1; }
        const std::string& operation = operations[0];
        if( operation == "project-onto-line" ) { return run< snark::points_calc::project::onto_line::traits >( options ); }
        if( operation == "project-onto-plane" ) { return run< snark::points_calc::project::onto_plane::traits >( options ); }
        if( operation == "plane-intersection-with-trajectory" ) { return run< snark::points_calc::plane_intersection_with_trajectory::traits >( options ); }
        if( vector_calc::has_operation( operation ) ) { vector_calc::process(operation, options, csv); return 0; }
        if( operation == "plane-intersection" ) { snark::points_calc::plane_intersection::traits::process(options, csv); return 0; }
        if( operation == "distance" )
        {
            if( options.exists("--output-fields" )){ std::cout << "distance" << std::endl; return 0; }
            if( options.exists("--output-format" )){ std::cout << "d" << std::endl; return 0; }
            bool propagate = options.exists( "--propagate" );
            if(    csv.has_field( "first" )   || csv.has_field( "second" )
                || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
            {
                calculate_distance_for_pairs( propagate );
                return 0;
            }
            if ( options.exists( "--next" ) ) { calculate_distance_next( propagate ); }
            else { calculate_distance( false, propagate ); }
            return 0;
        }
        if( operation == "cumulative-distance" )
        {
            if( options.exists("--output-fields" )){ std::cout << "distance" << std::endl; return 0; }
            if( options.exists("--output-format" )){ std::cout << "d" << std::endl; return 0; }
            calculate_distance( true );
            return 0;
        }
        if( operation == "angle-axis" )
        {
            if( options.exists("--output-fields" )){ std::cout << comma::join( comma::csv::names< Eigen::AngleAxis< double > >( false ), ',' ) << std::endl; return 0; }
            if( options.exists("--output-format" )){ std::cout << comma::csv::format::value< Eigen::AngleAxis< double > >() << std::endl; return 0; }

            if(    csv.has_field( "first" )   || csv.has_field( "second" )
                || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
            {
                angle_axis_for_pairs();
                return 0;
            }
            if ( options.exists( "--next" ) ) { angle_axis_next(); }
            else { angle_axis(); }
            return 0;
        }
        if( operation == "nearest" )
        {
            Eigen::Vector3d point = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--point,--to" ) );
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
            std::string record;
            double min_distance = std::numeric_limits< double >::max();
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                double d = ( *p - point ).norm();
                if( d >= min_distance ) { continue; }
                min_distance = d;
                record = csv.binary() ? std::string( istream.binary().last(), csv.format().size() ) : comma::join( istream.ascii().last(), csv.delimiter );
            }
            if( !record.empty() ) { std::cout << record; }
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &min_distance ), sizeof( double ) );
                if( csv.flush ) { std::cout.flush(); }
            }
            else { std::cout << csv.delimiter << min_distance << std::endl; }
            return 0;
        }
        if( operation == "thin" )
        {
            if( !options.exists( "--resolution" ) ) { std::cerr << "points-calc: --resolution is not specified " << std::endl; return 1; }
            double resolution = options.value( "--resolution" , 0.0 );
            thin( resolution );
            return 0;
        }
        if( operation == "discretise" || operation == "discretize" )
        {
            double step = options.value< double >( "--step" );
            if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
            // the last discretised point can be very close to the end of the interval, in which case the last two points can be identical in the output since ascii.put uses 12 digits by default
            // setting --tolerance=1e-12 will not allow the last discretised point to be too close to the end of the interval and therefore the output will have two distinct points at the end
            double tolerance = options.value( "--tolerance", std::numeric_limits< double >::min() );
            if( tolerance < 0 ) { std::cerr << "points-calc: expected non-negative tolerance, got " << tolerance << std::endl; return 1; }
            discretise( step, tolerance );
            return 0;
        }
        if( operation == "cumulative-discretise" || operation == "cumulative-discretize" || operation == "sample" )
        {
            return cumulative_discretise( options );
        }
        if( operation == "local-max" || operation == "local-min" ) // todo: if( operation == "local-calc" ? )
        {
            double sign = operation == "local-max" ? 1 : -1;
            if( csv.fields.empty() ) { csv.fields = "x,y,z,scalar"; }
            csv.full_xpath = false;
            bool has_id = csv.has_field( "id" );
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            std::deque< local_operation::record > records;
            double radius = options.value< double >( "--radius" );
            bool trace = options.exists( "--trace" );
            Eigen::Vector3d resolution( radius, radius, radius );
            snark::math::closed_interval< double, 3 > extents;
            comma::uint32 id = 0;
            if( verbose ) { std::cerr << "points-calc: reading input points..." << std::endl; }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const local_operation::point* p = istream.read();
                if( !p ) { break; }
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                local_operation::point q = *p;
                if( !has_id ) { q.id = id++; }
                records.push_back( local_operation::record( q, line ) );
                records.back().reference_record = &records.back();
                extents.set_hull( p->coordinates );
            }
            if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
            typedef std::vector< local_operation::record* > voxel_t; // todo: is vector a good container? use deque
            typedef snark::voxel_map< voxel_t, 3 > grid_t;
            grid_t grid( extents.min(), resolution );
            for( std::size_t i = 0; i < records.size(); ++i ) { ( grid.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] ); }
            if( verbose ) { std::cerr << "points-calc: searching for local extrema..." << std::endl; }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                grid_t::index_type i;
                for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
                {
                    for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                    {
                        for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                        {
                            grid_t::iterator git = grid.find( i );
                            if( git == grid.end() ) { continue; }
                            for( voxel_t::iterator vit = it->second.begin(); vit != it->second.end(); ++vit )
                            {
                                for( std::size_t k = 0; k < git->second.size() && ( *vit )->is_extremum; ++k )
                                {
                                    local_operation::evaluate_local_extremum( *vit, git->second[k], radius, sign );
                                }
                            }
                        }
                    }
                }
            }
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: filling extrema grid..." << std::endl; }
            grid_t extrema( extents.min(), resolution );
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                if( records[i].is_extremum )
                { 
                    ( extrema.touch_at( records[i].point.coordinates ) )->second.push_back( &records[i] );
                }
                else
                { 
                    records[i].reference_id = local_operation::record::invalid_id; // quick and dirty for now
//                     if( records[i].reference_id == local_operation::record::invalid_id ) { continue; }
//                     while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
//                     {
//                         records[i].reference_record = records[i].reference_record->reference_record;
//                     }
//                     records[i].reference_id = records[i].reference_record->point.id;
                }
            }
            if( verbose ) { std::cerr << "points-calc: calculating distances to " << extrema.size() << " local extrema..." << std::endl; }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                grid_t::index_type i;
                for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
                {
                    for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
                    {
                        for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                        {
                            grid_t::iterator git = extrema.find( i );
                            if( git == extrema.end() ) { continue; }
                            for( std::size_t n = 0; n < it->second.size(); ++n )
                            {                            
                                for( std::size_t k = 0; k < git->second.size(); ++k )
                                {
                                    local_operation::update_nearest_extremum( it->second[n], git->second[k], radius, trace );
                                }
                            }
                        }
                    }
                }
            }
            if( trace )
            {
                if( verbose ) { std::cerr << "points-calc: tracing extrema..." << std::endl; }
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    if( records[i].reference_id == local_operation::record::invalid_id ) { continue; }
                    while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
                    {
                        records[i].reference_record = records[i].reference_record->reference_record;
                    }
                }
            }
            
            
            // todo: move output to a function
            
            
            if( verbose ) { std::cerr << "points-calc: " << operation << ": outputting..." << std::endl; }
            std::string endl;
            std::string delimiter;
            comma::csv::options output_csv;
            if( csv.binary() )
            {
                output_csv.format( "ui,d" );
            }
            else
            {
                std::ostringstream oss;
                oss << std::endl;
                endl = oss.str();
                delimiter = std::string( 1, csv.delimiter );
            }
            output_csv.flush = csv.flush;
            if( options.exists( "--output-full-record,--full-record,--full" ) )
            {
                unsigned int discarded = 0;
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    if( !records[i].reference_record ) { ++discarded; continue; }
                    std::cout.write( &records[i].line[0], records[i].line.size() );
                    std::cout.write( &delimiter[0], delimiter.size() );
                    std::cout.write( &records[i].reference_record->line[0], records[i].reference_record->line.size() );
                    std::cout.write( &endl[0], endl.size() );
                    if( csv.flush ) { std::cout.flush(); }
                }
                if( verbose ) { std::cerr << "points-calc: " << operation << ": discarded " << discarded << " point(s) of " << records.size() << std::endl; }
            }
            else
            {
                comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    std::cout.write( &records[i].line[0], records[i].line.size() );
                    std::cout.write( &delimiter[0], delimiter.size() );
                    ostream.write( records[i].output( false ) );
                }
            }
            if( verbose ) { std::cerr << "points-calc: " << operation << ": done!" << std::endl; }
            return 0;
        }
        if( operation == "nearest-max" || operation == "nearest-min" || operation == "nearest-any" || operation == "nearest-point" )
        {
            std::deque< local_operation::record > records;
            double sign = operation == "nearest-max" ? 1 : -1;
            bool any = operation == "nearest-any" || operation == "nearest-point";
            if( csv.fields.empty() ) { csv.fields = any ? "x,y,z" : "x,y,z,scalar"; }
            csv.full_xpath = false;
            bool has_id = csv.has_field( "id" );
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            double radius = options.value< double >( "--radius" );
            Eigen::Vector3d resolution( radius, radius, radius );
            snark::math::closed_interval< double, 3 > extents;
            comma::uint32 id = 0;
            comma::uint32 block = 0;
            if( verbose ) { std::cerr << "points-calc " << operation << ": reading input points of block: " << block << std::endl; }
#ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
#endif
            options.assert_mutually_exclusive( "--output-full-record,--full-record,--full", "--output-extremums-only,--output-extremums,--extremums" );
            bool const output_full_record = options.exists( "--output-full-record,--full-record,--full" );
            bool output_extremums_only = options.exists( "--output-extremums-only,--output-extremums,--extremums" );
            std::string endl;
            std::string delimiter;
            comma::csv::options output_csv;
            if( csv.binary() )
            {
                output_csv.format( "ui,d" );
            }
            else
            {
                std::ostringstream oss;
                oss << std::endl;
                endl = oss.str();
                delimiter = std::string( 1, csv.delimiter );
            }
            output_csv.flush = csv.flush;
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const local_operation::point* p = istream.read();
                if ( !p || block != p->block )
                {
                    if( verbose ) { std::cerr << "points-calc " << operation << ": processing points of block: " << block << std::endl; }
                    local_operation::process_nearest_extremum_block( records, extents, resolution, sign, radius, any );
                    if( verbose ) { std::cerr << "points-calc " << operation << ": outputting points of block: " << block << std::endl; }
                    output_nearest_extremum_block( records, output_csv, endl, delimiter, output_full_record, output_extremums_only );
                    extents = snark::math::closed_interval< double, 3 >();
                    records.clear();
                    if( verbose ) { std::cerr << "points-calc " << operation << ": reading input points of block: " << block << std::endl; }
                }
                if( !p ) { break; }
                block = p->block;
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                local_operation::point q = *p;
                if( !has_id ) { q.id = id++; }
                records.push_back( local_operation::record( q, line ) );
                records.back().reference_record = &records.back();
                extents.set_hull( p->coordinates );
            }
            return 0;
        }
        if( operation == "find-outliers" )
        {
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            csv.full_xpath = false;
            unsigned int size = options.value< unsigned int >( "--min-number-of-points-per-voxel,--size" );
            double r = options.value< double >( "--resolution" );
            Eigen::Vector3d resolution( r, r, r );
            bool no_antialiasing = options.exists( "--no-antialiasing" );            
            bool discard=options.exists("--discard");
            comma::csv::input_stream< remove_outliers::point > istream( std::cin, csv );
            remove_outliers::app app(csv,verbose);
            unsigned block=0;
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: reading input points..." << std::endl; }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const remove_outliers::point* p = istream.read();
                if ( !p || block != p->block )
                {
                    app.process(resolution,no_antialiasing,size);
                    app.write_output(discard);
                    app.extents=snark::math::closed_interval< double, 3 >();
                    app.records.clear();
                }
                if( !p ) { break; }
                block = p->block;
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                app.records.push_back( remove_outliers::record( *p, line ) );
                app.extents.set_hull( p->coordinates );
            }
            if( verbose ) { std::cerr << "points-calc: done!" << std::endl; }
            return 0;
        }
        std::cerr << "points-calc: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-calc: unknown exception" << std::endl; }
    return 1;
}
