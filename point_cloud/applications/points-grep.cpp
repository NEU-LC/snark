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

/// @author abdallah kassir

#include <iostream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include "../../math/roll_pitch_yaw.h"
#include "../../math/rotation_matrix.h"
#include "../../math/geometry/polytope.h"
#include "../../math/applications/frame.h"
#include "../../visiting/traits.h"

struct bounds_t
{
    bounds_t(): front(0), back(0), right(0), left(0), top(0), bottom(0) {}
    double front;
    double back;
    double right;
    double left;
    double top;
    double bottom;
};

struct point
{
    point(): coordinates( Eigen::Vector3d::Zero() ), block(0), id(0), flag(1){}
    boost::posix_time::ptime timestamp;
    Eigen::Vector3d coordinates;
    comma::uint32 block;
    comma::uint32 id;
    comma::uint32 flag;
};

typedef snark::applications::frame::position_type bounding_point;

struct joined_point
{
    point bounded;
    bounding_point bounding;
};

struct position
{
    Eigen::Vector3d coordinates;
    snark::roll_pitch_yaw orientation;
    
    position() : coordinates( Eigen::Vector3d::Zero() ) {}
};

struct shape_input : public Eigen::Vector3d
{
    shape_input() : Eigen::Vector3d( Eigen::Vector3d::Zero() ) {}
    ::position shape; // quick and dirty
};

struct shape_output
{
    shape_output( bool included = false ) : included( included ) {}
    bool included;
};

namespace comma { namespace visiting {
    
template <> struct traits< ::position >
{
    template < typename K, typename V > static void visit( const K& k, ::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
    
    template < typename K, typename V > static void visit( const K& k, const ::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
};
    
template <> struct traits< shape_input >
{
    template < typename K, typename V > static void visit( const K& k, shape_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "shape", t.shape );
    }
    
    template < typename K, typename V > static void visit( const K& k, const shape_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "shape", t.shape );
    }
};

template <> struct traits< shape_output >
{
    template < typename K, typename V > static void visit( const K& k, const shape_output& t, V& v ) { v.apply( "included", t.included ); }
};

template <> struct traits< bounds_t >
{
    template < typename K, typename V > static void visit( const K&, bounds_t& p, V& v )
    {
        v.apply( "front", p.front );
        v.apply( "back", p.back );
        v.apply( "right", p.right );
        v.apply( "left", p.left );
        v.apply( "top", p.top );
        v.apply( "bottom", p.bottom );
    }

    template < typename K, typename V > static void visit( const K&, const bounds_t& p, V& v )
    {
        v.apply( "front", p.front );
        v.apply( "back", p.back );
        v.apply( "right", p.right );
        v.apply( "left", p.left );
        v.apply( "top", p.top );
        v.apply( "bottom", p.bottom );
    }
};
template <> struct traits< point >
{
    template < typename K, typename V > static void visit( const K&, point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "coordinates", p.coordinates );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "flag", p.flag );
    }

    template < typename K, typename V > static void visit( const K&, const point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "coordinates", p.coordinates );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
        v.apply( "flag", p.flag );
    }
};

template <> struct traits< joined_point >
{
    template < typename K, typename V > static void visit( const K&, joined_point& p, V& v )
    {
        v.apply( "bounded", p.bounded );
        v.apply( "bounding", p.bounding );
    }
    template < typename K, typename V > static void visit( const K&, const joined_point& p, V& v )
    {
        v.apply( "bounded", p.bounded );
        v.apply( "bounding", p.bounding );
    }
};

} } // namespace comma { namespace visiting { 

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "filter points from dynamic objects represented by a position and orientation stream" << std::endl;
    std::cerr << "input: point-cloud, bounding stream" << std::endl;
    std::cerr << "       bounding data may either be joined to each point or provided through a separate stream in which points-grep will time-join the two streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-grep <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "     shape, deprecated" << std::endl;
    std::cerr << "     stream, deprecated" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  stream <stream>: points are time-joined with a position stream and filtered based on a bounding box around the positions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      attention: deprecated interface, will be refactored soon" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      <stream>: example: \"nav.csv;fields=t,x,y,z,roll,pitch,yaw\" " << std::endl;
    std::cerr << "      fields" << std::endl;
    std::cerr << "          bounded: " << comma::join( comma::csv::names< point >( false ), ',' ) << std::endl;
    std::cerr << "                   default: t,x,y,z" << std::endl;
    std::cerr << "          bounding: " << comma::join( comma::csv::names< bounding_point >( false ), ',' ) << std::endl;
    std::cerr << "                    or shorthand: bounded,bounding" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      <options>:" << std::endl;
    std::cerr << "          --bounds=<front>,<back>,<right>,<left>,<top>,<bottom> the values represent the distances of the faces of the bounding box to the centre of the bounding stream in the bounding frame" << std::endl;
    std::cerr << "          --error-margin=<margin> error margin value added to bounds (for user convenience), default: 0.5" << std::endl;
    std::cerr << "          --output-all: output all points" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "       output: each point is tagged with a flag of type ui" << std::endl;
    std::cerr<< "           the value of flag is 0 for filtered points and 1 otherwise" << std::endl;
    std::cerr<< "           if --output-all is not specified only non-filtered points are output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  shape: points are assumed to be joined with the bounding stream" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      attention: deprecated interface, will be refactored soon" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      fields: " << comma::join( comma::csv::names< joined_point >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "      <options>:" << std::endl;
    std::cerr << "          --bounds=<front>,<back>,<right>,<left>,<top>,<bottom> the values represent the distances of the faces of the bounding box to the centre of the bounding stream in the bounding frame" << std::endl;
    std::cerr << "          --error-margin=<margin> error margin value added to bounds (for user convenience), default: 0.5" << std::endl;
    std::cerr << "          --output-all: output all points" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "       output: each point is tagged with a flag of type ui" << std::endl;
    std::cerr<< "           the value of flag is 0 for filtered points and 1 otherwise" << std::endl;
    std::cerr<< "           if --output-all is not specified only non-filtered points are output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat points.csv | points-grep stream --fields=bounded \"nav.csv;fields=t,x,y,z,roll,pitch,yaw\" --bounds=1.0,2.0,1.0,2.0,1.0,2.0 --error-margin=0.1" << std::endl;
    std::cerr << "    cat points.csv | points-grep stream --fields=t,coordinates,block,flag \"nav.csv;fields=t,x,y,z,roll,pitch,yaw\" --bounds=1.0,2.0,1.0,2.0,1.0,2.0 --error-margin=0.1" << std::endl;
    std::cerr << "    cat points.csv | points-grep stream --fields=bounded/t,bounded/coordinates,bounded/block,bounded/flag \"local:/tmp/nav;fields=t,x,y,z,roll,pitch,yaw\" --bounds=1.0,2.0,1.0,2.0,1.0,2.0 --error-margin=0.5" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cat points.csv | points-grep shape --fields=bounded,bounding --bounds=1.0,2.0,1.0,2.0,1.0,2.0" << std::endl;
    std::cerr << "    cat points.csv | points-grep shape --fields=bounded/t,bounded/coordinates,bounded/flag,bounding/t,bounding/x,bounding/y,bounding/z,bounding/roll,bounding/pitch,bounding/yaw --bounds=1.0,1.0,1.0,1.0,1.0,1.0 --error-margin=1.0" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static double offset;
static bounds_t bounds;

template < typename P > // quick and dirty for now
static snark::geometry::convex_polytope make_polytope( const P& position, const bounds_t& b, double error_margin = 0 )
{
    //get polygon from bounds
    Eigen::MatrixXd origins(3,7); //origin, front, back, right, left, top, bottom


    origins<<0, b.front + error_margin, -b.back - error_margin, 0, 0, 0, 0,
             0, 0, 0, b.right + error_margin, -b.left - error_margin, 0, 0,
             0, 0, 0, 0, 0, -b.top - error_margin, b.bottom + error_margin;


    //convert vehicle bounds to world coordinates
    //get transformation

    Eigen::MatrixXd rot_matrix = snark::rotation_matrix::rotation(position.orientation);

    for(int cntr2=0; cntr2<origins.cols(); cntr2++)
    {
        origins.col(cntr2)=rot_matrix*origins.col(cntr2)+position.coordinates;
    }

    //get plane equations
    Eigen::MatrixXd A(6,3);
    Eigen::VectorXd c(6);

    for(int cntr2=1; cntr2<origins.cols(); cntr2++)
    {
        A.row(cntr2-1)<<origins.col(0).transpose()-origins.col(cntr2).transpose();
        c(cntr2-1)=origins.col(cntr2).transpose()*(origins.col(0)-origins.col(cntr2));
    }

    //check polygon in 3d
    // use if statement not assignment because point might already be filtered
    return snark::geometry::convex_polytope( A, c );
}

static snark::geometry::convex_polytope make_polytope( const joined_point& pq, const bounds_t& b, double error_margin ) { return make_polytope( pq.bounding.value, b, error_margin ); }

static bool included( const joined_point& pq, const snark::geometry::convex_polytope& polytope ) { return polytope.has( pq.bounded.coordinates ); }

static bool included( const joined_point& pq ) { return included( pq, make_polytope( pq, bounds, offset ) ); }

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        const std::vector< std::string >& unnamed = options.unnamed("--output-all,--verbose,-v,--flush","-.*");
        std::string operation = unnamed[0];
        comma::csv::options csv( options );
        csv.full_xpath = true;
        if( operation == "shape" )
        {
            if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
            std::vector< std::string > fields = comma::split( csv.fields, ',' );
            for( unsigned int i = 0; i < fields.size(); ++i )
            {
                if( fields[i] == "shape/x" ) { fields[i] = "shape/coordinates/x"; }
                else if( fields[i] == "shape/y" ) { fields[i] = "shape/coordinates/y"; }
                else if( fields[i] == "shape/z" ) { fields[i] = "shape/coordinates/z"; }
                else if( fields[i] == "shape/roll" ) { fields[i] = "shape/orientation/roll"; }
                else if( fields[i] == "shape/pitch" ) { fields[i] = "shape/orientation/pitch"; }
                else if( fields[i] == "shape/yaw" ) { fields[i] = "shape/orientation/yaw"; }
            }
            csv.fields = comma::join( fields, ',' );
            bool output_all = options.exists( "--output-all" );
            shape_input default_input;
            default_input.shape = comma::csv::ascii< ::position >().get( options.value< std::string >( "--shape-position,--position", "0,0,0,0,0,0" ) );
            // todo: refactor bounds
            bounds_t shape_bounds = comma::csv::ascii< bounds_t >().get( options.value< std::string >( "--shape-bounds,--bounds", "0,0,0,0,0,0" ) );
            boost::optional< snark::geometry::convex_polytope > polytope;
            if( !csv.has_field( "shape,shape/coordinates,shape/coordinates/x,shape/coordinates/y,shape/coordinates/z,shape/orientation,shape/orientation/roll,shape/orientation/pitch,shape/orientation/yaw" ) ) { polytope = make_polytope( default_input.shape, shape_bounds ); }
            comma::csv::input_stream< shape_input > istream( std::cin, csv, default_input );
            comma::csv::output_stream< shape_output > ostream( std::cout, csv.binary() );
            comma::csv::passed< shape_input > passed( istream, std::cout );
            comma::csv::tied< shape_input, shape_output > tied( istream, ostream );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const shape_input* p = istream.read();
                if( !p ) { break; }
                bool keep = polytope ? make_polytope( p->shape, shape_bounds ).has( *p ) : polytope->has( *p );
                if( output_all ) { tied.append( shape_output( keep ) ); }
                else if( keep ) { passed.write(); }
                if( csv.flush ) { std::cout.flush(); }
            }
            return 0;
        }
        if( operation == "stream" )
        {
            bool output_all = options.exists( "--output-all");
            offset=options.value("--error-margin",0.5);
            bounds=comma::csv::ascii<bounds_t>().get(options.value("--bounds",std::string("0,0,0,0,0,0")));
            comma::csv::input_stream<joined_point> istream(std::cin,csv);
            comma::csv::output_stream<joined_point> ostream(std::cout,csv);
            std::vector<std::string> fields=comma::split(csv.fields,csv.delimiter);
            if( csv.fields.empty() ) { csv.fields = "t,coordinates"; }
            bool flag_exists = csv.has_field( "flag" );
            std::string bounded_string("bounded/");
            for(unsigned int i=0; i<fields.size(); i++)
            {
                if(fields[i].substr(0,bounded_string.size())!=bounded_string)
                {
                    fields[i]=bounded_string+fields[i];
                }
            }
            csv.fields=comma::join( fields, csv.delimiter );
            flag_exists = csv.has_field( "bounded/flag" );
            joined_point pq;
            comma::signal_flag is_shutdown;
            if(unnamed.size()<2){ usage(); }
            comma::name_value::parser parser( "filename" );
            comma::csv::options bounding_csv = parser.get< comma::csv::options >( unnamed[1] ); // get stream options
            comma::io::istream bounding_is( comma::split(unnamed[1],';')[0], bounding_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii ); // get stream name

            //bounding stream
            std::deque<bounding_point> bounding_queue;
            comma::csv::input_stream<bounding_point> bounding_istream(*bounding_is, bounding_csv);

            comma::io::select istream_select;
            comma::io::select bounding_istream_select;

            istream_select.read().add(0);
            istream_select.read().add(bounding_is.fd());
            bounding_istream_select.read().add(bounding_is.fd());

            bool next=true;

            while(!is_shutdown && ( istream.ready() || ( std::cin.good() && !std::cin.eof() ) ))
            {
                bool bounding_data_available =  bounding_istream.ready() || ( bounding_is->good() && !bounding_is->eof());

                //check so we do not block
                bool bounding_istream_ready=bounding_istream.ready();
                bool istream_ready=istream.ready();

                if(next)
                {
                    //only check istream if we need a new point
                    if(!bounding_istream_ready || !istream_ready)
                    {
                    if(!bounding_istream_ready && !istream_ready)
                    {
                        istream_select.wait(boost::posix_time::milliseconds(10));
                    }
                    else
                    {
                        istream_select.check();
                    }
                    if(istream_select.read().ready(bounding_is.fd()))
                    {
                        bounding_istream_ready=true;
                    }
                    if(istream_select.read().ready(0))
                    {
                        istream_ready=true;
                    }
                    }
                }
                else
                {
                    if(!bounding_istream_ready)
                    {
                        bounding_istream_select.wait(boost::posix_time::milliseconds(10));
                        if(bounding_istream_select.read().ready(bounding_is.fd())) { bounding_istream_ready=true; }
                    }
                }

                //keep storing available bounding data
                if(bounding_istream_ready)
                {
                    const bounding_point* q = bounding_istream.read();
                    if( q )
                    {
                        bounding_queue.push_back(*q);
                    }
                    else
                    {
                        bounding_data_available=false;
                    }
                }

                //if we are done with the last bounded point get next
                if(next)
                {
                    if(!istream_ready) { continue; }
                    const joined_point* pq_ptr = istream.read();
                    if( !pq_ptr ) { break; }
                    pq=*pq_ptr;
                }

                //get bound
                while(bounding_queue.size()>=2)
                {
                    if( pq.bounded.timestamp < bounding_queue[1].t ) { break; }
                    bounding_queue.pop_front();
                }

                if(bounding_queue.size()<2)
                {
                    //bound not found
                    //do we have more data?
                    if(!bounding_data_available) { break; }
                    next=false;
                    continue;
                }

                //bound available
                next=true; //get new point on next iteration

                //discard late points
                if(pq.bounded.timestamp < bounding_queue[0].t)
                {
                    continue;
                }

                //match
                bool is_first=( pq.bounded.timestamp - bounding_queue[0].t < bounding_queue[1].t - pq.bounded.timestamp );
                pq.bounding = is_first ? bounding_queue[0] : bounding_queue[1]; // assign bounding point

                //filter out object points
                pq.bounded.flag = !included( pq ); // todo: invert logic, now it is really weird

                if(!pq.bounded.flag && !output_all)
                {
                    continue;
                }

                if(flag_exists)
                {
                    ostream.write(pq);
                    ostream.flush();
                    continue;
                }

                //append flag
                if(ostream.is_binary())
                {
                    ostream.write(pq,istream.binary().last());
                    std::cout.write( reinterpret_cast< const char* >( &pq.bounded.flag ), sizeof( comma::uint32 ) );
                }
                else
                {
                    std::string line=comma::join( istream.ascii().last(), csv.delimiter );
                    line+=csv.delimiter+boost::lexical_cast<std::string>(pq.bounded.flag);
                    ostream.write(pq,line);
                }
                ostream.flush();
            }
            return 0;
        }
        std::cerr << "points-grep: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-grep: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-grep: unknown exception" << std::endl; }
    return 1;
}
