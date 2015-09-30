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
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <snark/point_cloud/voxel_grid.h>
#include <snark/point_cloud/voxel_map.h>
#include <snark/visiting/eigen.h>

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;

static comma::csv::options csv;
static comma::csv::ascii< Eigen::Vector3d > ascii;
static bool verbose;

static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates from stdin, perform calculations, and output result to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat points.csv | points-calc distance > results.csv" << std::endl;
    std::cerr << "    echo -e \"0\\n1\\n3\\n6\" | points-calc distance --fields x --next" << std::endl;
    std::cerr << "    cat points.csv | points-calc cumulative-distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc thin --resolution <resolution> > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc discretise --step <step> > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    distance" << std::endl;
    std::cerr << "    cumulative-distance" << std::endl;
    std::cerr << "    thin" << std::endl;
    std::cerr << "    discretise" << std::endl;
    std::cerr << "    find-outliers" << std::endl;
    std::cerr << "    local-min" << std::endl;
    std::cerr << "    local-max" << std::endl;
    std::cerr << "    nearest-min" << std::endl;
    std::cerr << "    nearest-max" << std::endl;
    std::cerr << "    nearest-any" << std::endl;
    std::cerr << "    nearest" << std::endl;
    std::cerr << "    plane-intersection" << std::endl;
    std::cerr << "    cross" << std::endl;
    std::cerr << "    dot" << std::endl;
    std::cerr << "    norm" << std::endl;
    std::cerr << "    scale" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: distance between subsequent points or, if input is pairs, between the points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << "        options: " << std::endl;
    std::cerr << "            --next: for subsequent points only, append distance to next point (default: append distance to previous point)" << std::endl;
    std::cerr << "                    fake zero is appended to the final point (since there is no next point)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-distance: cumulative distance between subsequent points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    local-min: output local minimums inside of given radius" << std::endl;
    std::cerr << "    local-max: output local maximums inside of given radius" << std::endl;
    std::cerr << "    nearest-min: for each point, output nearest minimums inside of given radius" << std::endl;
    std::cerr << "    nearest-max: for each point, output local maximums inside of given radius" << std::endl;
    std::cerr << "    nearest-any: for each point, output any nearest point inside of given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --radius=<metres>: radius of the local region to search" << std::endl;
    std::cerr << "            --trace: local min/max only; if a points's reference point is not local" << std::endl;
    std::cerr << "                     extremum, replace it with its reference" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,scalar,id" << std::endl;
    std::cerr << "        output fields: <input line>,extremum_id,distance (todo: make outputs optional)" << std::endl;
    std::cerr << "        example: get local height maxima in the radius of 5 metres:" << std::endl;
    std::cerr << "            cat xyz.csv | points-calc local-max --fields=x,y,scalar --radius=5" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest: find point nearest to the given point" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --point,--to=<x>,<y>,<z>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    thin: read input data and thin them down by the given --resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    find-outliers: find points in low density areas" << std::endl;
    std::cerr << "                   currently quick and dirty, may have aliasing problems" << std::endl;
    std::cerr << "                   unless --no-antialiasing defined, will not remove points" << std::endl;
    std::cerr << "                   on the border of a denser cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        output: input line with appended 0 for outliers, 1 for non-outliers" << std::endl;
    std::cerr << "                binary output: flag as unsigned byte (ub)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --resolution=<resolution>: size of the voxel to remove outliers" << std::endl;
    std::cerr << "            --min-number-of-points-per-voxel,--size=<number>: min number of points for a voxel to keep" << std::endl;
    std::cerr << "            --no-antialiasing: don't check neighbour voxels, which is faster, but may remove points in borderline voxels" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    plane-intersection: read points and plane normals from stdin, output intersection of the ray with the plane" << std::endl;
    std::cerr << "        input fields: x,y,z,normal/x,normal/y,normal/z" << std::endl;
    std::cerr << "        the plane passes through the normal and is perpendicular to it" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --input-fields: print input field names" << std::endl;
    std::cerr << "            --output-fields: print output field names" << std::endl;
    std::cerr << "            --output-format: print output fields format" << std::endl;
    std::cerr << "            --normal=<x>,<y>,<z>: default normal; uses this value when normal is empty or any of its fields are omitted" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    discretise, discretize: read input data and discretise intervals between adjacent points with --step" << std::endl;
    std::cerr << "        skip discretised points that are closer to the end of the interval than --tolerance (default: --tolerance=0)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    vector calculations: following operations perform calculations on 3d vectors and scalars" << std::endl;
    std::cerr << "        common options:" << std::endl;
    std::cerr << "            --input-fields: print input field names and exit" << std::endl;
    std::cerr << "            --output-fields: print output field names and exit" << std::endl;
    std::cerr << "            --output-format: print output format and exit" << std::endl;
    std::cerr << "    " << std::endl;
    std::cerr << "        cross: calculate vector cross product; output: vector" << std::endl;
    std::cerr << "            input fields: v, u" << std::endl;
    std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
    std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        dot: calculate vector dot product; output: scalar" << std::endl;
    std::cerr << "            input fields: v, u" << std::endl;
    std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
    std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        norm: calculate vector's norm " << std::endl;
    std::cerr << "            --v=<x>,<y>,<z>: default value for vecotr" << std::endl;
    std::cerr << "            --square: calculate norm square" << std::endl;
    std::cerr << "            --invert: calculate invert of norm; use with --square for invert of norm square" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        scale: calculate multiplication of scalar and vector;output vector" << std::endl;
    std::cerr << "            input fields:  v,a (scalar)" << std::endl;
    std::cerr << "            --v=<x>,<y>,<z>: default value for vecotr" << std::endl;
    std::cerr << "            --scalar: default value for a" << std::endl;
    std::cerr << "            --invert: invert the scalar first and then multiply it by vecotr, i.e. (1/a)*v" << std::endl;
    std::cerr << "" << std::endl;
    exit( 0 );
}

static void calculate_distance( bool cumulative )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double distance = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        double norm = last ? ( *p - *last ).norm() : 0;
        distance = cumulative ? distance + norm : norm;
        last = *p;
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
        }
    }
}

static void calculate_distance_next()
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        if( last )
        {
            double distance = ( *p - *last ).norm();
            if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) ); }
            else { std::cout << csv.delimiter << distance << std::endl; }
        }
        if( csv.binary() ) { std::cout.write( istream.binary().last(), istream.binary().binary().format().size() ); }
        else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ); }
        last = *p;
    }
    if( last )
    {
        double fake_final_distance = 0;
        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &fake_final_distance ), sizeof( double ) ); }
        else { std::cout << csv.delimiter << fake_final_distance << std::endl; }
    }
}

static void calculate_distance_for_pairs()
{
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        double norm = ( p->first - p->second ).norm();
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &norm ), sizeof( double ) );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << norm << std::endl;
        }
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
        std::cout.flush();
    }
    else 
    {
        std::cout << ascii.put( p1 ) << csv.delimiter << ascii.put( p2 ) << std::endl; 
    }
}

static void discretise( double step, double tolerance )
{
    BOOST_STATIC_ASSERT( sizeof( Eigen::Vector3d ) == sizeof( double ) * 3 );
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > previous_point;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* current_point = istream.read();
        if( !current_point ) { break; }
        if( previous_point )
        {
            output_points( *previous_point, *previous_point );
            double distance = ( *previous_point - *current_point ).norm();
            if( comma::math::less( step, distance ) )
            {
                Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( *previous_point, *current_point );
                for( double t = step; comma::math::less( t + tolerance, distance ); t += step )
                {
                    Eigen::Vector3d point = line.pointAt( t );
                    output_points( *previous_point, point );
                }
            }
        }
        previous_point.reset( *current_point );
    }
    output_points( *previous_point, *previous_point );
}

namespace local_operation {

struct point
{
    Eigen::Vector3d coordinates;
    double scalar;
    comma::uint32 id;
    
    point() : coordinates( 0, 0, 0 ), scalar( 0 ) {}
    point( const Eigen::Vector3d& coordinates, double scalar ) : coordinates( coordinates ), scalar( scalar ) {}
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
    comma::uint32 extremum_id;
    double distance;
    record* reference_record;
    
    static comma::uint32 invalid_id;
    
    record() : is_extremum( true ), extremum_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    record( const local_operation::point& p, const std::string& line ) : point( p ), line( line ), is_extremum( true ), extremum_id( invalid_id ), distance( std::numeric_limits< double >::max() ), reference_record( NULL ) {}
    local_operation::output output( bool force = true ) const { return !force && extremum_id == invalid_id ? local_operation::output( invalid_id, 0 ) : local_operation::output( reference_record->point.id, ( reference_record->point.coordinates - point.coordinates ).norm() ); }
};

comma::uint32 record::invalid_id = std::numeric_limits< comma::uint32 >::max();

static void evaluate_local_extremum( record* i, record* j, double radius, double sign )
{
    if( i == j || !i->is_extremum ) { return; }
    if( ( i->point.coordinates - j->point.coordinates ).squaredNorm() > radius * radius ) { return; }
    i->is_extremum = !comma::math::less( ( i->point.scalar - j->point.scalar ) * sign, 0 );
    if( i->is_extremum ) { j->is_extremum = false; }
}

static void update_nearest_extremum( record* i, record* j, double radius )
{
    if( i->is_extremum )
    {
        if( i->extremum_id != record::invalid_id ) { return; }
        i->extremum_id = i->point.id;
        i->distance = 0;
        i->reference_record = i; // todo: dump extremum_id, reference_record is enough
        return;
    }
    if( !j->is_extremum ) { return; }
    double norm = ( i->point.coordinates - j->point.coordinates ).norm();
    if( norm > radius ) { return; }
    if( i->extremum_id != record::invalid_id && i->distance < norm ) { return; }
    i->extremum_id = j->point.id;
    i->distance = norm;
    i->reference_record = j; // todo: dump extremum_id, reference_record is enough
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

} // namespace local_operation {

namespace comma { namespace visiting {

template <> struct traits< local_operation::point >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
    }
    
    template< typename K, typename V > static void visit( const K&, local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
        v.apply( "id", t.id );
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

namespace remove_outliers {

struct record
{
    Eigen::Vector3d point;
    std::string line;
    bool rejected;
    
    record() : rejected( false ) {}
    record( const Eigen::Vector3d& p, const std::string& line ) : point( p ), line( line ), rejected( false ) {}
};

} // namespace remove_outliers {

struct plane_intersection
{
    struct input_t
    {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        input_t():point( Eigen::Vector3d::Zero() ),normal(Eigen::Vector3d::Zero()){}
        input_t( const Eigen::Vector3d&  default_normal):point( Eigen::Vector3d::Zero() ),normal(default_normal){}
    };
    typedef Eigen::Vector3d output_t;
    static void run( const comma::csv::options& csv_opt, const Eigen::Vector3d& normal)
    {
        comma::csv::input_stream<input_t> is( std::cin, csv_opt, input_t(normal));
        comma::csv::output_stream<output_t> os( std::cout, csv_opt.binary() );
        comma::csv::tied<input_t,output_t> tied(is,os);
        const double inf=std::numeric_limits<double>::infinity();
        while(is.ready() || std::cin.good())
        {
            const input_t* rec=is.read();
            if(!rec){break;}
            if(verbose)
            {
                std::cerr<<"p: "<<rec->point.x()<<", "<<rec->point.y()<<", "<<rec->point.z()<<std::endl;
                std::cerr<<"n: "<<rec->normal.x()<<", "<<rec->normal.y()<<", "<<rec->normal.z()<<std::endl;
            }
            double d=rec->point.dot(rec->normal);
            double n2=rec->normal.squaredNorm();
            if(verbose){std::cerr<<"d "<<d<<", n2 "<<n2<<std::endl;}
            output_t out = (d == 0) ? Eigen::Vector3d(inf,inf,inf) : Eigen::Vector3d(( n2/d)*rec->point);
            tied.append(out);
        }
    }
};

struct vector_calc
{
    typedef Eigen::Vector3d vector;
    //typedef double scalar;
    static vector vzero;
    template<typename input_t,typename output_t>
    struct operation_t
    {
        virtual output_t calc(const input_t& in)=0;
        input_t default_input;
        operation_t(const input_t& def):default_input(def){}
        operation_t(){}
        void run(const comma::csv::options& csv_opt)
        {
            comma::csv::input_stream<input_t> is( std::cin, csv_opt, default_input);
            comma::csv::output_stream<output_t> os( std::cout, csv_opt.binary() );
            comma::csv::tied<input_t,output_t> tied(is,os);
            while(is.ready() || std::cin.good())
            {
                const input_t* rec=is.read();
                if(!rec){break;}
                tied.append(calc(*rec));
            }
        }
        void input_fields(){std::cout<<comma::join( comma::csv::names<input_t>(true), ',' ) << std::endl; }
        void output_fields(){std::cout<<comma::join( comma::csv::names<output_t>(false), ',' ) << std::endl; }
        void output_format(){std::cout<<comma::csv::format::value<output_t>() << std::endl; }
    };
    struct vector_pair
    {
        vector v;
        vector u;
        vector_pair():v(vzero),u(vzero){}
        vector_pair(const vector& d1,const vector& d2):v(d1),u(d2){}
    };
    struct scalar
    {
        double a;
        scalar():a(0){}
        scalar(double d):a(d){}
    };
    struct cross:operation_t<vector_pair,vector>
    {
        cross(){}
        cross(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        vector calc(const vector_pair& in)
        {
            return in.v.cross(in.u);
        }
    };
    struct dot:operation_t<vector_pair,scalar>
    {
        dot(){}
        dot(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        scalar calc(const vector_pair& in)
        {
            return in.v.dot(in.u);
        }
    };
    struct norm:operation_t<vector,scalar>
    {
        norm(){}
        norm(const vector& default1, bool squre=false, bool invert=false ) :
            operation_t(default1)
            ,_squre(squre)
            ,_invert(invert)
        {
        }
        scalar calc(const vector& in)
        {
            double d=_squre?in.squaredNorm(): in.norm();
            return _invert?1/d:d;
        }
        bool _squre;
        bool _invert;
    };
    typedef std::pair<vector,scalar> vector_scalarpair;
    struct scale:operation_t<vector_scalarpair,vector>
    {
        scale(){}
        scale(const vector& v, const scalar& s, bool invert=false ) :
            operation_t(vector_scalarpair(v,s))
            ,_invert(invert)
        {
        }
        vector calc(const vector_scalarpair& in)
        {
            double d=_invert?1/in.second.a:in.second.a;
            return d*in.first;
        }
        bool _invert;
    };
};
vector_calc::vector vector_calc::vzero=Eigen::Vector3d::Zero();

namespace comma { namespace visiting {

template <> struct traits< plane_intersection::input_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::input_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.normal);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::input_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.normal);
    }
};

template <> struct traits< vector_calc::vector_pair >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::vector_pair& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::vector_pair& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
    }
};

template <> struct traits< vector_calc::vector_scalarpair >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::vector_scalarpair& t, V& v )
    {
        v.apply( "v", t.first);
        v.apply( "scalar", t.second.a);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::vector_scalarpair& t, V& v )
    {
        v.apply( "v", t.first);
        v.apply( "scalar", t.second.a);
    }
};

template <> struct traits< vector_calc::scalar >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::scalar& t, V& v )
    {
        v.apply( "scalar", t.a);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::scalar& t, V& v )
    {
        v.apply( "scalar", t.a);
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }
        csv = comma::csv::options( options );
        csv.full_xpath = true;
        ascii = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", csv.delimiter );
        const std::vector< std::string >& operations = options.unnamed( "--verbose,-v,--trace,--no-antialiasing,--next", "-.*" );
        if( operations.size() != 1 ) { std::cerr << "points-calc: expected one operation, got " << operations.size() << ": " << comma::join( operations, ' ' ) << std::endl; return 1; }
        const std::string& operation = operations[0];
        if(operation=="cross")
        {
            if(options.exists("--input-fields")){vector_calc::cross().input_fields(); return 0; }
            if(options.exists("--output-fields")){vector_calc::cross().output_fields(); return 0; }
            if(options.exists("--output-format")){vector_calc::cross().output_format(); return 0; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::cross(v1_default,v2_default).run(csv);
            return 0;
        }
        if(operation=="dot")
        {
            if(options.exists("--input-fields")){vector_calc::dot().input_fields(); return 0; }
            if(options.exists("--output-fields")){vector_calc::dot().output_fields(); return 0; }
            if(options.exists("--output-format")){vector_calc::dot().output_format(); return 0; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::dot(v1_default,v2_default).run(csv);
            return 0;
        }
        if(operation=="norm")
        {
            if(options.exists("--input-fields")){vector_calc::norm().input_fields(); return 0; }
            if(options.exists("--output-fields")){vector_calc::norm().output_fields(); return 0; }
            if(options.exists("--output-format")){vector_calc::norm().output_format(); return 0; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            vector_calc::norm(v1_default,options.exists("--square"),options.exists("--invert")).run(csv);
            return 0;
        }
        if(operation=="scale")
        {
            if(options.exists("--input-fields")){vector_calc::scale().input_fields(); return 0; }
            if(options.exists("--output-fields")){vector_calc::scale().output_fields(); return 0; }
            if(options.exists("--output-format")){vector_calc::scale().output_format(); return 0; }
            Eigen::Vector3d v=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            vector_calc::scalar s=comma::csv::ascii< double >().get(options.value< std::string >( "--scalar", "0"));
            vector_calc::scale(v, s, options.exists("--invert")).run(csv);
            return 0;
        }
        if( operation == "plane-intersection" )
        {
            if(options.exists("--input-fields")){std::cout<<comma::join( comma::csv::names<plane_intersection::input_t>(true), ',' ) << std::endl; return 0; }
            if(options.exists("--output-fields")){std::cout<<comma::join( comma::csv::names<plane_intersection::output_t>(false), ',' ) << std::endl; return 0; }
            if(options.exists("--output-format")){std::cout<<comma::csv::format::value<plane_intersection::output_t>() << std::endl; return 0; }
            plane_intersection::run(csv, comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--normal",  "0,0,0")));
            return 0;
        }
        if( operation == "distance" )
        {
            if(    csv.has_field( "first" )   || csv.has_field( "second" )
                || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
            {
                calculate_distance_for_pairs();
                return 0;
            }
            if ( options.exists( "--next" ) ) { calculate_distance_next(); }
            else { calculate_distance( false ); }
            return 0;
        }
        if( operation == "cumulative-distance" )
        {
            calculate_distance( true );
            return 0;
        }
        if( operation == "nearest" )
        {
            if( options.exists( "--point,--to" ) )
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
                if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &min_distance ), sizeof( double ) ); }
                else { std::cout << csv.delimiter << min_distance << std::endl; }
                return 0;
            }
            else
            {
                
                return 0;
            }
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
            if( !options.exists( "--step" ) ) { std::cerr << "points-calc: --step is not specified " << std::endl; return 1; }
            double step = options.value( "--step" , 0.0 );
            if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
            // the last discretised point can be very close to the end of the interval, in which case the last two points can be identical in the output since ascii.put uses 12 digits by default
            // setting --tolerance=1e-12 will not allow the last discretised point to be too close to the end of the interval and therefore the output will have two distinct points at the end
            double tolerance = options.value( "--tolerance" , 0.0 ); 
            if( tolerance < 0 ) { std::cerr << "points-calc: expected non-negative tolerance, got " << tolerance << std::endl; return 1; }
            discretise( step, tolerance );
            return 0;
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
                    records[i].extremum_id = local_operation::record::invalid_id; // quick and dirty for now
//                     if( records[i].extremum_id == local_operation::record::invalid_id ) { continue; }
//                     while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
//                     {
//                         records[i].reference_record = records[i].reference_record->reference_record;
//                     }
//                     records[i].extremum_id = records[i].reference_record->point.id;
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
                                    local_operation::update_nearest_extremum( it->second[n], git->second[k], radius );
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
                    if( records[i].extremum_id == local_operation::record::invalid_id ) { continue; }
                    while( records[i].reference_record->point.id != records[i].reference_record->reference_record->point.id )
                    {
                        records[i].reference_record = records[i].reference_record->reference_record;
                    }
                }
            }
            if( verbose ) { std::cerr << "points-calc: outputting..." << std::endl; }
            std::string endl = csv.binary() ? "" : "\n";
            std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
            comma::csv::options output_csv;
            if( csv.binary() ) { output_csv.format( "ui,d" ); }
            comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                std::cout.write( &records[i].line[0], records[i].line.size() );
                std::cout.write( &delimiter[0], delimiter.size() );
                ostream.write( records[i].output( false ) ); // quick and dirty
            }
            if( verbose ) { std::cerr << "points-calc: done!" << std::endl; }
            return 0;
        }
        if( operation == "nearest-max" || operation == "nearest-min" || operation == "nearest-any" )
        {
            double sign = operation == "nearest-max" ? 1 : -1;
            bool any = operation == "nearest-any";
            if( csv.fields.empty() ) { csv.fields = "x,y,z,scalar"; }
            csv.full_xpath = false;
            bool has_id = csv.has_field( "id" );
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            std::deque< local_operation::record > records;
            double radius = options.value< double >( "--radius" );
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
            if( verbose ) { std::cerr << "points-calc: searching for " << operation << "..." << std::endl; }
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
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: outputting..." << std::endl; }
            std::string endl = csv.binary() ? "" : "\n";
            std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
            comma::csv::options output_csv;
            if( csv.binary() ) { output_csv.format( "ui,d" ); }
            comma::csv::output_stream< local_operation::output > ostream( std::cout, output_csv );
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                std::cout.write( &records[i].line[0], records[i].line.size() );
                std::cout.write( &delimiter[0], delimiter.size() );
                ostream.write( records[i].output() );
            }
            if( verbose ) { std::cerr << "points-calc: done!" << std::endl; }
            return 0;
        }
        if( operation == "find-outliers" )
        {
            unsigned int size = options.value< unsigned int >( "--min-number-of-points-per-voxel,--size" );
            double r = options.value< double >( "--resolution" );
            Eigen::Vector3d resolution( r, r, r );
            bool no_antialiasing = options.exists( "--no-antialiasing" );            
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
            std::deque< remove_outliers::record > records;
            snark::math::closed_interval< double, 3 > extents;
            if( verbose ) { std::cerr << "points-calc: reading input points..." << std::endl; }
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
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
                records.push_back( remove_outliers::record( *p, line ) );
                extents.set_hull( *p );
            }
            if( verbose ) { std::cerr << "points-calc: loading " << records.size() << " points into grid..." << std::endl; }
            typedef std::vector< remove_outliers::record* > voxel_t; // todo: is vector a good container? use deque
            typedef snark::voxel_map< voxel_t, 3 > grid_t;
            grid_t grid( extents.min(), resolution );
            for( std::size_t i = 0; i < records.size(); ++i ) { ( grid.touch_at( records[i].point ) )->second.push_back( &records[i] ); }
            if( verbose ) { std::cerr << "points-calc: removing outliers..." << std::endl; }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                bool rejected = true;
                if( no_antialiasing )
                {
                    rejected = it->second.size() < size;
                }
                else
                {
                    grid_t::index_type i;
                    for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2 && rejected; ++i[0] )
                    {
                        for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2 && rejected; ++i[1] )
                        {
                            for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2 && rejected; ++i[2] )
                            {
                                grid_t::iterator git = grid.find( i );
                                rejected = git == grid.end() || git->second.size() < size;
                            }
                        }
                    }
                }
                if( rejected ) { for( std::size_t i = 0; i < it->second.size(); ++i ) { it->second[i]->rejected = true; } }
            }
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            if( verbose ) { std::cerr << "points-calc: outputting..." << std::endl; }
            std::string endl = csv.binary() ? "" : "\n";
            std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                char valid = records[i].rejected ? 0 : 1;
                std::cout.write( &records[i].line[0], records[i].line.size() );
                std::cout.write( &delimiter[0], delimiter.size() );
                std::cout.write( &valid, 1 );
                std::cout.write( &endl[0], endl.size() );
            }
            if( verbose ) { std::cerr << "points-calc: done!" << std::endl; }
            return 0;
        }
        std::cerr << "points-calc: please specify an operation" << std::endl;
        return 1;
    }
    catch( std::exception& ex )
    {
        std::cerr << "points-calc: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "points-calc: unknown exception" << std::endl;
    }
    return 1;
}
