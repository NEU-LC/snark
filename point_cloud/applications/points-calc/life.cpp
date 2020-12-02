// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <array>
#include <cmath>
#include <sstream>
#include <Eigen/Core>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../../point_cloud/voxel_map.h"
#include "../../../visiting/eigen.h"
#include "life.h"

namespace snark { namespace points_calc { namespace life {

std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    life\n"
        << "        options\n"
        << "            --todo\n"
        << std::endl
        << "        examples\n"
        << "            still trying to find good parameters \\\n"
        << "                ( echo 0,0,0; echo 0,0,1; echo 0,0,2 ) \\\n"
        << "                    | csv-to-bin 3f \\\n"
        << "                    | points-calc life --procreation 6 --extinction 10 --step 0.1 --binary 3f \\\n"
        << "                    | view-points '-;fields=x,y,z,,block;color=yellow;binary=4d,ui' --orthographic --scene-radius 100\n"
        << std::endl;
    return oss.str();
}

struct point
{
    Eigen::Vector3d coordinates;
    double weight;
    
    point( const Eigen::Vector3d& coordinates = Eigen::Vector3d( 0, 0, 0 ), double weight = 1 ): coordinates( coordinates ), weight( weight ) {}
};

typedef point input;

struct output
{
    life::point point;
    comma::uint32 block;
    
    output( const life::point& point = life::point(), comma::uint32 block = 0 ): point( point ), block( block ) {}
};

} } } // namespace snark { namespace points_calc { namespace visit {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::life::point >
{
    template < typename K, typename V > static void visit( const K&, snark::points_calc::life::point& p, V& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "weight", p.weight );
    }

    template < typename K, typename V > static void visit( const K&, const snark::points_calc::life::point& p, V& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "weight", p.weight );
    }
};

template <> struct traits< snark::points_calc::life::output >
{
    template < typename K, typename V > static void visit( const K&, snark::points_calc::life::output& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::points_calc::life::output& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace points_calc { namespace life {

std::string traits::input_fields() { return comma::join( comma::csv::names< snark::points_calc::life::input >( false ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< snark::points_calc::life::input >(); }

std::string traits::output_fields() { return comma::join( comma::csv::names< snark::points_calc::life::output >( false ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< snark::points_calc::life::output >(); }

int traits::run( const comma::command_line_options& options )
{
    double procreation_threshold = options.value( "--procreation-threshold,--procreation", 3.0 );
    double stability_threshold = options.value( "--stability-threshold,--stability,--extinction-threshold,--extinction", 4.0 );
    double max_vitality = options.value( "--max-vitality,--vitality", 1.0 );
    double step = options.value( "--step", 1.0 );
    bool verbose = options.exists( "--verbose,-v" );
    comma::csv::options csv( options, "x,y,z" );
    csv.full_xpath = false;
    comma::csv::options output_csv;
    output_csv.full_xpath = false;
    if( csv.binary() ) { output_csv.format( output_format() ); }
    unsigned int block = 0;
    comma::csv::input_stream< input > istream( std::cin, csv );
    comma::csv::output_stream< output > ostream( std::cout, output_csv );
    typedef snark::voxel_map< double, 3 > voxels_t;
    std::array< voxels_t, 2 > voxels = {{ voxels_t( Eigen::Vector3d( 1, 1, 1 ) ), voxels_t( Eigen::Vector3d( 1, 1, 1 ) ) }};
    comma::signal_flag is_shutdown;
    while( istream.ready() || std::cin.good() || is_shutdown )
    {
        const input* p = istream.read();
        if( !p ) { break; }
        voxels[0].touch_at( p->coordinates )->second = p->weight; // quick and dirty
    }
    bool changed = true;
    unsigned int current = 0;
    for( current = 0; !is_shutdown && !voxels[current].empty() && changed; current = 1 - current )
    {
        changed = false;
        unsigned int next = 1 - current;
        voxels[next].clear();
        for( auto v: voxels[current] )
        {
            ostream.write( output( point( Eigen::Vector3d( v.first[0], v.first[1], v.first[2] ), v.second ), block ) );
            if( csv.flush ) { std::cout.flush(); }
        }
        if( verbose ) { std::cerr << "pointc-calc: life: generation: " << block << " size: " << voxels[current].size() << std::endl; }
        voxels_t dead( Eigen::Vector3d( 1, 1, 1 ) ); // quick and dirty: todo
        for( auto v: voxels[current] ) // todo? quick and dirty, watch performance
        {
            if( is_shutdown ) { break; }
            voxels_t::index_type i;
            for( i[0] = v.first[0] - 1; i[0] < v.first[0] + 2; ++i[0] )
            {
                for( i[1] = v.first[1] - 1; i[1] < v.first[1] + 2; ++i[1] )
                {
                    for( i[2] = v.first[2] - 1; i[2] < v.first[2] + 2; ++i[2] )
                    {
                        if( dead.find( i ) != dead.end() || voxels[next].find( i ) != voxels[next].end() ) { continue; }
                        voxels_t::index_type j;
                        double sum = 0;
                        for( j[0] = i[0] - 1; j[0] < i[0] + 2; ++j[0] )
                        {
                            for( j[1] = i[1] - 1; j[1] < i[1] + 2; ++j[1] )
                            {
                                for( j[2] = i[2] - 1; j[2] < i[2] + 2; ++j[2] )
                                {
                                    if( i == j ) { continue; } // quick and dirty
                                    auto c = voxels[current].find( j );
                                    if( c != voxels[current].end() ) { sum += c->second; }
                                }
                            }
                        }
                        double s = sum < procreation_threshold || sum > stability_threshold ? -step : ( sum - procreation_threshold ) < ( stability_threshold - sum ) ? step : 0; // todo? optionally linearly changing step?
                        auto n = voxels[current].find( i );
                        double new_value = ( n == voxels[current].end() ? 0 : n->second ) + s;
                        ( new_value <= 0 ? dead[i] : voxels[next][i] ) = std::min( new_value, max_vitality );
                        changed = true;
                    }
                }
            }
        }
        ++block;
    }
    if( verbose )
    { 
        if( is_shutdown ) { std::cerr << "pointc-calc: life: caught signal" << std::endl; }
        else if( voxels[current].empty() ) { std::cerr << "pointc-calc: life: generation: " << block << " size: " << 0 << std::endl; }
        else if( !changed ) { std::cerr << "pointc-calc: life: generation: " << block << " stabilised" << std::endl; }
    }
    return 0;
}

} } } // namespace snark { namespace points_calc { namespace visit {
