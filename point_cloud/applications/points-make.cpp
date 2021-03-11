// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include <iostream>
#include <array>
#include <stdlib.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include "../../visiting/eigen.h"

namespace snark { namespace test_pattern {

struct color_t
{
    boost::array< unsigned char, 4 > rgba = {{ 0, 0, 0, 255 }};
    color_t() {}
    color_t( unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255 ): rgba( {{ red, green, blue, alpha }} ) {}

    unsigned char red() const   { return rgba[0]; }
    unsigned char green() const { return rgba[1]; }
    unsigned char blue() const  { return rgba[2]; }
    unsigned char alpha() const { return rgba[3]; }
};

// Define our own vertex type using our colour type.
// A more flexible alternative is to use snark::graphics::qt3d::vertex_t
// but this type works better with view-points.
struct vertex_t
{
    Eigen::Vector3f position;
    color_t color;

    vertex_t() {}
    vertex_t( const Eigen::Vector3f& position, const color_t& color )
        : position( position ), color( color ) {}
};

template< typename T >
struct operation_t
{
    static std::string output_fields() { return comma::join( comma::csv::names< T >( false ), ',' ); }
    static std::string output_format() { return comma::csv::format::value< T >(); }
};

struct cube : public operation_t< vertex_t >
{
    static const unsigned int default_num_points = 100000;
    static constexpr float default_width = 1.0f;
    static constexpr float default_thickness = 0.01f;

    static void usage()
    {
        std::cerr << "\n";
        std::cerr << "\n    test-cube";
        std::cerr << "\n        --num,-n=<val>:     number of points in cube (default: " << default_num_points << ")";
        std::cerr << "\n        --width,-w=<val>:   cube width (default: " << default_width << ")";
        std::cerr << "\n        --thickness=<val>:  cube thickness (default: " << default_thickness << ")";
    }

    static void examples()
    {
        std::cerr << "\n    " << "points-make test-cube | view-points --fields x,y,z,r,g,b,a";
    }

    static const char* completion_options() { return " test-cube --num -n --width -w --thickness"; }

    static int run( const comma::command_line_options& options )
    {
        unsigned int num_points = options.value< unsigned int >( "--num,-n", default_num_points );
        float width = options.value< float >( "--width,-w", default_width );
        float thickness = options.value< float >( "--thickness", default_thickness );

        comma::csv::options output_csv( options );
        output_csv.full_xpath = false;
        if( options.exists( "--binary,-b" )) { output_csv.format( comma::csv::format::value< vertex_t >() ); }
        comma::csv::output_stream< vertex_t > ostream( std::cout, output_csv );

        for( unsigned int i = 0; i < num_points; i++ )
        {
            Eigen::Vector3f p = make_point( width, thickness );
            vertex_t vertex( p, color_t( point_to_color( p.x(), width, thickness )
                                       , point_to_color( p.y(), width, thickness )
                                       , point_to_color( p.z(), width, thickness )
                                       , 255 ));
            ostream.write( vertex );
        }
        return 0;
    }

private:
    static unsigned char point_to_color( float p, float width, float thickness ) { return (unsigned char)( fabs( p / ( width + thickness )) * 256 ); }
    static float random( float min, float max ) { return (float)::random() / RAND_MAX * ( max - min ) + min; }
    static int random_sign() { return ::random() % 2 * 2 - 1; }

    static Eigen::Vector3f make_point( float width, float thickness )
    {
        float min = width - thickness;
        float max = width + thickness;
        float x_min = 0;
        float y_min = 0;
        float z_min = 0;
        switch( ::random() % 3 )
        {
            case 0: x_min = min; break;
            case 1: y_min = min; break;
            default: case 2: z_min = min; break;
        }
        float x = random_sign() * random( x_min, max );
        float y = random_sign() * random( y_min, max );
        float z = random_sign() * random( z_min, max );
        return Eigen::Vector3f( x, y, z );
    }
};

struct grid : public operation_t< Eigen::Vector3f >
{
    static constexpr float default_width = 6.0f;
    static constexpr float default_spacing = 1.0f;
    static constexpr float default_z_offset = 0.0f;

    static void usage()
    {
        std::cerr << "\n";
        std::cerr << "\n    grid";
        std::cerr << "\n        --width,-w=<val>:   grid width; default: " << default_width;
        std::cerr << "\n        --spacing,-s=<val>: grid spacing; default: " << default_spacing;
        std::cerr << "\n        --z-offset=<val>:   grid z offset; default: " << default_z_offset;
    }

    static void examples()
    {
        std::cerr << "\n    " << "points-make test-grid | view-points --shape lines";
    }

    static const char* completion_options() { return " test-grid --width -w --spacing -s --z-offset"; }

    static int run( const comma::command_line_options& options )
    {
        float width = options.value< float >( "--width,-w", default_width );
        float spacing = options.value< float >( "--spacing,-s", default_spacing );
        float z_offset = options.value< float >( "--z-offset", default_z_offset );

        comma::csv::options output_csv( options );
        output_csv.full_xpath = false;
        if( options.exists( "--binary,-b" )) { output_csv.format( comma::csv::format::value< Eigen::Vector3f >() ); }

        comma::csv::output_stream< Eigen::Vector3f > ostream( std::cout, output_csv );

        int sign = 1;
        float offset = width / 2.0f;
        for( float y = -offset; y <= offset; y += spacing )
        {
            ostream.write( Eigen::Vector3f( -offset * sign, y, z_offset ));
            ostream.write( Eigen::Vector3f(  offset * sign, y, z_offset ));
            sign *= -1;
        }
        sign = 1;
        for( float x = -offset; x <= offset; x += spacing )
        {
            ostream.write( Eigen::Vector3f( x,  offset * sign, z_offset ));
            ostream.write( Eigen::Vector3f( x, -offset * sign, z_offset ));
            sign *= -1;
        }
        return 0;
    }
};

} } // namespace snark { namespace test_pattern {

namespace comma { namespace visiting {

template <> struct traits< snark::test_pattern::color_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::test_pattern::color_t& p, Visitor& v )
    {
        v.apply( "r", p.rgba[0] );
        v.apply( "g", p.rgba[1] );
        v.apply( "b", p.rgba[2] );
        v.apply( "a", p.rgba[3] );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::test_pattern::color_t& p, Visitor& v )
    {
        v.apply( "r", p.red() );
        v.apply( "g", p.green() );
        v.apply( "b", p.blue() );
        v.apply( "a", p.alpha() );
    }
};

template <> struct traits< snark::test_pattern::vertex_t >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::test_pattern::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::test_pattern::vertex_t& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

template < typename Operation > static int run( const comma::command_line_options& options )
{
    if( options.exists( "--output-fields" ) ) { std::cout << Operation::output_fields() << std::endl; return 0; }
    if( options.exists( "--output-format" ) ) { std::cout << Operation::output_format() << std::endl; return 0; }
    return Operation::run( options );
}

static void bash_completion( unsigned const ac, char const * const * av )
{
    std::cout << "--help -h --output-fields --output-format --binary -b --seed"
              << snark::test_pattern::cube::completion_options()
              << snark::test_pattern::grid::completion_options()
              << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\noutput point clouds";
    std::cerr << "\n";
    std::cerr << "\nusage: " << "points-make <operation> [<options>]";
    std::cerr << "\n";
    std::cerr << "\n<operation>";
    std::cerr << "\n    cube: todo: document...";
    std::cerr << "\n    grid: todo: document...";
    std::cerr << "\n";
    std::cerr << "\n<options>";
    std::cerr << "\n    --help,-h:       show this help, --help --verbose for more help";
    std::cerr << "\n    --output-fields: show output fields and exit";
    std::cerr << "\n    --output-format: show binary output format and exit";
    std::cerr << "\n    --binary,-b:     output in binary (default is ascii)";
    std::cerr << "\n    --seed=<seed>:   seed for random generator (see srand)";
    std::cerr << "\n                     <seed>: for random generator";
    std::cerr << "\n                         number, e.g. --seed=55";
    std::cerr << "\n                         'time', use time as seed, e.g. --seed=time";
    std::cerr << "\n                         default: 0";
    snark::test_pattern::cube::usage();
    snark::test_pattern::grid::usage();
    std::cerr << "\n";
    std::cerr << "\nexamples";
    snark::test_pattern::cube::examples();
    snark::test_pattern::grid::examples();
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) { bash_completion( argc, argv ); }
        unsigned seed = options.value< unsigned int >( "--seed", time( 0 ) );
        srand( seed );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h", "-.*,--.*" );
        if( unnamed.empty() ) { std::cerr << "points-make: please specify operation" << std::endl; return 1; }
        std::string operation = unnamed[0];
        if( operation == "test-cube" ) { return run< snark::test_pattern::cube >( options ); }
        else if( operation == "test-grid" ) { return run< snark::test_pattern::grid >( options ); }
        std::cerr << "points-make: expected operation, got: '" << operation << "'" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "points-make: " << ex.what() << std::endl;  }
    catch( ... ) { std::cerr << "points-make: unknown exception" << std::endl; }
    return 1;
}
