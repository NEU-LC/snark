#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <comma/base/exception.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/impl/program_options.h>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include <snark/point_cloud/voxel_map.h>

struct input_point
{
    Eigen::Vector3d point;
    comma::uint32 block;
    
    input_point() : block( 0 ) {}
};

struct Centroid
{
    boost::array< comma::int32, 3 > index;
    Eigen::Vector3d mean;
    comma::uint32 size;
    comma::uint32 block;
    
    Centroid() : size( 0 ), block( 0 ) {}
    
    void operator+=( const Eigen::Vector3d& point )
    {
        ++size;
        mean = ( mean * ( size - 1 ) + point ) / size;
    }
};

namespace comma { namespace visiting {

template <> struct traits< input_point >
{
    template < typename K, typename V > static void visit( const K&, input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }
};

template <> struct traits< Centroid >
{
    template < typename K, typename V > static void visit( const K&, Centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const Centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

int main( int argc, char** argv )
{
    try
    {
        std::string binary;        
        std::string origin_string;
        std::string resolution_string;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" );
        description.add( comma::csv::program_options::description( "x,y,z,block" ) );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "downsample a point cloud using a voxel map" << std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: cat points.csv | points-to-voxels [options] > voxels.csv" << std::endl;
            std::cerr << std::endl;
            std::cerr << "input: points: x,y,z[,block]; default: x,y,z,block" << std::endl;
            std::cerr << "output: voxels with indices, centroids, and weights (number of points): i,j,k,x,y,z,weight,[block]" << std::endl;
            std::cerr << "binary output format: 3ui,3d,ui[,ui]" << std::endl;
            std::cerr << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            return 1;
        }
        if( vm.count( "resolution" ) == 0 ) { COMMA_THROW( comma::exception, "please specify --resolution" ); }        
        comma::csv::options csv = comma::csv::program_options::get( vm );
        Eigen::Vector3d origin;
        Eigen::Vector3d resolution;
        comma::csv::ascii< Eigen::Vector3d >().get( origin, origin_string );
        if( resolution_string.find_first_of( ',' ) == std::string::npos ) { resolution_string = resolution_string + ',' + resolution_string + ',' + resolution_string; }
        comma::csv::ascii< Eigen::Vector3d >().get( resolution, resolution_string );
        comma::csv::input_stream< input_point > istream( std::cin, csv );
        comma::csv::options output_csv = csv;
        output_csv.full_xpath = true;
        if( csv.has_field( "block" ) ) // todo: quick and dirty, make output fields configurable?
        {
            output_csv.fields = "index,mean,size,block";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui,ui" ); }
        }
        else
        {
            output_csv.fields = "index,mean,size";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui" ); }
        }
        comma::csv::output_stream< Centroid > ostream( std::cout, output_csv );
        comma::signal_flag is_shutdown;
        unsigned int block = 0;
        const input_point* last = NULL;
        while( !is_shutdown && !std::cin.eof() && std::cin.good() )
        {
            snark::voxel_map< Centroid, 3 > voxels( origin, resolution );
            if( last ) { voxels.touch_at( last->point )->second += last->point; }
            while( !is_shutdown && !std::cin.eof() && std::cin.good() )
            {
                last = istream.read();
                if( !last || last->block != block ) { break; }
                voxels.touch_at( last->point )->second += last->point;
            }
            if( is_shutdown ) { break; }
            for( snark::voxel_map< Centroid, 3 >::iterator it = voxels.begin(); it != voxels.end(); ++it )
            {
                it->second.block = block;
                it->second.index = snark::voxel_map< Centroid, 3 >::index_of( it->second.mean, origin, resolution );
                ostream.write( it->second );
            }
            if( !last ) { break; }
            block = last->block;
        }
        if( is_shutdown ) { std::cerr << "points-to-voxels: caught signal" << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << argv[0] << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << argv[0] << ": unknown exception" << std::endl;
    }
    return 1;
}
