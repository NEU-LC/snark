#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <comma/application/command_line_options.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include <snark/graph/search.h>
#include <snark/graph/search_graph.h>
#include <snark/graph/serialization.h>
#include <snark/graph/traits.h>
#include <snark/visiting/traits.h>

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct node
{
    Eigen::Vector3d position;
    double distance;
    
    node() : distance( 0 ) {}
    node( const Eigen::Vector3d& position, double distance = 0 ) : position( position ), distance( distance ) {}
};

struct edge {};

namespace comma { namespace visiting {

template <> struct traits< node >
{
    template < typename K, typename V > static void visit( const K&, node& n, V& v )
    {
        v.apply( "position", n.position );
        v.apply( "distance", n.distance );
    }
    
    template < typename K, typename V > static void visit( const K&, const node& n, V& v )
    {
        v.apply( "position", n.position );
        v.apply( "distance", n.distance );
    }
};

template <> struct traits< edge >
{
    template < typename K, typename V > static void visit( const K&, edge& n, V& v ) {}
    template < typename K, typename V > static void visit( const K&, const edge& n, V& v ) {}
};
    
} } // namespace comma { namespace visiting {

static bool verbose = false;

template < typename Graph > static void load_( Graph& graph
                                             , const std::string& vertices_option
                                             , const std::string& edges_option )
{
    comma::csv::options vertex_csv = comma::name_value::parser( "filename", ';' ).get< comma::csv::options >( vertices_option );
    comma::csv::options edge_csv = comma::name_value::parser( "filename", ';' ).get< comma::csv::options >( edges_option );
    if( vertex_csv.fields.empty() ) { vertex_csv.fields = "value/position/x,value/position/y,value/position/z,id"; }
    vertex_csv.full_xpath = true;
    edge_csv.full_xpath = true;
    std::ifstream vif( &vertex_csv.filename[0] );
    if( !vif.is_open() ) { std::cerr << "graph-search: failed to open " << vertex_csv.filename << std::endl; exit( 1 ); }
    std::ifstream eif( &edge_csv.filename[0] );
    if( !eif.is_open() ) { std::cerr << "graph-search: failed to open " << edge_csv.filename << std::endl; exit( 1 ); }
    if( verbose ) { std::cerr << "graph-search: loading vertices from " << vertex_csv.filename << "..." << std::endl; }
    snark::read_vertices( graph, vif, vertex_csv );
    if( verbose ) { std::cerr << "graph-search: loading edges from " << edge_csv.filename << "..." << std::endl; }
    snark::read_edges( graph, eif, edge_csv );
    if( verbose ) { std::cerr << "graph-search: loaded graph: " << boost::num_vertices( graph ) << " vertices " << boost::num_edges( graph ) << " edges" << std::endl; }
}

static double objective_function( const node& n ) { return -n.distance; }

static boost::optional< node > advance( const node& from, const node& to ) { return node( to.position, from.distance + ( to.position - from.position ).norm() ); }

static bool valid( const node& n ) { return true; }

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        comma::csv::options output_csv( options );
        typedef snark::search_graph< node, edge > search_graph_t;
        typedef search_graph_t::type graph_t;
        typedef search_graph_t::vertex_iter vertex_iterator;
        typedef search_graph_t::vertex_desc vertex_descriptor;
        graph_t graph;
        load_( graph, options.value< std::string >( "--vertices,--nodes" ), options.value< std::string >( "--edges" ) );
        unsigned int source_id = options.value< unsigned int >( "--start,--start-id,--source,--origin" );
        unsigned int target_id = options.value< unsigned int >( "--target,--destination" );
        vertex_descriptor source = NULL;
        vertex_descriptor target = NULL;
        for( std::pair< vertex_iterator, vertex_iterator > d = boost::vertices( graph ); d.first != d.second; ++d.first )
        {
            if( graph[ *d.first ].id == source_id ) { source = *d.first; }
            else if( graph[ *d.first ].id == target_id ) { target = *d.first; }
            if( source && target ) { break; }
        }
        if( !source ) { std::cerr << "graph-search: source id " << source_id << " not found in the graph" << std::endl; return 1; }
        if( !target ) { std::cerr << "graph-search: target id " << target_id << " not found in the graph" << std::endl; return 1; }
        if( verbose ) { std::cerr << "graph-search: searching..." << std::endl; }
        snark::forward_search( graph, source, &advance, &objective_function, &valid );
        if( verbose ) { std::cerr << "graph-search: extracting best path..." << std::endl; }
        const std::vector< vertex_descriptor >& best_path = snark::best_path( graph, source, target );
        output_csv.full_xpath = true;
        comma::csv::output_stream< search_graph_t::node > os( std::cout, output_csv );
        if( verbose ) { std::cerr << "graph-search: outputting best path of " << best_path.size() << " node(s)..." << std::endl; }
        for( std::size_t i = 0; i < best_path.size(); ++i ) { os.write( graph[ best_path[i] ] ); }
        if( verbose ) { std::cerr << "graph-search: done" << std::endl; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "graph-search: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "graph-search: unknown exception" << std::endl;
    }
    return 1;
}
