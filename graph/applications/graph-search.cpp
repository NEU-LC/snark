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

struct edge {};

namespace comma { namespace visiting {

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



int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        typedef snark::search_graph< Eigen::Vector3d, edge >::type graph_t;
        typedef snark::search_graph< Eigen::Vector3d, edge >::vertex_iter vertex_iterator;
        typedef snark::search_graph< Eigen::Vector3d, edge >::vertex_desc vertex_descriptor;
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
        
        //inline void forward_search( G& graph, const D& start, const A& advance, const O& objective_function, const V& valid )
        
        // todo
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
