// Copyright (c) 2011 The University of Sydney

/// @author vsevolod vlaskine

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <boost/static_assert.hpp>
#include <Eigen/Geometry>
#include <comma/application/command_line_options.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include "../../visiting/traits.h"
#include "../search.h"
#include "../search_graph.h"
#include "../serialization.h"
#include "../traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "search graph" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: graph-search <options> > best_path.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: --help --verbose for more help" << std::endl;
    std::cerr << "    --edge-fields: output edge fields and exit" << std::endl;
    std::cerr << "    --edges=<filename>[,<csv options>]: graph edges" << std::endl;
    std::cerr << "        fields: source,target,cost" << std::endl;
    std::cerr << "        default fields" << std::endl;
    std::cerr << "            if x, y, or z fields present for nodes: source,target" << std::endl;
    std::cerr << "            otherwise: source,target,cost" << std::endl;
    std::cerr << "    --node-fields: output node fields and exit" << std::endl;
    std::cerr << "    --nodes,--vertices=[<filename>[,<csv options>]]: graph nodes" << std::endl;
    std::cerr << "        fields: x,y,z,id" << std::endl;
    std::cerr << "        default fields: id" << std::endl;
    std::cerr << "    --permissive; if present and only --source given, skip non-existing or unreachable targets" << std::endl;
    std::cerr << "    --source,--start-id,--from,--start,--origin: source node id" << std::endl;
    std::cerr << "        if --target specified, find the best way to target" << std::endl;
    std::cerr << "        else read target ids on stdin (i.e. search graph once, pull best path many times)" << std::endl;
    std::cerr << "    --target,--target-id,--to,--destination: target node id" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    graph defined by edges only; find best path from node 0 to node 4" << std::endl;
    std::cerr << "        > cat <<eof > simple-edges.csv" << std::endl;
    std::cerr << "        0,1" << std::endl;
    std::cerr << "        1,2" << std::endl;
    std::cerr << "        2,3" << std::endl;
    std::cerr << "        3,4" << std::endl;
    std::cerr << "        0,2" << std::endl;
    std::cerr << "        2,4" << std::endl;
    std::cerr << "        eof" << std::endl;
    std::cerr << "        > ( echo 0 ; echo 4 ) | graph-search --edges=\"simple-edges.csv\"" << std::endl;
    std::cerr << "    find best path from node 1 to node 100" << std::endl;
    std::cerr << "        > ( echo 1 ; echo 100 ; ) | graph-search --nodes=\"nodes.csv\" --edges=\"edges.csv\"" << std::endl;
    std::cerr << "    find best path by euclidean distance between the nodes" << std::endl;
    std::cerr << "        > ( echo 1 ; echo 100 ; ) | graph-search --nodes=\"nodes.csv;fields=x,y,z,id\" --edges=\"edges.csv\"" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct node
{
    Eigen::Vector3d position;
    double cost;
    std::string record;
    node() : position( 0, 0, 0 ), cost( std::numeric_limits< double >::max() ) {}
    node( const Eigen::Vector3d& position, double cost = std::numeric_limits< double >::max() ) : position( position ), cost( cost ) {}
};

struct edge
{
    double cost;
    edge() : cost( 1 ) {}
};

struct record { comma::uint32 id; };

namespace comma { namespace visiting {

template <> struct traits< node >
{
    template < typename K, typename V > static void visit( const K&, node& n, V& v )
    {
        v.apply( "position", n.position );
        v.apply( "cost", n.cost );
    }

    template < typename K, typename V > static void visit( const K&, const node& n, V& v )
    {
        v.apply( "position", n.position );
        v.apply( "cost", n.cost );
    }
};

template <> struct traits< edge >
{
    template < typename K, typename V > static void visit( const K&, edge& n, V& v ) { v.apply( "cost", n.cost ); }
    template < typename K, typename V > static void visit( const K&, const edge& n, V& v ) { v.apply( "cost", n.cost ); }
};

template <> struct traits< record >
{
    template < typename K, typename V > static void visit( const K&, record& n, V& v ) { v.apply( "id", n.id ); }
    template < typename K, typename V > static void visit( const K&, const record& n, V& v ) { v.apply( "id", n.id ); }
};
    
} } // namespace comma { namespace visiting {

typedef snark::search_graph< node, edge > search_graph_t;
typedef search_graph_t::type graph_t;
typedef search_graph_t::vertex_iter vertex_iterator;
typedef search_graph_t::vertex_desc vertex_descriptor;
typedef boost::unordered_map< comma::uint32, std::string > records_t;
static bool verbose = false;
static bool by_distance = false;
static graph_t graph;
static records_t records;

static const std::string& node_record( comma::uint32 id )
{
    if( !records.empty() ) { return records[id]; }
    static std::string s;
    s = boost::lexical_cast< std::string >( id );
    return s;
}

static void load_( graph_t& graph
                 , const boost::optional< comma::csv::options >& node_csv
                 , const comma::csv::options& edge_csv )
{
    std::ifstream eif( &edge_csv.filename[0] );
    if( !eif.is_open() ) { std::cerr << "graph-search: failed to open " << edge_csv.filename << std::endl; exit( 1 ); }
    if( node_csv )
    {
        std::ifstream vif( &node_csv->filename[0] );
        if( !vif.is_open() ) { std::cerr << "graph-search: failed to open " << node_csv->filename << std::endl; exit( 1 ); }
        if( verbose ) { std::cerr << "graph-search: loading vertices from " << node_csv->filename << "..." << std::endl; }
        snark::read_vertices( graph, vif, *node_csv );
        vif.close();
    }
    else
    {
        std::cerr << "graph-search: --nodes not given, vertices implied from edges: " << edge_csv.filename << std::endl;
    }
    if( verbose ) { std::cerr << "graph-search: loading edges from " << edge_csv.filename << "..." << std::endl; }
    snark::read_edges( graph, eif, edge_csv, !node_csv );
    eif.close();
    if( verbose ) { std::cerr << "graph-search: loaded graph: " << boost::num_vertices( graph ) << " vertices " << boost::num_edges( graph ) << " edges" << std::endl; }
}

static void load_records_( records_t& r, const comma::csv::options& csv )
{
    std::ifstream ifs( &csv.filename[0] );
    if( !ifs.is_open() ) { std::cerr << "graph-search: failed to open " << csv.filename << std::endl; exit( 1 ); }
    comma::csv::input_stream< search_graph_t::node > stream( ifs, csv );
    while( stream.ready() || ( ifs.good() && !ifs.eof() ) )
    {
        const search_graph_t::node* v = stream.read();
        if( !v ) { break; }
        r[ v->id ] = csv.binary() ? std::string( stream.binary().last(), csv.format().size() ) : ( comma::join( stream.ascii().last(), csv.delimiter ) );
    }
    ifs.close();
}

static double objective_function( const node& n ) { return -n.cost; }

static boost::optional< node > advance( const node& from, const node& to, const edge& e )
{
    return node( to.position, from.cost + ( by_distance ? ( to.position - from.position ).norm() : e.cost ) );
}

static bool valid( const node& n ) { return true; }

static void reset_graph()
{
    for( std::pair< vertex_iterator, vertex_iterator > d = boost::vertices( graph ); d.first != d.second; ++d.first )
    {
        search_graph_t::node& n = graph[ *d.first ];
        n.best_parent.reset();
        n.value.cost = std::numeric_limits< double >::max();
    }
}

static void forward_search( vertex_descriptor source )
{
    if( verbose ) { std::cerr << "graph-search: searching..." << std::endl; }
    graph[ source ].value.cost = 0; // todo: is it right at all?
    snark::forward_search( graph, source, &advance, &objective_function, &valid );
}

static std::pair< vertex_descriptor, vertex_descriptor > forward_search( comma::uint32 source_id, boost::optional< comma::uint32 > target_id = boost::none )
{
    reset_graph();
    vertex_descriptor source = nullptr;
    vertex_descriptor target = nullptr;
    for( std::pair< vertex_iterator, vertex_iterator > d = boost::vertices( graph ); d.first != d.second && ( !source || ( target_id && !target ) ); ++d.first )
    {
        if( graph[ *d.first ].id == source_id ) { source = *d.first; }
        if( target_id && graph[ *d.first ].id == *target_id ) { target = *d.first; }
    }
    if( !source ) { std::cerr << "graph-search: source id " << source_id << " not found in the graph" << std::endl; exit( 1 ); }
    forward_search( source );
    return std::make_pair( source, target );
}

static std::vector< vertex_descriptor > best_path( comma::uint32 source_id, comma::uint32 target_id )
{    
    auto p = forward_search( source_id, target_id );
    if( verbose ) { std::cerr << "graph-search: extracting best path from " << source_id << " to " << target_id << "..." << std::endl; }
    return p.first == p.second ? std::vector< vertex_descriptor >( 1, p.first ) : snark::best_path( graph, p.first, p.second );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        bool permissive = options.exists( "--permissive,--skip-non-existing-targets" );
        if( options.exists( "--edge-fields" ) ) { std::cout << "source,target," << comma::join( comma::csv::names< ::edge >(), ',' ) << std::endl; return 0; } // quick and dirty
        if( options.exists( "--node-fields" ) ) { std::cout << "id," << comma::join( comma::csv::names< ::node >( false ), ',' ) << std::endl; return 0; } // quick and dirty
        boost::optional< comma::csv::options > node_csv;
        if( options.exists( "--vertices,--nodes" ) )
        { 
            node_csv = comma::name_value::parser( "filename", ';' ).get< comma::csv::options >( options.value< std::string >( "--vertices,--nodes" ) );
            if( node_csv->fields.empty() ) { node_csv->fields = "id"; }
            std::vector< std::string > v = comma::split( node_csv->fields, ',' );
            for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "x" || v[i] == "y" || v[i] == "z" ) { v[i] = "value/position/" + v[i]; by_distance = true; } }
            node_csv->fields = comma::join( v, ',' );
            node_csv->full_xpath = true;
        }
        comma::csv::options edge_csv = comma::name_value::parser( "filename", ';' ).get< comma::csv::options >( options.value< std::string >( "--edges" ) );
        edge_csv.full_xpath = true;
        if( edge_csv.fields.empty() ) { edge_csv.fields = "source,target"; } //if( by_distance && edge_csv.fields.empty() ) { edge_csv.fields = "source,target"; }
        load_( graph, node_csv, edge_csv );
        if( node_csv ) { load_records_( records, *node_csv ); }
        boost::optional< unsigned int > source_id = options.optional< unsigned int >( "--start,--start-id,--from,--source,--origin" );
        boost::optional< unsigned int > target_id = options.optional< unsigned int >( "--target,--target-id,--to,--destination" );
        if( !source_id && target_id ) { std::cerr << "graph-search: --target specified, thus, please specify --source" << std::endl; return 1; }
        if( source_id )
        {
            if( target_id )
            {
                const std::vector< vertex_descriptor >& p = best_path( *source_id, *target_id );
                for( std::size_t i = 0; i < p.size(); ++i )
                {
                    const std::string& s = node_record( graph[ p[i] ].id );
                    std::cout.write( &s[0], s.size() );
                    if( !node_csv || !node_csv->binary() ) { std::cout << std::endl; }
                }
            }
            else
            {
                if( verbose ) { std::cerr << "graph-search: source id given, reading target ids from stdin..." << std::endl; }
                comma::csv::options csv( options );
                comma::csv::input_stream< record > istream( std::cin, csv );
                if( node_csv && csv.binary() != node_csv->binary() ) { std::cerr << "graph-search: expected stdin and " << node_csv->filename << " of the same type; got stdin " << ( csv.binary() ? "binary" : "ascii" ) << ", " << node_csv->filename << ": " << ( node_csv->binary() ? "binary" : "ascii" ) << std::endl; }
                auto p = forward_search( *source_id );
                std::unordered_map< comma::uint32, vertex_descriptor > descriptors;
                for( std::pair< vertex_iterator, vertex_iterator > d = boost::vertices( graph ); d.first != d.second; descriptors[ graph[ *d.first ].id ] = *d.first, ++d.first );
                auto source = descriptors.find( *source_id );
                if( source == descriptors.end() ) { std::cerr << "graph-search: source node with id " << *source_id << " not found in the graph" << std::endl; return 1; }
                forward_search( source->second );
                std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
                std::string endl = csv.binary() ? "" : std::string( 1, '\n' );
                while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
                {
                    const record* r = istream.read();
                    if( !r ) { break; }
                    auto it = descriptors.find( r->id );
                    if( it == descriptors.end() )
                    { 
                        if( permissive ) { if( verbose ) { std::cerr << "graph-search: target node with id " << r->id << " not found in the graph; discarded" << std::endl; } }
                        else { std::cerr << "graph-search: target node with id " << r->id << " not found in the graph" << std::endl; return 1; }
                    }
                    const auto& path = it->second == source->second ? std::vector< vertex_descriptor >( 1, p.first ) : snark::best_path( graph, source->second, it->second );
                    if( path.empty() )
                    { 
                        if( permissive ) { if( verbose ) { std::cerr << "graph-search: failed to find path from " << *source_id << " to " << r->id << "; discarded" << std::endl; } }
                        else { std::cerr << "graph-search: failed to find path from " << *source_id << " to " << r->id << std::endl; return 1; }
                    }
                    const auto& s = istream.last();
                    for( std::size_t i = 0; i < path.size(); ++i )
                    {
                        const std::string& n = node_record( graph[ path[i] ].id );
                        std::cout.write( &n[0], n.size() );
                        std::cout << delimiter;
                        std::cout.write( &s[0], s.size() );
                        std::cout << endl;
                    }
                }
            }
        }
        else
        {
            if( verbose ) { std::cerr << "graph-search: source and target ids not given, reading from stdin..." << std::endl; }
            comma::uint32 last_id = 0;
            std::string last;
            comma::csv::options csv( options );
            comma::csv::input_stream< record > istream( std::cin, csv );
            if( node_csv && csv.binary() != node_csv->binary() ) { std::cerr << "graph-search: expected stdin and " << node_csv->filename << " of the same type; got stdin " << ( csv.binary() ? "binary" : "ascii" ) << ", " << node_csv->filename << ": " << ( node_csv->binary() ? "binary" : "ascii" ) << std::endl; }
            std::string delimiter = csv.binary() ? "" : std::string( 1, csv.delimiter );
            std::string endl = csv.binary() ? "" : std::string( 1, '\n' );
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const record* r = istream.read();
                if( !r ) { break; }
                if( !last.empty() )
                {
                    const std::vector< vertex_descriptor >& p = best_path( last_id, r->id );
                    if( p.empty() ) { std::cerr << "graph-search: failed to find path from " << last_id << " to " << r->id << std::endl; return 1; }
                    for( std::size_t i = 0; i < p.size(); ++i )
                    {
                        const std::string& s = node_record( graph[ p[i] ].id );
                        std::cout.write( &s[0], s.size() );
                        std::cout << delimiter;
                        std::cout.write( &last[0], last.size() );
                        std::cout << endl;
                    }
                }
                last_id = r->id;
                last = csv.binary() ? std::string( istream.binary().last(), csv.format().size() ) : comma::join( istream.ascii().last(), csv.delimiter );
            }
        }
        if( verbose ) { std::cerr << "graph-search: done" << std::endl; }    
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "graph-search: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "graph-search: unknown exception" << std::endl; }
    return 1;
}
