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

/// @author vsevolod vlaskine

#ifndef SNARK_GRAPH_SERIALIZATION_H_
#define SNARK_GRAPH_SERIALIZATION_H_

#include <iostream>
#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>

namespace snark {

/// read graph vertices from a csv input stream
template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
void read_vertices( boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                  , std::istream& is
                  , const comma::csv::options& csv = comma::csv::options() );

/// read graph edges from a csv input stream
template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
void read_edges( boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
               , std::istream& os
               , const comma::csv::options& csv = comma::csv::options() );

/// write graph vertices to a csv output stream
template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
void write_vertices( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                   , std::ostream& os
                   , const comma::csv::options& csv = comma::csv::options() );

/// write graph edges from to csv output stream
template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
void write_egdes( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                , std::ostream& os
                , const comma::csv::options& csv = comma::csv::options() );

/// write graph edges with full source and target vertices to to csv output stream
template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
void write_edges_with_vertices( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                              , std::ostream& os
                              , const comma::csv::options& csv = comma::csv::options() );


template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void read_vertices( boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                         , std::istream& is
                         , const comma::csv::options& c )
{
    comma::csv::options csv = c;
    if( csv.fields.empty() ) { csv.full_xpath = true; } // quick and dirty
    comma::csv::input_stream< V > stream( is, csv );
    while( stream.ready() || ( is.good() && !is.eof() ) )
    {
        const V* v = stream.read();
        if( !v ) { break; }
        boost::add_vertex( *v, graph );
    }
}

template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void write_vertices( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                          , std::ostream& os
                          , const comma::csv::options& c )
{
    comma::csv::options csv = c;
    if( csv.fields.empty() ) { csv.full_xpath = true; } // quick and dirty
    typedef boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E > graph_t;
    comma::csv::output_stream< V > stream( os, csv );
    std::pair< typename graph_t::vertex_iterator, typename graph_t::vertex_iterator > vertices = boost::vertices( graph );
    for( typename graph_t::vertex_iterator it = vertices.first; it != vertices.second; ++it ) { stream.write( graph[ *it ] ); }
}

namespace impl {

template < typename T, typename E > struct serialized_edge
{
    T source;
    T target;
    E value;

    serialized_edge() {}
    serialized_edge( const T& source, const T& target, const E& e ) : source( source ), target( target ), value( e ) {}
};

} // namespace impl {

} // namespace snark {

namespace comma { namespace visiting {

template < typename Id, typename E > struct traits< snark::impl::serialized_edge< Id, E > >
{
    template < typename K, typename V > static void visit( const K& k, snark::impl::serialized_edge< Id, E >& t, V& v )
    {
        v.apply( "source", t.source );
        v.apply( "target", t.target );
        comma::visiting::traits< E >::visit( k, t.value, v );
    }

    template < typename K, typename V > static void visit( const K& k, const snark::impl::serialized_edge< Id, E >& t, V& v )
    {
        v.apply( "source", t.source );
        v.apply( "target", t.target );
        comma::visiting::traits< E >::visit( k, t.value, v );
    }
};

} } // namespace comma { namespace visiting {

namespace snark {

namespace impl {

struct output_id;
struct output_all;

template < typename P, typename T = output_all > struct edge_traits
{
    template < typename E, typename G > static const P& source( const E& e, const G& g ) { return g[ boost::source( e, g ) ]; }
    template < typename E, typename G > static const P& target( const E& e, const G& g ) { return g[ boost::target( e, g ) ]; }
};

template < typename P > struct edge_traits< P, output_id >
{
    template < typename E, typename G > static const P& source( const E& e, const G& g ) { return g[ boost::source( e, g ) ].id; }
    template < typename E, typename G > static const P& target( const E& e, const G& g ) { return g[ boost::target( e, g ) ].id; }
};

template < typename T, typename P, typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void write_edges_( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                        , std::ostream& os
                        , const comma::csv::options& c )
{
    comma::csv::options csv = c;
    if( csv.fields.empty() ) { csv.full_xpath = true; } // quick and dirty
    typedef boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E > graph_t;
    typedef impl::serialized_edge< P, E > edge_t;
    comma::csv::output_stream< edge_t > stream( os, csv );
    std::pair< typename graph_t::edge_iterator, typename graph_t::edge_iterator > edges = boost::edges( graph );
    for( typename graph_t::edge_iterator it = edges.first; it != edges.second; ++it )
    {
        const typename graph_t::edge_descriptor& e = *it;
        const P& source = impl::edge_traits< P, T >::source( e, graph );
        const P& target = impl::edge_traits< P, T >::target( e, graph );
        stream.write( edge_t( source, target, graph[ *it ] ) );
    }
}

} // namespace impl {

template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void write_edges( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                       , std::ostream& os
                       , const comma::csv::options& csv )
{
    impl::write_edges_< impl::output_id, typename V::id_type >( graph, os, csv );
}

template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void write_edges_with_vertices( const boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                                     , std::ostream& os
                                     , const comma::csv::options& csv )
{
    impl::write_edges_< impl::output_all, V >( graph, os, csv );
}

template < typename V, typename E, typename Directed, typename VertexStorage, typename EdgeStorage >
inline void read_edges( boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E >& graph
                      , std::istream& is
                      , const comma::csv::options& c )
{
    comma::csv::options csv = c;
    if( csv.fields.empty() ) { csv.full_xpath = true; } // quick and dirty
    typedef boost::adjacency_list< VertexStorage, EdgeStorage, Directed, V, E > graph_t;
    typedef typename V::id_type id_type;
    typedef impl::serialized_edge< id_type, E > edge_t;
    typedef boost::unordered_map< id_type, typename graph_t::vertex_iterator > vertices_map;
    vertices_map vertices; // super-lame
    std::pair< typename graph_t::vertex_iterator, typename graph_t::vertex_iterator > begin_end = boost::vertices( graph );
    for( typename graph_t::vertex_iterator it = begin_end.first; it != begin_end.second; vertices[ graph[ *it ].id ] = it, ++it );
    comma::csv::input_stream< edge_t > stream( is, csv );
    while( stream.ready() || ( is.good() && !is.eof() ) )
    {
        const edge_t* e = stream.read();
        if( !e ) { break; }
        typename vertices_map::const_iterator source = vertices.find( e->source );
        if( source == vertices.end() ) { COMMA_THROW( comma::exception, "vertex not found for key: " << e->source ); }
        typename vertices_map::const_iterator target = vertices.find( e->target );
        if( target == vertices.end() ) { COMMA_THROW( comma::exception, "vertex not found for key: " << e->target ); }
        typename graph_t::edge_descriptor d;
        bool linked = false;
        boost::tie( d, linked ) = boost::add_edge( *( source->second ), *( target->second ), graph );
        if( linked ) { graph[d] = e->value; }
    }
}

} // namespace snark {

#endif // SNARK_GRAPH_SERIALIZATION_H_
