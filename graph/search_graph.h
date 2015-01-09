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
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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

/// @author jason gan
/// @author vsevolod vlaskine

#ifndef SNARK_GRAPH_SEARCH_GRAPH_H_
#define SNARK_GRAPH_SEARCH_GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/subgraph.hpp>
#include <comma/base/types.h>

namespace snark {

namespace detail {

/// real quick and dirty, just because boost::optional is poorly supported in comma
/// @todo certainly refactor
template < typename T >
class optional
{
    public:
        optional() : is_set_( false ) {}
        optional( const T& t ) : is_set_( true ), value_( t ) {}
        operator bool() const { return is_set_; }
        T& operator*() { return value_; }
        const T& operator*() const { return value_; }
        T* operator->() { return &value_; }
        const T* operator->() const { return &value_; }
        void reset() { is_set_ = false; value_ = T(); }
        const optional& operator=( const T& t ) { is_set_ = true; value_ = t; return *this; }

    private:
        T value_;
        bool is_set_;
};

} // namespace detail {

template < typename Node, typename Edge >
struct search_node
{
    typedef Node value_type;
    typedef comma::uint32 id_type;

    comma::uint32 id;
    detail::optional< comma::uint32 > best_parent;
    value_type value;

    search_node() : id( 0 ) {}
    search_node( const Node& value, const detail::optional< comma::uint32 >& best_parent = detail::optional< comma::uint32 >() ) : id( 0 ), best_parent( best_parent ), value( value ) {}
    search_node( comma::uint32 id, const Node& value, const detail::optional< comma::uint32 >& best_parent = detail::optional< comma::uint32 >() ) : id( id ), best_parent( best_parent ), value( value ) {}
};

template < typename Node, typename Edge >
struct search_graph
{
    typedef Node node_type;
    typedef Edge edge_type;
    typedef search_node< Node, Edge > node;
    typedef Edge edge;
    typedef boost::adjacency_list< boost::setS, boost::setS, boost::bidirectionalS, node, edge > type;
    //typedef boost::subgraph< boost::adjacency_list< boost::setS, boost::setS, boost::bidirectionalS, node, edge > > type;

    typedef typename boost::graph_traits< type >::vertices_size_type vertex_size;
    typedef typename boost::graph_traits< type >::vertex_iterator vertex_iter;
    typedef std::pair<vertex_iter, vertex_iter> vertex_iter_pair;
    typedef typename boost::graph_traits< type >::vertex_descriptor vertex_desc;
    typedef typename boost::graph_traits< type >::edges_size_type edge_size;
    typedef typename boost::graph_traits< type >::edge_iterator edge_iter;
    typedef std::pair<edge_iter, edge_iter> edge_iter_pair;
    typedef typename boost::graph_traits< type >::edge_descriptor edge_desc;
    typedef typename boost::graph_traits< type >::out_edge_iterator out_edge_iter;
    typedef std::pair<out_edge_iter, out_edge_iter> out_edge_iter_pair;
    typedef typename boost::graph_traits< type >::in_edge_iterator in_edge_iter;
    typedef std::pair<in_edge_iter, in_edge_iter> in_edge_iter_pair;
};

} // namespace snark {

#endif // SNARK_GRAPH_SEARCH_GRAPH_H_
