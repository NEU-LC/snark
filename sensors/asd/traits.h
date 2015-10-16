#pragma once
#include "commands.h"
#include <comma/visiting/traits.h>
#include <boost/graph/graph_concepts.hpp>

namespace comma { namespace visiting {
    
template < > struct traits< snark::asd::commands::reply_header >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::reply_header& t, V& v )
    {
        v.apply( "header", t.header() );
        v.apply( "error", t.error() );
    }
};
template < > struct traits< snark::asd::commands::name_value >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::name_value& t, V& v )
    {
        v.apply( "name", std::string(t.name) );
        v.apply( "value", t.value() );
    }
};

template < > struct traits< snark::asd::commands::version::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::version::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        traits<snark::asd::commands::name_value>::visit(k,t.entry,v);
        v.apply( "type", t.type() );
        v.apply( "type_description", snark::asd::commands::version::reply::type_description(t.type()) );
    }
};

template < > struct traits< snark::asd::commands::abort::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::abort::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "name", std::string(t.entry.name) );
        //not used
        //v.apply( "value", t.value() );
        //v.apply( "count", t.count() );
    }
};

template < > struct traits< snark::asd::commands::optimize::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::optimize::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "itime", t.itime() );
        v.apply( "gain[0]",t.gain[0]() );
        v.apply( "gain[1]", t.gain[1]() );
        v.apply( "offset[0]", t.offset[0]() );
        v.apply( "offset[1]", t.offset[1]() );
    }
};

template < > struct traits< snark::asd::commands::restore::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::restore::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        for(int i=0;i<snark::asd::commands::restore::entry_count;i++)
        {
            v.apply( "entry[" + to_string(i) + "]", t.entry[i]() );
        }
        v.apply( "count", t.count() );
        v.apply( "verify", t.verify() );
    }
};

} }
