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
/*
template < > struct traits< snark::asd::commands::restore::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::restore::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        for(int i=0;i<snark::asd::commands::restore::entry_count;i++)
        {
            v.apply( "entry[" + to_string(i) + "]", t.entry[i] );
        }
        v.apply( "count", t.count() );
        v.apply( "verify", t.verify() );
    }
};
*/
} }
