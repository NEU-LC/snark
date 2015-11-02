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
#include <comma/packed/traits.h>

namespace comma { namespace visiting {

template<std::size_t N>
std::string truncate(const comma::packed::string<N>& f)
{
    std::string s=f();
    std::size_t i=s.find('\0');
    if(i==std::string::npos)
        return "";
    else
        return s.substr(0,i);
}


template < std::size_t M, std::size_t N > struct traits< boost::array<comma::packed::string<M>, N> >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array<comma::packed::string<M>, N>& t, V& v )
    {
        for( std::size_t i=0;i<t.size();i++)
        {
            v.apply( i, truncate(t[i]) );
        }
    }
};


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
        v.apply( "name", truncate(t.name) );
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
    }
};

template < > struct traits< snark::asd::commands::abort::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::abort::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "name", truncate(t.entry.name) );
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
        v.apply( "gain",t.gain );
        v.apply( "offset", t.offset );
    }
};

template < > struct traits< snark::asd::commands::restore::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::restore::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "names", t.names );
        v.apply( "values", t.values );
        v.apply( "count", t.count() );
        v.apply( "verify", t.verify() );
    }
};

template < > struct traits< snark::asd::commands::init::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::init::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "entry", t.entry );
        v.apply( "count", t.count() );
    }
};

template < > struct traits< snark::asd::commands::save::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::save::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "names", t.names );
        v.apply( "values", t.values );
        v.apply( "count", t.count() );
        v.apply( "verify", t.verify() );
    }
};

template < > struct traits< snark::asd::commands::erase::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::erase::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "names", t.names );
        v.apply( "values", t.values );
        v.apply( "count", t.count() );
        v.apply( "verify", t.verify() );
    }
};

template < > struct traits< snark::asd::commands::instrument_gain_control::reply >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::instrument_gain_control::reply& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "detector", t.detector() );
        v.apply( "command_type",t.command_type() );
        v.apply( "value", t.value() );
    }
};

template < > struct traits< snark::asd::commands::acquire_data::vnir_header >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::acquire_data::vnir_header& t, V& v )
    {
        v.apply( "integration_time", t.integration_time() );
        v.apply( "scans", t.scans() );
        v.apply( "max_channel", t.max_channel() );
        v.apply( "min_channel", t.min_channel() );
        v.apply( "saturation", t.saturation() );
        v.apply( "shutter", t.shutter() );
    }
};

template < > struct traits< snark::asd::commands::acquire_data::swir_header >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::acquire_data::swir_header& t, V& v )
    {
        v.apply( "tec_status", t.tec_status() );
        v.apply( "tec_current", t.tec_current() );
        v.apply( "max_channel", t.max_channel() );
        v.apply( "min_channel", t.min_channel() );
        v.apply( "saturation", t.saturation() );
        v.apply( "a_scans", t.a_scans() );
        v.apply( "b_scans", t.b_scans() );
        v.apply( "dark_current", t.dark_current() );
        v.apply( "gain", t.gain() );
        v.apply( "offset", t.offset() );
        v.apply( "scan_size_1", t.scan_size_1() );
        v.apply( "scan_size_2", t.scan_size_2() );
    }
};

template < > struct traits< snark::asd::commands::acquire_data::spectrum_header >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::acquire_data::spectrum_header& t, V& v )
    {
        traits<snark::asd::commands::reply_header>::visit(k,t.header,v);
        v.apply( "sample_count", t.sample_count() );
        v.apply( "trigger", t.trigger() );
        v.apply( "voltage", t.voltage() );
        v.apply( "current", t.current() );
        v.apply( "temprature", t.temprature() );
        v.apply( "motor_current", t.motor_current() );
        v.apply( "instrument_hours", t.instrument_hours() );
        v.apply( "instrument_minutes", t.instrument_minutes() );
        v.apply( "instrument_type", t.instrument_type() );
        v.apply( "ab", t.ab() );
        v.apply( "v_header", t.v_header );
        v.apply( "s1_header", t.s1_header );
        v.apply( "s2_header", t.s2_header );
    }
};

template < > struct traits< snark::asd::commands::acquire_data::spectrum_data >
{
    template< typename K, typename V > static void visit( const K& k, const snark::asd::commands::acquire_data::spectrum_data& t, V& v )
    {
        traits<snark::asd::commands::acquire_data::spectrum_header>::visit(k,t.header,v);
        v.apply( "values", t.values );
    }
};

} } //namespace comma { namespace visiting {
