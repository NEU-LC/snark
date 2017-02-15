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

#include <vector>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/core/core.hpp>
#include "serialization.h"

namespace snark{ namespace cv_mat {
    
struct header {
    boost::posix_time::ptime timestamp;
    std::vector< char > buffer;
    header() {}
    header(boost::posix_time::ptime t, const std::vector< char >& b ) : timestamp(t), buffer(b) {}
};

typedef header header_type;

template < typename T > struct header_traits;
template <> struct header_traits< boost::posix_time::ptime > { static boost::posix_time::ptime timestamp( boost::posix_time::ptime t ) { return t; } };
template <> struct header_traits< header > { static boost::posix_time::ptime timestamp( const header& h ) { return h.timestamp; } };

template < typename T > struct reader;
template < > struct reader< boost::posix_time::ptime > { 
    static std::pair< boost::posix_time::ptime, cv::Mat > read( serialization& s, std::istream& is ) { return s.read(is); } 
};
template < > struct reader< header_type > { 
    typedef std::pair< header_type, cv::Mat > type;
    static type read( serialization& s, std::istream& is ) 
    { 
        std::pair< boost::posix_time::ptime, cv::Mat > value = s.read(is); 
        return type( header(value.first, s.header_vector()), value.second );
    } 
};

template < typename T > struct writer;
template < > struct writer< boost::posix_time::ptime > { 
    typedef std::pair< boost::posix_time::ptime, cv::Mat > type;
    
    static void write( serialization& s, std::ostream& os, const type& m ) { return s.write(os, m); } 
    static std::size_t size( const serialization& s, const type& m ) { return s.size( m ); }
};
template < > struct writer< header_type > { 
    typedef std::pair< header_type, cv::Mat > type;
    typedef std::pair< serialization::header::buffer_t, cv::Mat > buffer_type;
    static void write( serialization& s, std::ostream& os, const type& m ) { 
        s.write( os, std::pair< serialization::header::buffer_t, cv::Mat >( m.first.buffer, m.second ) );
    } 
    static std::size_t size( const serialization& s, const type& m ) { return s.size( buffer_type( m.first.buffer, m.second ) ); }
};

template < typename Output = cv::Mat, typename H = boost::posix_time::ptime >
struct operation
{
    typedef std::pair< H, cv::Mat > input_type;
    typedef std::pair< H, Output > output_type;
    typedef input_type value_type; // quick and dirty for now
    operation( boost::function< output_type( value_type ) > f, bool p = true ): filter_function( f ), parallel( p ) {}
    boost::function< output_type( value_type ) > filter_function;
    bool parallel;
};

typedef operation<> filter;
typedef operation< cv::Mat, header_type > filter_with_header;

namespace impl {

template < typename H = boost::posix_time::ptime >
struct filters
{
    /// value type
    typedef std::pair< H, cv::Mat > value_type;
    typedef operation< cv::Mat, H > filter_type;
    
    /// return filters from name-value string
    static std::vector< filter_type > make( const std::string& how, unsigned int default_delay = 1 );
    
    /// apply filters (a helper)
    static value_type apply( std::vector< filter_type >& filters, value_type m );
    
    /// return filter usage
    static const std::string& usage( const std::string & operation = "" );
};

} // namespace impl {

typedef impl::filters<> filters;
typedef impl::filters< header_type > filters_with_header; // todo: a better name

/// a helper: e.g. take CV_8UC3, return CV_8UC1
int single_channel_type( int t );
std::string type_as_string( int t );

template < typename H >
bool is_empty( typename impl::filters< H >::value_type m ) { return ( header_traits< H >::timestamp( m.first ) == boost::posix_time::not_a_date_time ) && m.second.empty(); }

} }  // namespace snark { namespace cv_mat {
