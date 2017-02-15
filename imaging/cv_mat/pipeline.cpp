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

#include <boost/bind.hpp>
#include <tbb/tbb_thread.h>
#include <comma/base/last_error.h>
#include "pipeline.h"

namespace snark{ namespace imaging { namespace applications {
    
namespace impl {

/// constructor
/// @param fields csv output fields
/// @param format csv output format
/// @param filters string describing the filters
/// @param size max buffer size
/// @param mode output mode
template < typename H >
pipeline< H >::pipeline( cv_mat::serialization& output
                  , const std::string& filters
                  , tbb::bursty_reader< pair >& reader
                  , cv_mat::serialization::binary_type binary
                  , unsigned int number_of_threads )
    : m_output( output )
    , m_filters( filters_type::make( filters, binary ) )
    , m_reader( reader )
    , m_pipeline( number_of_threads )
{
    setup_pipeline_();
}

template < typename H >
pipeline< H >::pipeline( cv_mat::serialization& output
                  , const std::vector< filter_type >& filters
                  , tbb::bursty_reader< pair >& reader
                  , unsigned int number_of_threads )
    : m_output( output )
    , m_filters( filters )
    , m_reader( reader )
    , m_pipeline( number_of_threads )
{
    setup_pipeline_();
}

/// write frame to std out
template < typename H >
void pipeline< H >::write_( pair p )
{
    if( p.second.empty() )
    {
        m_reader.stop();
        return;
    }
    if( !std::cout.good() )
    {
        m_reader.stop();
    }
    // We use write_to_stdout() rather than write() because we saw issues with using std::cout.
    // See serialization.cpp for details.
    try { m_output.write_to_stdout( p ); }
    catch( const comma::last_error::exception& ex )
    {
        if( ex.value != EPIPE ) { m_error = ex.what(); }
        m_reader.stop();
    }
    catch( const comma::exception& ex )
    {
        m_error = ex.what();
        m_reader.stop();
    }
}

template < typename H >
void pipeline< H >::null_( pair p )
{
    if( p.second.empty() || !std::cout.good() )
    {
        m_reader.stop();
    }
}

/// setup the pipeline
/// @param filters name-value string describing the filters
template < typename H >
void pipeline< H >::setup_pipeline_()
{
    if( m_filters.empty() )
    {
        m_filter = ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( &pipeline< H >::write_, this, _1 ) );
    }
    else
    {
        ::tbb::filter_t< pair, pair > all_filters;
        bool has_null = false;
        for( std::size_t i = 0; i < m_filters.size(); ++i )
        {
            ::tbb::filter::mode mode = ::tbb::filter::serial_in_order;
            if( m_filters[i].parallel )
            {
                mode = ::tbb::filter::parallel;
            }
            if( !m_filters[i].filter_function ) { has_null = true; break; }
            ::tbb::filter_t< pair, pair > filter( mode, boost::bind( m_filters[i].filter_function, _1 ) );
            all_filters = i == 0 ? filter : ( all_filters & filter );
        }
        m_filter = all_filters & ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( has_null ? &pipeline< H >::null_ : &pipeline< H >::write_, this, _1 ) );
    }
}

/// run the pipeline
template < typename H >
void pipeline< H >::run() { m_pipeline.run( m_reader, m_filter ); m_reader.join(); }

template class pipeline< boost::posix_time::ptime >;

} //namespace impl {

} } }

