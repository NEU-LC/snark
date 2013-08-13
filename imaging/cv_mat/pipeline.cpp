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


#include <snark/imaging/cv_mat/pipeline.h>
#include <tbb/tbb_thread.h>
#include <boost/bind.hpp>

namespace snark{ namespace imaging { namespace applications {

/// constructor
/// @param fields csv output fields
/// @param format csv output format
/// @param filters string describing the filters
/// @param size max buffer size
/// @param mode output mode
pipeline::pipeline( cv_mat::serialization& output
                  , const std::string& filters
                  , tbb::bursty_reader< pair >& reader
                  , unsigned int number_of_threads )
    : m_output( output )
    , m_filters( snark::cv_mat::filters::make( filters ) )
    , m_reader( reader )
    , m_pipeline( number_of_threads )
{
    setup_pipeline_();
}

pipeline::pipeline( cv_mat::serialization& output
                  , const std::vector< cv_mat::filter >& filters
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
void pipeline::write_( pair p )
{
    if( p.second.size().width == 0 )
    {
        m_reader.stop();
        return;
    }
    if( std::cout.bad() || !std::cout.good() ) // if( std::cout.bad() || !std::cout.good() || is_shutdown_ )
    {
        m_reader.stop();
    }
    m_output.write( std::cout, p );
}

void pipeline::null_( pair p )
{
    if( p.second.size().width == 0 || std::cout.bad() || !std::cout.good() ) // if( p.second.size().width == 0 || std::cout.bad() || !std::cout.good() || is_shutdown_ )
    {
        m_reader.stop();
    }
}

/// setup the pipeline
/// @param filters name-value string describing the filters
void pipeline::setup_pipeline_()
{
    if( m_filters.empty() )
    {
        m_filter = ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( &pipeline::write_, this, _1 ) );
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
        m_filter = all_filters & ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( has_null ? &pipeline::null_ : &pipeline::write_, this, _1 ) );
    }
}

/// run the pipeline
void pipeline::run() { m_pipeline.run( m_reader, m_filter ); }

} } }

