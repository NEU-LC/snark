// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_IMAGING_BURSTY_PIPELINE_H_
#define SNARK_IMAGING_BURSTY_PIPELINE_H_

#include <tbb/task_scheduler_init.h>
#include <tbb/pipeline.h>
#include <snark/tbb/bursty_reader.h>

namespace snark { namespace tbb {

/// run a tbb pipeline with bursty data using bursty_reader
template< typename T >
class bursty_pipeline
{
public:
    bursty_pipeline( unsigned int numThread = 0 );
    void run_once( bursty_reader< T >& reader, const ::tbb::filter_t< T, void >& filter );
    void run( bursty_reader< T >& reader, const ::tbb::filter_t< T, void >& filter );

private:
    unsigned int m_threads;
    ::tbb::task_scheduler_init m_init;
};

/// constructor
/// @param numThread maximum number of threads, 0 means auto
template< typename T >
bursty_pipeline< T >::bursty_pipeline( unsigned int numThread ):
    m_threads( numThread )
{
    if( numThread == 0 )
    {
        m_threads = m_init.default_num_threads();
    }
}

/// run the pipeline once
template< typename T >
void bursty_pipeline< T >::run_once( bursty_reader< T >& reader, const ::tbb::filter_t< T, void >& filter )
{
    ::tbb::parallel_pipeline( m_threads, reader.filter() & filter );
}

/// run the pipeline until the reader stops and the queue is empty
template< typename T >
void bursty_pipeline< T >::run( bursty_reader< T >& reader, const ::tbb::filter_t< T, void >& filter )
{
    const ::tbb::filter_t< void, void > f = reader.filter() & filter;
    while( reader.wait() )
    {
        ::tbb::parallel_pipeline( m_threads, f );
    }
}



} }

#endif // SNARK_IMAGING_BURSTY_PIPELINE_H_
