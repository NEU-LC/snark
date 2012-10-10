#ifndef SNARK_IMAGING_BURSTY_PIPELINE_H_
#define SNARK_IMAGING_BURSTY_PIPELINE_H_

#include <tbb/task_scheduler_init.h>
#include <tbb/pipeline.h>
#include <boost/thread.hpp>
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
