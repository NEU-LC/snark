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

#ifndef SNARK_TBB_BURSTY_READER_H_
#define SNARK_TBB_BURSTY_READER_H_

#include <snark/tbb/queue.h>
#include <boost/thread.hpp>
#include <tbb/pipeline.h>

namespace snark{ namespace tbb{ 

/// the user can provide a method to validate the data,
/// the pipeline will be shut down if invalid data is received
template< typename T >
struct bursty_reader_traits
{
    static bool valid( const T& t ) { return true; }
};

/// helper class to run a tbb pipeline with bursty data
/// the pipeline has to be closed when no data is received to prevent the main thread to spin
template< typename T >
class bursty_reader
{
public:
    bursty_reader( boost::function0< T > read, unsigned int size = 0 );
    bursty_reader( boost::function0< T > read, unsigned int size, unsigned int capacity );
    ~bursty_reader();

    bool wait();
    void stop();
    void join();
    ::tbb::filter_t< void, T >& filter() { return m_read_filter; }

private:
    T read( ::tbb::flow_control& flow );
    void push();
    void push_thread();

    queue< T > m_queue;
    unsigned int m_size;
    bool m_running;
    boost::scoped_ptr< boost::thread > m_thread;
    boost::function0< T > m_read;
    ::tbb::filter_t< void, T > m_read_filter;
};


/// constructor
/// @param read the user-provided read functor that outputs the data
/// @param size maximum input queue size before discarding data, 0 means infinite
template< typename T >
bursty_reader< T >::bursty_reader( boost::function0< T > read, unsigned int size ):
    m_size( size ),
    m_running( true ),
    m_read( read ),
    m_read_filter( ::tbb::filter::serial_in_order, boost::bind( &bursty_reader< T >::read, this, _1 ) )
{
    m_thread.reset( new boost::thread( boost::bind( &bursty_reader< T >::push_thread, this ) ) );
}

/// constructor
/// @param read the user-provided read functor that outputs the data
/// @param size maximum input queue size before discarding data, 0 means infinite
/// @param capacity maximum input queue size before the reader thread blocks
template< typename T >
bursty_reader< T >::bursty_reader( boost::function0< T > read, unsigned int size, unsigned int capacity ):
    m_queue( capacity ),
    m_size( size ),
    m_running( true ),
    m_read( read ),
    m_read_filter( ::tbb::filter::serial_in_order, boost::bind( &bursty_reader< T >::read, this, _1 ) )
{
    m_thread.reset( new boost::thread( boost::bind( &bursty_reader< T >::push_thread, this ) ) );
}

/// desctructor
template< typename T >
bursty_reader< T >::~bursty_reader()
{
    join();
}


/// wait until the queue is ready
/// @return true if the reader is running or the queue is not empty, ie. if the pipeline should be started
template< typename T >
bool bursty_reader< T >::wait()
{
    if( !m_running && m_queue.empty() ) { return false; }
    m_queue.wait();
    return !m_queue.empty();
}

/// stop pushing items in the queue, will not wait until the thread actually exits, call join() if
/// this is what you want
template< typename T >
void bursty_reader< T >::stop()
{
    m_running = false;
    m_queue.shutdown();
}

/// join the push thread 
template< typename T >
void bursty_reader< T >::join()
{
    stop();
    if( !m_thread ) { return; }
    m_thread->join();
    m_thread.reset();
}

/// try to pop a frame from the queue
/// @param flow pipeline flow control used to stop the pipeline when the queue is empty
template< typename T >
T bursty_reader< T >::read( ::tbb::flow_control& flow )
{
    if( m_queue.empty() )
    {
        flow.stop();
        return T();
    }
    if( m_size > 0 )
    {
        T t;
        unsigned int n = 0;
        while( m_queue.size() > m_size )
        {
            m_queue.pop( t );
            n++;
        }
//         if( n > 0 ) TODO how to warn the user that data is discarded ?
//         {
//             std::cerr << "warning: discarded " << n << " frame(s)" << std::endl;
//         }
    }
    T t;
    m_queue.pop( t );
    if( !bursty_reader_traits< T >::valid( t ) )
    {
        flow.stop();
        return T();
    }
    return t;
}


/// read an element from the source and push it to the queue
template< typename T >
void bursty_reader< T >::push()
{
    T t = m_read();
    if( !bursty_reader_traits< T >::valid( t ) )
    {
        m_running = false;
        m_queue.push( T() ); // HACK to signal m_queue.wait
    }
    else
    {
        m_queue.push( t );
    }
}

/// push data to the queue in a separate thread
template< typename T >
void bursty_reader< T >::push_thread()
{
    while( m_running )
    {
        push();
    }
}


} } 

#endif // SNARK_TBB_BURSTY_READER_H_
