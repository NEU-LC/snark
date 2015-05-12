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


#ifndef SNARK_TBB_BURSTY_READER_H_
#define SNARK_TBB_BURSTY_READER_H_

#include <snark/tbb/queue.h>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
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
