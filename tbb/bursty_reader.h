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

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <tbb/pipeline.h>
#include "queue.h"

namespace snark { namespace tbb {

/// the user can provide a method to validate the data,
/// the pipeline will be shut down if invalid data is received
template< typename T >
struct bursty_reader_traits
{
    static bool valid( const T& t ) { return true; }
};

/// helper class to run a tbb pipeline with bursty data
/// the pipeline has to be closed when no data is received to prevent the main thread to spin
template < typename T >
class bursty_reader
{
public:
    bursty_reader( boost::function0< T > read, unsigned int size = 0 );
    
    bursty_reader( boost::function0< T > read, unsigned int size, unsigned int capacity );
    
    ~bursty_reader();

    bool wait();
    
    void stop();
    
    void join();
    
    ::tbb::filter_t< void, T >& filter() { return read_filter_; }

private:
    T read( ::tbb::flow_control& flow );
    void push();

    queue< T > queue_;
    unsigned int size_;
    bool running_;
    boost::function0< T > read_;
    ::tbb::filter_t< void, T > read_filter_;
    boost::thread thread_;
};


/// constructor
/// @param read the user-provided read functor that outputs the data
/// @param size maximum input queue size before discarding data, 0 means infinite
template< typename T >
bursty_reader< T >::bursty_reader( boost::function0< T > read, unsigned int size )
    : size_( size )
    , running_( true )
    , read_( read )
    , read_filter_( ::tbb::filter::serial_in_order, boost::bind( &bursty_reader< T >::read, this, _1 ) )
    , thread_( boost::bind( &bursty_reader< T >::push, this ) )
{
}

/// constructor
/// @param read the user-provided read functor that outputs the data
/// @param size maximum input queue size before discarding data, 0 means infinite
/// @param capacity maximum input queue size before the reader thread blocks
template< typename T >
bursty_reader< T >::bursty_reader( boost::function0< T > read, unsigned int size, unsigned int capacity )
    : queue_( capacity )
    , size_( size )
    , running_( true )
    , read_( read )
    , read_filter_( ::tbb::filter::serial_in_order, boost::bind( &bursty_reader< T >::read, this, _1 ) )
    , thread_( boost::bind( &bursty_reader< T >::push, this ) )
{
}

/// destructor
template< typename T >
inline bursty_reader< T >::~bursty_reader()
{
    join();
}

/// wait until the queue is ready
/// @return true if the reader is running or the queue is not empty, i.e. if the pipeline should be started
template< typename T >
inline bool bursty_reader< T >::wait()
{
    if( !running_ && queue_.empty() ) { return false; }
    queue_.wait();
    return !queue_.empty();
}

/// stop pushing items in the queue, will not wait until the thread actually exits, call join() if
/// this is what you want
template < typename T >
inline void bursty_reader< T >::stop()
{
    running_ = false;
    queue_.shutdown();
}

/// join the push thread
template < typename T >
inline void bursty_reader< T >::join()
{
    stop();
    thread_.join();
}

/// try to pop a frame from the queue
/// @param flow pipeline flow control used to stop the pipeline when the queue is empty
template < typename T >
inline T bursty_reader< T >::read( ::tbb::flow_control& flow )
{
    if( queue_.empty() )
    {
        flow.stop();
        return T();
    }
    if( size_ > 0 )
    {
        while( queue_.size() > size_ ) { T t; queue_.pop( t ); }
    }
    T t;
    queue_.pop( t );
    if( bursty_reader_traits< T >::valid( t ) ) { return t; }
    flow.stop();
    return T();
}

/// push data to the queue in a separate thread
template < typename T >
inline void bursty_reader< T >::push()
{
    while( running_ )
    {
        T t = read_();
        if( bursty_reader_traits< T >::valid( t ) )
        {
            queue_.push( t );
        }
        else
        {
            running_ = false;
            queue_.push( T() ); // HACK to signal queue_.wait
        }
    }
}

} } // namespace snark { namespace tbb {
