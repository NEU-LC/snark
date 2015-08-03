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

/// @author vsevolod vlaskine

#include <boost/array.hpp>
#include <Jai_Factory.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "error.h"
#include "camera.h"
#include "stream.h"

namespace snark { namespace jai {

template < typename T > struct value_traits {};
template <> struct value_traits < comma::uint32 > { typedef int64_t internal_type; };
template <> struct value_traits < comma::int32 > { typedef int64_t internal_type; };
template <> struct value_traits < comma::int64 > { typedef int64_t internal_type; };

struct jai::stream::impl
{
    struct buffer
    {
        BUF_HANDLE handle;
        std::vector< uint8_t > data;
        
        buffer() : handle( NULL ) {}
        
        void allocate( CAM_HANDLE device, std::size_t size )
        {
            data.resize( size );
            validate( "announcing buffer", J_DataStream_AnnounceBuffer( device, &data[0], size, NULL, &handle ) );
            validate( "queueing buffer", J_DataStream_QueueBuffer( device, handle ) );
        }
    };
    
    STREAM_HANDLE handle;
    CAM_HANDLE device;
    HANDLE event;
    EVT_HANDLE event_handle;
    J_COND_WAIT_RESULT condition;
    std::vector< buffer > buffers;
    
    impl( CAM_HANDLE device, std::size_t size, unsigned int buffers_size = 1 )
        : handle( NULL )
        , device( device )
        , event( NULL )
        , buffers( buffers_size )
    {
        validate( "creating data stream", J_Camera_CreateDataStreamMc( device, 0, &handle, 0  ));
        if( !handle ) { COMMA_THROW( comma::exception, "creating data stream failed" ); }
        validate( "creating condition", J_Event_CreateCondition( &event ) );
        if( !event ) { COMMA_THROW( comma::exception, "creating condition failed" ); }
        J_DataStream_RegisterEvent( handle, EVENT_NEW_BUFFER, event, (void **)&event_handle );
        for( unsigned int i = 0; i < buffers.size(); ++i ) { buffers[i].allocate( device, size ); }
        validate( "starting acquisition", J_DataStream_StartAcquisition( handle, ACQ_START_NEXT_IMAGE, 0 ) );
    }
    
    ~impl() { close(); }
    
    std::pair< boost::posix_time::ptime, cv::Mat > read()
    {
        // todo
        return std::pair< boost::posix_time::ptime, cv::Mat >();
    }
    
    void close()
    {
        if( !handle ) { return; }
        J_DataStream_StopAcquisition( handle, ACQ_STOP_FLAG_KILL );
        J_Event_ExitCondition( event );
        J_DataStream_FlushQueue( handle, ACQ_QUEUE_INPUT_TO_OUTPUT );
        J_DataStream_FlushQueue( handle, ACQ_QUEUE_OUTPUT_DISCARD );
        J_DataStream_Close( handle );
        handle = NULL;
    }

    bool closed() const { return handle == NULL; }    
};

jai::stream::stream::stream( const jai::camera& c, unsigned int number_of_buffers ) { new impl( c.handle(), c.width() * c.height() * J_MAX_BPP, number_of_buffers ); }

jai::stream::~stream() { if( pimpl_ ) { delete pimpl_; } }

std::pair< boost::posix_time::ptime, cv::Mat > jai::stream::read() { return pimpl_->read(); }

void jai::stream::close() { pimpl_->close(); }

bool jai::stream::closed() const { return pimpl_->closed(); }

} } // namespace snark { namespace jai {
