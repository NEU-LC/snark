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

unsigned int cv_type_from_jai( comma::uint32 pixel_type )
{
    switch( pixel_type )
    {
        case J_GVSP_PIX_MONO: return CV_8UC1;
        case J_GVSP_PIX_RGB: return CV_8UC3;
        case J_GVSP_PIX_CUSTOM: COMMA_THROW( comma::exception, "custom pixel format (J_GVSP_PIX_CUSTOM, 0x80000000) not supported" );
        case J_GVSP_PIX_COLOR_MASK: COMMA_THROW( comma::exception, "expected pixel format, got color mask (J_GVSP_PIX_COLOR_MASK, 0xFF000000)" );
        default: COMMA_THROW( comma::exception, "expected pixel format, got: " << pixel_type );
    }
}
    
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
    
    class event_buffer
    {
        public:
            event_buffer( STREAM_HANDLE h, EVT_HANDLE e ) : stream_( h )
            {
                uint32_t size = sizeof( void * );
                J_Event_GetData( e, &buffer_,  &size );
            }
            
            template < typename T > void get( _J_BUFFER_INFO_CMD_TYPE what, T& t ) const
            {
                uint32_t size = sizeof( T );
                J_DataStream_GetBufferInfo( stream_, buffer_, what, &t, &size );
            }
            
            BUF_HANDLE buffer() const { return buffer_; }
            
        private:
            mutable STREAM_HANDLE stream_;
            mutable BUF_HANDLE buffer_;
    };
    
    std::pair< boost::posix_time::ptime, cv::Mat > read()
    {
        std::pair< boost::posix_time::ptime, cv::Mat > pair;
        while( true )
        {
            static const unsigned int timeout = 1000;
            J_COND_WAIT_RESULT wait_result;
            validate( J_Event_WaitForCondition( event, timeout, &wait_result ) ); // todo: will exception work with tbb?
            if( closed() ) { return std::pair< boost::posix_time::ptime, cv::Mat >(); } // todo: make thread-safe?
            switch( wait_result )
            {
                case J_COND_WAIT_SIGNAL:
                    break;
                case J_COND_WAIT_EXIT:
                    return std::pair< boost::posix_time::ptime, cv::Mat >();
                case J_COND_WAIT_TIMEOUT:
                    continue;
                case J_COND_WAIT_ERROR:
                    COMMA_THROW( comma::exception, "error on wait" ); // todo: will exception work with tbb?
                default:
                    COMMA_THROW( comma::exception, "wait returned unexpected status: " << wait_result ); // todo: will exception work with tbb?
            }
        }
        pair.first = boost::posix_time::microsec_clock::universal_time(); // todo? use timestamp from the camera?
        event_buffer e( handle, event_handle );
        J_tIMAGE_INFO image_info;
        e.get( BUFFER_INFO_BASE, image_info.pImageBuffer );
        e.get( BUFFER_INFO_SIZE, image_info.iImageSize );
        e.get( BUFFER_INFO_PIXELTYPE, image_info.iPixelType );
        e.get( BUFFER_INFO_WIDTH, image_info.iSizeX );
        e.get( BUFFER_INFO_HEIGHT, image_info.iSizeY );
        e.get( BUFFER_INFO_TIMESTAMP, image_info.iTimeStamp );
        //e.get( BUFFER_INFO_NUM_PACKETS_MISSING, image_info.iMissingPackets ); todo? do we need that?
        e.get( BUFFER_INFO_XOFFSET, image_info.iOffsetX );
        e.get( BUFFER_INFO_YOFFSET, image_info.iOffsetY );
        // todo? example in stream_thread.cc also sets pointer to valid and queued buffers; do we even need it?
        J_tIMAGE_INFO tmp;
        validate( "image allocation", J_Image_Malloc( &image_info, &tmp ) );
        J_STATUS_TYPE r = J_Image_FromRawToImage( &image_info, &tmp );
        if( r != J_ST_SUCCESS )
        {
            J_Image_Free( &tmp ); // it sucks
            COMMA_THROW( comma::exception, "conversion from raw to image failed: " << error_to_string( r ) ); // todo: will exception work with tbb?
        }
        pair.second = cv::Mat( image_info.iSizeY, image_info.iSizeX, cv_type_from_jai( tmp.iPixelType ) );
        for( unsigned int i = 0; i < image_info.iSizeX; ++i ) // todo: replace with memcpy, once jai memory layout is clear
        {
            for( unsigned int j = 0; j < image_info.iSizeY; ++j )
            {
                POINT p;
                p.x = i;
                p.y = j;
                boost::array< char, 6 > pixel; // quick and dirty: up to 8 bytes
                J_Image_GetPixel( &tmp, &p, &pixel[0] );
                
                
                // todo: copy the pixel for pete's sake
                
                
            }
        }
        J_Image_Free( &tmp ); // it sucks
        J_DataStream_QueueBuffer( handle, e.buffer() );
        J_Event_SignalCondition( event );
    }
    
    void close()
    {
        if( !handle ) { return; }
        J_DataStream_StopAcquisition( handle, ACQ_STOP_FLAG_KILL );
        J_Event_CloseCondition( event );
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
