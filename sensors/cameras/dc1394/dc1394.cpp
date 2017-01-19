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

#include <limits>

#include <boost/optional.hpp>
#include <boost/static_assert.hpp>
#include "../../../timing/time.h"
#include <comma/base/exception.h>
#include <comma/packed/bits.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dc1394.h"

namespace snark { namespace camera {


/// default constructor    
dc1394::config::config():
    video_mode( DC1394_VIDEO_MODE_640x480_YUV422 ),
    operation_mode( DC1394_OPERATION_MODE_LEGACY ),
    iso_speed( DC1394_ISO_SPEED_400 ),
    frame_rate( DC1394_FRAMERATE_MAX ),
    relative_shutter( 0 ),
    relative_gain( 0 ),
    shutter( 0 ),
    gain( 0 ),
    exposure( 0 ),
    guid( 0 ),
    format7_left( 0 ),
    format7_top( 0 ),
    format7_width( 0 ),
    format7_height( 0 ),
    format7_packet_size( 0 ),
    format7_color_coding( DC1394_COLOR_CODING_MONO8 ),
    deinterlace( false )
{
 
}

void dc1394::manage_strobe_at_start()
{
    switch( m_strobe.command )
    {
        case STROBE_IGNORE: break;
        case STROBE_AUTO: trigger_strobe( true, m_strobe ); break;
        case STROBE_ON: trigger_strobe( true, m_strobe ); break;
        case STROBE_OFF: trigger_strobe( false, m_strobe ); break;
        default: COMMA_THROW( comma::exception, "encountered unexpected strobe command " << m_strobe.command );
    }
}

void dc1394::manage_strobe_at_stop()
{
    if( m_strobe.command == STROBE_AUTO  ) { trigger_strobe( false, m_strobe ); }
}

/// constructor
/// @param config camera config
dc1394::dc1394( const snark::camera::dc1394::config& c, const snark::camera::dc1394::strobe& s ):
      m_config( c )
    , m_epoch( timing::epoch )
    , m_strobe( s )
{
    //memset( &m_output_frame, 0, sizeof( m_output_frame ) );
    //m_output_frame.color_coding = DC1394_COLOR_CODING_RGB8;
 
    init_camera();

    /* First check if relative shutter has been set, if not check absolute
     * shutter. If neither of those are set, use the exposure */
    if( m_config.relative_shutter > 1e-8 )
    {
        set_relative_shutter_gain( m_config.relative_shutter, m_config.relative_gain );
    }
    else if( std::fabs( m_config.shutter ) > 1e-8 )
    {
        set_absolute_shutter_gain( m_config.shutter, m_config.gain );
    }
    else
    {
        set_exposure(m_config.exposure);
    }

        
    if ( !dc1394_is_video_mode_scalable( m_config.video_mode ) )
    {
        setup_camera();
    }
    else
    {
        setup_camera_format7();
    }

    if ( dc1394_capture_setup( m_camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not setup the camera" );
    }

    if ( m_config.deinterlace )
    {
        m_image = cv::Mat( m_height*2, m_width, CV_8UC1 ); //already checked dc type is MONO/RAW16, will be converted to 2 MONO/RAW8 images
    }
    else
    {
        m_image = cv::Mat( m_height, m_width, dc1394color_coding_to_cv_type(m_color_coding) );
    }
    
    if ( dc1394_video_set_transmission( m_camera, DC1394_ON ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not start the camera iso transmission" );
    }

    dc1394switch_t status = DC1394_OFF;
    unsigned int i = 0;
    while( status != DC1394_OFF )
    {
        ::usleep( 10000 );
        if ( dc1394_video_get_transmission( m_camera, &status ) != DC1394_SUCCESS )
        {
            COMMA_THROW( comma::exception, "could not get the camera transmission status" );
        }
        if( i == 10 )
        {
            COMMA_THROW( comma::exception, "camera could not start" );
        }
        i++;
    }
    
    manage_strobe_at_start();
}

/// destructor
dc1394::~dc1394()
{
    manage_strobe_at_stop(); 
    dc1394_capture_stop( m_camera );
    dc1394_video_set_transmission( m_camera, DC1394_OFF );
    dc1394_camera_free( m_camera );
    //std::cerr << "the camera has been closed" << std::endl;
}

/// acquire a frame from the camera
const cv::Mat& dc1394::read()
{
    
    if ( dc1394_capture_dequeue( m_camera, DC1394_CAPTURE_POLICY_WAIT, &m_frame ) < 0 )
    {
        COMMA_THROW( comma::exception, " no frame in buffer " );
    }
    if ( dc1394_capture_is_frame_corrupt ( m_camera, m_frame ) )
    {
        COMMA_THROW( comma::exception, "frame corrupted" );
    }

    if ( m_config.deinterlace )
    {
        if ( dc1394_deinterlace_stereo(m_frame->image, m_image.data, m_width, m_height*2) != DC1394_SUCCESS )
        {
            COMMA_THROW( comma::exception, "could not deinterlace image" );
        }
    }
    else
    {
        //convert dc big endian to little endian for 16bit modes
        // NOTE: not sure if it's mode or camera dependent
        //   Pika2 (Point Grey Flea2) = MONO16, needs swab
        //   Point Grey Ladybug2 = RAW16, needs memcpy
        //   adjust logic as future cameras dictate
        if( m_color_coding == DC1394_COLOR_CODING_MONO16 || m_color_coding == DC1394_COLOR_CODING_RGB16 || m_color_coding == DC1394_COLOR_CODING_MONO16S || m_color_coding == DC1394_COLOR_CODING_RGB16S )
        {
            swab( m_frame->image, m_image.data, m_frame->image_bytes );
        }
        else //direct copy 
        {
            memcpy( m_image.data, m_frame->image, m_frame->image_bytes ); //todo: assert sizes reported as equal
        }
    }

    //Get the time from the frame timestamp
    m_time = m_epoch + boost::posix_time::microseconds( m_frame->timestamp );
    dc1394_capture_enqueue( m_camera, m_frame ); // release the frame
    
    return m_image;
}

bool dc1394::poll()
{
    m_select.wait( m_frame_duration );
    return m_select.read().ready( m_fd ); 
}

void dc1394::list_cameras()
{
    dc1394_t* dc1394 = dc1394_new();
    dc1394camera_list_t * list;

    if ( dc1394_camera_enumerate( dc1394, &list) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "error scanning for cameras" );
    }

    for( unsigned int i = 0; i < list->num; i++ )
    {
        std::cerr << "camera found: " << list->ids[i].guid << std::endl;
    }
}

/// initialize dc1394 camera structure
void dc1394::init_camera()
{
    dc1394_t* dc1394 = dc1394_new();
    dc1394camera_list_t * list;

    if ( dc1394_camera_enumerate( dc1394, &list) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "error scanning for cameras" );
    }

    if ( list->num == 0 )
    {
        COMMA_THROW( comma::exception, "no camera found" );
    }

    unsigned int index = list->num - 1;
    if ( m_config.guid != 0 )
    {
        // look for specified camera on the bus
        bool found = false;
        unsigned int i = 0;
        while( i < list->num && !found )
        {
            if ( list->ids[i].guid == m_config.guid )
            {
                found = true;
                index = i;
            }
            i++;
        }
        if ( !found )
        {
            COMMA_THROW( comma::exception, "camera with guid " << m_config.guid << " not found " );
        }
    }

    m_camera = dc1394_camera_new( dc1394, list->ids[ index ].guid );
    if ( m_camera == NULL )
    {
        COMMA_THROW( comma::exception, "failed to initialize camera with guid " << list->ids[ m_config.guid ].guid );
    }

    dc1394_camera_free_list( list );
}

/// setup the camera capture for non-format7 modes
void dc1394::setup_camera()
{
    //set operation mode and check
    if( dc1394_video_set_operation_mode( m_camera, m_config.operation_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set operation mode" );
    }
    if( dc1394_video_get_operation_mode( m_camera, &m_operation_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get operation mode" );
    }
    if( m_operation_mode != m_config.operation_mode )
    {
        COMMA_THROW( comma::exception, "operation mode not set correctly" );
    }

    //set iso speed and check
    if( dc1394_video_set_iso_speed( m_camera, m_config.iso_speed ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set iso speed" );
    }
    if( dc1394_video_get_iso_speed( m_camera, &m_iso_speed ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get iso speed" );
    }
    if( m_iso_speed != m_config.iso_speed )
    {
        COMMA_THROW( comma::exception, "iso speed not set correctly" );
    }

    //set and check video mode
    if ( dc1394_video_set_mode( m_camera, m_config.video_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set video mode" );
    }
    if ( dc1394_video_get_mode( m_camera, &m_video_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get video mode" );
    }
    if( m_video_mode != m_config.video_mode )
    {
        COMMA_THROW( comma::exception, "iso speed not set correctly" );
    }

    //calculate framerate
    dc1394framerate_t framerate;
    dc1394framerates_t framerates;
    if ( dc1394_video_get_supported_framerates( m_camera, m_config.video_mode, &framerates ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "error getting framerate" );
    }
    if( framerates.num == 0 )
    {
        COMMA_THROW( comma::exception, "could not get framerate list" );
    }
    if( m_config.frame_rate == DC1394_FRAMERATE_MAX )
    {
        framerate = framerates.framerates[ framerates.num-1 ];
    }
    else
    {
        unsigned int i = 0;
        while( i < framerates.num && framerates.framerates[ i ] != m_config.frame_rate )
        {
            i++;
        }
        if( i == framerates.num )
        {
            COMMA_THROW( comma::exception, "invalid framerate" );
        }
        else
        {
            framerate = framerates.framerates[ i ];
        }
    }


    //set and check framerate
    if ( dc1394_video_set_framerate( m_camera, framerate ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set framerate" );
    }
    if ( dc1394_video_get_framerate( m_camera, &m_framerate ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get framerate" );
    }
    if( m_framerate != framerate )
    {
        COMMA_THROW( comma::exception, "framerate not set correctly" );
    }

    //in this mode, colour coding must come from video mode
    if ( dc1394_get_color_coding_from_video_mode( m_camera, m_config.video_mode, &m_color_coding ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get color coding from video mode" );
    }

    //check compatibility of deinterlace and color_mode
    if ( m_config.deinterlace && (m_color_coding != DC1394_COLOR_CODING_MONO16 && m_color_coding != DC1394_COLOR_CODING_RAW16) )
    {
        COMMA_THROW( comma::exception, "deinterlace set to true with color_coding=" << color_coding_to_string(m_color_coding) << " - must use DC1394_COLOR_CODING_MONO16 or DC1394_COLOR_CODING_RAW16" );
    }

    //todo THROW or warn if any format7 settings were given, as these are ignored in this mode
}

/// setup camera capture for format7 modes
void dc1394::setup_camera_format7()
{
    //set operation mode and check
    if( dc1394_video_set_operation_mode( m_camera, m_config.operation_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set operation mode" );
    }
    if( dc1394_video_get_operation_mode( m_camera, &m_operation_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get operation mode" );
    }
    if( m_operation_mode != m_config.operation_mode )
    {
        COMMA_THROW( comma::exception, "operation mode not set correctly" );
    }

    //set iso speed and check
    if( dc1394_video_set_iso_speed( m_camera, m_config.iso_speed ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set iso speed" );
    }
    if( dc1394_video_get_iso_speed( m_camera, &m_iso_speed ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get iso speed" );
    }
    if( m_iso_speed != m_config.iso_speed )
    {
        COMMA_THROW( comma::exception, "iso speed not set correctly" );
    }

    //set and check video mode
    if ( dc1394_video_set_mode( m_camera, m_config.video_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set video mode" );
    }
    if ( dc1394_video_get_mode( m_camera, &m_video_mode ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get video mode" );
    }
    if( m_video_mode != m_config.video_mode )
    {
        COMMA_THROW( comma::exception, "iso speed not set correctly" );
    }

    //set and check color coding
    if( dc1394_format7_set_color_coding( m_camera, m_config.video_mode, m_config.format7_color_coding ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not set format7 color coding" );
    } 
    if( dc1394_format7_get_color_coding( m_camera, m_config.video_mode, &m_color_coding ) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get format7 color coding" );
    }
    if( m_color_coding != m_config.format7_color_coding )
    {
        COMMA_THROW( comma::exception, "color coding not set correctly" );
    }

    //check compatibility of deinterlace and color_mode
    if ( m_config.deinterlace && (m_color_coding != DC1394_COLOR_CODING_MONO16 && m_color_coding != DC1394_COLOR_CODING_RAW16) )
    {
        COMMA_THROW( comma::exception, "deinterlace set to true with color_coding=" << color_coding_to_string(m_color_coding) << " - must use DC1394_COLOR_CODING_MONO16 or DC1394_COLOR_CODING_RAW16" );
    }


    //calculate packet-size
    unsigned int packet_size=0, min_size=0, max_size=0;
    if( dc1394_format7_get_packet_parameters( m_camera, m_config.video_mode, &min_size, &max_size) != DC1394_SUCCESS )
    {
        COMMA_THROW( comma::exception, "could not get packet size" );
    }
    if( m_config.format7_packet_size == 0 )
    {
        //std::cerr << "packet size set to max available: " << max_size << std::endl;
        packet_size=max_size;
    }
    else if( m_config.format7_packet_size > max_size )
    {
        //std::cerr << "warning: packet size (" << m_config.format7_packet_size << ") set to max available (" << max_size << ")" << std::endl;
        packet_size=max_size;
    }
    else if( m_config.format7_packet_size < min_size )
    {
        //std::cerr << "warning: packet size (" << m_config.format7_packet_size << ") set to min available (" << min_size << ")" << std::endl;
        packet_size=min_size;
    }
    else
    {
        packet_size=m_config.format7_packet_size;
    }
    //std::cerr << "pack size min, max, selected: " << min_size << ", " << max_size << ", " << packet_size << std::endl;


    //calculate based on ROI from config - if width && height ==0, then not using ROI
    unsigned int width=0,height=0,top=0,left=0;
    if( m_config.format7_width == 0 && m_config.format7_height == 0  )
    {
        if( m_config.format7_left!=0 || m_config.format7_top!=0 )
        {
            COMMA_THROW( comma::exception, "non-zero top or left, but width and height are both 0 (default), meaning no ROI." );
        }
        if( dc1394_format7_get_max_image_size( m_camera, m_config.video_mode, &width, &height) != DC1394_SUCCESS )
        {
            COMMA_THROW( comma::exception, "could not get max image size" );
        }
        if( width==0 || height==0 )
        {
            COMMA_THROW( comma::exception, "max image width or height returned 0" );
        }
    }
    else if( m_config.format7_width == 0 || m_config.format7_height == 0  )
    {
        COMMA_THROW( comma::exception, "ROI width and height must both be non zero, or both equal to 0 (default) for no ROI" );
    }
    else
    {
        top=m_config.format7_top;
        left=m_config.format7_left;
        width=m_config.format7_width;
        height=m_config.format7_height;
    }


    //set and check ROI
    if (dc1394_format7_set_roi( m_camera, m_config.video_mode, m_config.format7_color_coding, packet_size, left, top, width, height ) != DC1394_SUCCESS)
    {
        COMMA_THROW( comma::exception, "could not set roi" );
    }
    if (dc1394_format7_get_roi( m_camera, m_config.video_mode, &m_color_coding, &m_packet_size, &m_left, &m_top, &m_width, &m_height ) != DC1394_SUCCESS)
    {
        COMMA_THROW( comma::exception, "could not get roi" );
    }
    if (m_left != left || m_top != top || m_width != width || m_height != height)
    {
        //std::cerr << "left: " << left << " top: " << top << " width: " << width << " height: " << height << std::endl;
        //std::cerr << "left: " << m_left << " top: " << m_top << " width: " << m_width << " height: " << m_height << std::endl;
        COMMA_THROW( comma::exception, "ROI not set correctly" );
    }
    if (m_packet_size != packet_size)
    {
        COMMA_THROW( comma::exception, "packet size not set correctly" );
    }
    if (m_color_coding != m_config.format7_color_coding)
    {
        COMMA_THROW( comma::exception, "color coding not set correctly" );
    }


    //set and check image size
    if (dc1394_format7_set_image_size( m_camera, m_config.video_mode, width, height ) != DC1394_SUCCESS)
    {
        COMMA_THROW( comma::exception, "could not set image size" );
    }
    if (dc1394_format7_get_image_size( m_camera, m_config.video_mode, &m_width, &m_height ) != DC1394_SUCCESS)
    {
        COMMA_THROW( comma::exception, "could not get image size" );
    }
    if (width != m_width || height != m_height)
    {
        COMMA_THROW( comma::exception, "image width and height not set correctly" );
    }
}

/// set absolute shutter and gain
void dc1394::set_absolute_shutter_gain( float shutter, float gain )
{
    dc1394error_t err;

    /* 1. Turn exposure off */
    err = dc1394_feature_set_power(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    DC1394_ERR(err, "Failed to set exposure power");

    /* 2. Set gain to manual mode (and ideally minimum gain) */
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR(err, "Failed to set gain mode");
    err = dc1394_feature_set_absolute_control(m_camera, DC1394_FEATURE_GAIN, DC1394_ON);
    DC1394_ERR(err, "Failed to set gain absolute control mode");
    err = dc1394_feature_set_absolute_value(m_camera, DC1394_FEATURE_GAIN, gain);
    DC1394_ERR(err, "Failed to set absolute gain value");

    /* 3. Set the shutter to manual mode, and then the requested value. */
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR(err, "Failed to set shutter mode");
    err = dc1394_feature_set_absolute_control(m_camera, DC1394_FEATURE_SHUTTER, DC1394_ON);
    DC1394_ERR(err, "Failed to set shutter absolute control mode");
    err = dc1394_feature_set_absolute_value(m_camera, DC1394_FEATURE_SHUTTER, shutter);
    DC1394_ERR(err, "Failed to set absolute shutter value");
}

/// set relative shutter and gain
void dc1394::set_relative_shutter_gain( unsigned int shutter, unsigned int gain )
{    
    dc1394error_t err;

    /* 1. Turn exposure off */
    err = dc1394_feature_set_power(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    DC1394_ERR(err, "Failed to set exposure power");

    /* 2. Set gain to manual mode (and ideally minimum gain) */
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR(err, "Failed to set gain mode");
    err = dc1394_feature_set_absolute_control(m_camera, DC1394_FEATURE_GAIN, DC1394_OFF);
    DC1394_ERR(err, "Failed to set gain absolute control mode");
    err = dc1394_feature_set_value(m_camera, DC1394_FEATURE_GAIN, gain);
    DC1394_ERR(err, "Failed to set gain value");

    /* 3. Set the shutter to manual mode, and then the requested value. */
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR(err, "Failed to set shutter mode");
    err = dc1394_feature_set_absolute_control(m_camera, DC1394_FEATURE_SHUTTER, DC1394_OFF);
    DC1394_ERR(err, "Failed to set shutter absolute control mode");
    err = dc1394_feature_set_value(m_camera, DC1394_FEATURE_SHUTTER, shutter);
    DC1394_ERR(err, "Failed to set shutter value");
}

/// set exposure controls 
void dc1394::set_exposure( unsigned int exposure )
{
    dc1394error_t err;

    /* Turn shutter/gain to auto */
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
    DC1394_ERR(err, "Failed to set gain mode");
    err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
    DC1394_ERR(err, "Failed to set shutter mode");

    /* Turn auto exposure on */
    err = dc1394_feature_set_power(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR(err, "Failed to set exposure power");
   
    if(exposure > 0) 
    {
        /* If a particular exposure has been selected, choose it 
         * only capable of relative mode for now */
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
        DC1394_ERR(err, "Failed to set exposure mode");
        err = dc1394_feature_set_absolute_control(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
        DC1394_ERR(err, "Failed to set exposure absolute control mode");
        err = dc1394_feature_set_value(m_camera, DC1394_FEATURE_EXPOSURE, exposure);
        DC1394_ERR(err, "Failed to set exposure value");
    }
    else
    {
        /* Otherwise, auto exposure */
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);
        DC1394_ERR(err, "Failed to set exposure mode");
    }  
}


/// list all the dc1394 features of the camera
void dc1394::list_attributes()
{
    dc1394featureset_t featureset;
    dc1394_feature_get_all(m_camera, &featureset);
    dc1394_feature_print_all(&featureset, stderr);
}

void dc1394::verify_strobe_parameters( const dc1394::strobe& strobe )
{
    if( strobe.pin >= number_of_pins ) { COMMA_THROW( comma::exception, "expected GPIO pin from 0 to " << number_of_pins - 1  << ", got " << strobe.pin ); }
    uint32_t value;
    dc1394error_t err = dc1394_get_strobe_register( m_camera, strobe_inquiry_offsets( strobe.pin ), &value );
    DC1394_ERR( err, "Failed to get strobe register" );
    comma::packed::reversed_bits< strobe_inquiry > packed_inquiry = *reinterpret_cast< comma::packed::reversed_bits< strobe_inquiry >* >( &value );
    strobe_inquiry inquiry = packed_inquiry();
    if( !inquiry.presence ) { COMMA_THROW( comma::exception, "strobe inquiry feature is not present for pin " << strobe.pin ); }
    if( !inquiry.read_out ) { COMMA_THROW( comma::exception, "strobe inquiry value cannot be read for pin " << strobe.pin ); }
    if( !inquiry.on_off ) { COMMA_THROW( comma::exception, "strobe cannot be switched on and off for pin " << strobe.pin << ", check if this pin's direction control is set to 'Out'" ); }
    if( !inquiry.polarity ) { COMMA_THROW( comma::exception, "strobe's polarity cannot be changed for pin " << strobe.pin ); }
    if( strobe.polarity != STROBE_POLARITY_HIGH && strobe.polarity != STROBE_POLARITY_LOW ) { COMMA_THROW( comma::exception, "expected polarity to be " << STROBE_POLARITY_LOW << " (low) or " << STROBE_POLARITY_HIGH << " (high), got " << strobe.polarity ); }
    uint32_t min_value = inquiry.min_value;
    uint32_t max_value = inquiry.max_value;
    if( strobe.delay < min_value || strobe.delay > max_value ) { COMMA_THROW( comma::exception, "expected delay in the range [" << min_value << "," << max_value << "], got " << strobe.delay ); }
    if( strobe.duration < min_value || strobe.duration > max_value ) { COMMA_THROW( comma::exception, "expected duration in the range [" << min_value << "," << max_value << "], got " << strobe.duration ); }
}

void dc1394::trigger_strobe( const bool enable, const dc1394::strobe& strobe )
{
    verify_strobe_parameters( strobe );
    uint32_t value;
    dc1394error_t err = dc1394_get_strobe_register( m_camera, strobe_control_offsets( strobe.pin ), &value );
    DC1394_ERR( err, "Failed to get strobe register" );
    comma::packed::reversed_bits< strobe_control > packed_control = *reinterpret_cast< comma::packed::reversed_bits< strobe_control >* >( &value );
    strobe_control control = packed_control();
    if( !control.presence ) { COMMA_THROW( comma::exception, "strobe control feature is not present for pin " << strobe.pin ); }
    control.on_off = enable;
    control.polarity = strobe.polarity;
    control.delay = strobe.delay;
    control.duration = strobe.duration;
    packed_control = control;
    value = *reinterpret_cast< uint32_t* >( packed_control.data() );
    err = dc1394_set_strobe_register( m_camera, strobe_control_offsets( strobe.pin ), value );
    DC1394_ERR( err, "Failed to set strobe register" );
}

void dc1394::get_control_register( uint32_t* p, const uint64_t address )
{
    dc1394error_t error = dc1394_get_control_register( m_camera, address, p );
    DC1394_ERR( error, "Failed to get control register" );
}

void dc1394::set_control_register( const uint32_t value, const uint64_t address )
{
    dc1394error_t error = dc1394_set_control_register( m_camera, address, value );
    DC1394_ERR( error, "Failed to set control register" );
}

} } // namespace snark { namespace camera {
