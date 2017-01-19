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


#ifndef SNARK_SENSORS_DC1394_H_
#define SNARK_SENSORS_DC1394_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <dc1394/dc1394.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <comma/io/select.h>
#include <comma/visiting/traits.h>
#include "types.h"

namespace snark { namespace camera {
    
/// image acquisition from dc1394 camera
class dc1394
{
public:

    struct config
    {
        config();
        
        dc1394video_mode_t  video_mode;
        dc1394operation_mode_t operation_mode;
        dc1394speed_t iso_speed;
        // TODO framerate is not used in format7, as the way to set the framerate is different.
        // see http://damien.douxchamps.net/ieee1394/libdc1394/v2.x/faq/#How_do_I_set_the_frame_rate
        dc1394framerate_t frame_rate;
        unsigned int relative_shutter;
        unsigned int relative_gain;
        float shutter; // 0 means do not change
        float gain;
        unsigned int exposure;
        uint64_t guid;

        unsigned int format7_left;
        unsigned int format7_top;
        unsigned int format7_width;
        unsigned int format7_height;
        unsigned int format7_packet_size;
        dc1394color_coding_t format7_color_coding;

        bool deinterlace;
    };

    // see 4.11.3 Strobe Signal Output Function ( IIDC 1394-based Digital Camera Specification ver. 1.31 )
    struct strobe_inquiry 
    {
        strobe_inquiry() : presence( 0 ), unused1( 0 ), read_out( 0 ), on_off( 0 ), polarity( 0 ), unused2( 0 ), min_value( 0 ), max_value( 0 ) {}
        uint32_t presence: 1, unused1: 3, read_out: 1, on_off: 1, polarity: 1, unused2: 1, min_value: 12, max_value: 12;
    };
    struct strobe_control 
    {
        strobe_control() : presence( 0 ), unused( 0 ), on_off( 0 ), polarity( 0 ), delay( 0 ), duration( 0 ) {}
        uint32_t presence: 1, unused: 5, on_off: 1, polarity: 1, delay: 12, duration: 12;
    };
    static const std::size_t number_of_pins = 4;
    static uint64_t strobe_inquiry_offsets( unsigned int pin ) { static boost::array< unsigned int, number_of_pins > offsets = { 0x100, 0x104, 0x108, 0x10c }; return offsets.at( pin ); }
    static uint64_t strobe_control_offsets( unsigned int pin ) { static boost::array< unsigned int, number_of_pins > offsets = { 0x200, 0x204, 0x208, 0x20c }; return offsets.at( pin ); }
    typedef enum { STROBE_POLARITY_LOW = 0, STROBE_POLARITY_HIGH = 1 } strobe_polarity_t;
    typedef enum { STROBE_IGNORE, STROBE_ON, STROBE_OFF, STROBE_AUTO } strobe_command_t;
    struct strobe
    {
        strobe() : pin( 0 ), polarity( STROBE_POLARITY_HIGH ), delay( 0 ), duration( 0 ), command( STROBE_IGNORE ) {}
        unsigned int pin;
        strobe_polarity_t polarity;
        unsigned int delay;
        unsigned int duration;
        strobe_command_t command;
    };
    static strobe_polarity_t polarity_from_string( std::string s )
    {
        if( s == "low" ) { return STROBE_POLARITY_LOW; }
        else if( s == "high" ) { return STROBE_POLARITY_HIGH; }
        else { COMMA_THROW( comma::exception, "expected strobe polarity to be either \"high\" or \"low\", got " << s ); } 
    }
    static strobe_command_t command_from_string( std::string s )
    {
        if( s == "on" ) { return STROBE_ON; }
        else if( s == "off" ) { return STROBE_OFF; }
        else if ( s == "auto" ) { return STROBE_AUTO; }
        else { COMMA_THROW( comma::exception, "expected strobe command to be \"on\", \"off\" or \"auto\", got " << s ); }
    }

    dc1394( const config& c = config(), const strobe& s = strobe() );
    ~dc1394();

    const cv::Mat& read();
    boost::posix_time::ptime time() const { return m_time; }
    bool poll();
    static void list_cameras();
    void list_attributes();

    void get_control_register( uint32_t* p, const uint64_t address );
    void set_control_register( const uint32_t value, const uint64_t address );
    void verify_strobe_parameters( const strobe& strobe );
    void trigger_strobe( const bool enable, const strobe& strobe );

private:
    void init_camera();
    void setup_camera();
    void setup_camera_format7();

    void set_absolute_shutter_gain( float shutter, float gain );
    void set_relative_shutter_gain( unsigned int shutter, unsigned int gain );
    void set_exposure( unsigned int exposure );

    void manage_strobe_at_start();
    void manage_strobe_at_stop();

    config m_config;
    
    dc1394camera_t* m_camera;
    dc1394video_frame_t* m_frame;
    
    dc1394operation_mode_t m_operation_mode;
    dc1394speed_t m_iso_speed;
    dc1394video_mode_t m_video_mode;
    dc1394framerate_t m_framerate;
    dc1394color_coding_t m_color_coding;
    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_top;
    unsigned int m_left;
    unsigned int m_packet_size;
    
    cv::Mat m_image;
    const boost::posix_time::ptime m_epoch;
    boost::posix_time::ptime m_time;
    int m_fd;
    comma::io::select m_select;
    boost::posix_time::time_duration m_frame_duration;
    
    strobe m_strobe;
};

} } // namespace snark { namespace camera {

namespace comma { namespace visiting {

template <> struct traits< snark::camera::dc1394::config >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::camera::dc1394::config& c, Visitor& v )
    {
        std::string video_mode;
        std::string operation_mode;
        std::string iso_speed;
        std::string frame_rate;
        std::string color_coding="DC1394_COLOR_CODING_MONO8";
        v.apply( "video-mode", video_mode );
        v.apply( "operation-mode", operation_mode );
        v.apply( "iso-speed", iso_speed );
        v.apply( "frame-rate", frame_rate );
        v.apply( "color-coding", color_coding );
        v.apply( "left", c.format7_left );
        v.apply( "top", c.format7_top );
        v.apply( "width", c.format7_width );
        v.apply( "height", c.format7_height );
        v.apply( "packet-size", c.format7_packet_size );

        c.video_mode = snark::camera::video_mode_from_string( video_mode );
        c.operation_mode = snark::camera::operation_mode_from_string( operation_mode );
        c.iso_speed = snark::camera::iso_speed_from_string( iso_speed );
        c.frame_rate = snark::camera::frame_rate_from_string( frame_rate );
        c.format7_color_coding = snark::camera::color_coding_from_string( color_coding );

        v.apply( "relative-shutter", c.relative_shutter );
        v.apply( "relative-gain", c.relative_gain );
        v.apply( "shutter", c.shutter );
        v.apply( "gain", c.gain );
        v.apply( "exposure", c.exposure );
        v.apply( "guid", c.guid );
        v.apply( "deinterlace", c.deinterlace );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::camera::dc1394::config& c, Visitor& v )
    {
        std::string video_mode = snark::camera::video_mode_to_string( c.video_mode );
        std::string operation_mode = snark::camera::operation_mode_to_string( c.operation_mode );
        std::string iso_speed = snark::camera::iso_speed_to_string( c.iso_speed );
        std::string frame_rate = snark::camera::frame_rate_to_string( c.frame_rate );
        std::string color_coding = snark::camera::color_coding_to_string( c.format7_color_coding );

        v.apply( "video-mode", video_mode );
        v.apply( "operation-mode", operation_mode );
        v.apply( "iso-speed", iso_speed );
        v.apply( "frame-rate", frame_rate );
        v.apply( "color-coding", color_coding );
        v.apply( "left", c.format7_left );
        v.apply( "top", c.format7_top );
        v.apply( "width", c.format7_width );
        v.apply( "height", c.format7_height );
        v.apply( "packet-size", c.format7_packet_size );

        v.apply( "relative-shutter", c.relative_shutter );
        v.apply( "relative-gain", c.relative_gain );
        v.apply( "shutter", c.shutter );
        v.apply( "gain", c.gain );
        v.apply( "exposure", c.exposure );
        v.apply( "guid", c.guid );
        v.apply( "deinterlace", c.deinterlace );
    }
};

template <> struct traits< snark::camera::dc1394::strobe >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::camera::dc1394::strobe& c, Visitor& v )
    {
        std::string command;
        std::string polarity;
        v.apply( "command", command );
        v.apply( "polarity", polarity );
        if( !command.empty() ) { c.command = snark::camera::dc1394::command_from_string( command ); }
        if( !polarity.empty() ) { c.polarity = snark::camera::dc1394::polarity_from_string( polarity ); }
        v.apply( "pin", c.pin );
        v.apply( "delay", c.delay );
        v.apply( "duration", c.duration );
    }
    
};

} } // namespace comma { namespace visiting {

#endif // SNARK_SENSORS_DC1394_H_
