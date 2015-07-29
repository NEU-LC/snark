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

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <comma/base/exception.h>
#include "types.h"

namespace snark { namespace camera {

int dc1394color_coding_to_cv_type(dc1394color_coding_t color_coding)
{
    //std::cerr << "Color coding: " << color_coding_to_string( color_coding ) << std::endl;
    switch( color_coding )
    {
        case DC1394_COLOR_CODING_MONO8:
            return CV_8UC1;
        case DC1394_COLOR_CODING_RGB8:
            return CV_8UC3;
        case DC1394_COLOR_CODING_MONO16:
            return CV_16UC1;
        case DC1394_COLOR_CODING_RGB16:
            return CV_16UC3;
        case DC1394_COLOR_CODING_MONO16S:
            return CV_16SC1;
        case DC1394_COLOR_CODING_RGB16S:
            return CV_16SC3;
        case DC1394_COLOR_CODING_RAW8:
            return CV_8UC1;
        case DC1394_COLOR_CODING_RAW16:
            return CV_16UC1;
        case DC1394_COLOR_CODING_YUV411:
        case DC1394_COLOR_CODING_YUV422:
        case DC1394_COLOR_CODING_YUV444:
            COMMA_THROW( comma::exception, "unsupported color coding: " << color_coding);
        default:
            COMMA_THROW( comma::exception, "invalid color coding: " << color_coding);
    }
}


std::string video_mode_to_string ( dc1394video_mode_t mode )
{
    switch( mode )
    {
        case DC1394_VIDEO_MODE_160x120_YUV444:
            return "DC1394_VIDEO_MODE_160x120_YUV444";
        case DC1394_VIDEO_MODE_320x240_YUV422:
            return "DC1394_VIDEO_MODE_320x240_YUV422";
        case DC1394_VIDEO_MODE_640x480_YUV411:
            return "DC1394_VIDEO_MODE_640x480_YUV411";
        case DC1394_VIDEO_MODE_640x480_YUV422:
            return "DC1394_VIDEO_MODE_640x480_YUV422";
        case DC1394_VIDEO_MODE_640x480_RGB8:
            return "DC1394_VIDEO_MODE_640x480_RGB8";
        case DC1394_VIDEO_MODE_640x480_MONO8:
            return "DC1394_VIDEO_MODE_640x480_MONO8";
        case DC1394_VIDEO_MODE_640x480_MONO16:
            return "DC1394_VIDEO_MODE_640x480_MONO16";
        case DC1394_VIDEO_MODE_800x600_YUV422:
            return "DC1394_VIDEO_MODE_800x600_YUV422";
        case DC1394_VIDEO_MODE_800x600_RGB8:
            return "DC1394_VIDEO_MODE_800x600_RGB8";
        case DC1394_VIDEO_MODE_800x600_MONO8:
            return "DC1394_VIDEO_MODE_800x600_MONO8";
        case DC1394_VIDEO_MODE_1024x768_YUV422:
            return "DC1394_VIDEO_MODE_1024x768_YUV422";
        case DC1394_VIDEO_MODE_1024x768_RGB8:
            return "DC1394_VIDEO_MODE_1024x768_RGB8";
        case DC1394_VIDEO_MODE_1024x768_MONO8:
            return "DC1394_VIDEO_MODE_1024x768_MONO8";
        case DC1394_VIDEO_MODE_800x600_MONO16:
            return "DC1394_VIDEO_MODE_800x600_MONO16";
        case DC1394_VIDEO_MODE_1024x768_MONO16:
            return "DC1394_VIDEO_MODE_1024x768_MONO16";
        case DC1394_VIDEO_MODE_1280x960_YUV422:
            return "DC1394_VIDEO_MODE_1280x960_YUV422";
        case DC1394_VIDEO_MODE_1280x960_RGB8:
            return "DC1394_VIDEO_MODE_1280x960_RGB8";
        case DC1394_VIDEO_MODE_1280x960_MONO8:
            return "DC1394_VIDEO_MODE_1280x960_MONO8";
        case DC1394_VIDEO_MODE_1600x1200_YUV422:
            return "DC1394_VIDEO_MODE_1600x1200_YUV422";
        case DC1394_VIDEO_MODE_1600x1200_RGB8:
            return "DC1394_VIDEO_MODE_1600x1200_RGB8";
        case DC1394_VIDEO_MODE_1600x1200_MONO8:
            return "DC1394_VIDEO_MODE_1600x1200_MONO8";
        case DC1394_VIDEO_MODE_1280x960_MONO16:
            return "DC1394_VIDEO_MODE_1280x960_MONO16";
        case DC1394_VIDEO_MODE_1600x1200_MONO16:
            return "DC1394_VIDEO_MODE_1600x1200_MONO16";
        case DC1394_VIDEO_MODE_EXIF:
            return "DC1394_VIDEO_MODE_EXIF";
        case DC1394_VIDEO_MODE_FORMAT7_0:
            return "DC1394_VIDEO_MODE_FORMAT7_0";
        case DC1394_VIDEO_MODE_FORMAT7_1:
            return "DC1394_VIDEO_MODE_FORMAT7_1";
        case DC1394_VIDEO_MODE_FORMAT7_2:
            return "DC1394_VIDEO_MODE_FORMAT7_2";
        case DC1394_VIDEO_MODE_FORMAT7_3:
            return "DC1394_VIDEO_MODE_FORMAT7_3";
        case DC1394_VIDEO_MODE_FORMAT7_4:
            return "DC1394_VIDEO_MODE_FORMAT7_4";
        case DC1394_VIDEO_MODE_FORMAT7_5:
            return "DC1394_VIDEO_MODE_FORMAT7_5";
        case DC1394_VIDEO_MODE_FORMAT7_6:
            return "DC1394_VIDEO_MODE_FORMAT7_6";
        case DC1394_VIDEO_MODE_FORMAT7_7:
            return "DC1394_VIDEO_MODE_FORMAT7_7";
        default:
            COMMA_THROW( comma::exception, "invalid video mode: " << mode);
    }
}

std::string operation_mode_to_string ( dc1394operation_mode_t mode )
{
    switch( mode )
    {
        case DC1394_OPERATION_MODE_LEGACY:
            return "DC1394_OPERATION_MODE_LEGACY";
        case DC1394_OPERATION_MODE_1394B:
            return "DC1394_OPERATION_MODE_1394B";
        default:
            COMMA_THROW( comma::exception, "invalid operation mode: " << mode);
    }
}

std::string iso_speed_to_string ( dc1394speed_t speed )
{
    switch( speed )
    {
        case DC1394_ISO_SPEED_100:
            return "DC1394_ISO_SPEED_100";
        case DC1394_ISO_SPEED_200:
            return "DC1394_ISO_SPEED_200";
        case DC1394_ISO_SPEED_400:
            return "DC1394_ISO_SPEED_400";
        case DC1394_ISO_SPEED_800:
            return "DC1394_ISO_SPEED_800";
        case DC1394_ISO_SPEED_1600:
            return "DC1394_ISO_SPEED_1600";
        case DC1394_ISO_SPEED_3200:
            return "DC1394_ISO_SPEED_3200";
        default:
            COMMA_THROW( comma::exception, "invalid iso speed: " << speed );
    }
}

std::string frame_rate_to_string ( dc1394framerate_t frame_rate )
{    
    switch( frame_rate )
    {
        case DC1394_FRAMERATE_1_875:
            return "DC1394_FRAMERATE_1_875";
        case DC1394_FRAMERATE_3_75:
            return "DC1394_FRAMERATE_3_75";
        case DC1394_FRAMERATE_7_5:
            return "DC1394_FRAMERATE_7_5";
        case DC1394_FRAMERATE_15:
            return "DC1394_FRAMERATE_15";
        case DC1394_FRAMERATE_30:
            return "DC1394_FRAMERATE_30";
        case DC1394_FRAMERATE_60:
            return "DC1394_FRAMERATE_60";
        case DC1394_FRAMERATE_120:
            return "DC1394_FRAMERATE_120";
        case DC1394_FRAMERATE_240:
            return "DC1394_FRAMERATE_240";
        default:
            COMMA_THROW( comma::exception, "invalid frame rate: " << frame_rate);
    }
}

std::string color_coding_to_string( dc1394color_coding_t color_coding )
{
    switch( color_coding )
    {
        case DC1394_COLOR_CODING_MONO8:
            return "DC1394_COLOR_CODING_MONO8";
        case DC1394_COLOR_CODING_YUV411:
            return "DC1394_COLOR_CODING_YUV411";
        case DC1394_COLOR_CODING_YUV422:
            return "DC1394_COLOR_CODING_YUV422";
        case DC1394_COLOR_CODING_YUV444:
            return "DC1394_COLOR_CODING_YUV444";
        case DC1394_COLOR_CODING_RGB8:
            return "DC1394_COLOR_CODING_RGB8";
        case DC1394_COLOR_CODING_MONO16:
            return "DC1394_COLOR_CODING_MONO16";
        case DC1394_COLOR_CODING_RGB16:
            return "DC1394_COLOR_CODING_RGB16";
        case DC1394_COLOR_CODING_MONO16S:
            return "DC1394_COLOR_CODING_MONO16S";
        case DC1394_COLOR_CODING_RGB16S:
            return "DC1394_COLOR_CODING_RGB16S";
        case DC1394_COLOR_CODING_RAW8:
            return "DC1394_COLOR_CODING_RAW8";
        case DC1394_COLOR_CODING_RAW16:
            return "DC1394_COLOR_CODING_RAW16";
        default:
            COMMA_THROW( comma::exception, "invalid color coding: " << color_coding );
    }
}

dc1394video_mode_t video_mode_from_string ( const std::string& mode )
{
    if( mode == "DC1394_VIDEO_MODE_160x120_YUV444" )
    {
        return DC1394_VIDEO_MODE_160x120_YUV444;
    }
    else if( mode == "DC1394_VIDEO_MODE_320x240_YUV422" )
    {
        return DC1394_VIDEO_MODE_320x240_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_640x480_YUV411" )
    {
        return DC1394_VIDEO_MODE_640x480_YUV411;
    }
    else if( mode == "DC1394_VIDEO_MODE_640x480_YUV422" )
    {
        return DC1394_VIDEO_MODE_640x480_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_640x480_RGB8" )
    {
        return DC1394_VIDEO_MODE_640x480_RGB8;
    }
    else if( mode == "DC1394_VIDEO_MODE_640x480_MONO8" )
    {
        return DC1394_VIDEO_MODE_640x480_MONO8;
    }
    else if( mode == "DC1394_VIDEO_MODE_640x480_MONO16" )
    {
        return DC1394_VIDEO_MODE_640x480_MONO16;
    }
    else if( mode == "DC1394_VIDEO_MODE_800x600_YUV422" )
    {
        return DC1394_VIDEO_MODE_800x600_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_800x600_RGB8" )
    {
        return DC1394_VIDEO_MODE_800x600_RGB8;
    }
    else if( mode == "DC1394_VIDEO_MODE_800x600_MONO8" )
    {
        return DC1394_VIDEO_MODE_800x600_MONO8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1024x768_YUV422" )
    {
        return DC1394_VIDEO_MODE_1024x768_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_1024x768_RGB8" )
    {
        return DC1394_VIDEO_MODE_1024x768_RGB8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1024x768_MONO8" )
    {
        return DC1394_VIDEO_MODE_1024x768_MONO8;
    }
    else if( mode == "DC1394_VIDEO_MODE_800x600_MONO16" )
    {
        return DC1394_VIDEO_MODE_800x600_MONO16;
    }
    else if( mode == "DC1394_VIDEO_MODE_1024x768_MONO16" )
    {
        return DC1394_VIDEO_MODE_1024x768_MONO16;
    }
    else if( mode == "DC1394_VIDEO_MODE_1280x960_YUV422" )
    {
        return DC1394_VIDEO_MODE_1280x960_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_1280x960_RGB8" )
    {
        return DC1394_VIDEO_MODE_1280x960_RGB8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1280x960_MONO8" )
    {
        return DC1394_VIDEO_MODE_1280x960_MONO8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1600x1200_YUV422" )
    {
        return DC1394_VIDEO_MODE_1600x1200_YUV422;
    }
    else if( mode == "DC1394_VIDEO_MODE_1600x1200_RGB8" )
    {
        return DC1394_VIDEO_MODE_1600x1200_RGB8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1600x1200_MONO8" )
    {
        return DC1394_VIDEO_MODE_1600x1200_MONO8;
    }
    else if( mode == "DC1394_VIDEO_MODE_1280x960_MONO16" )
    {
        return DC1394_VIDEO_MODE_1280x960_MONO16;
    }
    else if( mode == "DC1394_VIDEO_MODE_1600x1200_MONO16" )
    {
        return DC1394_VIDEO_MODE_1600x1200_MONO16;
    }
    else if( mode == "DC1394_VIDEO_MODE_EXIF" )
    {
        return DC1394_VIDEO_MODE_EXIF;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_0" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_0;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_1" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_1;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_2" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_2;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_3" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_3;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_4" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_4;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_5" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_5;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_6" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_6;
    }
    else if( mode == "DC1394_VIDEO_MODE_FORMAT7_7" )
    {
        return DC1394_VIDEO_MODE_FORMAT7_7;
    }
    else
    {
        COMMA_THROW( comma::exception, "invalid video mode: " << mode);
    }
}

dc1394operation_mode_t operation_mode_from_string ( const std::string& mode )
{
    if( mode == "DC1394_OPERATION_MODE_LEGACY" )
    {
        return DC1394_OPERATION_MODE_LEGACY;
    }
    else if( mode == "DC1394_OPERATION_MODE_1394B" )
    {
        return DC1394_OPERATION_MODE_1394B;
    }
    else
    {
        COMMA_THROW( comma::exception, "invalid operation mode: " << mode );
    }
}

dc1394speed_t iso_speed_from_string ( const std::string& speed )
{
    if( speed == "DC1394_ISO_SPEED_100" )
    {
        return DC1394_ISO_SPEED_100;
    }
    else if( speed == "DC1394_ISO_SPEED_200" )
    {
        return DC1394_ISO_SPEED_200;
    }
    else if( speed == "DC1394_ISO_SPEED_400" )
    {
        return DC1394_ISO_SPEED_400;
    }
    else if( speed == "DC1394_ISO_SPEED_800" )
    {
        return DC1394_ISO_SPEED_800;
    }
    else if( speed == "DC1394_ISO_SPEED_1600" )
    {
        return DC1394_ISO_SPEED_1600;
    }
    else if( speed == "DC1394_ISO_SPEED_3200" )
    {
        return DC1394_ISO_SPEED_3200;
    }
    else
    {
        COMMA_THROW( comma::exception, "invalid iso speed: " << speed );
    }
}

dc1394framerate_t frame_rate_from_string ( const std::string& frame_rate )
{
    if( frame_rate == "DC1394_FRAMERATE_1_875" )
    {
        return DC1394_FRAMERATE_1_875;
    }
    else if( frame_rate == "DC1394_FRAMERATE_3_75" )
    {
        return DC1394_FRAMERATE_3_75;
    }
    else if( frame_rate == "DC1394_FRAMERATE_7_5" )
    {
        return DC1394_FRAMERATE_7_5;
    }
    else if( frame_rate == "DC1394_FRAMERATE_15" )
    {
        return DC1394_FRAMERATE_15;
    }
    else if( frame_rate == "DC1394_FRAMERATE_30" )
    {
        return DC1394_FRAMERATE_30;
    }
    else if( frame_rate == "DC1394_FRAMERATE_60" )
    {
        return DC1394_FRAMERATE_60;
    }
    else if( frame_rate == "DC1394_FRAMERATE_120" )
    {
        return DC1394_FRAMERATE_120;
    }
    else if( frame_rate == "DC1394_FRAMERATE_240" )
    {
        return DC1394_FRAMERATE_240;
    }
    else
    {
        COMMA_THROW( comma::exception, "invalid frame rate: " << frame_rate);
    }
}

dc1394color_coding_t color_coding_from_string( const std::string& color_coding )
{
    if( color_coding == "DC1394_COLOR_CODING_MONO8" )
    {
        return DC1394_COLOR_CODING_MONO8;
    }
    else if( color_coding == "DC1394_COLOR_CODING_YUV411" )
    {
        return DC1394_COLOR_CODING_YUV411;
    }
    else if( color_coding == "DC1394_COLOR_CODING_YUV422" )
    {
        return DC1394_COLOR_CODING_YUV422;
    }
    else if( color_coding == "DC1394_COLOR_CODING_YUV444" )
    {
        return DC1394_COLOR_CODING_YUV444;
    }
    else if( color_coding == "DC1394_COLOR_CODING_RGB8" )
    {
        return DC1394_COLOR_CODING_RGB8;
    }
    else if( color_coding == "DC1394_COLOR_CODING_MONO16" )
    {
        return DC1394_COLOR_CODING_MONO16;
    }
    else if( color_coding == "DC1394_COLOR_CODING_RGB16" )
    {
        return DC1394_COLOR_CODING_RGB16;
    }
    else if( color_coding == "DC1394_COLOR_CODING_MONO16S" )
    {
        return DC1394_COLOR_CODING_MONO16S;
    }
    else if( color_coding == "DC1394_COLOR_CODING_RGB16S" )
    {
        return DC1394_COLOR_CODING_RGB16S;
    }
    else if( color_coding == "DC1394_COLOR_CODING_RAW8" )
    {
        return DC1394_COLOR_CODING_RAW8;
    }
    else if( color_coding == "DC1394_COLOR_CODING_RAW16" )
    {
        return DC1394_COLOR_CODING_RAW16;
    }
    else
    {
        COMMA_THROW( comma::exception, "invalid color coding: \"" << color_coding << "\"" );
    }
}

void print_video_modes()
{
    for( unsigned int mode = DC1394_VIDEO_MODE_MIN; mode <= DC1394_VIDEO_MODE_MAX; mode++  )
    {
        std::cerr << "\t" << video_mode_to_string( static_cast< dc1394video_mode_t >( mode ) ) << std::endl;
    }
}

void print_operation_modes()
{
    std::cerr << "\tDC1394_OPERATION_MODE_LEGACY" << std::endl;
    std::cerr << "\tDC1394_OPERATION_MODE_1394B"  << std::endl;
}

void print_iso_speeds()
{
    for( unsigned int speed = DC1394_ISO_SPEED_MIN; speed <= DC1394_ISO_SPEED_MAX; speed++  )
    {
        std::cerr << "\t" << iso_speed_to_string( static_cast< dc1394speed_t >( speed ) ) << std::endl;
    }
}

void print_frame_rates()
{
    for( unsigned int rate = DC1394_FRAMERATE_MIN; rate <= DC1394_FRAMERATE_MAX; rate++  )
    {
        std::cerr << "\t" << frame_rate_to_string( static_cast< dc1394framerate_t >( rate ) ) << std::endl;
    }
}

void print_color_coding()
{
    for( unsigned int color_coding = DC1394_COLOR_CODING_MIN; color_coding <= DC1394_COLOR_CODING_MAX; color_coding++  )
    {
        std::cerr << "\t" << color_coding_to_string( static_cast< dc1394color_coding_t >( color_coding ) ) << std::endl;
    }
    std::cerr << "\t" << "Note: support for YUV modes not yet implemented" << std::endl;
}

} } // namespace snark { namespace camera {
