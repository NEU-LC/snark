#include <snark/sensors/dc1394/types.h>
#include <comma/base/exception.h>
#include <iostream>

namespace snark { namespace camera {

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
            COMMA_THROW( comma::exception, "invalid video mode");
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
            COMMA_THROW( comma::exception, "invalid operation mode");
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
            COMMA_THROW( comma::exception, "invalid iso speed");
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
            COMMA_THROW( comma::exception, "invalid frame rate");
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
        COMMA_THROW( comma::exception, "invalid video mode");
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
        COMMA_THROW( comma::exception, "invalid operation mode");
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
        COMMA_THROW( comma::exception, "invalid iso speed");
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
        COMMA_THROW( comma::exception, "invalid frame rate");
    }
}

void print_video_modes()
{
    for( unsigned int mode = DC1394_VIDEO_MODE_MIN; mode < DC1394_VIDEO_MODE_MAX; mode++  )
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
    for( unsigned int speed = DC1394_ISO_SPEED_MIN; speed < DC1394_ISO_SPEED_MAX; speed++  )
    {
        std::cerr << "\t" << iso_speed_to_string( static_cast< dc1394speed_t >( speed ) ) << std::endl;
    }
}

void print_frame_rates()
{
    for( unsigned int rate = DC1394_FRAMERATE_MIN; rate < DC1394_FRAMERATE_MAX; rate++  )
    {
        std::cerr << "\t" << frame_rate_to_string( static_cast< dc1394framerate_t >( rate ) ) << std::endl;
    }
}




} } // namespace snark { namespace camera {