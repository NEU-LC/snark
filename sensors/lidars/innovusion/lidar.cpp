// Copyright (c) 2020 Mission Systems Pty Ltd

#include "lidar.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>

namespace snark { namespace innovusion {

lidar::lidar()
    : handle( 0 )
{}

lidar::~lidar()
{
    if( handle != 0 )
    {
        inno_lidar_stop( handle );
        inno_lidar_close( handle );
    }
}

void lidar::init( const std::string& name
                , const std::string& address, unsigned int port
                , inno_lidar_alarm_callback_t alarm_callback
                , inno_lidar_frame_callback_t frame_callback )
{
    handle = inno_lidar_open_live( name.c_str(), address.c_str(), port, 1 );

    // TODO: set model (not required for live?) and config yaml file
    if( inno_lidar_set_parameters( handle, "", "", "" ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_parameters() failed" ); }

    if( inno_lidar_set_reflectance_mode( handle, REFLECTANCE_MODE_INTENSITY ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_reflectance_mode() failed" ); }

    if( inno_lidar_set_callbacks( handle, alarm_callback, frame_callback, nullptr, nullptr ) != 0 )
    { COMMA_THROW( comma::exception, "inno_lidar_set_callbacks() failed" ); }
}

void lidar::start()
{
    if( inno_lidar_start( handle ) != 0 ) { COMMA_THROW( comma::exception, "inno_lidar_start() failed" ); }
}

} } // namespace snark { namespace innovusion {
