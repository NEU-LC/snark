// Copyright (c) 2021 Mission Systems Pty Ltd

#pragma once

#include <inno_lidar_api.h>
#include <string>

namespace snark { namespace innovusion {

// thin wrapper around the Innovusion API

class lidar
{
public:
    lidar();
    ~lidar();

    void init( const std::string& name
             , const std::string& address, unsigned int port
             , inno_lidar_alarm_callback_t alarm_callback
             , inno_lidar_frame_callback_t frame_callback );
    void start();

private:
    int handle;
};

} } // namespace snark { namespace innovusion {
