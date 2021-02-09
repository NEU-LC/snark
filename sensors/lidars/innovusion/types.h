// Copyright (c) 2021 Mission Systems Pty Ltd

#pragma once

#include <inno_lidar_api.h>
#include <comma/base/types.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace snark { namespace innovusion {

struct output_data_t
{
    boost::posix_time::ptime t;
    comma::uint32 block_id;
    float x;
    float y;
    float z;
    float radius;
    comma::uint16 value;                // reflectance or intensity

    output_data_t()
        : block_id( 0 )
        , x( 0 )
        , y( 0 )
        , z( 0 )
        , radius( 0 )
        , value( 0 )
    {}

    output_data_t( const inno_point& point, comma::uint32 block_id );
};

} } // namespace snark { namespace innovusion {
