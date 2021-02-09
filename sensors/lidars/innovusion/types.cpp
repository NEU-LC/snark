// Copyright (c) 2021 Mission Systems Pty Ltd

#include "types.h"

namespace snark { namespace innovusion {

output_data_t::output_data_t( const inno_point& point, comma::uint32 block_id_ )
{
    t = boost::posix_time::microsec_clock::universal_time();
    block_id = block_id_;
    x = point.x;
    y = point.y;
    z = point.z;
    radius = point.radius;
    value = point.ref;
}

} } // namespace snark { namespace innovusion {
