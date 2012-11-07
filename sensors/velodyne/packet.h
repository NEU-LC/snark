// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_VELODYNE_PACKET_H_
#define SNARK_SENSORS_VELODYNE_PACKET_H_

#include <boost/array.hpp>
#include <boost/static_assert.hpp>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark {  namespace velodyne {

struct packet : public comma::packed::packed_struct< packet, 1206 >
{
    struct laser_return : public comma::packed::packed_struct< laser_return, 3 >
    {
        comma::packed::uint16 range;
        comma::packed::byte intensity;
    };
    
    struct laser_block : public comma::packed::packed_struct< laser_block, 2 + 2 + 32 * sizeof( laser_return ) >
    {
        comma::packed::string< 2 > id;
        comma::packed::uint16 rotation;
        boost::array< laser_return, 32 > lasers;
    };
    
    struct status : public comma::packed::packed_struct< status, 6 >
    {
        struct temperature : public comma::packed::packed_struct< temperature, 6 >
        {
            comma::packed::byte fractions;
            comma::packed::byte degrees;
            comma::packed::string< 4 > text;
            bool valid() const { return text() == "DegC"; }
        };
    
        struct version : public comma::packed::packed_struct< version, 6 >
        {
            comma::packed::string< 2 > padding;
            comma::packed::uint16 counter;
            comma::packed::uint16 number;
            bool valid() const { return ::memcmp( data() + 2, "DegC", 4 ) != 0; }
        };
    
        template < class T > const T& as() const { return reinterpret_cast< const T& >( *this ); }
        template < class T > T& As() { return reinterpret_cast< T& >( *this ); }
        boost::array< comma::packed::byte, 6 > value;
    };
    
    static const char* upper_block_id() { return "\xFF\xEE"; }
    
    static const char* lower_block_id() { return "\xFF\xDD"; }

    boost::array< laser_block, 12 > blocks;
    status status;
};

} } // namespace snark {  namespace velodyne {

#endif /*SNARK_SENSORS_VELODYNE_PACKET_H_*/
