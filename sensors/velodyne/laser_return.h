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


#ifndef SNARK_SENSORS_VELODYNE_LASERRETURN_H_
#define SNARK_SENSORS_VELODYNE_LASERRETURN_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/types.h>
#include <comma/visiting/traits.h>

namespace snark { namespace velodyne {

/// velodyne point corresponding to a single laser return
struct laser_return
{
    /// timestamp
    boost::posix_time::ptime timestamp;
    
    /// laser id
    comma::uint32 id;
    
    /// intensity
    unsigned char intensity;
    
    /// range (in metres, without range correction)
    double range;
    
    /// azimuth (in degrees, without angle correction)
    double azimuth;
};

} } // namespace snark  { namespace velodyne {

namespace comma { namespace visiting {

template <> struct traits< snark::velodyne::laser_return >
{
    template < typename Key, class Visitor >
    static void visit( Key k, snark::velodyne::laser_return& t, Visitor& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "id", t.id );
        v.apply( "intensity", t.intensity );
        v.apply( "range", t.range );
        v.apply( "azimuth", t.azimuth );
    }
    
    template < typename Key, class Visitor >
    static void visit( Key k, const snark::velodyne::laser_return& t, Visitor& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "id", t.id );
        v.apply( "intensity", t.intensity );
        v.apply( "range", t.range );
        v.apply( "azimuth", t.azimuth );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_SENSORS_VELODYNE_LASERRETURN_H_*/
