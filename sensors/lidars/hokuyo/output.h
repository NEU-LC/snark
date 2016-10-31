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

#ifndef SNARK_SENSORS_HOKUYO_OUTPUT_H
#define SNARK_SENSORS_HOKUYO_OUTPUT_H

#include <boost/static_assert.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <comma/base/types.h>
#include "sensors.h"

namespace snark { namespace hokuyo {

/// Represent a point relative to the laser's coordinate frame.
/// Note z and elevation are always zero as laser shoots out horizontally.
struct data_point
{
    data_point() : t( boost::posix_time::microsec_clock::universal_time() ), 
        x(0), y(0), z(0), range(0), bearing(0), elevation(0), intensity(0) {}
    
    bool is_nan() const { return ( x == 0 && y == 0 && z == 0 ); }
    
    /// Set the data point
    /// distance in meters, bearing in (radians)
    void set( double distance, comma::uint32 intensity, double bearing );
    
    boost::posix_time::ptime t;
    double x;
    double y;
    double z;
    double range; // meters
    double bearing; //radians
    double elevation;   // radians
/// Intensity is the reflected strength of the laser.
/// The reflected laser intensity value is represented by 18- bit data. It is a relative number without a unit.
/// Intensity may differ depending upon the distance, material and detection angle of the object. Therefore, users
/// should check the detection capability verification test.
    comma::uint32 intensity;   /// This is a relative, unit less number that is 18-bit
};
    
} } // namespace snark { namespace hokuyo {
    
    
#endif // SNARK_SENSORS_HOKUYO_OUTPUT_H
