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

#ifndef SNARK_MATH_SPHERICAL_GEOMETRY_COORDINATES_H_
#define SNARK_MATH_SPHERICAL_GEOMETRY_COORDINATES_H_

#include "../range_bearing_elevation.h"
#include "../angle.h"

namespace snark { namespace spherical {

/// coordinates, quick and dirty
struct coordinates
{
    double latitude;
    
    double longitude;
    
    static const double epsilon; // WAY quick and dirty

    coordinates() : latitude( 0 ), longitude( 0 ) {}
    
    coordinates( const double latitude, const double longitude );
    
    coordinates( const snark::bearing_elevation& be );
    
    coordinates( const Eigen::Vector3d& xyz );
    
    operator Eigen::Vector3d() const { return to_cartesian(); }
    
    bool is_near( const Eigen::Vector3d& c, double _epsilon = 0.0005 ) const;
    
    bool operator!=( const Eigen::Vector3d& rhs ) const { return !operator==( rhs ); }
    
    bool operator==( const Eigen::Vector3d& rhs ) const { return is_near( rhs, coordinates::epsilon ); }

    bool operator==( const coordinates& rhs ) const;
    
    bool operator!=( const coordinates& rhs ) const { return !operator==( rhs ); }

    /// @return true if the two coordinates are close to each other (within +/-epsilon)
    /// @note this is highly approximate, since longitude distorts with latitude
    /// @note an espilon of 0.0003 is about 1 minute of latitude
    bool is_near( const coordinates& c, double _epsilon = 0.0005 ) const;

    snark::bearing_elevation as_bearing_elevation() const { return snark::bearing_elevation( longitude, latitude ); }
    
    Eigen::Vector3d to_cartesian() const { return snark::range_bearing_elevation( 1, longitude, latitude ).to_cartesian(); }

    coordinates& operator+=( const coordinates& rhs );

    coordinates operator+( const coordinates& rhs ) const { coordinates r = *this; r += rhs; return r; }
    
    coordinates& operator-=( const coordinates& rhs ) { return operator+=( spherical::coordinates( -rhs.latitude, -rhs.longitude ) ); }

    coordinates operator-( const coordinates& rhs ) const { coordinates r = *this; r -= rhs; return r; }

    static coordinates from_degrees( double latitude, double longitude );

    std::pair< double, double > to_degrees() const;

    operator std::string() const;

    static bool is_near( const coordinates & l, const coordinates & r, double _epsilon = 0.005 );

    static size_t shortcut_latitude;
    static size_t shortcut_longitude;
    static size_t shortcut_none;
};

/// convert to navigation frame at given coordinates
/// @param c coordinates
/// @param v vector to convert
/// @return converted vector
Eigen::Vector3d to_navigation_frame( const coordinates& c, const Eigen::Vector3d& v );

} } // namespace snark { namespace spherical {

#endif // SNARK_MATH_SPHERICAL_GEOMETRY_COORDINATES_H_
