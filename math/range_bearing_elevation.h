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

#ifndef SNARK_MATH_RBE_H
#define SNARK_MATH_RBE_H

#include <Eigen/Core>
#include <comma/math/compare.h>

namespace snark {

/// polar point definition (range-bearing-elevation)
class range_bearing_elevation
{
public:
    /// constructors
    range_bearing_elevation(){}
    range_bearing_elevation( double r, double b, double e ): m_rbe( r, b, e ) {}

    /// return coordinates
    double range() const { return m_rbe[0]; }
    double bearing() const { return m_rbe[1]; }
    double elevation() const { return m_rbe[2]; }

    /// for brevity's sake
    double r() const { return range(); }
    double b() const { return bearing(); }
    double e() const { return elevation(); }

    /// set coordinates
    void range( double t );
    void bearing( double t );
    void elevation( double t );

    Eigen::Vector3d to_cartesian() const;
    const range_bearing_elevation& from_cartesian( double x, double y, double z );
    const range_bearing_elevation& from_cartesian( const Eigen::Vector3d& xyz );

private:
    Eigen::Vector3d m_rbe;
};

} // namespace snark {

#endif // SNARK_MATH_RBE_H
