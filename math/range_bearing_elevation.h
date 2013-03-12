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
    static range_bearing_elevation from_cartesian( double x, double y, double z );
    static range_bearing_elevation from_cartesian( Eigen::Vector3d xyz );

private:
    Eigen::Vector3d m_rbe;
};

void range_bearing_elevation::range( double t )
{
    if( !comma::math::less( t, 0 ) )
    {
        m_rbe[0] = t;
    }
    else
    {
        m_rbe[0] = -t;
        m_rbe[1] = -m_rbe[1];
        m_rbe[2] = -m_rbe[2];
    }
}

void range_bearing_elevation::bearing( double t )
{
    double b( std::fmod( t, ( double )( M_PI * 2 ) ) );
    if( !comma::math::less( b, M_PI ) ) { b -= ( M_PI * 2 ); }
    else if( comma::math::less( b, -M_PI ) ) { b += ( M_PI * 2 ); }
    m_rbe[1] = b;
}

void range_bearing_elevation::elevation( double t )
{
    double e( std::fmod( t, ( double )( M_PI * 2 ) ) );
    if( comma::math::less( e, 0 ) ) { e += M_PI * 2; }
    if( !comma::math::less( e, M_PI / 2 ) )
    {
        if( !comma::math::less( e, M_PI ) )
        {
            if( !comma::math::less( e, M_PI * 3 / 2 ) )
            {
                e = e - M_PI * 2;
            }
            else
            {
                e = M_PI - e;
                bearing( bearing() + M_PI );
            }
        }
        else
        {
            e = M_PI - e;
            bearing( bearing() + M_PI );
        }
    }
    m_rbe[2] = e;
}

Eigen::Vector3d range_bearing_elevation::to_cartesian() const
{
    double xyProjection( range() * std::cos( elevation() ) );
    return ::Eigen::Matrix< double, 3, 1 >( xyProjection * std::cos( bearing() )
                                    , xyProjection * std::sin( bearing() )
                                    , range() * std::sin( elevation() ) );
}

range_bearing_elevation range_bearing_elevation::from_cartesian( Eigen::Vector3d xyz )
{
    return from_cartesian( xyz[0], xyz[1], xyz[2] );
}

range_bearing_elevation range_bearing_elevation::from_cartesian( double x, double y, double z )
{
    long double projectionSquare ( x*x +  y*y );
    if ( comma::math::equal( projectionSquare, 0 ) )
    {
        if ( comma::math::equal( z, 0 ) ) { return range_bearing_elevation(); }
        return range_bearing_elevation( std::abs( z ), 0, comma::math::less( z, 0 ) ? -M_PI / 2 : M_PI / 2 );
    }
    long double rangeSquare( projectionSquare + z * z );
    long double range( std::sqrt( rangeSquare ) );
    long double elevation = 0;
    if ( !comma::math::equal( z, 0 ) )
    {
        long double r = z / range;
        if ( comma::math::less( ( long double )( 1.0 ), r ) ) { r = 1; }
        else if ( comma::math::less( r, ( long double ) ( -1.0 ) ) ) { r = -1; }
        elevation = std::asin( r );
    }

    long double r = x / std::sqrt( projectionSquare );
    if ( comma::math::less( ( long double )( 1.0 ), r ) ) { r = 1; }
    else if ( comma::math::less( r, ( long double ) ( -1.0 ) ) ) { r = -1; }
    long double bearing = std::acos( r );
    if ( comma::math::less( y, 0 ) ) { bearing = M_PI * 2 - bearing; }
    return range_bearing_elevation( range, bearing, elevation );
}

} // namespace snark {

#endif // SNARK_MATH_RBE_H
