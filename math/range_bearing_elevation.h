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

namespace snark{ 

/// polar point definition (range-bearing-elevation)
class range_bearing_elevation
{
public:
    /// constructors
    range_bearing_elevation(){}
    range_bearing_elevation( double r, double b, double e ): m_rbe( r, b, e ) {}

    /// return coordinates
    double range() const { return m_rbe[0]; }
    double bearing() const { return m_rbe[0]; }
    double elevation() const { return m_rbe[0]; }

    /// for brevity's sake
    double r() const { return range(); }
    double b() const { return bearing(); }
    double e() const { return elevation(); }

    /// set coordinates
    void range( double t );
    void bearing( double t );
    void elevation( double t );

    Eigen::Vector3d to_cartesian() const;
    
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

} 

namespace comma { namespace visiting {

/// visiting traits
template <>
struct traits< snark::range_bearing_elevation >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::range_bearing_elevation& p, Visitor& v )
    {
        v.apply( "range", p.range() );
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( Key, snark::range_bearing_elevation& p, Visitor& v )
    {
        double r;
        double b;
        double e;
        v.apply( "range", r );
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::range_bearing_elevation( r, b, e );
    }
};

} } // namespace comma { namespace visiting {
  
#endif // SNARK_MATH_RBE_H
