// This file is part of Ark, a generic and flexible library 
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// Ark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Ark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License 
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with Ark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_MATH_ANGLE_H_
#define SNARK_MATH_ANGLE_H_

#include <comma/math/cyclic.h>

namespace snark{ namespace math{

/// forward declaration
struct degrees;

/// strongly typed radians 
struct radians
{
    double value;
    explicit radians( double d ) : value( d ) {}
    radians( degrees d );
};

/// strongly typed degrees
struct degrees
{
    double value;
    explicit degrees( double d ) : value( d ) {}
    degrees( radians r );
};

/// angle varying between 0 and 360 degrees
/// @todo phase out radians and degrees and use boost::quantity and boost::units
/// @todo add constructors (or template parameters) for angles varying from -180 to 180, etc
template < typename T >
class angle : public comma::math::cyclic< T >
{
    public:
        /// constructors
        angle() : comma::math::cyclic< T >( comma::math::interval< T >( 0, 360 ), 0 ) {}
        angle( degrees d ) : comma::math::cyclic< T >( comma::math::interval< T >( 0, 360 ), static_cast< T >( d.value ) ) {}
        angle( radians d ) : comma::math::cyclic< T >( comma::math::interval< T >( 0, 360 ), static_cast< T >( degrees( d ).value ) ) {}
        angle( const angle& rhs ) : comma::math::cyclic< T >( rhs.interval(), rhs() ) {}
    
        /// accessors
        T asdegrees() const { return comma::math::cyclic< T >::operator()(); }
        double asradians() const { return math::radians( degrees(comma::math::cyclic< T >::operator()()) ).value; }
    
        /// operators
        const angle& operator+=( const angle& rhs ) { comma::math::cyclic< T >::operator+=( rhs ); return *this; }
        const angle& operator-=( const angle& rhs ) { comma::math::cyclic< T >::operator-=( rhs ); return *this; }
        const angle& operator+=( T t ) { comma::math::cyclic< T >::operator+=( t ); return *this; }
        const angle& operator-=( T t ) { comma::math::cyclic< T >::operator-=( t ); return *this; }
        const angle& operator*=( T t ) { comma::math::cyclic< T >::operator=( comma::math::cyclic< T >::operator()() * t ); return *this; }
        const angle& operator/=( T t ) { comma::math::cyclic< T >::operator=( comma::math::cyclic< T >::operator()() / t ); return *this; }
        angle operator+( const angle& rhs ) const { angle a( *this ); a += rhs; return a; }
        angle operator-( const angle& rhs ) const { angle a( *this ); a -= rhs; return a; }
        angle operator+( T rhs ) const { angle a( *this ); a += rhs; return a; }
        angle operator-( T rhs ) const { angle a( *this ); a -= rhs; return a; }
        angle operator*( T rhs ) const { angle a( *this ); a *= rhs; return a; }
        angle operator/( T rhs ) const { angle a( *this ); a /= rhs; return a; }
    
    private:
        T operator()() const { return comma::math::cyclic< T >::operator()(); }
};

inline radians::radians( degrees d ) { value = d.value * static_cast< double >( M_PI ) / 180; }

inline degrees::degrees( radians d ) { value = d.value * 180 / static_cast< double >( M_PI ); }

} } // namespace snark{ namespace math{

#endif // SNARK_MATH_ANGLE_H_
