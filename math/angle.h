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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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
        T as_degrees() const { return comma::math::cyclic< T >::operator()(); }
        double as_radians() const { return math::radians( degrees(comma::math::cyclic< T >::operator()()) ).value; }
    
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

inline radians::radians( degrees d ) { value = static_cast< long double >( d.value ) * M_PI / 180; }

inline degrees::degrees( radians d ) { value = static_cast< long double >( d.value ) * 180 / M_PI; }

} } // namespace snark{ namespace math{

#endif // SNARK_MATH_ANGLE_H_
