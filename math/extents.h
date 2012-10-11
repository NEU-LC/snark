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

#ifndef SNARK_MATH_EXTENTS_HEADER_GUARD_
#define SNARK_MATH_EXTENTS_HEADER_GUARD_

#include <boost/optional.hpp>
#include <comma/math/compare.h>
#include <snark/math/traits.h>
#include <comma/visiting/visit.h>

namespace snark {

/// a utility class to find extents of a set of array-like objects (e.g. points)
template < typename P >
class extents
{
public:
    /// extents pair type
    typedef std::pair< P, P > pair;
    
    /// constructor from min and max
    extents( const P& min, const P& max );
    
    /// constructor from a container
    template < typename It >
    extents( It begin, It end );
    
    /// constructor from min/max pair
    extents( const pair& extents );
    
    /// default constructor
    extents();
    
    /// copy constructor
    extents( const extents& rhs );
    
    /// add point
    const pair& add( const P& p );
    
    /// add extents
    const pair& add( const extents< P >& rhs );
    
    /// return extents, throw, if no points have been added yet
    const pair& operator()() const { return *m_extents; }

    /// return min
    const P& min() const { return m_extents->first; }

    /// return max
    const P& max() const { return m_extents->second; }
    
    /// return box size, i.e. max - min
    P boxSize() const { return m_extents->second - m_extents->first; }

    /// return number of points added
    std::size_t size() const { return m_size; }
    
    /// return true, if extents contain given value
    bool has( const P& p ) const;
    
private:
    boost::optional< pair > m_extents;
    std::size_t m_size;
};

/// output operator
template < typename Stream, typename P >
Stream& operator<<( Stream& lhs, const extents< P >& rhs )
{
    lhs << rhs().first << "," << rhs().second;
    return lhs;
}

template < typename P >
extents< P >::extents() : m_size( 0 ) {}

template < typename P >
extents< P >::extents( const extents< P >& rhs ) { this->operator=( rhs ); }

template < typename P >
extents< P >::extents( const P& min, const P& max )
    : m_size( 0 ) /// @todo what does size need to be?
{
    add( min );
    add( max );
}

template < typename P >
extents< P >::extents( const typename extents< P >::pair& extents )
    : m_extents( extents )
    , m_size( 0 ) /// @todo what does size need to be?
{
}

template < typename P >
const typename extents< P >::pair& extents< P >::add( const P& p )
{
    if( !m_extents )
    {
        m_extents = std::make_pair( p, p );
    }
    else
    {
        for( unsigned int i = 0; i < math::traits< P >::size; ++i )
        {
            if( comma::math::less( p[i], m_extents->first[i] ) ) { m_extents->first[i] = p[i]; }
            else if( comma::math::less( m_extents->second[i], p[i] ) ) { m_extents->second[i] = p[i]; }
        }
    }
    ++m_size;
    return *m_extents;
}

template < typename P >
const typename extents< P >::pair& extents< P >::add( const extents< P >& rhs )
{
    add( rhs.min() );
    add( rhs.max() );
    m_size += rhs.m_size;
    return *m_extents;
}

template < typename P >
bool extents< P >::has( const P& p ) const
{
    extents< P > e( *this );
    e.add( p );
    return comma::math::equal( this->min(), e.min() ) && comma::math::equal( this->max(), e.max() );}

template < typename P >
template < typename It >
extents< P >::extents( It begin, It end )
{
    for( It it = begin; it != end; ++it ) { add( *it ); }
}  

} // namespace snark{ 

namespace comma { namespace visiting {

/// visiting traits
template < typename T > struct traits< snark::extents< T > >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::extents< T >& p, Visitor& v ) // very quick and dirty, very non-generic; need to think how to do it better
    {
        if( p.size() == 0 ) // real frigging quick and dirty, just to make things working again and think later
        {
            T t;
            v.apply( "min", t );
            v.apply( "max", t );
        }
        else
        {
            v.apply( "min", p.min() );
            v.apply( "max", p.max() );
        }
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( Key, snark::extents< T >& p, Visitor& v ) // very quick and dirty, very non-generic; need to think how to do it better
    {
        snark::extents< T > e;
        T t;
        v.apply( "min", t );
        e.add( t );
        v.apply( "max", t );
        e.add( t );
        p = e;
    }
};

} } // namespace comma { namespace visiting {

#endif // #ifndef SNARK_MATH_EXTENTS_HEADER_GUARD_
