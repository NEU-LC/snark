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

#ifndef SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_
#define SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_

#include <snark/timing/time.h>
#include <comma/visiting/traits.h>

namespace snark{ namespace applications {

/// a quick helper class for simple input/output of timestamped values
template < typename T, typename TimeType = boost::posix_time::ptime >
struct timestamped
{
    TimeType t;
    T value;
};

namespace detail {

inline std::string toString( const boost::posix_time::ptime& t ) { return boost::posix_time::to_iso_string( t ); }

} // namespace detail {

} } // namespace snark{ namespace applications {

template < typename T, typename S >
inline std::ostream& operator<<( std::ostream& os, const snark::applications::timestamped< T, S >& rhs ) { os << snark::applications::detail::toString( rhs.t ) << "," << rhs.value; return os; }

namespace comma { namespace visiting {

template < typename T, typename S > struct traits< snark::applications::timestamped< T, S > >
{
    template < typename Key, class Visitor >
    static void visit( Key, const snark::applications::timestamped< T, S >& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "value", p.value );
    }
    
    template < typename Key, class Visitor >
    static void visit( Key, snark::applications::timestamped< T, S >& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "value", p.value );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_*/
